#include "flow_meter.h"

#include <math.h>

#include "config_store.h"
#include "constants.h"
#include "pins.h"

// Volatile shared state (specification section 4.4).
// Counters are 32-bit: on the ESP32-C3 (RV32) aligned 32-bit loads and
// stores are atomic, so the ISR increments and main-loop reads need no
// critical section.
static volatile uint32_t gFlowPulseCount = 0;
static volatile uint32_t gFlowDoseBase = 0;
static volatile uint32_t gFlowCalStartPulse = 0;
static volatile uint32_t gFlowLastAcceptedUs = 0;

// Flow-meter pulse ISR (GPIO4, falling edge).
// Budget: one micros() read, one interval comparison, one counter increment.
// No logging, no math beyond the debounce compare, no heap, no NVS,
// no Web, no MQTT. Must stay IRAM_ATTR.
static void IRAM_ATTR flowPulseIsr() {
    const uint32_t nowUs = micros();
    if (nowUs - gFlowLastAcceptedUs < constants::FLOW_MIN_PULSE_INTERVAL_US) {
        return; // bounce / noise
    }
    gFlowLastAcceptedUs = nowUs;
    gFlowPulseCount++;
}

void FlowMeter::begin() {
    pinMode(PIN_FLOW_PULSE, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_PULSE), flowPulseIsr, FALLING);
}

void FlowMeter::stop() {
    detachInterrupt(digitalPinToInterrupt(PIN_FLOW_PULSE));
}

void FlowMeter::setCalibration(float pulsesPerLiter) {
    if (isfinite(pulsesPerLiter) && pulsesPerLiter > 0.0f) {
        calibrationPpl_ = pulsesPerLiter;
    }
}

// 32-bit aligned reads on RV32 are atomic: no critical section needed.
uint32_t FlowMeter::getPulseCount() {
    return gFlowPulseCount;
}

uint32_t FlowMeter::getDoseBasePulseCount() {
    return gFlowDoseBase;
}

float FlowMeter::getWaterVolumeLitersSinceDose() {
    // Snapshot count, then base: worst case a pulse lands between the two
    // reads, so the volume is off by one pulse at most.
    const uint32_t count = gFlowPulseCount;
    const uint32_t base = gFlowDoseBase;
    if (calibrationPpl_ <= 0.0f) {
        return 0.0f;
    }
    return (float)(count - base) / calibrationPpl_;
}

float FlowMeter::getBootWaterVolumeLiters() {
    if (calibrationPpl_ <= 0.0f) {
        return 0.0f;
    }
    return (float)getPulseCount() / calibrationPpl_;
}

float FlowMeter::getEstimatedWaterMillilitersSinceDose() {
    return getWaterVolumeLitersSinceDose() * 1000.0f;
}

void FlowMeter::resetDoseBase() {
    gFlowDoseBase = gFlowPulseCount;
}

void FlowMeter::beginFlowCalibration() {
    gFlowCalStartPulse = gFlowPulseCount;
    calibrationActive_ = true;
}

uint32_t FlowMeter::getCalibrationPulsesSinceStart() {
    return gFlowPulseCount - gFlowCalStartPulse;
}

bool FlowMeter::completeFlowCalibration(ConfigStore &store, float measuredVolumeLiters) {
    if (!calibrationActive_) {
        return false;
    }

    // Input validation (Step 2 prompt requirement 10).
    if (!isfinite(measuredVolumeLiters) ||
        measuredVolumeLiters <= 0.0f ||
        measuredVolumeLiters > constants::MAX_FLOW_CAL_VOLUME_LITER) {
        cancelFlowCalibration();
        return false;
    }

    const uint32_t deltaPulses = getCalibrationPulsesSinceStart();
    if (deltaPulses == 0) {
        cancelFlowCalibration();
        return false;
    }

    const float pulsesPerLiter = (float)deltaPulses / measuredVolumeLiters;
    const bool valid = isfinite(pulsesPerLiter) &&
        pulsesPerLiter > constants::MIN_FLOW_PULSES_PER_LITER &&
        pulsesPerLiter <= constants::MAX_FLOW_PULSES_PER_LITER;

    bool ok = false;
    if (valid && store.setFlowCalibration(pulsesPerLiter)) {
        setCalibration(pulsesPerLiter);
        lastResultPpl_ = pulsesPerLiter;
        ok = true;
    }

    // Always end the session; on failure the previous value is retained.
    cancelFlowCalibration();
    return ok;
}

void FlowMeter::cancelFlowCalibration() {
    calibrationActive_ = false;
}

void FlowMeter::update(uint32_t nowMs) {
    const uint32_t count = getPulseCount();
    if (count != lastSeenCount_) {
        lastSeenCount_ = count;
        lastFlowActiveMs_ = nowMs;
    }
}

bool FlowMeter::isFresh(uint32_t maxAgeMs) const {
    const uint32_t nowMs = millis();
    if (lastFlowActiveMs_ == 0) {
        return false;
    }
    return (nowMs - lastFlowActiveMs_) < maxAgeMs;
}

#ifdef ENABLE_TEST_INJECTION
void FlowMeter::testInject(uint32_t pulses) {
    gFlowPulseCount += pulses;
}
#endif
