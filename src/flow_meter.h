// FlowMeter / water volume tracker.
// Per acid-dosing-fw-specification.md 3.4, 4.4, 4.5 and the Step 2 prompt.
//
// GPIO4 falling-edge hardware interrupt increments a volatile 32-bit pulse
// counter (debounced to a 50 microsecond minimum interval). The counter is
// 32-bit, so aligned loads/stores on the ESP32-C3 (RV32) are atomic and no
// critical section is needed for main-loop reads. Volume tracking,
// dose-base handling and flow calibration live in the main loop. The ISR does
// no logging, math, heap, NVS, Web or MQTT work.
#ifndef ACID_DOSE_FLOW_METER_H
#define ACID_DOSE_FLOW_METER_H

#include <Arduino.h>

#include <cstdint>

class ConfigStore;

class FlowMeter {
public:
    // Attaches the GPIO4 falling-edge interrupt. Call after Hal::setup().
    void begin();

    // Detaches the interrupt. Pulse counter state is preserved.
    void stop();

    // Calibration factor. Main loop only.
    void setCalibration(float pulsesPerLiter);
    float getPulsesPerLiter() const { return calibrationPpl_; }

    // 32-bit counters are read atomically on the ESP32-C3: no critical
    // section needed.
    uint32_t getPulseCount();
    uint32_t getDoseBasePulseCount();

    float getWaterVolumeLitersSinceDose();
    float getBootWaterVolumeLiters();
    float getEstimatedWaterMillilitersSinceDose();

    // Sets the dose base to the current pulse count. Call after a
    // successful dose.
    void resetDoseBase();

    // Flow calibration. All main loop only.
    void beginFlowCalibration();
    uint32_t getCalibrationPulsesSinceStart();
    // Validates the measured volume and the resulting pulses/liter, then
    // persists the new calibration through ConfigStore on success and
    // updates the RAM calibration. Always ends the calibration session.
    bool completeFlowCalibration(ConfigStore &store, float measuredVolumeLiters);
    void cancelFlowCalibration();
    bool isCalibrationActive() const { return calibrationActive_; }
    // Last successfully completed pulses/liter value (0 if none yet).
    float getLastCalibratedPulsesPerLiter() const { return lastResultPpl_; }

    // Main-loop hook: refreshes bookkeeping (flow-freshness tracking).
    void update(uint32_t nowMs);
    // True when the last accepted pulse is younger than maxAgeMs.
    bool isFresh(uint32_t maxAgeMs) const;

#ifdef ENABLE_TEST_INJECTION
    // Test hook: adds N virtual pulses to the main pulse counter.
    // Compiled out entirely when ENABLE_TEST_INJECTION is not defined.
    void testInject(uint32_t pulses);
#endif

private:
    float calibrationPpl_ = 0.0f;
    float lastResultPpl_ = 0.0f;
    bool calibrationActive_ = false;
    uint32_t lastSeenCount_ = 0;
    uint32_t lastFlowActiveMs_ = 0;
};

#endif /* ACID_DOSE_FLOW_METER_H */
