// StepperEngine - DRV8825 STEP/DIR/ENABLE control with linear ramp.
// Per acid-dosing-fw-specification.md 3.6, 4.4, 4.5 and the Step 3 prompt.

#include "stepper_engine.h"

#include <driver/gpio.h>

#include "constants.h"
#include "hal.h"
#include "pins.h"

// ---------------------------------------------------------------------------
// Volatile shared state (specification section 4.4, names kept as specified).
// The esp_timer dispatcher callback (timerIsr) and the Arduino loopTask are
// the only executors touching these fields. Individual words are 32-bit or
// smaller (atomic on this 32-bit core) and every multi-word transition is
// sequenced with gMotorGen / gMotorMux.
// ---------------------------------------------------------------------------
static volatile uint8_t gMotorMode = 0; // StepperEngine::Mode value
static volatile uint32_t gMotorStepsDone = 0;
static volatile uint32_t gMotorTargetSteps = 0;
static volatile uint32_t gMotorTargetSpeedHz = constants::DEFAULT_MOTOR_SPEED_HZ;
static volatile uint32_t gMotorCurrentPeriodUs = 0;

static volatile uint8_t gMotorPhase = 0; // 0 off, 1 STEP rising edge armed, 2 falling edge armed
static volatile bool gMotorStopRequested = false;
static volatile bool gMotorFaultLatched = false;
static volatile bool gMotorMeterComplete = false;
static volatile bool gMotorRunning = false;
static volatile uint32_t gMotorGen = 0; // bumped by each main-loop control action
static volatile uint32_t gMotorOpStartMs = 0; // millis() time base of current op

// Section 4.5: portMUX for mode/state changes from the main loop.
static portMUX_TYPE gMotorMux = portMUX_INITIALIZER_UNLOCKED;

// Main-loop-only state (never read by the timer callback):
static uint32_t stepperPrimeDeadlineMs = 0;
static uint32_t stepperPurgeDeadlineMs = 0;
static volatile bool stepperPrimeTimedOut = false;
static volatile bool stepperPurgeTimedOut = false;

// ---------------------------------------------------------------------------
// Timer callback body (spec section 3.6 "timerIsr").
// Permitted work only: GPIO writes, ramp arithmetic, one GPIO level read
// (driver fault), esp_timer re-arm. No logging, no heap, no millis(),
// no NVS/Web/MQTT.
// ---------------------------------------------------------------------------
IRAM_ATTR void StepperEngine::timerIsr() {
    const uint32_t gen = gMotorGen;

    if (gMotorStopRequested || gMotorPhase == 0) {
        gMotorPhase = 0;
        return;
    }

    // Driver fault input (active low, external pull-up): one register read.
    if (gpio_get_level((gpio_num_t)PIN_DRV_FAULT) == 0) {
        gMotorFaultLatched = true;
        gMotorPhase = 0;
        gMotorRunning = false;
        gMotorMode = (uint8_t)Mode::FAULT;
        WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1UL << PIN_STEP));
        WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1UL << PIN_ENABLE)); // active low -> disabled
        return;
    }

    if (gMotorPhase == 1) {
        // One STEP pulse edge: raise STEP, arm the 2 us falling edge.
        gMotorPhase = 2;
        WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1UL << PIN_STEP));
        esp_timer_start_once(timerHandle_, constants::STEP_PULSE_US);
        return;
    }

    // Falling edge: close the pulse, count the step, plan the next one.
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1UL << PIN_STEP));

    if (gMotorMode == (uint8_t)Mode::FORWARD_METER) {
        gMotorStepsDone++;
        if (gMotorStepsDone >= gMotorTargetSteps) {
            // Exact stop: no further pulses are armed.
            gMotorStepsDone = gMotorTargetSteps;
            gMotorMeterComplete = true;
            gMotorRunning = false;
            gMotorPhase = 0;
            gMotorMode = (uint8_t)Mode::IDLE;
            WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1UL << PIN_ENABLE)); // motor disabled
            return;
        }
    }

    // Linear ramp (specification 3.6 formula), millisecond precision.
    // NOTE: esp_timer_get_time() below and millis() (used to set
    // gMotorOpStartMs in start*()) derive from the same 1 us sTick core
    // clock in this ESP-IDF/Arduino-ESP32 version, so the comparison is
    // valid; if either timebase changes, both ends must be updated
    // (AI review M4 note).
    const uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
    uint32_t elapsedMs = nowMs - gMotorOpStartMs;
    if (elapsedMs > constants::RAMP_MS) {
        elapsedMs = constants::RAMP_MS;
    }
    const uint32_t target = gMotorTargetSpeedHz;
    uint32_t hz = constants::RAMP_START_HZ +
                 ((target - constants::RAMP_START_HZ) * elapsedMs) / constants::RAMP_MS;
    if (hz > target) {
        hz = target;
    }

    uint32_t periodUs = 1000000UL / hz; // period_us = 1000000 / frequency_hz
    if (periodUs < constants::STEP_PULSE_US + 1) {
        periodUs = constants::STEP_PULSE_US + 1;
    }

    // A stop/start may have raced with this dispatch: re-check before re-arming.
    if (gMotorStopRequested || (gMotorGen != gen)) {
        return;
    }

    gMotorCurrentPeriodUs = periodUs;
    gMotorPhase = 1;
    esp_timer_start_once(timerHandle_, periodUs - constants::STEP_PULSE_US);
}

void StepperEngine::timerCallback(void *arg) {
    StepperEngine *self = static_cast<StepperEngine *>(arg);
    if (self != nullptr) {
        self->timerIsr();
    }
}

// ---------------------------------------------------------------------------
// Main-loop interface.
// ---------------------------------------------------------------------------
bool StepperEngine::begin() {
    if (timerHandle_ != nullptr) {
        return true;
    }

    esp_timer_create_args_t args = {};
    args.callback = &StepperEngine::timerCallback;
    args.arg = this;
    args.name = "stepper_gen";
    const esp_err_t err = esp_timer_create(&args, &timerHandle_);
    if (err != ESP_OK || timerHandle_ == nullptr) {
        timerHandle_ = nullptr;
        return false;
    }

    // Deterministic start state: idle, motor disabled, STEP low.
    gMotorMode = (uint8_t)Mode::IDLE;
    gMotorStepsDone = 0;
    gMotorTargetSteps = 0;
    gMotorTargetSpeedHz = constants::DEFAULT_MOTOR_SPEED_HZ;
    gMotorCurrentPeriodUs = 0;
    gMotorPhase = 0;
    gMotorStopRequested = false;
    gMotorFaultLatched = false;
    gMotorMeterComplete = false;
    gMotorRunning = false;
    gMotorGen++;

    Hal::setStep(false);
    Hal::disableMotor();
    return true;
}

void StepperEngine::setSpeedHz(uint32_t speedHz) {
    if (speedHz < constants::MIN_MOTOR_SPEED_HZ) {
        speedHz = constants::MIN_MOTOR_SPEED_HZ;
    }
    if (speedHz > constants::MAX_MOTOR_SPEED_HZ) {
        speedHz = constants::MAX_MOTOR_SPEED_HZ;
    }
    portENTER_CRITICAL(&gMotorMux);
    gMotorTargetSpeedHz = speedHz;
    portEXIT_CRITICAL(&gMotorMux);
}

uint32_t StepperEngine::getSpeedHz() const {
    return (uint32_t)gMotorTargetSpeedHz;
}

void StepperEngine::startForwardPrime(uint32_t timeoutMs) {
    if (timerHandle_ == nullptr) {
        return; // begin() failed: motor operations unavailable
    }
    // Never (re)arm while a driver fault is latched in FAULT mode: that
    // would clear the latch and re-enable the driver with a live fault
    // (e.g. asserted between the caller's fatal check and this call).
    // The caller's fault handling covers that path (AI review M4).
    if (getMode() == Mode::FAULT) {
        return;
    }
    if (!gMotorRunning) {
        gMotorFaultLatched = false; // a fresh run clears an old non-fatal latch
    }
    if (timeoutMs > constants::MAX_FORWARD_PRIME_MS) {
        timeoutMs = constants::MAX_FORWARD_PRIME_MS;
    }
    stepperPrimeTimedOut = false;
    const uint32_t deadlineMs = (uint32_t)millis() + timeoutMs;

    portENTER_CRITICAL(&gMotorMux);
    gMotorMode = (uint8_t)Mode::FORWARD_PRIME;
    gMotorStepsDone = 0;
    gMotorTargetSteps = 0;
    gMotorMeterComplete = false;
    gMotorStopRequested = false;
    gMotorRunning = true;
    gMotorOpStartMs = (uint32_t)millis();
    gMotorGen++;
    portEXIT_CRITICAL(&gMotorMux);
    stepperPrimeDeadlineMs = deadlineMs;

    // DIR is set before ENABLE (specification 3.6).
    if (timerHandle_ != nullptr) {
        esp_timer_stop(timerHandle_);
    }
    Hal::setDir(false);
    Hal::enableMotor();

    gMotorPhase = 1; // first event: STEP rising edge at the 100 Hz ramp period
    esp_timer_start_once(timerHandle_,
                         1000000UL / constants::RAMP_START_HZ - constants::STEP_PULSE_US);
}

void StepperEngine::startForwardMeter(uint32_t steps) {
    if (timerHandle_ == nullptr) {
        return;
    }
    // Refuse to clear the fault latch / re-arm while FAULT is latched
    // (AI review M4); the caller's fatal-fault check owns that path.
    if (getMode() == Mode::FAULT) {
        return;
    }
    gMotorFaultLatched = false;
    stepperPrimeTimedOut = false;

    portENTER_CRITICAL(&gMotorMux);
    gMotorMode = (uint8_t)Mode::FORWARD_METER;
    gMotorStepsDone = 0;
    gMotorTargetSteps = steps;
    gMotorMeterComplete = false;
    gMotorStopRequested = false;
    gMotorRunning = true;
    gMotorOpStartMs = (uint32_t)millis();
    gMotorGen++;
    portEXIT_CRITICAL(&gMotorMux);

    if (steps == 0) {
        // Nothing to dispense: mark complete, keep the motor disabled.
        portENTER_CRITICAL(&gMotorMux);
        gMotorMeterComplete = true;
        gMotorRunning = false;
        gMotorPhase = 0;
        gMotorMode = (uint8_t)Mode::IDLE;
        gMotorGen++;
        portEXIT_CRITICAL(&gMotorMux);
        Hal::setStep(false);
        Hal::disableMotor();
        return;
    }

    if (timerHandle_ != nullptr) {
        esp_timer_stop(timerHandle_);
    }
    Hal::setDir(false);
    Hal::enableMotor();

    gMotorPhase = 1;
    esp_timer_start_once(timerHandle_,
                         1000000UL / constants::RAMP_START_HZ - constants::STEP_PULSE_US);
}

void StepperEngine::startReversePurge(uint32_t timeoutMs) {
    if (timerHandle_ == nullptr) {
        return;
    }
    // Refuse to re-arm while FAULT is latched (AI review M4).
    if (getMode() == Mode::FAULT) {
        return;
    }
    if (!gMotorRunning) {
        gMotorFaultLatched = false;
    }
    if (timeoutMs > constants::MAX_PURGE_MS) {
        timeoutMs = constants::MAX_PURGE_MS;
    }
    stepperPurgeTimedOut = false;
    const uint32_t deadlineMs = (uint32_t)millis() + timeoutMs;

    portENTER_CRITICAL(&gMotorMux);
    gMotorMode = (uint8_t)Mode::REVERSE_PURGE;
    gMotorStepsDone = 0;
    gMotorTargetSteps = 0;
    gMotorMeterComplete = false;
    gMotorStopRequested = false;
    gMotorRunning = true;
    gMotorOpStartMs = (uint32_t)millis();
    gMotorGen++;
    portEXIT_CRITICAL(&gMotorMux);
    stepperPurgeDeadlineMs = deadlineMs;

    if (timerHandle_ != nullptr) {
        esp_timer_stop(timerHandle_);
    }
    Hal::setDir(true);
    Hal::enableMotor();

    gMotorPhase = 1;
    esp_timer_start_once(timerHandle_,
                         1000000UL / constants::RAMP_START_HZ - constants::STEP_PULSE_US);
}

void StepperEngine::stopNow() {
    portENTER_CRITICAL(&gMotorMux);
    gMotorStopRequested = true;
    gMotorRunning = false;
    gMotorPhase = 0;
    if (gMotorMode != (uint8_t)Mode::FAULT) {
        gMotorMode = (uint8_t)Mode::IDLE;
    }
    gMotorGen++;
    portEXIT_CRITICAL(&gMotorMux);

    if (timerHandle_ != nullptr) {
        esp_timer_stop(timerHandle_);
    }
    Hal::setStep(false);
    Hal::disableMotor(); // ENABLE immediately high
}

bool StepperEngine::isBusy() const {
    return gMotorRunning;
}

StepperEngine::Mode StepperEngine::getMode() const {
    return (Mode)(uint8_t)gMotorMode;
}

uint32_t StepperEngine::getStepsDone() const {
    uint32_t done = 0;
    portENTER_CRITICAL(&gMotorMux);
    done = (uint32_t)gMotorStepsDone;
    portEXIT_CRITICAL(&gMotorMux);
    return done;
}

uint32_t StepperEngine::getTargetSteps() const {
    uint32_t target = 0;
    portENTER_CRITICAL(&gMotorMux);
    target = (uint32_t)gMotorTargetSteps;
    portEXIT_CRITICAL(&gMotorMux);
    return target;
}

float StepperEngine::getCurrentStepFrequencyHz() const {
    const uint32_t periodUs = (uint32_t)gMotorCurrentPeriodUs;
    if (periodUs == 0) {
        return 0.0f;
    }
    return 1000000.0f / (float)periodUs;
}

bool StepperEngine::isPrimeTimedOut() const {
    const bool busy = gMotorRunning &&
        (uint8_t)gMotorMode == (uint8_t)Mode::FORWARD_PRIME;
    const uint32_t now = (uint32_t)millis();
    const bool due = stepperPrimeDeadlineMs != 0 && now >= stepperPrimeDeadlineMs;
    if (busy && due) {
        stepperPrimeTimedOut = true; // lazy latch, survives stopNow()
    }
    return stepperPrimeTimedOut;
}

bool StepperEngine::isPurgeTimedOut() const {
    const bool busy = gMotorRunning &&
        (uint8_t)gMotorMode == (uint8_t)Mode::REVERSE_PURGE;
    const uint32_t now = (uint32_t)millis();
    const bool due = stepperPurgeDeadlineMs != 0 && now >= stepperPurgeDeadlineMs;
    if (busy && due) {
        stepperPurgeTimedOut = true;
    }
    return stepperPurgeTimedOut;
}

bool StepperEngine::isMeterComplete() const {
    return gMotorMeterComplete;
}

bool StepperEngine::isDriverFaultDetected() const {
    return gMotorFaultLatched;
}
