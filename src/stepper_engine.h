// StepperEngine - DRV8825 STEP/DIR/ENABLE control with linear ramp.
// Per acid-dosing-fw-specification.md 3.6 (and its required constants),
// 4.4/4.5 (volatile shared state, critical section) and the Step 3 prompt.
//
// Single speed parameter rule: the ONLY user-adjustable motor speed value is
// targetMotorSpeedHz (clamped to 100..800 Hz). Every pump operation
// (forward prime, metered dose, reverse purge, and later pump calibration)
// uses this same value. There is no second speed parameter.
//
// Step generation: ESP32-C3 has only two 54-bit hardware timers and both are
// owned by the OS (core timer / esp_timer service), therefore step pulses are
// generated with esp_timer one-shot alarms, which the Step 3 prompt
// explicitly allows ("ESP32 Arduino hardware timer or esp_timer with ISR
// dispatch; do not create a task"). Each timer event drives exactly one
// STEP edge, so the step frequency can change on the very next edge -
// this is what makes the 100 Hz -> target linear ramp and the exact
// "stop after N steps" behavior possible.
//
// The per-event callback (timerIsr()) runs in the esp_timer dispatcher
// context and performs, at most: one GPIO write, the ramp arithmetic,
// one short GPIO level read (driver fault) and one esp_timer re-arm.
// It never logs, never allocates, never calls millis(), and never touches
// NVS, Web, MQTT or JSON.
#ifndef ACID_DOSE_STEPPER_ENGINE_H
#define ACID_DOSE_STEPPER_ENGINE_H

#include <Arduino.h>

#include <cstdint>
#include <esp_timer.h>

class StepperEngine {
public:
    enum class Mode : uint8_t {
        IDLE = 0,
        FORWARD_PRIME = 1,
        FORWARD_METER = 2,
        REVERSE_PURGE = 3,
        FAULT = 4
    };

    // Creates the esp_timer one-shot handle and initializes state.
    // Must run after Hal::setup() (which configures STEP/DIR/ENABLE).
    // Returns true on success. Allocation happens once, at setup.
    bool begin();

    // The single user speed parameter. Out-of-range values are clamped
    // to MIN_MOTOR_SPEED_HZ..MAX_MOTOR_SPEED_HZ (100..800 Hz).
    void setSpeedHz(uint32_t speedHz);
    uint32_t getSpeedHz() const;

    // Start a forward run that continues until stopNow() (acid prime).
    // timeoutMs is clamped to MAX_FORWARD_PRIME_MS and is evaluated only
    // in the main loop via isPrimeTimedOut() (millis-based, not in ISR).
    void startForwardPrime(uint32_t timeoutMs);

    // Start a forward run of exactly `steps` STEP pulses. The pulse
    // generator stops itself, disables the motor and sets isMeterComplete()
    // exactly after the last pulse; no extra step occurs.
    void startForwardMeter(uint32_t steps);

    // Start a reverse run that continues until stopNow() (line purge).
    // timeoutMs is clamped to MAX_PURGE_MS, evaluated in the main loop
    // via isPurgeTimedOut().
    void startReversePurge(uint32_t timeoutMs);

    // Immediate stop for the main loop: disables the motor (ENABLE high),
    // forces STEP low, cancels the next pulse. Mode returns to IDLE unless
    // a driver fault latched mode FAULT.
    void stopNow();
    bool isBusy() const;

    Mode getMode() const;
    uint32_t getStepsDone() const;
    uint32_t getTargetSteps() const;

    // Current step frequency based on the last computed period (Hz),
    // 0.0f when no operation is running. Exposed for HIL testing.
    float getCurrentStepFrequencyHz() const;

    bool isPrimeTimedOut() const;
    bool isPurgeTimedOut() const;
    // True once a metered run delivered its full step count.
    bool isMeterComplete() const;
    // True when the driver fault input was seen asserted (active low).
    bool isDriverFaultDetected() const;

    // Per-event step generator body. Dispatched by the esp_timer service;
    // kept public (spec 3.6 API) - do not call directly from the main loop.
    IRAM_ATTR void timerIsr();

private:
    esp_timer_handle_t timerHandle_ = nullptr;

    // Operation start time (ms, millis() time base) lives in the file-local
    // volatile uint32_t gMotorOpStartMs: a 32-bit word is atomic on this
    // core, so the dispatch callback can read it safely. Written under
    // gMotorMux by the main loop; the ramp math is millisecond precision
    // per specification 3.6.

    static void timerCallback(void *arg);
};

#endif /* ACID_DOSE_STEPPER_ENGINE_H */
