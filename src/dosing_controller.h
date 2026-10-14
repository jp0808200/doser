// DosingController: dosing state machine and dose calculation.
//
// Per acid-dosing-fw-specification.md 3.8 (dose calculation, required
// clamping constants, dosing sequence, failure rules) and the Step 5/6
// prompt requirements.
//
// Dose calculation (specification 3.8):
//     water_ml      = water_since_dose_L * 1000
//     required_ml   = water_ml * dose_factor  (clamped to MAX_DOSE_ML)
//     target_steps  = round(required_ml / pump_ml_per_step),
//                     minimum 1 when required_ml > 0,
//                     clamped to MAX_DOSE_STEPS
//
// Step 6: full dosing sequence (specification 3.8 sequence + failure rules):
//     IDLE -> DOSE_READY
//         on a scheduler request or a manual request; the required ml and
//         target steps are computed from the current water volume.
//     DOSE_READY -> DOSING (forward prime)
//         only when dosing conditions are met (no fatal fault, calibrations
//         valid) and required steps > 0. Tank empty is a warning only and
//         does not block dosing. Zero steps:
//         warning logged, back to IDLE, water accumulator NOT reset.
//     DOSING / FORWARD_PRIME
//         engine forward until the acid line detector is present.
//         line present      -> start metered run with the target steps
//         prime timeout     -> warning LINE_NO_ACID, stop, reverse purge;
//                              purge success -> IDLE without resetting the
//                              water accumulator (aborted dose)
//     DOSING / FORWARD_METER
//         engine delivers exactly the target steps (ISP exact stop).
//         meter complete    -> stop, start reverse purge (dose complete)
//         line lost         -> warning LINE_LOST, stop, reverse purge,
//                              water accumulator NOT reset (aborted dose)
//     DOSING / REVERSE_PURGE
//         engine reverse until the line detector is empty.
//         line empty        -> dose complete? POST_DOSE : IDLE (aborted)
//         purge timeout     -> stop, ERROR (LINE_NOT_EMPTY)
//     POST_DOSE -> IDLE
//         completed metered ml = completed steps * pump_ml_per_step,
//         totals + last-dose fields updated, stats flushed to NVS,
//         water accumulator dose base reset, DOSE_COMPLETED logged.
//
// Abort rules (specification 3.8 failure rules, amended): a DRV8825 fault
// at any moment stops the motor immediately, the water accumulator is
// preserved (never reset except in POST_DOSE), DOSE_ABORTED is logged and
// the state becomes ERROR. Tank empty is a WARNING only: it is surfaced on
// the web UI / MQTT but does not affect the dosing functions. AppCore's safe stop applies the motor disable
// in the same loop pass; this controller additionally stops the engine so
// the sequence state machines are deterministic.
//
// Constraints honored: runs in the main loop only (spec 3.13 order item 7),
// non-blocking, no allocation, no direct Web/MQTT calls (alerts go through
// the alert queue, spec 3.12 / 4.8), the scheduler and other modules never
// drive the motor (only this controller does, spec 3.7/3.9).
#ifndef ACID_DOSE_DOSING_CONTROLLER_H
#define ACID_DOSE_DOSING_CONTROLLER_H

#include <Arduino.h>

#include <stdint.h>

#include "config_store.h"
#include "dose_scheduler.h"
#include "flow_meter.h"
#include "safety_monitor.h"
#include "stepper_engine.h"
#include "types.h"

class CalibrationManager;

class DosingController {
public:
    // Sub-states of the high-level DOSING state (specification 3.8 dosing
    // sequence steps 2-4). Exposed for HIL/test inspection; the web/MQTT
    // layers report the high-level SystemState plus this phase.
    enum class Phase : uint8_t {
        NONE = 0,
        FORWARD_PRIME = 1,
        FORWARD_METER = 2,
        REVERSE_PURGE = 3
    };

    // Attaches the cooperating components (all owned by AppCore).
    void begin(FlowMeter &flowMeter, ConfigStore &store,
               SafetyMonitor &safety, StepperEngine &engine,
               DoseScheduler &scheduler, CalibrationManager &calibration);

    // Main-loop hook: advances the dosing state machine. Call exactly once
    // per main loop, in AppCore's update order (spec 3.13 item 7).
    void update(uint32_t nowMs);

    // Manual dose request (Step 6 req. 9; the HIL serial hook is the
    // ENABLE_TEST_INJECTION `manual_dose` command in AppCore).
    // rejected (log + alert) when a dose is active, any calibration is
    // active (flow or pump, Step 7), a fatal fault (driver fault) is active,
    // the motor is otherwise busy, or dosing conditions (calibrations)
    // are not met. Tank empty is a warning only and does not reject.
    // Accepted (state -> DOSE_READY) otherwise.
    bool requestManualDose();

    // Abort the active dose: stops the motor immediately, keeps the water
    // accumulator (it is only reset after a successful dose in POST_DOSE),
    // logs DOSE_ABORTED and queues the alert. Safe to call when no dose is
    // active (no-op for the state machine).
    void abortDose(FaultCode code, const char *message);

    // Status fields (specification 3.8 API).
    SystemState getState() const { return (SystemState)state_; }
    Phase getPhase() const { return (Phase)phase_; }
    bool isDoseActive() const;
    bool isDosePending() const { return ((SystemState)state_ == SystemState::DOSE_READY); }
    float getRequiredDoseMilliliters() const { return requiredMl_; }
    uint32_t getRequiredSteps() const { return requiredSteps_; }
    // Exact metered steps delivered in the active (during FORWARD_METER) or
    // the last completed dose (0 otherwise).
    uint32_t getCompletedMeteredSteps() const;
    float getWaterSinceDoseMilliliters() const;
    // Dosing conditions (specification 3.8 DOSE_READY checks, excluding the
    // required-steps check which is handled in the transition).
    bool canDose() const;

    // Recompute the required dose (ml + steps) from the current water
    // volume, calibrations and the spec 3.8 clamp rules. Pure math, no
    // motor or state side effects - used by the DOSE_READY transition and
    // by the HIL `dose` inspection command.
    void computeDose();

private:
    static bool calibrationsValid(const AppConfig &cfg);

    FlowMeter *flowMeter_ = nullptr;
    ConfigStore *store_ = nullptr;
    SafetyMonitor *safety_ = nullptr;
    StepperEngine *engine_ = nullptr;
    DoseScheduler *scheduler_ = nullptr;
    CalibrationManager *calibration_ = nullptr;

    volatile uint8_t state_ = (uint8_t)SystemState::IDLE;
    volatile uint8_t phase_ = (uint8_t)Phase::NONE;
    bool fatalHandled_ = false;   // abort log throttling for a persistent fault
    bool pendingManual_ = false;  // manual request awaiting IDLE entry
    bool doseCompleted_ = false;  // metered phase finished (drives POST_DOSE vs. IDLE after purge)

    float requiredMl_ = 0.0f;
    uint32_t requiredSteps_ = 0;
    uint32_t completedSteps_ = 0; // exact steps delivered by the last metered phase
    float doseWaterMl_ = 0.0f;    // water since dose captured at dosing time
};

#endif /* ACID_DOSE_DOSING_CONTROLLER_H */
