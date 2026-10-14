// CalibrationManager: flow-meter and pump calibration (Step 7).
// Per acid-dosing-fw-specification.md 3.9 (functional description, static
// interface, flow/pump calibration procedures, internal constraints).
//
// Flow calibration (spec 3.9 flow procedure):
//     NONE -> FLOW_ACTIVE          FlowMeter snapshots the pulse count;
//                                  the user lets a known volume pass.
//     FLOW_ACTIVE -> FLOW_WAIT_VOLUME  no pulse for FLOW_CAL_WAIT_SETTLE_MS;
//                                  the session awaits the volume entry.
//     FLOW_WAIT_VOLUME/FLOW_ACTIVE - stopFlowCalibration(volumeL):
//                                  delta = now - start; ppl = delta/L is
//                                  validated (> 10 and <= 100000 pulses/L,
//                                  0 < L <= 1000 L) and persisted via
//                                  ConfigStore::setFlowCalibration(); on
//                                  invalid input or failed write the
//                                  previous calibration is retained.
//     cancelFlowCalibration(): discards the session, previous value kept.
//
// Pump calibration (spec 3.9 pump procedure):
//     NONE -> PUMP_PRIMING         forward run until the acid line detector
//                                  is present (prime timeout -> warn, safe
//                                  reverse purge, abort).
//     PUMP_PRIMING -> PUMP_METERING  exactly PUMP_CAL_STEPS (1600) steps
//                                  forward at the single global motor speed.
//     PUMP_METERING -> PUMP_PURGING  reverse run until the line detector is
//                                  empty (timeout -> warn, abort).
//     PUMP_PURGING -> PUMP_WAIT_VOLUME   the ~unknown volume has been
//                                  dispensed; the user enters it in ml.
//     PUMP_WAIT_VOLUME: confirmPumpCalibration(ml) computes
//                                  ml/1600.0f; valid when finite and in
//                                  0.001..10.0 ml/step -> persisted via
//                                  ConfigStore::setPumpCalibration().
//                                  Invalid input -> previous value retained.
//     cancelPumpCalibration(): stops the motor, reverse purges to a dry
//     line state if the line was primed, discards the calibration.
//
// A fatal fault (tank empty / DRV8825 fault) at any moment aborts the
// active calibration: the motor is stopped here (AppCore applies the
// global safe stop in the same loop pass) and the state returns to NONE.
//
// Constraints honored (spec 3.9 internal constraints):
//  - StepperEngine is only driven after normal dosing is prevented
//    (no active/pending dose, engine idle);
//  - the metered run is exactly 1600 steps regardless of motor speed;
//  - only the single global motor speed parameter is used;
//  - fatal fault aborts the calibration (error handling is global);
//  - cancel stops the motor and purges to a dry line when primed;
//  - no dynamic allocation, no NVS write from callback context (all
//    store writes happen in the main-loop facade methods only).
#ifndef ACID_DOSE_CALIBRATION_MANAGER_H
#define ACID_DOSE_CALIBRATION_MANAGER_H

#include <stdint.h>

class ConfigStore;
class DosingController;
class FlowMeter;
class SafetyMonitor;
class StepperEngine;

class CalibrationManager {
public:
    // Sub-states (specification 3.9 static interface).
    enum class SubState : uint8_t {
        NONE = 0,
        FLOW_ACTIVE = 1,
        FLOW_WAIT_VOLUME = 2,
        PUMP_PRIMING = 3,
        PUMP_METERING = 4,
        PUMP_PURGING = 5,
        PUMP_WAIT_VOLUME = 6
    };

    // Attaches the cooperating components (all owned by AppCore).
    // Called after the FlowMeter, SafetyMonitor, StepperEngine and
    // DosingController are constructed.
    void begin(FlowMeter &flowMeter, ConfigStore &store,
               SafetyMonitor &safety, StepperEngine &engine,
               DosingController &dosing);

    // Main-loop hook: advances the flow/pump calibration sub-state
    // machine. Call exactly once per main loop (AppCore update order,
    // after the dosing controller pass).
    void update(uint32_t nowMs);

    // Flow calibration session (spec 3.9 flow procedure). All main loop.
    bool startFlowCalibration();
    // Validates the measured volume, computes deltas pulses, persists the
    // new pulses/liter on success (previous value retained otherwise) and
    // always ends the session.
    bool stopFlowCalibration(float volumeLiters);
    // Discards the active flow session; the previous calibration is kept.
    void cancelFlowCalibration();

    // Pump calibration session (spec 3.9 pump procedure). All main loop.
    // Starts the forward prime; the metered/purge phases run in update().
    bool startPumpCalibration();
    // Valid only in PUMP_WAIT_VOLUME: computes volume_ml / PUMP_CAL_STEPS,
    // validates (finite, 0.001..10.0 ml/step) and persists the new
    // ml/step on success (previous value retained otherwise).
    bool confirmPumpCalibration(float volumeMilliliters);
    // Stops the motor, reverse purges to a dry line state when the line
    // was primed and discards the calibration data.
    void cancelPumpCalibration();

    SubState getSubState() const { return (SubState)subState_; }
    bool isFlowCalibrationActive() const;
    // True while any pump-calibration sub-state (priming, metering,
    // purging or awaiting volume) is active.
    bool isPumpCalibrationActive() const;
    bool isActive() const;

    // Pulses counted since the active flow calibration start (0 when no
    // flow session is active).
    uint64_t getFlowPulsesSinceStart() const;
    // Exact metered steps delivered by the last pump calibration run
    // (live engine progress while PUMP_METERING is active).
    uint32_t getPumpStepsCompleted() const;
    // Pulses counted during the last successfully saved flow calibration
    // (0 if none yet completed).
    float getFlowLastDeltaPulses() const { return flowLastDeltaPulses_; }
    // Volume (ml) from the last successfully saved pump calibration
    // (0 if none yet completed).
    float getPumpCalibrationVolumeMl() const { return pumpCalVolumeMl_; }

private:
    // Shared guard: a calibration may only be started when normal dosing
    // is prevented and the system is safe (spec 3.9 constraints).
    bool calibrationsBlocked() const;

    FlowMeter *flowMeter_ = nullptr;
    ConfigStore *store_ = nullptr;
    SafetyMonitor *safety_ = nullptr;
    StepperEngine *engine_ = nullptr;
    DosingController *dosing_ = nullptr;

    volatile uint8_t subState_ = (uint8_t)SubState::NONE;
    // PUMP_PURGING context flag: the purge was started by a cancel or an
    // aborted run (prime timeout). When set, a completed purge returns to
    // NONE (discard); when clear, a completed purge enters PUMP_WAIT_VOLUME.
    bool purgeAbortsAfter_ = false;

    uint32_t pumpStepsDelivered_ = 0;
    float flowLastDeltaPulses_ = 0.0f;
    float pumpCalVolumeMl_ = 0.0f;
};

#endif /* ACID_DOSE_CALIBRATION_MANAGER_H */
