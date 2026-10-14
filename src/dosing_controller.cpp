// DosingController implementation - full dosing sequence (Step 6).
// Sequence, failure rules and calculations per
// acid-dosing-fw-specification.md 3.8 and the Step 6 prompt.
#include "dosing_controller.h"

#include <math.h>

#include "alert_queue.h"
#include "calibration_manager.h"
#include "constants.h"
#include "logger.h"

void DosingController::begin(FlowMeter &flowMeter, ConfigStore &store,
                             SafetyMonitor &safety, StepperEngine &engine,
                             DoseScheduler &scheduler, CalibrationManager &cal) {
    flowMeter_ = &flowMeter;
    store_ = &store;
    safety_ = &safety;
    engine_ = &engine;
    scheduler_ = &scheduler;
    calibration_ = &cal;
    state_ = (uint8_t)SystemState::IDLE;
    phase_ = (uint8_t)Phase::NONE;
    fatalHandled_ = false;
    pendingManual_ = false;
    doseCompleted_ = false;
    requiredMl_ = 0.0f;
    requiredSteps_ = 0;
    completedSteps_ = 0;
    doseWaterMl_ = 0.0f;
}

bool DosingController::calibrationsValid(const AppConfig &cfg) {
    // Dosing-readiness calibration ranges (Step 6 requirements 3/9,
    // specification 3.8 DOSE_READY checks):
    //   pump_ml_per_step between 0.001 and 10.0
    //   pulses_per_liter between 10 and 100000
    return cfg.pumpMlPerStep >= 0.001f && cfg.pumpMlPerStep <= 10.0f
        && cfg.pulsesPerLiter >= constants::MIN_FLOW_PULSES_PER_LITER
        && cfg.pulsesPerLiter <= constants::MAX_FLOW_PULSES_PER_LITER
        && cfg.doseFactorMlPerLiter >= 0.001f;
}

void DosingController::computeDose() {
    // Dose calculation per specification 3.8 exactly (float math, rounded
    // to the nearest step count):
    //   water_ml     = water_since_dose_L * 1000
    //   required_ml  = water_ml * dose_factor (clamped to MAX_DOSE_ML)
    //   target_steps = round(required_ml / pump_ml_per_step)
    //   if water_ml>0 && required_ml>0 && steps==0 -> 1 step
    //   target_steps clamped to MAX_DOSE_STEPS
    const float waterL = flowMeter_->getWaterVolumeLitersSinceDose();
    const float waterMl = (waterL > 0.0f) ? waterL * 1000.0f : 0.0f;

    const AppConfig &cfg = store_->getAppConfig();
    float requiredMl = waterL * cfg.doseFactorMlPerLiter;
    if (requiredMl < 0.0f || !isfinite(requiredMl)) {
        requiredMl = 0.0f; // defensive: no negative or non-finite dose
    }

    uint32_t steps = 0;
    if (cfg.pumpMlPerStep > 0.0f) {
        steps = (uint32_t)lroundf(requiredMl / cfg.pumpMlPerStep);
    }
    if (requiredMl > 0.0f && steps == 0) {
        steps = 1; // minimum single step for a non-zero dose
    }
    if (steps > constants::MAX_DOSE_STEPS) {
        steps = constants::MAX_DOSE_STEPS;
    }
    if (requiredMl > constants::MAX_DOSE_ML) {
        requiredMl = constants::MAX_DOSE_ML;
    }

    requiredMl_ = requiredMl;
    requiredSteps_ = steps;
    doseWaterMl_ = waterMl;
}

bool DosingController::canDose() const {
    if (safety_ == nullptr) {
        return false;
    }
    // Step 6 requirement 3: no fatal fault (tank empty is warning-only and
    // does not block dosing).
    if (safety_->isFatalFaultActive()) {
        return false;
    }
    if (flowMeter_ == nullptr || flowMeter_->isCalibrationActive()) {
        return false;
    }
    // Step 7: normal dosing is prevented while any calibration (flow or
    // pump) is active (spec 3.9 constraint).
    if (calibration_ != nullptr && calibration_->isActive()) {
        return false;
    }
    return calibrationsValid(store_->getAppConfig());
}

float DosingController::getWaterSinceDoseMilliliters() const {
    return (flowMeter_ != nullptr) ? flowMeter_->getEstimatedWaterMillilitersSinceDose() : 0.0f;
}

uint32_t DosingController::getCompletedMeteredSteps() const {
    // During an active metered phase report live progress; after the run
    // (purge/post-dose/idle) report the exact delivered count of the last
    // completed metered phase.
    if ((SystemState)state_ == SystemState::DOSING
        && (Phase)phase_ == Phase::FORWARD_METER && engine_ != nullptr) {
        return engine_->getStepsDone();
    }
    return completedSteps_;
}

bool DosingController::isDoseActive() const {
    const SystemState s = (SystemState)state_;
    return s == SystemState::DOSE_READY || s == SystemState::DOSING || s == SystemState::POST_DOSE;
}

bool DosingController::requestManualDose() {
    // Rejection rules (Step 6 requirement 9): calibration active, dose
    // active, fatal fault (driver fault), motor busy, not dosing-ready.
    // Tank empty is a WARNING only and does NOT reject a manual dose.
    // The exact rejection reason is logged and, for faults, queued so the
    // web layer (Step 8) can surface it.
    if (isDoseActive()) {
        Logger::warning("manual dose rejected: dose_active");
        return false;
    }
    if ((flowMeter_ != nullptr && flowMeter_->isCalibrationActive())
        || (calibration_ != nullptr && calibration_->isActive())) {
        // Step 7: rejects both the flow and the pump calibration sessions
        // (web API reports this as calibration_active from Step 8 on).
        Logger::warning("manual dose rejected: calibration_active");
        return false;
    }
    if (safety_ != nullptr && safety_->isFatalFaultActive()) {
        Logger::warning("manual dose rejected: fatal_fault");
        return false;
    }
    if (engine_ != nullptr && engine_->isBusy()) {
        // A manual test run or calibration owns the motor; do not clobber
        // it (the web API reports this as motor_busy from Step 8 on).
        Logger::warning("manual dose rejected: motor_busy");
        return false;
    }
    if (!canDose()) {
        Logger::warning("manual dose rejected: calibration/config not dosing-ready");
        return false;
    }
    // Accepted: the IDLE -> DOSE_READY transition happens in the next
    // update() pass (both run in the AppCore main loop, spec 3.13 order).
    pendingManual_ = true;
    Serial.println("manual dose      : requested");
    return true;
}

void DosingController::abortDose(FaultCode code, const char *message) {
    // Step 6 requirement 10: abortDose MUST disable the motor and must NOT
    // reset the water accumulator (only POST_DOSE after a successful dose
    // does), and logs DOSE_ABORTED.
    const SystemState s = (SystemState)state_;
    const bool wasDosing = (s == SystemState::DOSING || s == SystemState::DOSE_READY
                            || s == SystemState::POST_DOSE);

    if (engine_ != nullptr && engine_->isBusy()) {
        engine_->stopNow(); // ENABLE high immediately
    }
    if (wasDosing && !fatalHandled_) {
        // fatalHandled_ doubles as the abort-log throttle so a persistent
        // fault does not re-log an unbounded number of abort events.
        fatalHandled_ = true;
        Logger::event(EVENT_DOSE_ABORTED, (message != nullptr) ? message : "dose aborted");
    }
    if (code != FaultCode::NONE) {
        // Tank empty is warning-only; driver fault is the fatal abort.
        Alerts::queue().push(
            code == FaultCode::DRIVER_FAULT
                ? AlertSeverity::ERROR : AlertSeverity::WARNING,
            code, (message != nullptr) ? message : "dose aborted", millis());
    }
}

void DosingController::update(uint32_t nowMs) {
    (void)nowMs; // engine timeouts are millis-based and evaluated in-engine
    const SystemState s = (SystemState)state_;

    // --- Tank empty / DRV8825 fault during any active dose --------------
    // Specification 3.8 failure rules (Step 6 reqs. 5-7): stop the motor,
    // enter ERROR, keep the water accumulator. AppCore::handleFatalFault()
    // applies the global safe stop in the same loop pass; stopping the
    // engine here keeps this state machine deterministic.
    if (s == SystemState::DOSING || s == SystemState::DOSE_READY
        || s == SystemState::POST_DOSE) {
        bool fatal = false;
        FaultCode code = FaultCode::NONE;
        if (safety_ != nullptr && safety_->isFatalFaultActive()) {
            fatal = true;
            code = safety_->getFaultCode();
            if (code == FaultCode::NONE) {
                code = FaultCode::DRIVER_FAULT; // driver fault is the only fatal
            }
        }
        if (engine_ != nullptr && engine_->getMode() == StepperEngine::Mode::FAULT) {
            fatal = true;
            code = (code != FaultCode::NONE) ? code : FaultCode::DRIVER_FAULT;
        }
        if (fatal) {
            abortDose(code, "fatal fault during dose");
            fatalHandled_ = true;
            phase_ = (uint8_t)Phase::NONE;
            state_ = (uint8_t)SystemState::ERROR;
            Serial.println("dose               : ABORTED (fatal fault), state=ERROR, accumulator kept");
            return;
        }
    }

    switch (s) {
        case SystemState::IDLE: {
            // Consume a scheduler request (spec 4.8: Scheduler ->
            // DosingController) or a queued manual request.
            bool fromScheduler = false;
            if (scheduler_ != nullptr && scheduler_->requestPending()) {
                (void)scheduler_->consumeRequest();
                fromScheduler = true;
            }
            if (pendingManual_) {
                pendingManual_ = false;
            } else if (!fromScheduler) {
                return; // nothing requested this pass
            }

            // IDLE -> DOSE_READY: compute the dose from the current water
            // volume, calibrations and clamping rules (pure math).
            phase_ = (uint8_t)Phase::NONE;
            fatalHandled_ = false;
            doseCompleted_ = false;
            completedSteps_ = 0;
            computeDose();
            state_ = (uint8_t)SystemState::DOSE_READY;
            Serial.printf("dose               : %s, required %.2f ml / %lu steps (water %.1f ml)\n",
                          fromScheduler ? "scheduled" : "manual",
                          (double)requiredMl_, (unsigned long)requiredSteps_,
                          (double)doseWaterMl_);
            Logger::event(EVENT_DOSE_STARTED, "dose sequence beginning");
            break;
        }

        case SystemState::DOSE_READY: {
            // Step 6 reqs. 3/4: verify dosing conditions, then start the
            // forward prime (specification 3.8 sequence step 2).
            if (!canDose()) {
                // Conditions were not dosing-ready (calibration invalid or
                // a fatal fault between request and now).
                FaultCode code = FaultCode::PUMP_NO_CAL;
                const AppConfig &cfg = store_->getAppConfig();
                if (cfg.pulsesPerLiter < constants::MIN_FLOW_PULSES_PER_LITER
                    || cfg.pulsesPerLiter > constants::MAX_FLOW_PULSES_PER_LITER) {
                    code = FaultCode::FLOW_NO_CAL;
                }
                abortDose(code, "dosing conditions not met at DOSE_READY");
                phase_ = (uint8_t)Phase::NONE;
                state_ = (uint8_t)SystemState::ERROR;
                Serial.println("dose               : ABORTED at DOSE_READY (not dosing-ready), state=ERROR");
                return;
            }
            if (requiredSteps_ == 0) {
                // No water (or a zero dose): warn and return to IDLE WITHOUT
                // touching the water accumulator (specification 3.8 seq. 1,
                // Step 6 req. 4, Step 6 verification "aborted dose does not
                // reset water accumulator").
                Serial.println("dose               : zero dose required, no acid dispensed, accumulator kept");
                Logger::warning("zero dose steps, returning to IDLE (accumulator kept)");
                Alerts::queue().push(AlertSeverity::WARNING, FaultCode::NONE,
                                     "dose skipped: zero required volume", millis());
                phase_ = (uint8_t)Phase::NONE;
                state_ = (uint8_t)SystemState::IDLE;
                return;
            }
            // Start the forward prime (DIR forward, ramp 100 Hz -> target).
            // The pump runs until the acid line detector is present or the
            // prime timeout (both evaluated in the DOSING passes below).
            phase_ = (uint8_t)Phase::FORWARD_PRIME;
            engine_->startForwardPrime(constants::MAX_FORWARD_PRIME_MS);
            state_ = (uint8_t)SystemState::DOSING;
            Serial.printf("dose               : DOSING, forward prime started "
                          "(target: %lu metered steps)\n", (unsigned long)requiredSteps_);
            break;
        }

        case SystemState::DOSING: {
            switch ((Phase)phase_) {
                // ----------------------------------------------------------
                // 2. DOSING_FORWARD_PRIME (Step 6 req. 5, spec 3.8 seq. 2)
                //    Run until line present; timeout -> warning + purge.
                // ----------------------------------------------------------
                case Phase::FORWARD_PRIME: {
                    if (safety_->isLinePresent()) {
                        // Acid reached the line: begin the exact metered run.
                        phase_ = (uint8_t)Phase::FORWARD_METER;
                        engine_->startForwardMeter(requiredSteps_);
                        Logger::event(EVENT_LINE_DETECTED, "line present, metered dose started");
                        Serial.printf("dose               : line present, metering %lu steps\n",
                                      (unsigned long)requiredSteps_);
                        break;
                    }
                    if (engine_->isPrimeTimedOut()) {
                        // Prime timeout: no acid detected (LINE_NO_ACID
                        // warning). Stop forward, start reverse purge; if
                        // the purge succeeds the dose is ABORTED - the water
                        // accumulator is NOT reset (Step 6 req. 5).
                        Serial.println("dose               : PRIME TIMEOUT (LINE_NO_ACID), "
                                       "stopping forward, starting reverse purge");
                        Logger::warning("prime timeout: no acid detected in line (LINE_NO_ACID)");
                        Alerts::queue().push(AlertSeverity::WARNING, FaultCode::LINE_NO_ACID,
                                             "prime timeout: no acid detected in line", millis());
                        engine_->stopNow();
                        doseCompleted_ = false;  // purge success -> IDLE, not POST_DOSE
                        phase_ = (uint8_t)Phase::REVERSE_PURGE;
                        engine_->startReversePurge(constants::MAX_PURGE_MS);
                    }
                    break;
                }

                // ----------------------------------------------------------
                // 3. DOSING_FORWARD_METER (Step 6 req. 6, spec 3.8 seq. 3)
                //    Exact step count; line loss or fatal -> abort rules.
                // ----------------------------------------------------------
                case Phase::FORWARD_METER: {
                    if (engine_->isMeterComplete()) {
                        // Exact step count delivered (the ISP stopped the
                        // engine at the last pulse). Capture the exact step
                        // count BEFORE starting the purge: startReversePurge
                        // zeroes the engine's step counters.
                        completedSteps_ = engine_->getStepsDone();
                        doseCompleted_ = true;
                        phase_ = (uint8_t)Phase::REVERSE_PURGE;
                        engine_->startReversePurge(constants::MAX_PURGE_MS);
                        Serial.printf("dose               : metered complete (%lu steps), "
                                      "reverse purge started\n",
                                      (unsigned long)completedSteps_);
                        break;
                    }
                    if (safety_->isLineEmpty()) {
                        // Line lost while metering (spec 3.8 failure rules,
                        // Step 6 req. 6): stop forward, reverse purge, warn;
                        // the water accumulator is NOT reset and the dose
                        // does not reach POST_DOSE (purge success -> IDLE).
                        Serial.println("dose               : LINE LOST during metering, "
                                       "stopping forward, starting reverse purge");
                        Logger::event(EVENT_LINE_LOST, "line lost during metering dose");
                        Alerts::queue().push(AlertSeverity::WARNING, FaultCode::LINE_NO_ACID,
                                             "line lost during metering dose", millis());
                        engine_->stopNow();
                        doseCompleted_ = false; // abort: accumulator preserved
                        phase_ = (uint8_t)Phase::REVERSE_PURGE;
                        engine_->startReversePurge(constants::MAX_PURGE_MS);
                    }
                    break;
                }

                // ----------------------------------------------------------
                // 4. DOSING_REVERSE_PURGE (Step 6 req. 7, spec 3.8 seq. 4)
                //    Reverse until line empty; timeout -> ERROR.
                // ----------------------------------------------------------
                case Phase::REVERSE_PURGE: {
                    if (safety_->isLineEmpty()) {
                        // Line safe/dry: stop the motor.
                        engine_->stopNow();
                        Logger::event(EVENT_LINE_PURGE_COMPLETE, "reverse purge complete, line empty");
                        if (doseCompleted_) {
                            // Successful dose: bookkeeping in POST_DOSE.
                            phase_ = (uint8_t)Phase::NONE;
                            state_ = (uint8_t)SystemState::POST_DOSE;
                            Serial.println("dose               : line empty, dose complete, posting stats");
                        } else {
                            // Aborted dose (prime timeout or line loss):
                            // return to IDLE WITHOUT resetting the water
                            // accumulator (Step 6 verification: "aborted dose
                            // does not reset water accumulator").
                            phase_ = (uint8_t)Phase::NONE;
                            fatalHandled_ = false;
                            state_ = (uint8_t)SystemState::IDLE;
                            Serial.println("dose               : aborted (no acid delivered), "
                                           "line purged back to IDLE, accumulator kept");
                        }
                        break;
                    }
                    if (engine_->isPurgeTimedOut()) {
                        // Purge timeout: line still not empty -> ERROR
                        // (spec 3.8 seq. 4, Step 6 req. 7).
                        engine_->stopNow();
                        abortDose(FaultCode::LINE_NOT_EMPTY,
                                  "reverse purge timeout: line not empty");
                        phase_ = (uint8_t)Phase::NONE;
                        state_ = (uint8_t)SystemState::ERROR;
                        Serial.println("dose               : PURGE TIMEOUT, state=ERROR, accumulator kept");
                        return;
                    }
                    break;
                }

                case Phase::NONE:
                    // Should not occur; fall through to the other phases
                    // deterministically doing nothing until a fault/abort.
                    break;
            }
            break;
        }

        case SystemState::POST_DOSE: {
            // POST_DOSE bookkeeping (Step 6 req. 8, spec 3.8 sequence
            // step 5). completedSteps_ holds the EXACT metered step count
            // delivered (captured at meter completion).
            const AppConfig &cfg = store_->getAppConfig();
            const float completedMl = (float)completedSteps_ * cfg.pumpMlPerStep;

            // last_dose fields, totals, dose counter.
            store_->setLastDose(doseWaterMl_ / 1000.0f, completedMl);
            store_->addTotalAcidMilliliters(completedMl);
            store_->addTotalWaterLiters(doseWaterMl_ / 1000.0f);
            store_->incrementTotalDoses();
            store_->flushStats();       // NVS write from main loop, after a successful dose
            flowMeter_->resetDoseBase(); // reset water accumulator dose base

            Logger::event(EVENT_DOSE_COMPLETED, "dose completed");
            Alerts::queue().push(AlertSeverity::INFO, FaultCode::NONE,
                                 "dose completed", millis());
            Serial.printf("dose               : COMPLETED, %.2f ml acid (%lu steps), %.1f ml water\n",
                          (double)completedMl, (unsigned long)completedSteps_,
                          (double)doseWaterMl_);

            // Back to IDLE (Step 6 req. 11: the alert queue entry above and
            // AppCore's flow-status refresh publish the dose event; no
            // direct Web/MQTT calls from here, spec 3.8/4.8).
            phase_ = (uint8_t)Phase::NONE;
            doseCompleted_ = false;
            pendingManual_ = false;
            state_ = (uint8_t)SystemState::IDLE;
            break;
        }

        case SystemState::ERROR:
        case SystemState::POWER_ON:
        case SystemState::COUNTING:
        case SystemState::CALIBRATION:
        case SystemState::SAFE_SHUTDOWN:
            // Outside this controller's transitions. ERROR persists until
            // reset/ack (Step 10 adds SAFE_SHUTDOWN timing).
            break;
    }
}
