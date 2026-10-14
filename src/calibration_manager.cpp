// CalibrationManager implementation (Step 7).
// Flow/pump calibration per acid-dosing-fw-specification.md 3.9 procedures,
// event IDs from types.h (EVENT_FLOW_CAL_START/SAVED,
// EVENT_PUMP_CAL_START/SAVED, EVENT_LINE_DETECTED/Lost/PURGE_COMPLETE,
// FaultCode::CAL_INVALID_INPUT), alert queue usage per 3.12/4.8.
#include "calibration_manager.h"

#include <Arduino.h>
#include <math.h>

#include "alert_queue.h"
#include "config_store.h"
#include "constants.h"
#include "dosing_controller.h"
#include "flow_meter.h"
#include "logger.h"
#include "safety_monitor.h"
#include "stepper_engine.h"
#include "types.h"

void CalibrationManager::begin(FlowMeter &flowMeter, ConfigStore &store,
                               SafetyMonitor &safety, StepperEngine &engine,
                               DosingController &dosing) {
    flowMeter_ = &flowMeter;
    store_ = &store;
    safety_ = &safety;
    engine_ = &engine;
    dosing_ = &dosing;
    subState_ = (uint8_t)SubState::NONE;
    purgeAbortsAfter_ = false;
    pumpStepsDelivered_ = 0;
    flowLastDeltaPulses_ = 0.0f;
    pumpCalVolumeMl_ = 0.0f;
}

bool CalibrationManager::isFlowCalibrationActive() const {
    const SubState s = (SubState)subState_;
    return (s == SubState::FLOW_ACTIVE || s == SubState::FLOW_WAIT_VOLUME);
}

bool CalibrationManager::isPumpCalibrationActive() const {
    const SubState s = (SubState)subState_;
    return (s == SubState::PUMP_PRIMING || s == SubState::PUMP_METERING
            || s == SubState::PUMP_PURGING || s == SubState::PUMP_WAIT_VOLUME);
}

bool CalibrationManager::isActive() const {
    return isFlowCalibrationActive() || isPumpCalibrationActive();
}

uint64_t CalibrationManager::getFlowPulsesSinceStart() const {
    if (!isFlowCalibrationActive() || flowMeter_ == nullptr) {
        return 0;
    }
    return (uint64_t)flowMeter_->getCalibrationPulsesSinceStart();
}

uint32_t CalibrationManager::getPumpStepsCompleted() const {
    if ((SubState)subState_ == SubState::PUMP_METERING && engine_ != nullptr) {
        return engine_->getStepsDone(); // live progress
    }
    return pumpStepsDelivered_;
}

// Shared start guard (spec 3.9 "calibration uses StepperEngine only after
// normal dosing is prevented" + start preconditions).
bool CalibrationManager::calibrationsBlocked() const {
    if (isFlowCalibrationActive() || isPumpCalibrationActive()) {
        return true;
    }
    if (dosing_ != nullptr
        && (dosing_->isDoseActive() || dosing_->isDosePending())) {
        return true;
    }
    if (safety_ != nullptr && safety_->isFatalFaultActive()) {
        // Tank empty is a warning only and does not block calibration.
        return true;
    }
    return false;
}

// ---------------------------------------------------------------
// Flow calibration (spec 3.9 flow procedure)
// ---------------------------------------------------------------
bool CalibrationManager::startFlowCalibration() {
    if (flowMeter_ == nullptr) {
        return false;
    }
    if (calibrationsBlocked()) {
        Logger::warning("flow calibration rejected: calibration/dose active "
                        "or safety fault active");
        return false;
    }

    // FlowMeter snapshots the current pulse count (spec 3.9 flow step 2);
    // the user then lets a known volume pass and stops the session.
    flowMeter_->beginFlowCalibration();
    subState_ = (uint8_t)SubState::FLOW_ACTIVE;
    Logger::event(EVENT_FLOW_CAL_START, "flow calibration started");
    Alerts::queue().push(AlertSeverity::INFO, FaultCode::NONE,
                         "flow calibration started", millis());
    Serial.println("cal flow           : started (let a known water volume pass)");
    return true;
}

bool CalibrationManager::stopFlowCalibration(float volumeLiters) {
    if (!isFlowCalibrationActive() || flowMeter_ == nullptr) {
        Logger::warning("flow calibration stop rejected: no active session");
        return false;
    }

    // Input validation (spec 3.9 flow step 7; FlowMeter repeats its own
    // check and enforces the 10..100000 pulses/liter result range).
    if (!isfinite(volumeLiters) || volumeLiters <= 0.0f
        || volumeLiters > constants::MAX_FLOW_CAL_VOLUME_LITER) {
        flowMeter_->cancelFlowCalibration();
        subState_ = (uint8_t)SubState::NONE;
        Logger::error("flow calibration rejected: invalid volume input");
        Alerts::queue().push(AlertSeverity::ERROR, FaultCode::CAL_INVALID_INPUT,
                            "flow calibration rejected: invalid volume", millis());
        Serial.println("cal flow           : INVALID VOLUME, previous calibration retained");
        return false;
    }

    const uint32_t deltaPulses = flowMeter_->getCalibrationPulsesSinceStart();
    const bool ok = flowMeter_->completeFlowCalibration(*store_, volumeLiters);
    if (ok) {
        flowLastDeltaPulses_ = (float)deltaPulses;
        Logger::event(EVENT_FLOW_CAL_SAVED, "flow calibration saved");
        Alerts::queue().push(AlertSeverity::INFO, FaultCode::NONE,
                             "flow calibration saved", millis());
        Serial.printf("cal flow           : saved %.1f pulses/liter (%lu pulses / %.3f L)\n",
                      (double)flowMeter_->getPulsesPerLiter(),
                      (unsigned long)deltaPulses, (double)volumeLiters);
    } else {
        // Invalid result (zero deltas, out of range) or failed NVS write:
        // the previous calibration value is retained (spec 3.9/3.2).
        Logger::error("flow calibration rejected, previous value retained");
        Alerts::queue().push(AlertSeverity::ERROR, FaultCode::CAL_INVALID_INPUT,
                            "flow calibration failed, previous value retained",
                            millis());
        Serial.println("cal flow           : rejected, previous calibration retained");
    }
    subState_ = (uint8_t)SubState::NONE; // session always ends
    return ok;
}

void CalibrationManager::cancelFlowCalibration() {
    if (!isFlowCalibrationActive()) {
        return;
    }
    if (flowMeter_ != nullptr) {
        flowMeter_->cancelFlowCalibration();
    }
    subState_ = (uint8_t)SubState::NONE;
    Logger::info("flow calibration canceled, previous value retained");
    Serial.println("cal flow           : canceled, previous calibration retained");
}

// ---------------------------------------------------------------
// Pump calibration (spec 3.9 pump procedure)
// ---------------------------------------------------------------
bool CalibrationManager::startPumpCalibration() {
    if (engine_ == nullptr || safety_ == nullptr) {
        return false;
    }
    if (calibrationsBlocked()) {
        Logger::warning("pump calibration rejected: calibration/dose active "
                        "or safety fault active");
        return false;
    }
    if (engine_->isBusy() || engine_->getMode() == StepperEngine::Mode::FAULT) {
        Logger::warning("pump calibration rejected: motor busy");
        return false;
    }

    // Step 3: run forward until the acid line detector is present. The
    // PUMP_PRIMING pass in update() promotes to PUMP_METERING immediately
    // when the line is already present.
    purgeAbortsAfter_ = false;
    pumpStepsDelivered_ = 0;
    subState_ = (uint8_t)SubState::PUMP_PRIMING;
    engine_->startForwardPrime(constants::MAX_FORWARD_PRIME_MS);
    Logger::event(EVENT_PUMP_CAL_START, "pump calibration started (1600 steps)");
    Alerts::queue().push(AlertSeverity::INFO, FaultCode::NONE,
                         "pump calibration started", millis());
    Serial.printf("cal pump           : started, priming (metered run: %lu steps)\n",
                  (unsigned long)constants::PUMP_CAL_STEPS);
    return true;
}

bool CalibrationManager::confirmPumpCalibration(float volumeMilliliters) {
    if ((SubState)subState_ != SubState::PUMP_WAIT_VOLUME) {
        Logger::warning("pump calibration confirm rejected: not awaiting volume");
        return false;
    }

    // Compute + validate (spec 3.9 pump steps 7-10):
    //   pump_ml_per_step = volume_ml / 1600.0f
    //   valid when finite and 0.001 <= result <= 10.0
    if (!isfinite(volumeMilliliters) || volumeMilliliters <= 0.0f) {
        Logger::error("pump calibration rejected: invalid volume input");
        Alerts::queue().push(AlertSeverity::ERROR, FaultCode::CAL_INVALID_INPUT,
                            "pump calibration rejected: invalid volume", millis());
        Serial.println("cal pump           : INVALID VOLUME, previous calibration retained");
        subState_ = (uint8_t)SubState::NONE; // discard, previous value kept
        return false;
    }

    const float mlPerStep = volumeMilliliters / (float)constants::PUMP_CAL_STEPS;
    const bool valid = isfinite(mlPerStep)
        && mlPerStep >= constants::PUMP_CAL_MIN_ML_PER_STEP
        && mlPerStep <= constants::PUMP_CAL_MAX_ML_PER_STEP;

    if (valid && store_ != nullptr && store_->setPumpCalibration(mlPerStep)) {
        pumpCalVolumeMl_ = volumeMilliliters;
        Logger::event(EVENT_PUMP_CAL_SAVED, "pump calibration saved");
        Alerts::queue().push(AlertSeverity::INFO, FaultCode::NONE,
                             "pump calibration saved", millis());
        Serial.printf("cal pump           : saved %.4f ml/step (%.1f ml / %lu steps)\n",
                      (double)mlPerStep, (double)volumeMilliliters,
                      (unsigned long)constants::PUMP_CAL_STEPS);
        subState_ = (uint8_t)SubState::NONE;
        return true;
    }

    // Invalid result or failed config write (spec 3.2: abort calibration
    // and retain the previous value).
    Logger::error("pump calibration rejected, previous value retained");
    Alerts::queue().push(AlertSeverity::ERROR, FaultCode::CAL_INVALID_INPUT,
                        "pump calibration failed, previous value retained", millis());
    Serial.println("cal pump           : rejected, previous calibration retained");
    subState_ = (uint8_t)SubState::NONE; // discard, previous value kept
    return false;
}

void CalibrationManager::cancelPumpCalibration() {
    if (!isPumpCalibrationActive()) {
        return;
    }
    const SubState s = (SubState)subState_;
    if (s == SubState::PUMP_WAIT_VOLUME) {
        // The line is already in its post-purge state; nothing to purify.
        Logger::info("pump calibration canceled, previous value retained");
        Serial.println("cal pump           : canceled, previous calibration retained");
        subState_ = (uint8_t)SubState::NONE;
        return;
    }
    // Spec 3.9 constraint: cancel stops the motor and reverse purges to a
    // safe dry line state when the line was primed, then discards.
    if (engine_ != nullptr && engine_->isBusy()) {
        engine_->stopNow();
    }
    if (safety_ != nullptr && safety_->isLinePresent()
        && engine_ != nullptr && !engine_->isBusy()) {
        // Line contains acid: purge it before finishing the cancel.
        purgeAbortsAfter_ = true;
        subState_ = (uint8_t)SubState::PUMP_PURGING;
        engine_->startReversePurge(constants::MAX_PURGE_MS);
        Logger::info("pump calibration cancel: reverse purging to dry line");
        Serial.println("cal pump           : cancel, reverse purging line...");
    } else {
        Logger::info("pump calibration canceled, previous value retained");
        Serial.println("cal pump           : canceled, previous calibration retained");
        subState_ = (uint8_t)SubState::NONE;
    }
}

// ---------------------------------------------------------------
// State machine (spec 3.9 pump procedure + failure rules)
// ---------------------------------------------------------------
void CalibrationManager::update(uint32_t nowMs) {
    const SubState s = (SubState)subState_;

    // --- Flow: FLOW_ACTIVE -\u003e FLOW_WAIT_VOLUME when flow has settled -------
    if (s == SubState::FLOW_ACTIVE && flowMeter_ != nullptr
        && !flowMeter_->isFresh(constants::FLOW_CAL_WAIT_SETTLE_MS)) {
        subState_ = (uint8_t)SubState::FLOW_WAIT_VOLUME;
        Serial.printf("cal flow           : no flow for %lu ms (pulses since start: %lu), "
                      "enter the measured volume\n",
                      (unsigned long)constants::FLOW_CAL_WAIT_SETTLE_MS,
                      (unsigned long)flowMeter_->getCalibrationPulsesSinceStart());
        return;
    }

    // --- Fatal fault during any pump phase (spec 3.9 constraint) --------------
    if (isPumpCalibrationActive()) {
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
            if (engine_ != nullptr && engine_->isBusy()) {
                engine_->stopNow();
            }
            Logger::event(EVENT_SAFE_SHUTDOWN, "pump calibration aborted (fatal fault)");
            Alerts::queue().push(AlertSeverity::ERROR, code,
                                "pump calibration aborted: fatal fault", millis());
            Serial.println("cal pump           : ABORTED (fatal fault), previous calibration retained");
            subState_ = (uint8_t)SubState::NONE;
            return;
        }
    }

    switch (s) {
        case SubState::PUMP_PRIMING: {
            if (engine_ == nullptr) {
                subState_ = (uint8_t)SubState::NONE;
                return;
            }
            if (safety_->isLinePresent()) {
                // Acid reached the line: the exact 1600-step metered run.
                pumpStepsDelivered_ = 0;
                subState_ = (uint8_t)SubState::PUMP_METERING;
                engine_->startForwardMeter(constants::PUMP_CAL_STEPS);
                Logger::event(EVENT_LINE_DETECTED, "pump calibration: line present, metering 1600 steps");
                Serial.printf("cal pump           : line present, metering %lu steps\n",
                              (unsigned long)engine_->getTargetSteps());
                break;
            }
            if (engine_->isPrimeTimedOut()) {
                // No acid in the line: warn, reverse purge for safety, then
                // abort (discard) the calibration (spec 3.9 failure handling).
                Logger::warning("pump calibration prime timeout (LINE_NO_ACID)");
                Alerts::queue().push(AlertSeverity::WARNING, FaultCode::LINE_NO_ACID,
                                    "pump calibration: no acid detected in line",
                                    millis());
                Serial.println("cal pump           : PRIME TIMEOUT, purging line, aborting");
                engine_->stopNow();
                purgeAbortsAfter_ = true;
                subState_ = (uint8_t)SubState::PUMP_PURGING;
                engine_->startReversePurge(constants::MAX_PURGE_MS);
            }
            break;
        }

        case SubState::PUMP_METERING: {
            if (engine_ == nullptr) {
                subState_ = (uint8_t)SubState::NONE;
                return;
            }
            if (engine_->isMeterComplete()) {
                // Exact step count delivered (the engine stopped at the last
                // pulse). Capture it BEFORE the purge zeroes the counters.
                pumpStepsDelivered_ = engine_->getStepsDone();
                subState_ = (uint8_t)SubState::PUMP_PURGING;
                purgeAbortsAfter_ = false; // success path -\u003e PUMP_WAIT_VOLUME
                engine_->startReversePurge(constants::MAX_PURGE_MS);
                Serial.printf("cal pump           : metered %lu steps, reverse purge started\n",
                              (unsigned long)pumpStepsDelivered_);
            }
            break;
        }

        case SubState::PUMP_PURGING: {
            if (engine_ == nullptr) {
                subState_ = (uint8_t)SubState::NONE;
                return;
            }
            if (safety_->isLineEmpty()) {
                // Line safe/dry: stop the motor.
                engine_->stopNow();
                Logger::event(EVENT_LINE_PURGE_COMPLETE, "reverse purge complete, line empty");
                if (purgeAbortsAfter_) {
                    // Cancel or abort (prime timeout): discard the run.
                    Serial.println("cal pump           : purged, calibration discarded, "
                                   "previous calibration retained");
                    subState_ = (uint8_t)SubState::NONE;
                } else {
                    // Spec 3.9 pump step 6: line empty before volume input
                    // -\u003e await the dispensed volume (ml).
                    Serial.printf("cal pump           : line empty (%lu steps dispensed), "
                                  "enter the measured volume in ml\n",
                                  (unsigned long)pumpStepsDelivered_);
                    subState_ = (uint8_t)SubState::PUMP_WAIT_VOLUME;
                }
                break;
            }
            if (engine_->isPurgeTimedOut()) {
                // Line still not empty after the purge window: stop the
                // motor and abort (same rule as dosing, spec 3.8/3.9).
                engine_->stopNow();
                Logger::warning("pump calibration purge timeout (LINE_NOT_EMPTY)");
                Alerts::queue().push(AlertSeverity::WARNING, FaultCode::LINE_NOT_EMPTY,
                                    "pump calibration: reverse purge timeout",
                                    millis());
                Serial.println("cal pump           : PURGE TIMEOUT, aborting calibration");
                subState_ = (uint8_t)SubState::NONE;
            }
            break;
        }

        case SubState::NONE:
        case SubState::FLOW_ACTIVE:
        case SubState::FLOW_WAIT_VOLUME:
        case SubState::PUMP_WAIT_VOLUME:
            // No pump-phase action required this pass.
            break;
    }
    (void)nowMs;
}
