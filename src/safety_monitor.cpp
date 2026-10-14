#include "safety_monitor.h"

#include <Arduino.h>

#include "constants.h"
#include "hal.h"
#include "logger.h"

void SafetyMonitor::begin() {
    lastLineMs_ = 0;
    lastTankMs_ = 0;
    lastFaultMs_ = 0;
    lineRaw_ = false;
    tankRaw_ = false;
    faultRaw_ = false;
    lineStreak_ = 0;
    tankStreak_ = 0;
    faultStreak_ = 0;
    linePresent_ = false;
    tankEmpty_ = false;
    driverFault_ = false;
    fatalFaultActive_ = false;
    warningActive_ = false;
    safeStop_ = false;
    lastCode_ = FaultCode::NONE;
    Logger::info("safety monitor armed (line 10ms, tank 1000ms, fault 10ms poll)");
}

void SafetyMonitor::update(uint32_t nowMs) {
    if (nowMs - lastLineMs_ >= LINE_POLL_MS) {
        lastLineMs_ = nowMs;
        const bool raw = sampleLine();
        if (raw != lineRaw_) {
            lineRaw_ = raw;
            lineStreak_ = 1;
        } else {
            if (lineStreak_ < 0xFFFFu) {
                lineStreak_++;
            }
        }
        if (lineStreak_ >= LINE_CONFIRM_SAMPLES && lineRaw_ != linePresent_) {
            if(lineRaw_) {
                linePresent_ = lineRaw_;
                onLineConfirmed(linePresent_);
            } else {
                //allows 1 second for possible air pubble to clear before triggering
                if(lineStreak_ >= 100) {
                    linePresent_= lineRaw_;
                    onLineConfirmed(linePresent_);
                }
            }
            
            
        }
    }

    if (nowMs - lastTankMs_ >= TANK_POLL_MS) {
        lastTankMs_ = nowMs;
        const bool raw = sampleTankEmpty();
        if (raw != tankRaw_) {
            tankRaw_ = raw;
            tankStreak_ = 1;
        } else {
            if (tankStreak_ < 0xFFFFu) {
                tankStreak_++;
            }
        }
        if (tankStreak_ >= TANK_CONFIRM_SAMPLES && tankRaw_ != tankEmpty_) {
            tankEmpty_ = tankRaw_;
            onTankConfirmed(tankEmpty_);
        }
    }

    if (nowMs - lastFaultMs_ >= DRV_FAULT_POLL_MS) {
        lastFaultMs_ = nowMs;
        const bool raw = sampleDriverFault();
        if (raw != faultRaw_) {
            faultRaw_ = raw;
            faultStreak_ = 1;
        } else {
            if (faultStreak_ < 0xFFFFu) {
                faultStreak_++;
            }
        }
        if (faultStreak_ >= FAULT_CONFIRM_SAMPLES && faultRaw_ != driverFault_) {
            driverFault_ = faultRaw_;
            onDriverFaultConfirmed(driverFault_);
        }
    }
}

// --- Confirmed transition handlers (main loop only) -------------------------

void SafetyMonitor::onLineConfirmed(bool present) {
    // Line state changes are LOGGED here and exposed live via isLinePresent()
    // (/api/state). The WARNING *alerts* are only raised by the modules that
    // know the context (DosingController: line loss mid-dose, spec 3.8;
    // CalibrationManager), so a normal successful dose - which ends with an
    // expected line-empty transition - no longer injects bogus warnings into
    // the alert queue / MQTT (AI review M1).
    if (present) {
        // Liquid present in the line -> state change for log/UI only.
        Logger::event(EVENT_LINE_DETECTED, "acid line: liquid present");
        lastCode_ = FaultCode::LINE_NOT_EMPTY;
    } else {
        // Liquid gone from the line -> state change for log/UI only.
        Logger::event(EVENT_LINE_LOST, "acid line: liquid lost");
        lastCode_ = FaultCode::LINE_NO_ACID;
    }
}

void SafetyMonitor::onTankConfirmed(bool tankEmpty) {
    if (tankEmpty) {
        // WARNING only: tank empty. Do not latch the fatal flag or request a
        // safe stop; the condition is surfaced via /api/state (tank_empty),
        // the alert queue (web UI + MQTT) and getFaultCode(). Dosing is NOT
        // affected by tank empty (product decision; supersedes spec 3.5).

        Logger::event(EVENT_TANK_EMPTY, "tank empty (ALERT)");
        fatalFaultActive_ = false;
        safeStop_ = false;
        lastCode_ = FaultCode::TANK_EMPTY;
        Alerts::queue().push(AlertSeverity::WARNING,
                             FaultCode::TANK_EMPTY,
                             "TANK_EMPTY",
                             millis());
    } else {
        // Tank refill confirmed. Tank empty is warning-only, so the fatal
        // latch is only held by a concurrent driver fault.
        if (!driverFault_) {
            lastCode_ = FaultCode::NONE;
        }
    }
}

void SafetyMonitor::onDriverFaultConfirmed(bool asserted) {
    if (asserted) {
        // FATAL: DRV8825 fault asserted (active low input went low and held).
        Logger::event(EVENT_DRV_FAULT, "DRV8825 fault (FATAL)");
        fatalFaultActive_ = true;
        safeStop_ = true;
        lastCode_ = FaultCode::DRIVER_FAULT;
        Alerts::queue().push(AlertSeverity::ERROR,
                             FaultCode::DRIVER_FAULT,
                             "DRIVER_FAULT",
                             millis());
    } else {
        // Fault released: the fatal latch clears. Tank empty is only a
        // warning and never holds the fatal latch.
        fatalFaultActive_ = false;
        lastCode_ = FaultCode::NONE;
    }
}

// --- Accessors ---------------------------------------------------------------

bool SafetyMonitor::isLinePresent() const { return linePresent_; }
bool SafetyMonitor::isTankEmpty() const { return tankEmpty_; }
bool SafetyMonitor::isDriverFault() const { return driverFault_; }

bool SafetyMonitor::isFatalFaultActive() const {
    return fatalFaultActive_ || driverFault_;
}

FaultCode SafetyMonitor::getFaultCode() const {
    // Driver fault is more severe than tank empty (warning).
    if (driverFault_) {
        return FaultCode::DRIVER_FAULT;
    }
    if (tankEmpty_) {
        return FaultCode::TANK_EMPTY;
    }
    if (warningActive_) {
        return lastCode_;
    }
    return FaultCode::NONE;
}

void SafetyMonitor::clearWarning() {
    warningActive_ = false;
    if (lastCode_ == FaultCode::LINE_NO_ACID || lastCode_ == FaultCode::LINE_NOT_EMPTY) {
        if (!isFatalFaultActive()) {
            lastCode_ = FaultCode::NONE;
        }
    }
    Logger::info("safety warning acknowledged");
}

void SafetyMonitor::clearFaultAfterPowerCycle() {
    tankEmpty_ = false;
    driverFault_ = false;
    fatalFaultActive_ = false;
    warningActive_ = false;
    safeStop_ = false;
    lastCode_ = FaultCode::NONE;
    Logger::info("safety fault latches cleared (after power cycle)");
}

// --- Sampling ----------------------------------------------------------------

bool SafetyMonitor::sampleLine() const {
#ifdef ENABLE_TEST_INJECTION
    if (injectMode_) {
        return injectLineLevel_;
    }
#endif
    return Hal::readAcidLine(); // high = acid present
}

bool SafetyMonitor::sampleTankEmpty() const {
#ifdef ENABLE_TEST_INJECTION
    if (injectMode_) {
        return injectTankLevel_;
    }
#endif
    return Hal::readTankEmpty(); // high = tank empty
}

bool SafetyMonitor::sampleDriverFault() const {
#ifdef ENABLE_TEST_INJECTION
    if (injectMode_) {
        return injectFaultLevel_;
    }
#endif
    return Hal::readDriverFault(); // low = fault asserted
}

#ifdef ENABLE_TEST_INJECTION
void SafetyMonitor::injectLine(bool present) {
    injectMode_ = true;
    injectLineLevel_ = present;
    Serial.printf("test inject: line_present=%d\n", (int)present);
}

void SafetyMonitor::injectTankEmpty(bool tankEmpty) {
    injectMode_ = true;
    injectTankLevel_ = tankEmpty;
    Serial.printf("test inject: tank_empty=%d\n", (int)tankEmpty);
}

void SafetyMonitor::injectDriverFault(bool faultAsserted) {
    injectMode_ = true;
    injectFaultLevel_ = faultAsserted;
    Serial.printf("test inject: driver_fault=%d\n", (int)faultAsserted);
}

void SafetyMonitor::clearInjection() {
    injectMode_ = false;
    injectLineLevel_ = false;
    injectTankLevel_ = false;
    injectFaultLevel_ = false;
    Serial.println("test inject: cleared (physical pins active again)");
}

void SafetyMonitor::report() {
    Serial.printf("safety : line_present=%d tank_empty=%d driver_fault=%d "
                  "fatal=%d warning=%d safe_stop=%d code=%d "
                  "alerts_queued=%u alerts_dropped=%u\n",
                  (int)isLinePresent(),
                  (int)isTankEmpty(),
                  (int)isDriverFault(),
                  (int)isFatalFaultActive(),
                  (int)isWarningActive(),
                  (int)safeStopRequested(),
                  (unsigned)(uint16_t)getFaultCode(),
                  (unsigned)Alerts::queue().size(),
                  (unsigned)Alerts::queue().droppedCount());
}
#endif
