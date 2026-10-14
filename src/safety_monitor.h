// SafetyMonitor: tank level, acid line detector and DRV8825 fault polling
// with debounced output flags and fault/warning alert generation.
//
// Per acid-dosing-fw-specification.md 3.5 (safety monitor: polling rules,
// input interpretation, fatal-fault rules, behaviour), 4.4/4.5 (shared
// flags and critical sections), the Step 4 prompt and the alert queue
// requirements (3.12).
//
// Input interpretation (specification 3.5 behaviour / Step 4 requirement 4):
//   ACID_LINE_DET high -> acid liquid present in the line
//   TANK_LEVEL    high -> tank empty (WARNING only; dosing is unaffected)
//   DRV_FAULT     low  -> DRV8825 fault asserted (FATAL)
//
// Debounce rules (specification 3.5 / Step 4 requirement 3):
//   line detector : two consecutive 10 ms samples must agree
//   tank empty    : two consecutive samples must agree (1 s poll)
//   driver fault  : 200 ms of consecutive asserted samples (20 x 10 ms)
//
// The monitor itself performs NO motor control. On a confirmed fatal fault
// it only (a) latches the state, (b) logs the event and (c) pushes the
// alert into the alert queue; the immediate motor disable (safe stop) is
// applied by AppCore via Hal in the same loop iteration (Step 4 req. 10),
// which then enters the ERROR state without resetting the system.
#ifndef ACID_DOSE_SAFETY_MONITOR_H
#define ACID_DOSE_SAFETY_MONITOR_H

#include <cstdint>

#include <stddef.h>

#include "alert_queue.h"
#include "types.h"

class SafetyMonitor {
public:
    // Polling intervals (specification 3.5 polling rules / Step 4 req. 2).
    static constexpr uint32_t LINE_POLL_MS      = 10;
    static constexpr uint32_t TANK_POLL_MS      = 1000;
    static constexpr uint32_t DRV_FAULT_POLL_MS = 10;
    static constexpr uint32_t FAULT_CONFIRM_MS  = 200;

    void begin();

    // Non-blocking: samples inputs according to the polling intervals,
    // advances the debounce counters and publishes state-change events
    // (log + alerts) exactly once per confirmed transition. Call once per
    // main loop, main-loop context only.
    void update(uint32_t nowMs);

    // Debounced output flags (specification 3.5 API).
    bool isLinePresent() const;
    bool isLineEmpty() const { return !isLinePresent(); }
    bool isTankEmpty() const;
    bool isDriverFault() const;

    // True when a fatal fault (DRV8825 driver fault) is confirmed or was
    // latched; stays true until clearFaultAfterPowerCycle(). Tank empty is
    // a WARNING condition and does NOT count as fatal.
    bool isFatalFaultActive() const;

    // True when a non-fatal warning (line no acid / line not empty) is
    // currently latched and not yet acknowledged/cleared.
    bool isWarningActive() const { return warningActive_; }

    // Most severe unresolved fault/warning code, FaultCode::NONE if clear.
    FaultCode getFaultCode() const;

    // Clear the non-fatal warning latch (Step 4 requirement 5 API:
    // clearWarning). Fatal faults are NOT cleared by this.
    void clearWarning();

    // Clear latches after the fault has been serviced (power cycle / ack).
    // (intentionally unconnected: the fatal-fault path ends in deep sleep
    //  - SAFE_SHUTDOWN - so the device always recovers via a real power
    //  cycle; kept for a future re-armable-fault design (AI review M2).)
    void clearFaultAfterPowerCycle();

    // True when the monitor latched a fatal fault and the safe stop (motor
    // disable) has been requested; AppCore applies and tracks this.
    bool safeStopRequested() const { return safeStop_; }

#ifdef ENABLE_TEST_INJECTION
    // HIL test injection (specification 5.2): overrides the physical pin
    // samples for the debounce path. Compiled out of production builds.
    void injectLine(bool present);
    void injectTankEmpty(bool tankEmpty);
    void injectDriverFault(bool faultAsserted);
    void clearInjection();
    bool injectionActive() const { return injectMode_; }
    void report();
#endif

private:
    bool sampleLine() const;
    bool sampleTankEmpty() const;
    bool sampleDriverFault() const;

    void onLineConfirmed(bool present);
    void onTankConfirmed(bool tankEmpty);
    void onDriverFaultConfirmed(bool asserted);

    // Debounce bookkeeping; only ever touched from the main loop.
    uint32_t lastLineMs_ = 0;
    uint32_t lastTankMs_ = 0;
    uint32_t lastFaultMs_ = 0;

    // Per input: level observed in the last sample, number of consecutive
    // samples at that level, and the confirmed (debounced) output flag.
    bool lineRaw_ = false;
    uint16_t lineStreak_ = 0;
    bool linePresent_ = false;

    bool tankRaw_ = false;
    uint16_t tankStreak_ = 0;
    bool tankEmpty_ = false;

    bool faultRaw_ = false;
    uint16_t faultStreak_ = 0;
    bool driverFault_ = false;

    // Latches (specification 3.4-style volatile shared flags).
    volatile bool fatalFaultActive_ = false;
    volatile bool warningActive_ = false;
    volatile bool safeStop_ = false;
    FaultCode lastCode_ = FaultCode::NONE;

    // Consecutive-sample confirmation counts derived from the above rules.
    static constexpr uint16_t LINE_CONFIRM_SAMPLES = 2;
    static constexpr uint16_t TANK_CONFIRM_SAMPLES = 2;
    static constexpr uint16_t FAULT_CONFIRM_SAMPLES =
        (uint16_t)(FAULT_CONFIRM_MS / DRV_FAULT_POLL_MS); // 20

#ifdef ENABLE_TEST_INJECTION
    bool injectMode_ = false;
    bool injectLineLevel_ = false;
    bool injectTankLevel_ = false;
    bool injectFaultLevel_ = false;
#endif
};

#endif /* ACID_DOSE_SAFETY_MONITOR_H */
