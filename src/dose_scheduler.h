// DoseScheduler: up to four daily (HH:MM) dosing time slots.
//
// Per acid-dosing-fw-specification.md 3.7 (schedule rules, API, internal
// constraints) and the Step 5 prompt requirements 1, 3, 5 and 6.
//
// Rules implemented:
//   - fixed four slots, each with hour (0..23), minute (0..59), enabled,
//     lastDayOfYear (day key, prevents double firing within one day);
//   - the check runs once per second (SCHED_CHECK_PERIOD_MS) from the main
//     loop and is budgeted well under 100 microseconds (a few int
//     comparisons per slot);
//   - time is read as LOCAL time (time() + localtime_r()) through the
//     ClockManager, which applies the configured timezone and NTP/manual
//     source;
//   - a slot fires only when: time is known, the slot is enabled, local
//     hour and minute match, the day key is not equal to today's
//     day-of-year, and the AppCore guard allows dosing (state is IDLE or
//     COUNTING, no fatal fault, no calibration active, no dose active);
//   - firing emits at most one pending request; AppCore/DosingController
//     retrieve it with requestPending()/consumeRequest().
//
// The scheduler performs NO motor control and NO allocation; it only raises
// a single pending-request flag consumed by the DosingController in the
// same main-loop pass (spec 4.8: "Scheduler -> DosingController").
#ifndef ACID_DOSE_DOSE_SCHEDULER_H
#define ACID_DOSE_DOSE_SCHEDULER_H

#include <Arduino.h>

#include <stdint.h>

#include "clock_manager.h"
#include "config_store.h"
#include "types.h"

// "Pending" sentinel for the request slot id (0..3 are valid slots).
enum { SCHED_NO_REQUEST = 0xFF };

class DoseScheduler {
public:
    // MayDose: main-loop guard. Non-null implementations must keep the
    // check cheap (flags only) because it runs once per second inside the
    // scheduler's budget. context is the pointer passed to attach().
    using MayDoseGuard = bool (*)(void *context);

    // Loads the four slots (incl. their day keys) from ConfigStore and
    // attaches the time source and dosing guard. ClockManager is required;
    // a null MayDoseGuard means "no extra guard".
    void begin(ClockManager &clock, ConfigStore &store,
               MayDoseGuard mayDose = nullptr, void *guardContext = nullptr);

    // Main-loop hook: runs the once-per-second schedule check.
    // Must be called exactly once per main loop (AppCore update order).
    void update(uint32_t nowMs);

    // Pending request interface (spec 3.7 API).
    bool requestPending() const { return (requestSlot_ != SCHED_NO_REQUEST); }
    // Consumes the pending request and returns the slot id (0..3).
    // Only meaningful when requestPending() is true.
    uint8_t consumeRequest();

    bool isTimeKnown() const;

    // Manual time set (Step 5 requirement 4): stored in RAM via the core
    // clock, forwarded to the ClockManager.
    bool syncTime(uint32_t epochSeconds);
    bool setTimezoneOffsetMinutes(int16_t offsetMinutes);

    // Slot management (hour 0..23, minute 0..59).
    bool setSlot(uint8_t slot, uint8_t hour, uint8_t minute, bool enabled);
    bool getSlot(uint8_t slot, uint8_t &hour, uint8_t &minute, bool &enabled) const;
    // Persists the fired day key (day-of-year) for a slot to ConfigStore.
    // (intentionally unconnected: firing goes through fireSlot()/
    //  setSlotFiredDay() in the main loop; AI review n2.)
    bool noteSlotFired(uint8_t slot, uint16_t dayOfYear);

    // Today's day-of-year per the core clock. 0 when time is unknown.
    uint16_t dayOfYear() const;

    // Current local time fields (for status display), false when unknown.
    // (intentionally unconnected: consumers use ClockManager directly;
    //  AI review n2.)
    bool getLocalTime(uint8_t &hour, uint8_t &minute) const;

private:
    void fireSlot(uint8_t slot, uint16_t today);

    ClockManager *clock_ = nullptr;
    ConfigStore *store_ = nullptr;
    MayDoseGuard mayDose_ = nullptr;
    void *guardContext_ = nullptr;
    uint32_t lastCheckMs_ = 0;
    volatile uint8_t requestSlot_ = SCHED_NO_REQUEST;
};

#endif /* ACID_DOSE_DOSE_SCHEDULER_H */
