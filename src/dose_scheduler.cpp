// DoseScheduler implementation.
//
// Budget: the once-per-second check is four (hour/minute/enabled/day-key)
// integer comparisons per slot plus a single localtime_r() call. This is
// far below the 100 microsecond scheduler budget and does no allocation,
// logging, NVS, motor, or network work.
#include "dose_scheduler.h"

#include "constants.h"
#include "logger.h"

void DoseScheduler::begin(ClockManager &clock, ConfigStore &store,
                          MayDoseGuard mayDose, void *guardContext) {
    clock_ = &clock;
    store_ = &store;
    mayDose_ = mayDose;
    guardContext_ = guardContext;
    lastCheckMs_ = 0;      // force the first update() to run its check
    requestSlot_ = SCHED_NO_REQUEST;
}

void DoseScheduler::update(uint32_t nowMs) {
    // Once per second, driven by the main loop (spec 3.7, Step 5 req. 3).
    if (lastCheckMs_ != 0 && nowMs - lastCheckMs_ < constants::SCHED_CHECK_PERIOD_MS) {
        return;
    }
    lastCheckMs_ = nowMs;

    // Guard: no dose when time is unknown, or when AppCore says dosing is
    // not allowed (state not IDLE/COUNTING, fatal fault, calibration or an
    // active dose).
    if (!clock_->isTimeKnown()) {
        return;
    }
    if (mayDose_ != nullptr && !mayDose_(guardContext_)) {
        return;
    }

    struct tm lt;
    if (!clock_->fillLocalTime(lt)) {
        return;
    }
    const uint16_t today = (uint16_t)lt.tm_yday; // 0..365 day-of-year

    const AppConfig &cfg = store_->getAppConfig();
    for (uint8_t slot = 0; slot < constants::DEFAULT_SCHEDULE_COUNT; ++slot) {
        const ScheduleSlot &s = cfg.schedule[slot];
        if (!s.enabled) {
            continue;
        }
        // Local hour/minute must both match this slot.
        if (s.hour != (uint8_t)lt.tm_hour || s.minute != (uint8_t)lt.tm_min) {
            continue;
        }
        // Day key: never re-fire a slot that already fired today.
        if (s.lastDayOfYear == today+1) {
            continue;
        }
        // If a request is already pending we do not overwrite it; the
        // pending slot will be consumed by the DosingController before the
        // next once-per-second tick.
        if (requestPending()) {
            continue;
        }
        fireSlot(slot, today);
    }
}

void DoseScheduler::fireSlot(uint8_t slot, uint16_t today) {
    // Claim the slot atomically with respect to the once-per-second tick:
    // set the pending id first, then persist the day key so a reboot in the
    // same minute cannot cause a double fire. All main-loop only.
    requestSlot_ = slot;
    store_->setSlotFiredDay(slot, today+1);
    Serial.printf("scheduler          : slot %u matched, dose requested\n",
                  (unsigned)slot);
    Logger::info("scheduler: slot fired, dose requested");
}

uint8_t DoseScheduler::consumeRequest() {
    if (!requestPending()) {
        return SCHED_NO_REQUEST;
    }
    const uint8_t slot = requestSlot_;
    requestSlot_ = SCHED_NO_REQUEST;
    return slot;
}

bool DoseScheduler::isTimeKnown() const {
    return clock_ != nullptr && clock_->isTimeKnown();
}

bool DoseScheduler::syncTime(uint32_t epochSeconds) {
    return clock_ != nullptr && clock_->setEpoch(epochSeconds);
}

bool DoseScheduler::setTimezoneOffsetMinutes(int16_t offsetMinutes) {
    return clock_ != nullptr && clock_->updateTimezone(offsetMinutes);
}

bool DoseScheduler::setSlot(uint8_t slot, uint8_t hour, uint8_t minute, bool enabled) {
    return store_ != nullptr && store_->setScheduleSlot(slot, hour, minute, enabled);
}

bool DoseScheduler::getSlot(uint8_t slot, uint8_t &hour, uint8_t &minute, bool &enabled) const {
    if (store_ == nullptr || slot >= constants::DEFAULT_SCHEDULE_COUNT) {
        return false;
    }
    const ScheduleSlot &s = store_->getAppConfig().schedule[slot];
    hour = s.hour;
    minute = s.minute;
    enabled = s.enabled;
    return true;
}

bool DoseScheduler::noteSlotFired(uint8_t slot, uint16_t dayOfYear) {
    return store_ != nullptr && store_->setSlotFiredDay(slot, dayOfYear);
}

uint16_t DoseScheduler::dayOfYear() const {
    struct tm lt;
    if (clock_ == nullptr || !clock_->fillLocalTime(lt)) {
        return 0;
    }
    return (uint16_t)lt.tm_yday;
}

bool DoseScheduler::getLocalTime(uint8_t &hour, uint8_t &minute) const {
    struct tm lt;
    if (clock_ == nullptr || !clock_->fillLocalTime(lt)) {
        return false;
    }
    hour = (uint8_t)lt.tm_hour;
    minute = (uint8_t)lt.tm_min;
    return true;
}
