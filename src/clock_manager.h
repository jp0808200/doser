// ClockManager: system clock ownership for the acid-dosing firmware.
//
// Per acid-dosing-fw-specification.md 3.7 (scheduler uses localtime_r and a
// timezone offset), 3.2 (ntpEnabled / ntpServer / timezoneOffsetMinutes
// configuration fields) and the Step 5 prompt requirement 4 (time handling):
//
//  - NTP is the time source (the device is expected to be network connected).
//    The framework's built-in, non-blocking SNTP client is used
//    (Arduino core `configTime()`, an existing public facility - no new
//    library, no application task is created by it) together with newlib's
//    time()/localtime_r().
//  - Manual epoch set is supported (Step 5 requirement 4 / Step 8 /api/time)
//    through settimeofday().
//  - The timezone offset (minutes, from configuration) is applied to the
//    local time conversion, so all consumers work on LOCAL time.
//  - `timeKnown` latches high once the system clock reports a sane epoch
//    (>= 2020-01-01). The DoseScheduler must never auto-dose while the
//    clock is unknown.
//
// All methods are non-blocking and must be called from the main loop only.
#ifndef ACID_DOSE_CLOCK_MANAGER_H
#define ACID_DOSE_CLOCK_MANAGER_H

#include <Arduino.h>

#include <time.h>

#include "types.h"

class ClockManager {
public:
    // Earliest epoch accepted as "time known" (2020-01-01 00:00:00 UTC).
    static constexpr time_t MIN_VALID_EPOCH = 1577836800;

    // Applies the timezone and starts the non-blocking NTP client when the
    // configuration enables it and a server name is present.
    // Must run from the main loop after ConfigStore::load().
    void begin(const AppConfig &cfg);

    // Main-loop hook: refreshes the timeKnown latch. Call once per loop (or
    // at any rate >= 1/10 s); the underlying clock is read lazily.
    void update(uint32_t nowMs);

    // True once the system clock holds a sane epoch (NTP sync or manual set).
    bool isTimeKnown() const { return timeKnown_; }

    // Current system epoch seconds (time()), 0 when unknown.
    uint32_t getEpoch() const;

    // Manual time set (Step 5 requirement 4: epoch value persists in RAM).
    // Rejects epochs older than MIN_VALID_EPOCH.
    bool setEpoch(uint32_t epochSeconds);

    // Applies a new timezone offset (minutes, clamped to +-13 hours) to the
    // local time conversion and re-arms NTP if it was enabled.
    bool updateTimezone(int16_t offsetMinutes);
    int16_t timezoneOffsetMinutes() const { return offsetMinutes_; }

    // Applies new NTP settings (Step 8 /api/time and /api/config changes).
    // When enabled with a non-empty server, (re-)arms the SNTP client.
    bool setNtp(bool enabled, const char *server);
    bool ntpEnabled() const { return ntpEnabled_; }
    const char *ntpServer() const { return ntpServer_; }

    // Fills `lt` with the current LOCAL time. Returns false when the clock
    // is not yet known. Main loop / web context only.
    bool fillLocalTime(struct tm &lt) const;

    // Formats LOCAL time as "YYYY-MM-DD HH:MM:SS" (NUL-terminated).
    // Returns false when the clock is not yet known.
    bool formatLocal(char *buffer, size_t bufferSize) const;

private:
    bool applyTimezoneAndNtp();

    bool timeKnown_ = false;
    int16_t offsetMinutes_ = 0;
    bool ntpEnabled_ = false;
    char ntpServer_[64] = {0};
    uint32_t lastCheckMs_ = 0;
    uint32_t lastLogMs_ = 0;
};

#endif /* ACID_DOSE_CLOCK_MANAGER_H */
