// ClockManager implementation.
//
// Time source: NTP via the arduino-esp32 core's non-blocking SNTP client
// (configTime(), an existing public facility of the framework - no new
// library dependency, no FreeRTOS task of our own is created). Manual
// epoch set via settimeofday(). Both update the same system clock that
// time() and localtime_r() read, so a single clock source is shared by the
// scheduler, web and MQTT layers.
#include "clock_manager.h"

#include <sys/time.h>

#include "constants.h"
#include "logger.h"

void ClockManager::begin(const AppConfig &cfg) {
    offsetMinutes_ = cfg.timezoneOffsetMinutes;
    ntpEnabled_ = cfg.ntpEnabled;
    ntpServer_[sizeof(ntpServer_) - 1] = '\0';
    const size_t n = strnlen(cfg.ntpServer, sizeof(cfg.ntpServer));
    for (size_t i = 0; i < n && i < sizeof(ntpServer_) - 1; ++i) {
        ntpServer_[i] = cfg.ntpServer[i];
    }

    if (ntpEnabled_ && ntpServer_[0] != '\0') {
        // configTime() is non-blocking: it programs the core's SNTP client
        // which syncs automatically once a network route is available.
        // gmtOffset is the timezone in seconds (spec field: minutes).
        configTime((long)offsetMinutes_ * 60L, 0L, ntpServer_, nullptr, nullptr);
        Serial.printf("clock              : NTP enabled, server '%s', utc offset %d min\n",
                      ntpServer_, (int)offsetMinutes_);
        Logger::info("clock: NTP enabled, server configured");
    } else {
        Serial.println("clock              : NTP disabled, manual time set only");
    }

    // Latch timeKnown immediately in case the clock already holds a sane
    // value (e.g. an earlier sync or a manual set this boot is impossible,
    // but a fast re-check keeps update() cheap later).
    timeKnown_ = (time(nullptr) >= MIN_VALID_EPOCH);
}

void ClockManager::update(uint32_t nowMs) {
    if (timeKnown_) {
        return;
    }
    // Polling the wall clock every 1 s is far within the scheduler's
    // budget and gives a visible sync delay of at most one second.
    if (lastCheckMs_ != 0 && nowMs - lastCheckMs_ < 1000) {
        return;
    }
    lastCheckMs_ = nowMs;
    if (time(nullptr) >= MIN_VALID_EPOCH) {
        timeKnown_ = true;
        if (nowMs - lastLogMs_ > 1000) {
            lastLogMs_ = nowMs;
            Serial.println("clock              : time known, scheduler armed");
            Logger::info("clock: time known, scheduler armed");
        }
    }
}

uint32_t ClockManager::getEpoch() const {
    const time_t t = time(nullptr);
    if (t < MIN_VALID_EPOCH) {
        return 0;
    }
    return (uint32_t)t;
}

bool ClockManager::setEpoch(uint32_t epochSeconds) {
    if (epochSeconds < (uint32_t)MIN_VALID_EPOCH) {
        Serial.println("clock: setEpoch rejected (epoch before 2020-01-01)");
        return false;
    }
    struct timeval tv;
    tv.tv_sec = (time_t)epochSeconds;
    tv.tv_usec = 0;
    if (settimeofday(&tv, nullptr) != 0) {
        Serial.println("clock: settimeofday failed");
        return false;
    }
    // timeKnown latches on the next update() tick (<= 1 s) or immediately
    // if the write path verified the value, which it must have (newlib's
    // settimeofday returns 0 on success).
    timeKnown_ = (time(nullptr) >= MIN_VALID_EPOCH);
    Serial.printf("clock              : manual epoch set to %lu\n",
                  (unsigned long)epochSeconds);
    Logger::info("clock: manual epoch set");
    return true;
}

bool ClockManager::updateTimezone(int16_t offsetMinutes) {
    // Reasonable bound: +-13 h covers real-world offsets with margin.
    if (offsetMinutes < -780 || offsetMinutes > 780) {
        return false;
    }
    offsetMinutes_ = offsetMinutes;
    return applyTimezoneAndNtp();
}

bool ClockManager::setNtp(bool enabled, const char *server) {
    ntpEnabled_ = enabled;
    ntpServer_[sizeof(ntpServer_) - 1] = '\0';
    if (server != nullptr) {
        const size_t n = strnlen(server, sizeof(ntpServer_)-1);
        for (size_t i = 0; i < n && i < sizeof(ntpServer_) - 1; ++i) {
            ntpServer_[i] = server[i];
        }
    }
    return applyTimezoneAndNtp();
}

bool ClockManager::applyTimezoneAndNtp() {
    if (ntpServer_[0] == '\0' || !ntpEnabled_) {
        return false;
    }
    // Re-programming the core SNTP client is non-blocking and safe from
    // the main loop (no task of our own is created).
    configTime((long)offsetMinutes_ * 60L, 0L, ntpServer_, nullptr, nullptr);
    return true;
}

bool ClockManager::fillLocalTime(struct tm &lt) const {
    const time_t t = time(nullptr);
    if (t < MIN_VALID_EPOCH) {
        return false;
    }
    // The timezone was applied to the core clock conversion in
    // configTime()/applyTimezoneAndNtp(), so localtime_r already yields
    // local time (spec 3.7: scheduler works on local time via localtime_r).
    if (localtime_r(&t, &lt) == nullptr) {
        return false;
    }
    return true;
}

bool ClockManager::formatLocal(char *buffer, size_t bufferSize) const {
    struct tm lt;
    if (bufferSize == 0 || !fillLocalTime(lt)) {
        return false;
    }
    const int written = snprintf(buffer, bufferSize, "%04d-%02d-%02d %02d:%02d:%02d",
                                 lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                                 lt.tm_hour, lt.tm_min, lt.tm_sec);
    return (written > 0 && (size_t)written < bufferSize);
}
