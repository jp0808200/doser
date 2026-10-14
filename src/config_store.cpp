#include "config_store.h"

#include <Arduino.h>
#include <math.h>
#include <optional>
#include <string.h>

#include "constants.h"
#include "logger.h"

namespace {

// Read helpers. A missing key or invalid NVS entry yields an empty Opt.
// No heap allocation: values are read directly into destination buffers.

std::optional<uint32_t> optU32(nvs_handle_t h, const char *key) {
    uint32_t v = 0;
    if (nvs_get_u32(h, key, &v) == ESP_OK) {
        return v;
    }
    return std::nullopt;
}

std::optional<float> optF32(nvs_handle_t h, const char *key) {
    // This ESP-IDF build has no nvs f32 API: persist the IEEE-754 bit
    // pattern of the float as a u32 under the same stable key name.
    std::optional<uint32_t> bits = optU32(h, key);
    if (!bits) {
        return std::nullopt;
    }
    float v;
    memcpy(&v, &(*bits), sizeof(v));
    return v;
}

esp_err_t setF32(nvs_handle_t h, const char *key, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return nvs_set_u32(h, key, bits);
}

std::optional<bool> optBool(nvs_handle_t h, const char *key) {
    int8_t v = 0;
    if (nvs_get_i8(h, key, &v) == ESP_OK) {
        return v != 0;
    }
    return std::nullopt;
}

std::optional<uint16_t> optU16(nvs_handle_t h, const char *key) {
    uint16_t v = 0;
    if (nvs_get_u16(h, key, &v) == ESP_OK) {
        return v;
    }
    return std::nullopt;
}

std::optional<int16_t> optI16(nvs_handle_t h, const char *key) {
    int16_t v = 0;
    if (nvs_get_i16(h, key, &v) == ESP_OK) {
        return v;
    }
    return std::nullopt;
}

std::optional<uint16_t> optDayOfYear(nvs_handle_t h, const char *key) {
    uint16_t v = 0;
    if (nvs_get_u16(h, key, &v) == ESP_OK) {
        return v;
    }
    return std::nullopt;
}

// Succeeds only when the stored string fits dst (including NUL).
bool optStr(nvs_handle_t h, const char *key, char *dst, size_t dstSize) {
    if (dst == nullptr || dstSize == 0) {
        return false;
    }
    size_t len = dstSize;
    if (nvs_get_str(h, key, dst, &len) != ESP_OK) {
        return false;
    }
    dst[dstSize - 1] = '\0';
    return true;
}

bool floatInRange(float v, float minV, float maxV) {
    return isfinite(v) && v >= minV && v <= maxV;
}

bool copyStr(const char *src, char *dst, size_t dstSize) {
    if (src == nullptr || dst == nullptr || dstSize == 0) {
        return false;
    }
    const size_t len = strlen(src);
    if (len + 1 > dstSize) {
        return false;
    }
    memcpy(dst, src, len + 1);
    return true;
}

// Fills below are written as NVS_TRY(expr): run the call, and return its
// error code immediately if it failed (ESP_OK on success).
#define NVS_TRY(expr) do { esp_err_t rv_ = (expr); if (rv_ != ESP_OK) return rv_; } while (0)

// Opens a namespace for writing, applies fill(), and commits.
// Transient NVS/flash glitches (e.g. 0x1021 invalid-state while the WiFi
// driver is active) are retried once. Every failing step is logged with the
// namespace, stage and NVS error code so the root cause is visible on the
// serial monitor (and in the web debug log via Logger).
bool writeNamespace(const char *ns,
                    esp_err_t (*fill)(nvs_handle_t, const ConfigStore *),
                    const ConfigStore *self) {
    for (int attempt = 1; attempt <= 2; attempt++) {
        nvs_handle_t h = 0;
        esp_err_t err = nvs_open(ns, NVS_READWRITE, &h);
        const char *stage = "open";
        if (err == ESP_OK) {
            err = fill(h, self);
            stage = "fill";
            if (err == ESP_OK) {
                err = nvs_commit(h);
                stage = "commit";
            }
            nvs_close(h);
        }
        if (err == ESP_OK) {
            return true;
        }
        Serial.printf("configstore: NVS '%s' %s failed: 0x%x (attempt %d)\n",
                      ns, stage, (unsigned)err, attempt);
        Logger::error("nvs write failed");
    }
    return false;
}

bool isWatchdogResetCause(ResetCause cause) {
    return cause == ResetCause::WDT_CPU ||
           cause == ResetCause::WDT_BROWNOUT ||
           cause == ResetCause::WDT_SUPER ||
           cause == ResetCause::WDT_TASK;
}

} // namespace

void ConfigStore::applyConfigDefaults(AppConfig &cfg) const {
    cfg.motorSpeedHz = constants::DEFAULT_MOTOR_SPEED_HZ;

    cfg.pulsesPerLiter        = constants::DEFAULT_FLOW_PULSES_PER_LITER;
    cfg.pumpMlPerStep         = constants::DEFAULT_PUMP_ML_PER_STEP;
    cfg.doseFactorMlPerLiter  = constants::DEFAULT_DOSE_FACTOR_ML_PER_L;

    for (uint8_t i = 0; i < constants::DEFAULT_SCHEDULE_COUNT; i++) {
        cfg.schedule[i].hour            = constants::DEFAULT_SCHEDULE_HOURS[i];
        cfg.schedule[i].minute          = constants::DEFAULT_SCHEDULE_MINUTES[i];
        cfg.schedule[i].enabled         = constants::DEFAULT_SCHEDULE_ENABLED[i];
        cfg.schedule[i].lastDayOfYear   = 0;
    }

    cfg.mqttEnabled = false;
    snprintf(cfg.mqttHost, sizeof(cfg.mqttHost), "%s", constants::DEFAULT_MQTT_HOST);
    cfg.mqttPort = constants::DEFAULT_MQTT_PORT;
    snprintf(cfg.mqttUser, sizeof(cfg.mqttUser), "%s", constants::DEFAULT_MQTT_USER);
    snprintf(cfg.mqttPass, sizeof(cfg.mqttPass), "%s", constants::DEFAULT_MQTT_PASS);
    snprintf(cfg.mqttBaseTopic, sizeof(cfg.mqttBaseTopic), "%s", constants::DEFAULT_BASE_TOPIC);

    snprintf(cfg.wifiSsid, sizeof(cfg.wifiSsid), "%s", constants::DEFAULT_WIFI_SSID);
    snprintf(cfg.wifiPass, sizeof(cfg.wifiPass), "%s", constants::DEFAULT_WIFI_PASS);

    snprintf(cfg.webUser, sizeof(cfg.webUser), "%s", constants::DEFAULT_WEB_USER);
    snprintf(cfg.webPass, sizeof(cfg.webPass), "%s", constants::DEFAULT_WEB_PASS);

    cfg.ntpEnabled = true;
    snprintf(cfg.ntpServer, sizeof(cfg.ntpServer), "%s", constants::DEFAULT_NTP_SERVER);
    cfg.timezoneOffsetMinutes = constants::DEFAULT_TIMEZONE_OFFSET_MINUTES;

    cfg.statusPublishIntervalSec = constants::DEFAULT_STATUS_PUBLISH_INTERVAL_SEC;
    cfg.statsPublishIntervalSec  = constants::DEFAULT_STATS_PUBLISH_INTERVAL_SEC;
}

bool ConfigStore::readConfigNs() {
    nvs_handle_t h = 0;
    if (nvs_open(constants::NVS_NS_CONFIG, NVS_READONLY, &h) != ESP_OK) {
        return false; // namespace absent: first boot, defaults already applied
    }

    auto repair = [this]() { this->configRepaired_ = true; };
    auto takeU32 = [&](const char *key,
                       uint32_t &dst,
                       uint32_t minV,
                       uint32_t maxV) {
        std::optional<uint32_t> v = optU32(h, key);
        if (v && *v >= minV && *v <= maxV) {
            dst = *v;
            return true;
        }
        return false;
    };
    auto takeF32 = [&](const char *key,
                       float &dst,
                       float minV,
                       float maxV) {
        std::optional<float> v = optF32(h, key);
        if (v && floatInRange(*v, minV, maxV)) {
            dst = *v;
            return true;
        }
        return false;
    };

    char key[24];
    bool found = false;

    if (takeU32("motorSpeedHz", appConfig_.motorSpeedHz,
                constants::MIN_MOTOR_SPEED_HZ, constants::MAX_MOTOR_SPEED_HZ)) {
        found = true;
    } else {
        repair();
    }

    if (takeF32("pulsesPerLiter", appConfig_.pulsesPerLiter,
                constants::MIN_FLOW_PULSES_PER_LITER, constants::MAX_FLOW_PULSES_PER_LITER)) {
        found = true;
    } else {
        repair();
    }

    if (takeF32("pumpMlPerStep", appConfig_.pumpMlPerStep,
                constants::MIN_PUMP_ML_PER_STEP, constants::MAX_PUMP_ML_PER_STEP)) {
        found = true;
    } else {
        repair();
    }

    if (takeF32("doseMlPerL", appConfig_.doseFactorMlPerLiter,
                constants::MIN_DOSE_FACTOR_ML_PER_L, constants::MAX_DOSE_FACTOR_ML_PER_L)) {
        found = true;
    } else {
        repair();
    }

    for (uint8_t i = 0; i < constants::DEFAULT_SCHEDULE_COUNT; i++) {
        bool slotOk = true;

        snprintf(key, sizeof(key), "sched%u_hour", i);
        std::optional<uint32_t> hour = optU32(h, key);
        if (hour && *hour <= 23) {
            appConfig_.schedule[i].hour = static_cast<uint8_t>(*hour);
        } else {
            slotOk = false;
        }

        snprintf(key, sizeof(key), "sched%u_minute", i);
        std::optional<uint32_t> minute = optU32(h, key);
        if (minute && *minute <= 59) {
            appConfig_.schedule[i].minute = static_cast<uint8_t>(*minute);
        } else {
            slotOk = false;
        }

        snprintf(key, sizeof(key), "sched%u_enabled", i);
        std::optional<bool> en = optBool(h, key);
        if (en) {
            appConfig_.schedule[i].enabled = *en;
        }

        snprintf(key, sizeof(key), "sched%u_lastDay", i);
        std::optional<uint16_t> day = optDayOfYear(h, key);
        if (day) {
            appConfig_.schedule[i].lastDayOfYear = *day;
        }

        found = true;
        if (!slotOk) {
            repair();
        }
    }

    std::optional<bool> b = optBool(h, "mqttEnabled");
    if (b) {
        appConfig_.mqttEnabled = *b;
        found = true;
    }

    if (optStr(h, "mqttHost", appConfig_.mqttHost, sizeof(appConfig_.mqttHost))) {
        found = true;
    } else {
        repair();
    }

    std::optional<uint16_t> port = optU16(h, "mqttPort");
    if (port && *port != 0) {
        appConfig_.mqttPort = *port;
        found = true;
    } else {
        repair();
    }

    if (optStr(h, "mqttUser", appConfig_.mqttUser, sizeof(appConfig_.mqttUser))) {
        found = true;
    } else {
        repair();
    }

    if (optStr(h, "mqttPass", appConfig_.mqttPass, sizeof(appConfig_.mqttPass))) {
        found = true;
    } else {
        repair();
    }

    if (!optStr(h, "mqttBaseTopic", appConfig_.mqttBaseTopic, sizeof(appConfig_.mqttBaseTopic))) {
        repair();
    }

    if (optStr(h, "wifiSsid", appConfig_.wifiSsid, sizeof(appConfig_.wifiSsid))) {
        found = true;
    } else {
        repair();
    }

    if (optStr(h, "wifiPass", appConfig_.wifiPass, sizeof(appConfig_.wifiPass))) {
        found = true;
    } else {
        repair();
    }

    if (optStr(h, "webUser", appConfig_.webUser, sizeof(appConfig_.webUser))) {
        found = true;
    } else {
        repair();
    }

    if (optStr(h, "webPass", appConfig_.webPass, sizeof(appConfig_.webPass))) {
        found = true;
    } else {
        repair();
    }

    b = optBool(h, "ntpEnabled");
    if (b) {
        appConfig_.ntpEnabled = *b;
        found = true;
    }

    if (!optStr(h, "ntpServer", appConfig_.ntpServer, sizeof(appConfig_.ntpServer))) {
        repair();
    }

    std::optional<int16_t> tz = optI16(h, "tzOffsetMin");
    if (tz) {
        appConfig_.timezoneOffsetMinutes = *tz;
        found = true;
    } else {
        repair();
    }

    if (takeU32("statusPubSec", appConfig_.statusPublishIntervalSec,
                constants::MIN_PUBLISH_INTERVAL_SEC, constants::MAX_PUBLISH_INTERVAL_SEC)) {
        found = true;
    } else {
        repair();
    }

    if (takeU32("statsPubSec", appConfig_.statsPublishIntervalSec,
                constants::MIN_PUBLISH_INTERVAL_SEC, constants::MAX_PUBLISH_INTERVAL_SEC)) {
        found = true;
    } else {
        repair();
    }

    nvs_close(h);
    return found;
}

bool ConfigStore::readStatsNs() {
    nvs_handle_t h = 0;
    if (nvs_open(constants::NVS_NS_STATS, NVS_READONLY, &h) != ESP_OK) {
        return false; // first boot: statistics start at zero
    }

    bool found = false;
    stats_ = StatsData{};

    std::optional<uint32_t> u = optU32(h, "totalDoses");
    if (u) {
        stats_.totalDoses = *u;
        found = true;
    }

    std::optional<float> f = optF32(h, "totWaterL");
    if (f && *f >= 0.0f && isfinite(*f)) {
        stats_.totalWaterLiters = *f;
        found = true;
    }

    f = optF32(h, "totAcidMl");
    if (f && *f >= 0.0f && isfinite(*f)) {
        stats_.totalAcidMilliliters = *f;
        found = true;
    }

    f = optF32(h, "lastDoseWL");
    if (f && *f >= 0.0f && isfinite(*f)) {
        stats_.lastDoseWaterLiters = *f;
        found = true;
    }

    f = optF32(h, "lastDoseAM");
    if (f && *f >= 0.0f && isfinite(*f)) {
        stats_.lastDoseAcidMilliliters = *f;
        found = true;
    }

    u = optU32(h, "lastDoseBootMs");
    if (u) {
        stats_.lastDoseBootMs = *u;
        found = true;
    }

    u = optU32(h, "lastDoseEpoch");
    if (u) {
        stats_.lastDoseEpoch = *u;
        found = true;
    }

    nvs_close(h);
    return found;
}

bool ConfigStore::load() {
    applyConfigDefaults(appConfig_);
    stats_ = StatsData{};
    configRepaired_ = false;
    configDirty_ = false;
    statsDirty_ = false;

    const bool configFound = readConfigNs();
    const bool statsFound = readStatsNs();

    // Boot bookkeeping in the doserboot namespace.
    bool bootFound = false;
    uint32_t bootCount = 0;
    uint32_t wdtCount = 0;
    uint32_t wdtStreak = 0;
    {
        nvs_handle_t h = 0;
        if (nvs_open(constants::NVS_NS_BOOT, NVS_READONLY, &h) == ESP_OK) {
            bootFound = true;
            std::optional<uint32_t> bc = optU32(h, "bootCount");
            if (bc) {
                bootCount = *bc;
            }
            std::optional<uint32_t> wc = optU32(h, "wdtResets");
            if (wc) {
                wdtCount = *wc;
            }
            std::optional<uint32_t> sc = optU32(h, "wdtStreak");
            if (sc) {
                wdtStreak = *sc;
            }
            nvs_close(h);
        }
    }

    const ResetCause cause = Logger::getResetCause();
    if (isWatchdogResetCause(cause)) {
        wdtCount++;
        // Consecutive reset loop (Step 10 post-mortem): cleared on any
        // non-WDT reset, bounded so a wrapped value restarts the streak.
        if (wdtStreak < 0xFFFFFFFFu) {
            wdtStreak++;
        }
    } else {
        wdtStreak = 0;
    }
    bootCount++;

    bootInfo_ = BootInfo{};
    bootInfo_.bootCount = bootCount;
    bootInfo_.watchdogResetCount = wdtCount;
    bootInfo_.watchdogStreak = wdtStreak;
    bootInfo_.lastResetCause = static_cast<uint32_t>(cause);
    bootInfo_.lastResetEpoch = 0;   // system clock not valid yet at boot
    bootInfo_.lastResetBootMs = 0;  // reset occurred before this boot's millis()

    // Persist boot counters and seed any missing namespaces.
    save();

    if (configRepaired_) {
        Logger::info("config repaired to defaults");
    }

    return configFound || statsFound || bootFound;
}

bool ConfigStore::save() {
    bool ok = true;

    if (!writeNamespace(constants::NVS_NS_CONFIG, &ConfigStore::fillConfigNs, this)) {
        ok = false;
    }

    if (!writeNamespace(constants::NVS_NS_STATS, &ConfigStore::fillStatsNs, this)) {
        ok = false;
    } else if (statsDirty_) {
        statsDirty_ = false;
        lastStatsFlushMs_ = static_cast<uint32_t>(millis());
    }

    if (!writeNamespace(constants::NVS_NS_BOOT, &ConfigStore::fillBootNs, this)) {
        ok = false;
    }

    return ok;
}

bool ConfigStore::flushStats() {
    if (writeNamespace(constants::NVS_NS_STATS, &ConfigStore::fillStatsNs, this)) {
        statsDirty_ = false;
        lastStatsFlushMs_ = static_cast<uint32_t>(millis());
        return true;
    }
    // Keep the RAM copy and the dirty flag so the flush is retried.
    statsDirty_ = true;
    Logger::error("stats NVS flush failed");
    return false;
}

bool ConfigStore::resetToDefaults() {
    applyConfigDefaults(appConfig_);
    stats_ = StatsData{};
    configRepaired_ = false;
    configDirty_ = true;
    statsDirty_ = true;
    return save();
}

void ConfigStore::update(uint32_t nowMs) {
    if (statsDirty_ &&
        nowMs - lastStatsFlushMs_ >= constants::STATS_FLUSH_MAX_INTERVAL_MS) {
        flushStats();
    }
    if (configDirty_) {
        // Rate-limit retries: on a permanent NVS fault (before the
        // 15-character key-name fix) this ran every loop iteration and
        // flooded the monitor; 2 s is plenty to catch a transient one.
        if (nowMs - lastConfigRetryMs_ >= constants::CONFIG_SAVE_RETRY_INTERVAL_MS) {
            lastConfigRetryMs_ = static_cast<uint32_t>(nowMs);
            if (writeNamespace(constants::NVS_NS_CONFIG, &ConfigStore::fillConfigNs, this)) {
                configDirty_ = false;
            }
        }
    }
}

bool ConfigStore::setMotorSpeedHz(uint32_t speedHz) {
    if (speedHz < constants::MIN_MOTOR_SPEED_HZ || speedHz > constants::MAX_MOTOR_SPEED_HZ) {
        return false;
    }
    appConfig_.motorSpeedHz = speedHz;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setFlowCalibration(float pulsesPerLiter) {
    if (!floatInRange(pulsesPerLiter, constants::MIN_FLOW_PULSES_PER_LITER,
                      constants::MAX_FLOW_PULSES_PER_LITER)) {
        return false;
    }
    appConfig_.pulsesPerLiter = pulsesPerLiter;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setPumpCalibration(float mlPerStep) {
    if (!floatInRange(mlPerStep, constants::MIN_PUMP_ML_PER_STEP,
                      constants::MAX_PUMP_ML_PER_STEP)) {
        return false;
    }
    appConfig_.pumpMlPerStep = mlPerStep;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setDoseFactor(float mlPerLiter) {
    if (!floatInRange(mlPerLiter, constants::MIN_DOSE_FACTOR_ML_PER_L,
                      constants::MAX_DOSE_FACTOR_ML_PER_L)) {
        return false;
    }
    appConfig_.doseFactorMlPerLiter = mlPerLiter;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setScheduleSlot(uint8_t slot, uint8_t hour, uint8_t minute, bool enabled) {
    if (slot >= constants::DEFAULT_SCHEDULE_COUNT || hour > 23 || minute > 59) {
        return false;
    }
    appConfig_.schedule[slot].hour = hour;
    appConfig_.schedule[slot].minute = minute;
    appConfig_.schedule[slot].enabled = enabled;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setSlotFiredDay(uint8_t slot, uint16_t dayOfYear) {
    if (slot >= constants::DEFAULT_SCHEDULE_COUNT) {
        return false;
    }
    appConfig_.schedule[slot].lastDayOfYear = dayOfYear;
    // Persist the day key so a slot cannot re-fire on the same day after a
    // reboot (one write per slot per day, negligible NVS wear).
    configDirty_ = true;
    return true;
}

bool ConfigStore::setWifiCredentials(const char *ssid, const char *password) {
    if (!copyStr(ssid, appConfig_.wifiSsid, sizeof(appConfig_.wifiSsid)) ||
        !copyStr(password, appConfig_.wifiPass, sizeof(appConfig_.wifiPass))) {
        return false;
    }
    configDirty_ = true;
    return true;
}

bool ConfigStore::setWebCredentials(const char *user, const char *password) {
    if (!copyStr(user, appConfig_.webUser, sizeof(appConfig_.webUser)) ||
        !copyStr(password, appConfig_.webPass, sizeof(appConfig_.webPass))) {
        return false;
    }
    configDirty_ = true;
    return true;
}

bool ConfigStore::setMqttSettings(bool enabled, const char *host, uint16_t port,
                                  const char *user, const char *password,
                                  const char *baseTopic) {
    char hostBuf[sizeof(appConfig_.mqttHost)] = {0};
    char userBuf[sizeof(appConfig_.mqttUser)] = {0};
    char passBuf[sizeof(appConfig_.mqttPass)] = {0};
    char topicBuf[sizeof(appConfig_.mqttBaseTopic)] = {0};

    if (port == 0 ||
        !copyStr(host, hostBuf, sizeof(hostBuf)) ||
        !copyStr(user, userBuf, sizeof(userBuf)) ||
        !copyStr(password, passBuf, sizeof(passBuf)) ||
        !copyStr(baseTopic, topicBuf, sizeof(topicBuf))) {
        return false;
    }

    appConfig_.mqttEnabled = enabled;
    memcpy(appConfig_.mqttHost, hostBuf, sizeof(hostBuf));
    appConfig_.mqttPort = port;
    memcpy(appConfig_.mqttUser, userBuf, sizeof(userBuf));
    memcpy(appConfig_.mqttPass, passBuf, sizeof(passBuf));
    memcpy(appConfig_.mqttBaseTopic, topicBuf, sizeof(topicBuf));
    configDirty_ = true;
    return true;
}

bool ConfigStore::setTimezoneOffsetMinutes(int16_t offsetMinutes) {
    appConfig_.timezoneOffsetMinutes = offsetMinutes;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setNtpEnabled(bool enabled) {
    appConfig_.ntpEnabled = enabled;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setNtpServer(const char *server) {
    if (!copyStr(server, appConfig_.ntpServer, sizeof(appConfig_.ntpServer))) {
        return false;
    }
    configDirty_ = true;
    return true;
}

bool ConfigStore::setStatusPublishIntervalSec(uint32_t intervalSec) {
    if (intervalSec < constants::MIN_PUBLISH_INTERVAL_SEC ||
        intervalSec > constants::MAX_PUBLISH_INTERVAL_SEC) {
        return false;
    }
    appConfig_.statusPublishIntervalSec = intervalSec;
    configDirty_ = true;
    return true;
}

bool ConfigStore::setStatsPublishIntervalSec(uint32_t intervalSec) {
    if (intervalSec < constants::MIN_PUBLISH_INTERVAL_SEC ||
        intervalSec > constants::MAX_PUBLISH_INTERVAL_SEC) {
        return false;
    }
    appConfig_.statsPublishIntervalSec = intervalSec;
    configDirty_ = true;
    return true;
}

bool ConfigStore::incrementTotalDoses() {
    stats_.totalDoses++;
    statsDirty_ = true;
    return true;
}

bool ConfigStore::addTotalWaterLiters(float liters) {
    if (!floatInRange(liters, 0.0f, 1e9f)) {
        return false;
    }
    stats_.totalWaterLiters += liters;
    statsDirty_ = true;
    return true;
}

bool ConfigStore::addTotalAcidMilliliters(float ml) {
    if (!floatInRange(ml, 0.0f, 1e9f)) {
        return false;
    }
    stats_.totalAcidMilliliters += ml;
    statsDirty_ = true;
    return true;
}

bool ConfigStore::setLastDose(float waterLiters, float acidMilliliters) {
    if (!floatInRange(waterLiters, 0.0f, 1e9f) || !floatInRange(acidMilliliters, 0.0f, 1e9f)) {
        return false;
    }
    stats_.lastDoseWaterLiters = waterLiters;
    stats_.lastDoseAcidMilliliters = acidMilliliters;
    stats_.lastDoseBootMs = static_cast<uint32_t>(millis());
    statsDirty_ = true;
    return true;
}

bool ConfigStore::saveBootInfo(uint32_t watchdogResetCount) {
    bootInfo_.watchdogResetCount = watchdogResetCount;
    return writeNamespace(constants::NVS_NS_BOOT, &ConfigStore::fillBootNs, this);
}

// Fill functions: serialize in-memory state to an open NVS handle.
// Returns the first NVS error (not bare false) so the failing call and
// error code are visible via writeNamespace()'s logging.
//
// NVS KEY-NAME LIMIT: this NVS implementation rejects key names longer
// than 15 characters (ESP_ERR_NVS_KEY_TOO_LONG, 0x1109) and *every* save
// then fails. Keep every key name in this file at 15 characters or fewer.

esp_err_t ConfigStore::fillConfigNs(nvs_handle_t h, const ConfigStore *self) {
    const AppConfig &c = self->appConfig_;

    NVS_TRY(nvs_set_u32(h, "motorSpeedHz", c.motorSpeedHz));
    NVS_TRY(setF32(h, "pulsesPerLiter", c.pulsesPerLiter));
    NVS_TRY(setF32(h, "pumpMlPerStep", c.pumpMlPerStep));
    NVS_TRY(setF32(h, "doseMlPerL", c.doseFactorMlPerLiter));

    char key[24];
    for (uint8_t i = 0; i < constants::DEFAULT_SCHEDULE_COUNT; i++) {
        snprintf(key, sizeof(key), "sched%u_hour", i);
        NVS_TRY(nvs_set_u32(h, key, c.schedule[i].hour));
        snprintf(key, sizeof(key), "sched%u_minute", i);
        NVS_TRY(nvs_set_u32(h, key, c.schedule[i].minute));
        snprintf(key, sizeof(key), "sched%u_enabled", i);
        NVS_TRY(nvs_set_i8(h, key, c.schedule[i].enabled ? 1 : 0));
        snprintf(key, sizeof(key), "sched%u_lastDay", i);
        NVS_TRY(nvs_set_u16(h, key, c.schedule[i].lastDayOfYear));
    }

    NVS_TRY(nvs_set_i8(h, "mqttEnabled", c.mqttEnabled ? 1 : 0));
    NVS_TRY(nvs_set_str(h, "mqttHost", c.mqttHost));
    NVS_TRY(nvs_set_u16(h, "mqttPort", c.mqttPort));
    NVS_TRY(nvs_set_str(h, "mqttUser", c.mqttUser));
    NVS_TRY(nvs_set_str(h, "mqttPass", c.mqttPass));
    NVS_TRY(nvs_set_str(h, "mqttBaseTopic", c.mqttBaseTopic));

    NVS_TRY(nvs_set_str(h, "wifiSsid", c.wifiSsid));
    NVS_TRY(nvs_set_str(h, "wifiPass", c.wifiPass));
    NVS_TRY(nvs_set_str(h, "webUser", c.webUser));
    NVS_TRY(nvs_set_str(h, "webPass", c.webPass));

    NVS_TRY(nvs_set_i8(h, "ntpEnabled", c.ntpEnabled ? 1 : 0));
    NVS_TRY(nvs_set_str(h, "ntpServer", c.ntpServer));
    NVS_TRY(nvs_set_i16(h, "tzOffsetMin", c.timezoneOffsetMinutes));

    NVS_TRY(nvs_set_u32(h, "statusPubSec", c.statusPublishIntervalSec));
    NVS_TRY(nvs_set_u32(h, "statsPubSec", c.statsPublishIntervalSec));

    return ESP_OK;
}

esp_err_t ConfigStore::fillStatsNs(nvs_handle_t h, const ConfigStore *self) {
    const StatsData &s = self->stats_;
    NVS_TRY(nvs_set_u32(h, "totalDoses", s.totalDoses));
    NVS_TRY(setF32(h, "totWaterL", s.totalWaterLiters));
    NVS_TRY(setF32(h, "totAcidMl", s.totalAcidMilliliters));
    NVS_TRY(setF32(h, "lastDoseWL", s.lastDoseWaterLiters));
    NVS_TRY(setF32(h, "lastDoseAM", s.lastDoseAcidMilliliters));
    NVS_TRY(nvs_set_u32(h, "lastDoseBootMs", s.lastDoseBootMs));
    NVS_TRY(nvs_set_u32(h, "lastDoseEpoch", s.lastDoseEpoch));
    return ESP_OK;
}

esp_err_t ConfigStore::fillBootNs(nvs_handle_t h, const ConfigStore *self) {
    const BootInfo &b = self->bootInfo_;
    NVS_TRY(nvs_set_u32(h, "bootCount", b.bootCount));
    NVS_TRY(nvs_set_u32(h, "wdtResets", b.watchdogResetCount));
    NVS_TRY(nvs_set_u32(h, "wdtStreak", b.watchdogStreak));
    NVS_TRY(nvs_set_u32(h, "lastResetCause", b.lastResetCause));
    return ESP_OK;
}