// ConfigStore / NVS manager.
// Per acid-dosing-fw-specification.md 3.2 and the Step 1 prompt.
// All NVS access happens from the Arduino loopTask context (never ISR).
#ifndef ACID_DOSE_CONFIG_STORE_H
#define ACID_DOSE_CONFIG_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <esp_err.h>
#include <nvs_flash.h>

#include "types.h"

class ConfigStore {
public:
    // Loads config, stats and boot info from NVS, repairing invalid values
    // to defaults. Increments bootCount, increments watchdogResetCount when
    // the last reset cause was a watchdog reset, and persists the results.
    // Returns true when pre-existing NVS data was present.
    bool load();

    // Persists all dirty namespaces.
    bool save();

    // Restores all configuration and statistics to defaults and saves them.
    // Wired to POST /api/reset_defaults (auth-gated) + soft reboot (field
    // recovery against a soft-bricked config, AI review n8).
    bool resetToDefaults();

    const AppConfig &getAppConfig() const { return appConfig_; }
    const StatsData &getStatsData() const { return stats_; }
    const BootInfo &getBootInfo() const { return bootInfo_; }

    bool setMotorSpeedHz(uint32_t speedHz);
    bool setFlowCalibration(float pulsesPerLiter);
    bool setPumpCalibration(float mlPerStep);
    bool setDoseFactor(float mlPerLiter);
    bool setScheduleSlot(uint8_t slot, uint8_t hour, uint8_t minute, bool enabled);
    // Records that a slot fired on the given day-of-year (RAM + NVS dirty
    // flag so the day key survives a reboot and the slot cannot re-fire the
    // same day). Used by DoseScheduler after a slot has fired.
    bool setSlotFiredDay(uint8_t slot, uint16_t dayOfYear);

    bool setWifiCredentials(const char *ssid, const char *password);
    bool setWebCredentials(const char *user, const char *password);
    bool setMqttSettings(bool enabled, const char *host, uint16_t port,
                         const char *user, const char *password,
                         const char *baseTopic);

    bool setTimezoneOffsetMinutes(int16_t offsetMinutes);
    bool setNtpEnabled(bool enabled);
    bool setNtpServer(const char *server);
    bool setStatusPublishIntervalSec(uint32_t intervalSec);
    bool setStatsPublishIntervalSec(uint32_t intervalSec);

    bool incrementTotalDoses();
    bool addTotalWaterLiters(float liters);
    bool addTotalAcidMilliliters(float ml);
    bool setLastDose(float waterLiters, float acidMilliliters);

    // Boot bookkeeping.
    bool saveBootInfo(uint32_t watchdogResetCount);
    uint32_t getBootCounter() const { return bootInfo_.bootCount; }
    uint32_t getWatchdogResetCount() const { return bootInfo_.watchdogResetCount; }
    // Step 10: consecutive watchdog resets (reset-storm post-mortem).
    uint32_t getWatchdogStreak() const { return bootInfo_.watchdogStreak; }

    // Main-loop hook: flushes dirty statistics after the wear-reduction
    // window and persists pending config changes.
    void update(uint32_t nowMs);

    // Flushes statistics to NVS immediately (e.g. after a successful dose).
    // On write failure the RAM copy is retained and the dirty flag set.
    bool flushStats();

    // True when any invalid NVS value was replaced by its default during load.
    bool isConfigRepaired() const { return configRepaired_; }

private:
    void applyConfigDefaults(AppConfig &cfg) const;
    bool readConfigNs();
    bool readStatsNs();

    // Serialize state to an open NVS handle; ESP_OK on success, otherwise
    // the error code of the first failing nvs_set_* call.
    static esp_err_t fillConfigNs(nvs_handle_t h, const ConfigStore *self);
    static esp_err_t fillStatsNs(nvs_handle_t h, const ConfigStore *self);
    static esp_err_t fillBootNs(nvs_handle_t h, const ConfigStore *self);

    AppConfig appConfig_;
    StatsData stats_;
    BootInfo bootInfo_;

    bool configRepaired_ = false;
    bool configDirty_ = false;
    bool statsDirty_ = false;
    uint32_t lastStatsFlushMs_ = 0;
    uint32_t lastConfigRetryMs_ = 0;
};

#endif /* ACID_DOSE_CONFIG_STORE_H */
