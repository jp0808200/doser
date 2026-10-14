// Core enums, log event IDs, and persistent data structures.
// Per acid-dosing-fw-specification.md sections 3.3 and 4.
#ifndef ACID_DOSE_TYPES_H
#define ACID_DOSE_TYPES_H

#include <stdint.h>

#ifdef __cplusplus

enum EventId : uint16_t {
    EVENT_BOOT = 1,
    EVENT_WIFI_CONNECTED = 2,
    EVENT_WIFI_DISCONNECTED = 3,
    EVENT_MQTT_CONNECTED = 4,
    EVENT_MQTT_DISCONNECTED = 5,
    EVENT_MQTT_PUBLISH_FAILED = 6,
    EVENT_DOSE_STARTED = 7,
    EVENT_DOSE_COMPLETED = 8,
    EVENT_DOSE_ABORTED = 9,
    EVENT_LINE_DETECTED = 10,
    EVENT_LINE_LOST = 11,
    EVENT_LINE_PURGE_COMPLETE = 12,
    EVENT_TANK_EMPTY = 13,
    EVENT_DRV_FAULT = 14,
    EVENT_FLOW_CAL_START = 15,
    EVENT_FLOW_CAL_SAVED = 16,
    EVENT_PUMP_CAL_START = 17,
    EVENT_PUMP_CAL_SAVED = 18,
    EVENT_CONFIG_SAVED = 19,
    EVENT_CONFIG_LOAD_FAILED = 20,
    EVENT_WATCHDOG_NEAR_TIMEOUT = 21,
    EVENT_SAFE_SHUTDOWN = 22,
    EVENT_DEEP_SLEEP_ENTERED = 23
};

enum class SystemState : uint8_t {
    POWER_ON = 0,
    IDLE = 1,
    COUNTING = 2,
    DOSE_READY = 3,
    DOSING = 4,
    POST_DOSE = 5,
    CALIBRATION = 6,
    ERROR = 7,
    SAFE_SHUTDOWN = 8
};

enum class FaultCode : uint16_t {
    NONE = 0,
    TANK_EMPTY = 1,
    DRIVER_FAULT = 2,
    LINE_NO_ACID = 3,
    LINE_NOT_EMPTY = 4,
    DOSE_TIMEOUT = 5,
    FLOW_NO_CAL = 6,
    PUMP_NO_CAL = 7,
    NVS_WRITE_ERROR = 8,
    WATCHDOG_RESET = 9,
    CAL_INVALID_INPUT = 10,
    MAX_DOSE_EXCEEDED = 11,
    UNKNOWN = 99
};

enum class AlertSeverity : uint8_t {
    INFO = 0,
    WARNING = 1,
    ERROR = 2
};

// Arduino.h defines EXTERNAL as a pin-mode macro; keep it out of the way
// only while this enum is declared, then restore it.
#ifdef EXTERNAL
#pragma push_macro("EXTERNAL")
#undef EXTERNAL
#define ACID_DOSE_TYPES_KEEP_EXTERNAL
#endif

enum class ResetCause : uint8_t {
    UNKNOWN = 0,
    POWER_ON = 1,
    EXTERNAL = 2,
    SOFT_RESET = 3,
    WDT_CPU = 4,
    WDT_BROWNOUT = 5,
    WDT_SUPER = 6,
    WDT_TASK = 7,
    BROWNOUT = 8
};

#ifdef ACID_DOSE_TYPES_KEEP_EXTERNAL
#pragma pop_macro("EXTERNAL")
#undef ACID_DOSE_TYPES_KEEP_EXTERNAL
#endif

struct ScheduleSlot {
    uint8_t hour;
    uint8_t minute;
    bool enabled;
    uint16_t lastDayOfYear;
};

struct AppConfig {
    uint32_t motorSpeedHz;

    float pulsesPerLiter;
    float pumpMlPerStep;
    float doseFactorMlPerLiter;

    ScheduleSlot schedule[4];

    bool mqttEnabled;
    char mqttHost[64];
    uint16_t mqttPort;
    char mqttUser[32];
    char mqttPass[64];
    char mqttBaseTopic[32];

    char wifiSsid[32];
    char wifiPass[64];

    char webUser[32];
    char webPass[64];

    bool ntpEnabled;
    char ntpServer[64];
    int16_t timezoneOffsetMinutes;

    uint32_t statusPublishIntervalSec;
    uint32_t statsPublishIntervalSec;
};

struct StatsData {
    uint32_t totalDoses;
    float totalWaterLiters;
    float totalAcidMilliliters;
    float lastDoseWaterLiters;
    float lastDoseAcidMilliliters;
    uint32_t lastDoseBootMs;
    uint32_t lastDoseEpoch;
};

struct BootInfo {
    uint32_t bootCount;
    uint32_t watchdogResetCount;
    uint32_t lastResetCause;
    uint32_t lastResetEpoch;
    uint32_t lastResetBootMs;
    // Step 10: consecutive watchdog resets with no clean boot in between
    // (cleared on any non-WDT reset). A tight WDT reset loop never survives
    // long enough to record its own death time, so the consecutive streak is
    // the on-NVS witness of the "3 within 10000 ms" reset storm (spec 3.13).
    uint32_t watchdogStreak;
};

#endif /* __cplusplus */

#endif /* ACID_DOSE_TYPES_H */
