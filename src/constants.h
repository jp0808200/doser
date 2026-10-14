// Stable NVS namespaces, default values, and limits.
// Per acid-dosing-fw-specification.md sections 3.2, 3.3, and the Step 1 prompt.
#ifndef ACID_DOSE_CONSTANTS_H
#define ACID_DOSE_CONSTANTS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
namespace constants {
#endif

// NVS namespaces (stable names).
constexpr char NVS_NS_CONFIG[] = "dosercfg";
constexpr char NVS_NS_STATS[]  = "doserstats";
constexpr char NVS_NS_BOOT[]   = "doserboot";

// Motor speed. Exactly one user-adjustable parameter (Hz).
constexpr uint32_t DEFAULT_MOTOR_SPEED_HZ = 800;
constexpr uint32_t MIN_MOTOR_SPEED_HZ     = 100;
constexpr uint32_t MAX_MOTOR_SPEED_HZ     = 800;

// Stepper ramp and pulse generation (specification 3.6 required constants).
// RAMP_START_HZ: every motor operation starts at this frequency and ramps
// linearly to the target speed over RAMP_MS. STEP_PULSE_US: STEP high time.
constexpr uint32_t RAMP_START_HZ          = 100;
constexpr uint32_t RAMP_MS                = 100;
constexpr uint32_t STEP_PULSE_US          = 2;
// Maximum prime/purge run times (ms); timeouts are evaluated in the main loop.
constexpr uint32_t MAX_FORWARD_PRIME_MS   = 30000;
constexpr uint32_t MAX_PURGE_MS           = 30000;

// Calibration defaults and validity ranges.
constexpr float DEFAULT_FLOW_PULSES_PER_LITER = 1380.0f;
constexpr float MIN_FLOW_PULSES_PER_LITER     = 10.0f;
constexpr float MAX_FLOW_PULSES_PER_LITER     = 100000.0f;
// Flow-meter pulse debounce: any two accepted pulses closer together
// than this are ignored as bounce/noise (specification 3.4).
constexpr uint32_t FLOW_MIN_PULSE_INTERVAL_US = 50;
// Maximum measured volume accepted by a flow calibration session (liters).
constexpr float MAX_FLOW_CAL_VOLUME_LITER     = 1000.0f;

// Dose clamping (specification 3.8 required constants): a dose caps at this
// acid volume and this step count.
constexpr float MAX_DOSE_ML                  = 200.0f;
constexpr uint32_t MAX_DOSE_STEPS            = 100000;

// Pump calibration (specification 3.9): the metered run always uses exactly
// this step count, and the resulting ml/step must land within this range
// to be accepted (0.001 .. 10.0 ml/step).
constexpr uint32_t PUMP_CAL_STEPS            = 16000;
constexpr float PUMP_CAL_MIN_ML_PER_STEP     = 0.001f;
constexpr float PUMP_CAL_MAX_ML_PER_STEP     = 10.0f;
// Flow calibration (specification 3.9): when no pulse has been seen for
// this long, the session transitions FLOW_ACTIVE -> FLOW_WAIT_VOLUME
// (prompting the volume input instead of waiting for more flow).
constexpr uint32_t FLOW_CAL_WAIT_SETTLE_MS   = 2500;

// Scheduler (specification 3.7): the once-per-second schedule check period.
constexpr uint32_t SCHED_CHECK_PERIOD_MS     = 1000;
constexpr float DEFAULT_PUMP_ML_PER_STEP      = 0.05f;
constexpr float MIN_PUMP_ML_PER_STEP          = 0.0001f;
constexpr float MAX_PUMP_ML_PER_STEP          = 100.0f;
constexpr float DEFAULT_DOSE_FACTOR_ML_PER_L  = 1.0f;
constexpr float MIN_DOSE_FACTOR_ML_PER_L      = 0.001f;
constexpr float MAX_DOSE_FACTOR_ML_PER_L      = 1000.0f;

// Credentials and endpoints.
constexpr char DEFAULT_MQTT_HOST[]     = "";
constexpr uint16_t DEFAULT_MQTT_PORT   = 1883;
constexpr char DEFAULT_MQTT_USER[]     = "";
constexpr char DEFAULT_MQTT_PASS[]     = "";
constexpr char DEFAULT_BASE_TOPIC[]    = "acid_doser";
constexpr char DEFAULT_WIFI_SSID[]     = "";
constexpr char DEFAULT_WIFI_PASS[]     = "";
constexpr char DEFAULT_WEB_USER[]      = "admin";
constexpr char DEFAULT_WEB_PASS[]      = "admin123";
constexpr char DEFAULT_NTP_SERVER[]    = "pool.ntp.org";
constexpr int16_t DEFAULT_TIMEZONE_OFFSET_MINUTES = 0;

// Publish intervals.
constexpr uint32_t DEFAULT_STATUS_PUBLISH_INTERVAL_SEC = 30;
constexpr uint32_t DEFAULT_STATS_PUBLISH_INTERVAL_SEC  = 300;
constexpr uint32_t MIN_PUBLISH_INTERVAL_SEC            = 1;
constexpr uint32_t MAX_PUBLISH_INTERVAL_SEC            = 86400;

// MQTT (specification 3.12 / Step 9): PubSubClient network buffer and
// fixed payload size (768 bytes), reconnect backoff (5000 ms).
constexpr uint32_t MQTT_PUBSUB_BUFFER      = 768;
constexpr uint32_t MQTT_RECONNECT_BACKOFF_MS = 5000;

// Schedule.
constexpr uint8_t DEFAULT_SCHEDULE_COUNT = 4;
constexpr uint8_t DEFAULT_SCHEDULE_HOURS[]   = {0, 0, 0, 0};
constexpr uint8_t DEFAULT_SCHEDULE_MINUTES[] = {0, 0, 0, 0};
constexpr bool DEFAULT_SCHEDULE_ENABLED[]    = {false, false, false, false};

// Statistics NVS flush wear reduction (ms of dirtiness before auto-flush).
constexpr uint32_t STATS_FLUSH_MAX_INTERVAL_MS = 300000;
// Minimum interval between config-save retries after a failed NVS commit,
// so a persistent NVS fault cannot spam the serial monitor every loop
// iteration (or hammer flash) instead of failing quietly and visibly once.
constexpr uint32_t CONFIG_SAVE_RETRY_INTERVAL_MS = 2000;

// Logger circular buffer.
constexpr size_t LOG_ENTRY_SIZE_BYTES = 64;
constexpr uint16_t LOG_MAX_ENTRIES    = 64; // 64 * 64 = 4096 bytes total
constexpr uint16_t LOG_MESSAGE_MAX_CHARS = 55; // room for NUL within 64-byte entry

// Status LED blink period (ms) for the Step 1 boot behavior.
constexpr uint32_t STATUS_LED_BLINK_PERIOD_MS = 1000;

// Wi-Fi (specification 3.10) and web (specification 3.11, Step 8).
constexpr uint8_t  WIFI_RECONNECT_MAX_STEP = 4;   // 5-entry backoff table
constexpr uint32_t WIFI_REBOOT_DELAY_MS    = 2000; // flush the HTTP response first
// Field-recovery: after this much persistent STA failure the setup AP is
// started alongside (STA retries continue) so the web UI stays reachable
// and a wrong password cannot permanently lock the unit out.
constexpr uint32_t WIFI_AP_FALLBACK_MS    = 120000;
// WiFi.status() can transiently report not-connected while the radio is in
// modem-sleep (WiFi.setSleep(true)) even though the STA link is healthy.
// As long as no real disconnect event has arrived, hold the previous
// "connected" state for up to this long so consumers (MQTT client
// lifecycle, web status) do not tear down a healthy link on every pass.
constexpr uint32_t WIFI_STATUS_FLAP_HOLD_MS = 1000;
constexpr uint32_t REBOOT_DELAY_MS         = 1500;

constexpr size_t  WEB_MAX_RESPONSE_BYTES = 2048; // spec 3.11 max response
constexpr size_t  WEB_MAX_BODY_BYTES     = 1024; // spec 3.11 max request body
constexpr uint8_t WEB_BODY_SLOTS         = 4;    // in-flight POST body holders
constexpr uint8_t WEB_LOG_MAX_ENTRIES    = 32;   // /debug/log page slice

// Config update queue (Step 8 / spec 4.8 web->main-loop mechanism).
constexpr size_t CONFIG_UPDATE_QUEUE_SIZE = 8;

// Alert queue (specification 3.12). Bounded ring; overwriting oldest on full.
constexpr size_t ALERT_QUEUE_SIZE = 8;

// Watchdog (Step 4 requirement 9) and main-loop cycle-time warning (spec 3.13).
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 5000;
constexpr uint32_t LOOP_CYCLE_WARN_MS = 500;

// Periodic main-loop heartbeat log (ms).
constexpr uint32_t LOGGER_HEARTBEAT_PERIOD_MS = 10000;

// Step 10 / spec 3.13 (safe shutdown + reset post-mortem).
constexpr uint32_t SAFE_SHUTDOWN_HOLD_MS = 2000;   // fatal-fault hold before deep sleep
constexpr uint32_t WDT_STORM_MAX_COUNT  = 3;      // >3 consecutive WDT resets -> safe shutdown
constexpr uint32_t LOOP_AVG_WINDOW_MS   = 1000;   // rolling loop-time average window

#ifdef __cplusplus
} // namespace constants
#endif

#endif /* ACID_DOSE_CONSTANTS_H */
