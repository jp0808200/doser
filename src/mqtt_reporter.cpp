#include "mqtt_reporter.h"

#include "app_core.h"
#include "config_store.h"
#include "constants.h"
#include "logger.h"
#include "names.h"

// Throttle for the publish-failure log so a dead broker does not flood the
// fixed 4096-byte log buffer (spec 3.3 buffer budget).
namespace {
constexpr uint32_t FAIL_LOG_THROTTLE_MS = 5000;
}

void MqttReporter::begin(AppCore &app) {
    app_ = &app;

    // Fixed 768-byte network buffer (spec 3.12). Library-internal.
    // (buffer_size is applied in buildConfig; the payload cap is PAYLOAD_MAX.)

    const AppConfig &cfg = app.configStore().getAppConfig();
    setTopicsFromConfig(cfg.mqttBaseTopic[0] != '\0'
                            ? cfg.mqttBaseTopic
                            : constants::DEFAULT_BASE_TOPIC);
    lastKeyValid_ = false;      // force initial configuration on first update

    const uint32_t nowMs = millis();
    uint32_t nextStatusMs = 0;
    intervalOf(cfg.statusPublishIntervalSec, nextStatusMs);
    nextStatusAtMs_ = nowMs + nextStatusMs;
    uint32_t nextStatsMs = 0;
    intervalOf(cfg.statsPublishIntervalSec, nextStatsMs);
    nextStatsAtMs_ = nowMs + nextStatsMs;

    Serial.printf("mqtt reporter        : topics %s / %s / %s, "
                  "status every %lus, stats every %lus, backoff %lus, QoS1 retain=false\n",
                  topicStatus_, topicAlert_, topicStats_,
                  (unsigned long)cfg.statusPublishIntervalSec,
                  (unsigned long)cfg.statsPublishIntervalSec,
                  (unsigned long)RECONNECT_BACKOFF_MS);
}

void MqttReporter::intervalOf(uint32_t seconds, uint32_t &outMs) const {
    // Deadlines are "no later than now + interval" and are re-anchored on
    // publish; the clamp keeps a misconfigured interval representable.
    const uint64_t ms = (uint64_t)(seconds ? seconds : 1) * 1000ULL;
    outMs = (ms > 0xFFFFFFFFULL) ? 0xFFFFFFFFU : (uint32_t)ms;
}

void MqttReporter::setTopicsFromConfig(const char *base) {
    snprintf(topicStatus_, sizeof(topicStatus_), "%s/status", base);
    snprintf(topicAlert_, sizeof(topicAlert_), "%s/alert", base);
    snprintf(topicStats_, sizeof(topicStats_), "%s/stats", base);
}

bool MqttReporter::cfgDiffers(const MqttKey &cand) const {
    if (!lastKeyValid_) {
        return true;
    }
    return (strncmp(lastKey_.host, cand.host, sizeof(lastKey_.host)) != 0
            || lastKey_.port != cand.port
            || strncmp(lastKey_.user, cand.user, sizeof(lastKey_.user)) != 0
            || strncmp(lastKey_.pass, cand.pass, sizeof(lastKey_.pass)) != 0
            || strncmp(lastKey_.base, cand.base, sizeof(lastKey_.base)) != 0);
}

// esp-mqtt event callback (library task context). Per spec 2.2 this context
// must only set flags and must not call logging/heap/JSON/NVS/Wi-Fi/MQTT API.
// Connection state is latched into `connected_`; the main loop (update())
// consumes that flag for logging and scheduling.
esp_err_t MqttReporter::onEvent(esp_mqtt_event_handle_t evt) {
    MqttReporter *self =
        static_cast<MqttReporter *>(evt->user_context);
    if (self == nullptr) {
        return ESP_OK;
    }
    switch (evt->event_id) {
        case MQTT_EVENT_CONNECTED:
            self->connected_ = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            self->connected_ = false;
            break;
        case MQTT_EVENT_ERROR:
            // esp-mqtt auto-reconnect handles failures; note it so update()
            // can treat the link as down.
            self->connected_ = false;
            break;
        default:
            break;
    }
    return ESP_OK;
}

void MqttReporter::buildConfig(const AppCore &app, const MqttKey &cfg,
                               esp_mqtt_client_config_t &out) {
    out = {};
    out.event_handle        = &MqttReporter::onEvent;
    out.user_context        = this;
    out.host                = cfg.host;
    out.port                = cfg.port;
    out.client_id           = app.deviceId();          // "ESP32C3_XXXX"
    // Auth only when BOTH username and password are present (Step 9 req. 4).
    out.username            = (cfg.user[0] != '\0' && cfg.pass[0] != '\0')
                                  ? cfg.user
                                  : nullptr;
    out.password            = (cfg.user[0] != '\0' && cfg.pass[0] != '\0')
                                  ? cfg.pass
                                  : nullptr;
    out.buffer_size         = (uint32_t)constants::MQTT_PUBSUB_BUFFER;       // 768
    out.reconnect_timeout_ms = (int)RECONNECT_BACKOFF_MS;                    // 5000 ms
    out.disable_auto_reconnect = false;                                      // library reconnect
    out.keepalive           = 30;                                            // MQTT keep-alive
    out.disable_keepalive   = false;
}

void MqttReporter::update(uint32_t nowMs) {
    if (app_ == nullptr) {
        return;
    }
    const AppConfig &cfg = app_->configStore().getAppConfig();

    const bool wantActive = cfg.mqttEnabled
                            && (cfg.mqttHost[0] != '\0')
                            && app_->wifi().isStaConnected();
    running_ = wantActive;

    // ---- not wanted: keep the client stopped (Step 9 req. 4) -------------
    // Hysteresis: a brief !wantActive (e.g. a transient STA status blip) must
    // not destroy a healthy link; require the condition to persist ~1 s.
    // (A live esp-mqtt client keeps its own reconnect logic meanwhile.)
    if (!wantActive) {
        if (wantActiveDownSinceMs_ == 0) {
            wantActiveDownSinceMs_ = nowMs;
        } else if ((uint32_t)(nowMs - wantActiveDownSinceMs_) >= 1000) {
            if (started_) {
                disconnectNow("disabled");
            }
        }
        return;
    }
    wantActiveDownSinceMs_ = 0;

    // ---- configuration snapshot (for change detection) -------------------
    MqttKey cand;
    strncpy(cand.host, cfg.mqttHost, sizeof(cand.host) - 1);
    cand.host[sizeof(cand.host) - 1] = '\0';
    cand.port = (cfg.mqttPort != 0) ? cfg.mqttPort
                                    : constants::DEFAULT_MQTT_PORT;
    memcpy(cand.user, cfg.mqttUser, sizeof(cand.user));
    memcpy(cand.pass, cfg.mqttPass, sizeof(cand.pass));
    // Effective base topic (same fallback rule as begin()) so that a
    // base-topic-only change also triggers a client reconfigure.
    strncpy(cand.base,
            cfg.mqttBaseTopic[0] != '\0'
                ? cfg.mqttBaseTopic
                : constants::DEFAULT_BASE_TOPIC,
            sizeof(cand.base) - 1);

    // ---- configuration changed mid-flight: tear down so the start branch
    // ---- below rebuilds the client with the new credentials -------------
    if (started_ && cfgDiffers(cand)) {
        Serial.println("mqtt: configuration changed, restarting client");
        if (mcli_ != nullptr) {
            esp_mqtt_client_stop(mcli_);
            mcli_ = nullptr;
        }
        started_ = false;
        connected_ = false;    // avoid a stale link-up flag during restart
        lastConnShown_ = false;
    }

    // ---- (re)start the client when it is not running yet -----------------
    if (!started_) {
        // Keep a persistent copy of the host/credentials in the member so
        // the strings outlive the update() frame (esp-mqtt stores them).
        lastKey_ = cand;
        lastKeyValid_ = true;
        // Re-derive the publish topics for this client instance (begin()
        // does the same at boot; a saved base-topic change lands here
        // after the tear-down above).
        setTopicsFromConfig(lastKey_.base);
        esp_mqtt_client_config_t out;
        buildConfig(*app_, lastKey_, out);
        if (mcli_ != nullptr) {
            // A previous client instance is still alive: drop it and make a
            // fresh one with the current credentials.
            esp_mqtt_client_stop(mcli_);
            mcli_ = nullptr;
        }
        mcli_ = esp_mqtt_client_init(&out);
        if (mcli_ == nullptr) {
            logFailure(nowMs, "esp_mqtt init failed");
            return;
        }
        const esp_err_t err = esp_mqtt_client_start(mcli_);
        if (err != ESP_OK) {
            logFailure(nowMs, "esp_mqtt start failed");
            esp_mqtt_client_stop(mcli_);
            mcli_ = nullptr;
            return;
        }
        started_ = true;
        Serial.printf("mqtt: client started, broker %s:%u (id %s)\n",
                      lastKey_.host, (unsigned)lastKey_.port, app_->deviceId());
    }

    // ---- connection state transition logging -----------------------------
    const bool nowConn = connected_;
    if (nowConn && lastConnShown_ == false) {
        lastConnShown_ = true;
        Serial.println("mqtt: connected");
        Logger::event(EVENT_MQTT_CONNECTED, "broker link up");
        // Reset the periodic anchors so the first publish is immediate.
        nextStatusAtMs_ = nowMs;
        nextStatsAtMs_ = nowMs;
        statusDue_ = true;
    } else if (!nowConn && lastConnShown_ == true) {
        lastConnShown_ = false;
        Serial.println("mqtt: disconnected");
        Logger::event(EVENT_MQTT_DISCONNECTED, "broker link lost");
    }
}

// Publish phase (spec 3.13 order item 10): called by AppCore AFTER the
// scheduler/dosing/calibration updates so the same-loop's events are
// included in the payload. One alert per pass (Step 9 requirement 7);
// status/stats only when their deadline passed (or was forced).
void MqttReporter::publishIfDue(uint32_t nowMs) {
    if (app_ == nullptr || !started_ || !connected_) {
        return;
    }
    const AppConfig &cfg = app_->configStore().getAppConfig();

    // One alert per loop (Step 9 requirement 7):
    drainOneAlert(nowMs);

    // Status publish deadline.
    // Wrap-safe deadline compare: (int32)(now - anchor) >= 0 means the anchor
    // (now + interval, set on the previous publish) has been reached.
    {
        uint32_t statusIntervalMs = 0;
        intervalOf(cfg.statusPublishIntervalSec, statusIntervalMs);
        const bool forced = statusDue_;
        if (forced || (int32_t)(nowMs - nextStatusAtMs_) >= 0) {
            statusDue_ = false;
            nextStatusAtMs_ = nowMs + statusIntervalMs;
            publishStatusNow(nowMs);
        }
    }

    // Stats publish deadline (also forced after each successful dose).
    {
        uint32_t statsIntervalMs = 0;
        intervalOf(cfg.statsPublishIntervalSec, statsIntervalMs);
        if (statsPublishForced_ || (int32_t)(nowMs - nextStatsAtMs_) >= 0) {
            statsPublishForced_ = false;
            nextStatsAtMs_ = nowMs + statsIntervalMs;
            publishStatsNow(nowMs);
        }
    }
}

bool MqttReporter::drainAlertNow(uint32_t nowMs) {
    if (app_ == nullptr || !started_ || !connected_) {
        return false;
    }
    return drainOneAlert(nowMs);
}

// ---------------------------------------------------------------------------
// Fixed-buffer payload builders (768 B, spec 3.12). All values are read from
// the AppCore facades; a truncation is reported as a failed publish so the
// caller state (alert queue) is left untouched.
namespace {
// Escapes backslashes and quotes; out must fit source (<=63 B) + escapes.
void jsonEscapeFixed(const char *in, char *out, size_t cap) {
    size_t o = 0;
    for (size_t i = 0; in[i] != '\0' && o + 3 < cap; i++) {
        const char c = in[i];
        if (c == '"' || c == '\\') {
            out[o++] = '\\';
            out[o++] = c;
        } else if (c == '\n') {
            out[o++] = '\\'; out[o++] = 'n';
        } else if (c == '\r') {
            out[o++] = '\\'; out[o++] = 'r';
        } else if (c == '\t') {
            out[o++] = '\\'; out[o++] = 't';
        } else {
            out[o++] = c;
        }
    }
    out[o] = '\0';
}
} // namespace

void MqttReporter::logFailure(uint32_t nowMs, const char *why) {
    failures_++;
    if ((uint32_t)(nowMs - lastFailLogMs_) >= FAIL_LOG_THROTTLE_MS) {
        lastFailLogMs_ = nowMs;
        Serial.printf("mqtt: %s (retry in %lus)\n", why,
                      (unsigned long)RECONNECT_BACKOFF_MS);
        Logger::event(EVENT_MQTT_PUBLISH_FAILED, why);
    }
}

// Shared publish: QoS 1, retain false (spec 3.12 / Step 9 req. 6).
bool MqttReporter::publish(const char *topic, const char *payload, size_t len,
                           uint32_t nowMs) {
    if (mcli_ == nullptr || !connected_) {
        logFailure(nowMs, "not connected");
        return false;
    }
    const int msgId = esp_mqtt_client_publish(
        mcli_, topic, payload, (int)len,
        /*qos*/ 1,
        /*retain*/ 0);
    if (msgId < 0) {
        logFailure(nowMs, topic);
        return false;
    }
    return true;
}

bool MqttReporter::publishStatusNow(uint32_t nowMs) {
    if (app_ == nullptr) {
        return false;
    }
    AppCore &app = *app_;
    const AppConfig &cfg = app.configStore().getAppConfig();
    const AppCore::FlowStatus &fs = app.getFlowStatus();
    const StatsData &st = app.configStore().getStatsData();
    char buf[PAYLOAD_MAX];   // fixed 768-byte payload (spec 3.12)
    const int r = snprintf(buf, sizeof(buf),
        "{\"device_id\":\"%s\","
        "\"state\":\"%s\","
        "\"state_id\":%u,"
        "\"calibration_state\":%u,"
        "\"time_known\":%s,"
        "\"epoch\":%lu,"
        "\"motor_speed_hz\":%lu,"
        "\"line_present\":%s,"
        "\"tank_empty\":%s,"
        "\"driver_fault\":%s,"
        "\"fatal_fault\":%s,"
        "\"water_since_dose_ml\":%.1f,"
        "\"total_doses\":%lu,"
        "\"last_dose_ml\":%.1f,"
        "\"flow_calibration_ppl\":%.1f,"
        "\"pump_calibration_ml_per_step\":%.4f,"
        "\"dose_factor_ml_per_l\":%.3f,"
        "\"loop_max_ms\":%lu,"
        "\"loop_avg_ms\":%lu,"
        "\"wifi_mode\":\"%s\","
        "\"mqtt_connected\":true}",
        app.deviceId(),
        systemStateName(app.getState()),
        (unsigned)app.getState(),
        (unsigned)app.calibration().getSubState(),
        app.clock().isTimeKnown() ? "true" : "false",
        (unsigned long)app.clock().getEpoch(),
        (unsigned long)cfg.motorSpeedHz,
        app.safetyMonitor().isLinePresent() ? "true" : "false",
        app.safetyMonitor().isTankEmpty() ? "true" : "false",
        app.safetyMonitor().isDriverFault() ? "true" : "false",
        app.isFatalFaultActive() ? "true" : "false",
        (double)fs.waterMlSinceDose,
        (unsigned long)st.totalDoses,
        (double)st.lastDoseAcidMilliliters,
        (double)cfg.pulsesPerLiter,
        (double)cfg.pumpMlPerStep,
        (double)cfg.doseFactorMlPerLiter,
        (unsigned long)app.getLoopMaxMs(),
        (unsigned long)app.getLoopAvgMs(),
        app.wifi().modeName());
    if (r < 0 || (size_t)r >= sizeof(buf)) {
        logFailure(nowMs, "status payload too large");
        return false;
    }
    if (publish(topicStatus_, buf, (size_t)r, nowMs)) {
        statusPublished_++;
        return true;
    }
    return false;
}

bool MqttReporter::publishStatsNow(uint32_t nowMs) {
    if (app_ == nullptr) {
        return false;
    }
    AppCore &app = *app_;
    const AppConfig &cfg = app.configStore().getAppConfig();
    const StatsData &st = app.configStore().getStatsData();
    const BootInfo &bi = app.configStore().getBootInfo();
    char buf[PAYLOAD_MAX];
    const int r = snprintf(buf, sizeof(buf),
        "{\"total_doses\":%lu,"
        "\"total_water_ml\":%.0f,"
        "\"total_acid_ml\":%.1f,"
        "\"last_dose_water_ml\":%.0f,"
        "\"last_dose_acid_ml\":%.1f,"
        "\"motor_speed_hz\":%lu,"
        "\"boot_count\":%lu,"
        "\"watchdog_reset_count\":%lu}",
        (unsigned long)st.totalDoses,
        (double)(st.totalWaterLiters * 1000.0f),
        (double)st.totalAcidMilliliters,
        (double)(st.lastDoseWaterLiters * 1000.0f),
        (double)st.lastDoseAcidMilliliters,
        (unsigned long)cfg.motorSpeedHz,
        (unsigned long)bi.bootCount,
        (unsigned long)bi.watchdogResetCount);
    if (r < 0 || (size_t)r >= sizeof(buf)) {
        logFailure(nowMs, "stats payload too large");
        return false;
    }
    if (publish(topicStats_, buf, (size_t)r, nowMs)) {
        statsPublished_++;
        return true;
    }
    return false;
}

// Drains at most ONE queued alert per call (Step 9 requirement 7). The
// entry is removed only after a successful (queued) publish, so a failed
// publish leaves it in place for the next attempt without touching the
// rest of the bounded queue.
bool MqttReporter::drainOneAlert(uint32_t nowMs) {
    if (app_ == nullptr) {
        return false;
    }
    AlertQueue::Entry e;
    if (!app_->alerts().at(0, e)) {
        return false;
    }
    char msgEsc[128];  // source 63 B; worst case ~2x after escaping
    jsonEscapeFixed(e.message, msgEsc, sizeof(msgEsc));
    char buf[PAYLOAD_MAX];
    const int r = snprintf(buf, sizeof(buf),
        "{\"severity\":\"%s\","
        "\"code\":%u,"
        "\"message\":\"%s\","
        "\"state\":\"%s\","
        "\"ts\":%lu}",
        severityName(e.severity),
        (unsigned)e.code,
        msgEsc,
        systemStateName(app_->getState()),
        (unsigned long)e.epochOrBootMs);
    if (r < 0 || (size_t)r >= sizeof(buf)) {
        return false;
    }
    if (!publish(topicAlert_, buf, (size_t)r, nowMs)) {
        return false; // alert stays queued
    }
    // Publish accepted: remove only now.
    AlertQueue::Entry tmp;
    app_->alerts().pop(tmp);
    alertPublished_++;
    Serial.printf("mqtt: alert published sev=%s code=%u \"%.60s\"\n",
                  severityName(e.severity), (unsigned)e.code, e.message);
    return true;
}

void MqttReporter::enqueueAlert(AlertSeverity severity, FaultCode code,
                                const char *message) {
    if (app_ == nullptr) {
        return;
    }
    uint32_t ts;
    if (app_->clock().isTimeKnown()) {
        ts = app_->clock().getEpoch(); // epoch when time is known (spec)
    } else {
        ts = millis();                 // ...else boot milliseconds
    }
    app_->alerts().push(severity, code, message, ts);
}

void MqttReporter::disconnectNow(const char *reason) {
    if (started_ && mcli_ != nullptr) {
        esp_mqtt_client_stop(mcli_);
        mcli_ = nullptr;
    }
    started_ = false;
    connected_ = false;
    if (lastConnShown_) {
        lastConnShown_ = false;
        Serial.printf("mqtt: link closed (%s)\n", reason);
        Logger::event(EVENT_MQTT_DISCONNECTED, reason);
    }
}
