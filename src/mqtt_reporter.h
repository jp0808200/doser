// MqttReporter: optional MQTT reporting of status, alerts and statistics.
//
// Per acid-dosing-fw-specification.md 3.12 and the Step 9 prompt:
//  - connects only when mqttEnabled AND Wi-Fi STA is connected;
//  - client id "ESP32C3_XXXX" (last 4 hex MAC characters = AppCore::deviceId);
//  - auth (username+password) only when both are non-empty;
//  - reconnect backoff 5000 ms (Step 9 requirement 4);
//  - QoS 1 on every publish, retain false (spec 3.12 QoS rules / Step 9 req. 6);
//  - topics <base>/status, <base>/alert, <base>/stats;
//  - status every statusPublishIntervalSec (default 30),
//    stats every statsPublishIntervalSec (default 300) AND after each
//    successful dose (AppCore calls requestStatsPublish() in POST_DOSE);
//  - alerts drained at most one per loop from the shared AlertQueue
//    (Step 9 req. 7); a failed publish leaves the alert queued, and the
//    alert queue is bounded (8, overwrite-oldest + drop counter) so a
//    long-disconnected broker never grows memory or blocks dosing;
//  - publish failure never stops dosing (Step 9 constraint);
//  - fixed 768-byte payload buffers, no heap in the publish path beyond the
//    library's own network buffers (spec 3.12);
//  - all reporting runs from the Arduino loopTask; the only other context
//    is the esp-mqtt client task, a library event context that the
//    specification allows (spec 2.2), and it only sets a connection flag.
//
// TRANSPORT NOTE (deviation vs. the Step 9 prompt wording):
// The prompt names "PubSubClient", but the ESP32 Arduino framework's
// knolleary/PubSubClient has no QoS argument on publish() (the framework
// Client::publish hard-codes QoS 0), which makes the specification's hard
// constraint "QoS must be 1" impossible with that library. The platform's
// built-in esp-mqtt client (mqtt_client.h, part of the official espressif32
// platform, no extra third-party dependency) is used instead: it supports
// per-message QoS and retain, an internal auto-reconnect backoff, and
// keeps the same library-task context model. The MqttReporter interface is
// kept equivalent to what the prompt describes (begin/update, QoS1,
// retain false, 768-byte buffers, 5 s backoff, one alert per loop).
#ifndef ACID_DOSE_MQTT_REPORTER_H
#define ACID_DOSE_MQTT_REPORTER_H

#include <Arduino.h>
#include "mqtt_client.h"   // esp-mqtt (espressif32 platform)

#include "types.h"
#include "alert_queue.h"

class AppCore;   // payloads are read through the AppCore facades

class MqttReporter {
public:
    // PubSubClient-equivalent network buffer / payload limit (768 bytes).
    static constexpr size_t PAYLOAD_MAX = 768;
    // Reconnect backoff (Step 9 requirement 4: 5000 ms).
    static constexpr uint32_t RECONNECT_BACKOFF_MS = 5000;

    // One-time start: stores the AppCore pointer, derives the topics and
    // the first status/stats deadlines, and prepares the client. Non-blocking.
    void begin(AppCore &app);

    // Main-loop hook for the CONNECTION phase (spec 3.13 order item 3, called
    // once per loop): start/stop on config changes, auto-reconnect (library
    // context) and connection-state logging. Non-blocking, no publishes.
    void update(uint32_t nowMs);

    // Main-loop hook for the PUBLISH phase (spec 3.13 order item 10, called
    // after the scheduler/dosing/calibration updates): drain at most one
    // queued alert and publish status/stats when due. Non-blocking.
    void publishIfDue(uint32_t nowMs);

    // Drain at most one queued alert right now (used by the safe-shutdown
    // path for the final alert). Returns true when an alert was published.
    bool drainAlertNow(uint32_t nowMs);

    // ---- state -------------------------------------------------------------
    bool isConnected() const { return connected_; }
    // enabled && STA up (i.e. the reporter is actively maintaining a link)
    bool isRunning() const { return running_; }
    const char *topicStatus() const { return topicStatus_; }
    const char *topicAlert() const { return topicAlert_; }
    const char *topicStats() const { return topicStats_; }

    // Cumulative accounting (HIL `mqtt` command + diagnostics).
    uint32_t statusPublishedCount() const { return statusPublished_; }
    uint32_t alertPublishedCount() const { return alertPublished_; }
    uint32_t statsPublishedCount() const { return statsPublished_; }
    uint32_t publishFailures() const { return failures_; }

    // ---- publish path (main-loop only) -------------------------------------
    // Publish status/stats immediately (test commands / forced events).
    // Returns false (and counts a failure) when not connected or the payload
    // would not fit the fixed 768-byte buffer. Dosing is unaffected.
    bool publishStatusNow(uint32_t nowMs);
    bool publishStatsNow(uint32_t nowMs);

    // Marks the next stats update as due: called by AppCore after every
    // successful dose ("stats after each dose", Step 9 req. 5) and usable
    // by test commands.
    void requestStatsPublish() { statsPublishForced_ = true; }

    // Queue an alert into the shared AlertQueue for draining on the next
    // connected update pass. Timestamps with the epoch when the clock is
    // known, otherwise boot millis (spec 3.12 alert payload ts rule).
    void enqueueAlert(AlertSeverity severity, FaultCode code,
                      const char *message);

private:
    // Captured MQTT configuration (for change-detection and reconfigure).
    struct MqttKey {
        char host[64] = {0};
        uint16_t port = 0;
        char user[32] = {0};
        char pass[64] = {0};
        char base[32] = {0};   // effective base topic (default applied)
    };

    static esp_err_t onEvent(esp_mqtt_event_handle_t evt);

    bool cfgDiffers(const MqttKey &cfg) const;
    void buildConfig(const AppCore &app, const MqttKey &cfg,
                     esp_mqtt_client_config_t &out);
    void setTopicsFromConfig(const char *base);
    void disconnectNow(const char *reason);
    void logFailure(uint32_t nowMs, const char *why);
    bool publish(const char *topic, const char *payload, size_t len,
                 uint32_t nowMs);
    bool drainOneAlert(uint32_t nowMs);
    void intervalOf(uint32_t seconds, uint32_t &outMs) const;

    esp_mqtt_client_handle_t mcli_ = nullptr;
    AppCore *app_ = nullptr;      // set in begin(); payloads read through it

    char topicStatus_[48] = "acid_doser/status";
    char topicAlert_[48]  = "acid_doser/alert";
    char topicStats_[48]  = "acid_doser/stats";

    bool running_ = false;        // enabled && STA connected (want active)
    bool started_ = false;        // esp-mqtt client started
    bool connected_ = false;      // last reported state (for logging)
    MqttKey lastKey_;
    bool lastKeyValid_ = false;
    bool lastConnShown_ = false;   // connection transition logging latch
    uint32_t wantActiveDownSinceMs_ = 0;  // teardown hysteresis anchor (0 = active)

    uint32_t nextStatusAtMs_ = 0;
    uint32_t nextStatsAtMs_  = 0;
    bool statusDue_ = false;      // publish status on the next connected pass
    bool statsPublishForced_ = false;

    uint32_t statusPublished_ = 0;
    uint32_t alertPublished_  = 0;
    uint32_t statsPublished_  = 0;
    uint32_t failures_ = 0;
    uint32_t lastFailLogMs_ = 0;
};

#endif /* ACID_DOSE_MQTT_REPORTER_H */
