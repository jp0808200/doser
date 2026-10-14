#include "wifi_controller.h"

#include <DNSServer.h>
#include <esp_wifi.h>

#include <string.h>

#include "constants.h"
#include "logger.h"

namespace {

DNSServer &dns() {
    static DNSServer server;
    return server;
}

// Single firmware-image instance of the controller, used only so the core
// Wi-Fi event callback (context: Wi-Fi stack library task) can raise the
// flag. Per spec 3.10 the callback only sets flags - no logging, no Wi-Fi
// calls, no heap; AppCore consumes the flags from the main loop.
WifiController *s_self = nullptr;

void wifiEventCallback(arduino_event_id_t event) {
    if (s_self == nullptr) {
        return;
    }
    s_self->noteEvent(event);
}

} // namespace

void WifiController::noteEvent(arduino_event_id_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            evStaConnected_ = true;
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            evStaDisconnected_ = true;
            break;
        default:
            break;
    }
}

void WifiController::begin(const AppConfig &cfg) {
    WiFi.persistent(false);
    s_self = this;
    // One global Wi-Fi event hook (context: Wi-Fi library task); only
    // raises flags (spec 3.10 constraint).
    WiFi.onEvent(wifiEventCallback);
    // Raw handler to latch the STA disconnect reason (library context,
    // stores state only).
    esp_event_handler_instance_register(
        WIFI_EVENT, (int32_t)WIFI_EVENT_STA_DISCONNECTED,
        [](void *arg, esp_event_base_t base, int32_t id, void *data) {
            (void)base;
            (void)id;
            if (data != nullptr) {
                static_cast<WifiController *>(arg)->noteDisconnectReason(
                    static_cast<wifi_event_sta_disconnected_t *>(data)->reason);
            }
        },
        this, nullptr);
    const char *ssid = (cfg.wifiSsid[0] != '\0') ? cfg.wifiSsid : nullptr;
    if (ssid != nullptr) {
        startSta(cfg.wifiSsid, cfg.wifiPass);
    } else {
        startAp(false);
    }
}

void WifiController::update(uint32_t nowMs) {
    if (apMode_) {
        // Captive portal: service DNS queries (redirect to the AP IP).
        dns().processNextRequest();
        if (!staActive_) {
            return; // pure setup AP (no saved SSID)
        }
        // else: AP+STA field-recovery mode - keep the STA retry below
    }

    const bool connected = (WiFi.status() == WL_CONNECTED);
    if (connected) {
        staConnected_ = true;
        staFlapHoldMs_ = 0;
        backoffStep_ = 0;
        staFailSinceMs_ = 0;
        const IPAddress ip = WiFi.localIP();
        snprintf(staIpBuf_, sizeof(staIpBuf_), "%d.%d.%d.%d",
                 (int)ip[0], (int)ip[1], (int)ip[2], (int)ip[3]);
        return;
    }

    // Modem-sleep status flap: while no REAL disconnect event has arrived
    // (peek only - AppCore still consumes it for logging) and the hold
    // window has not expired, keep the previous state instead of marking
    // the link down on a transient WiFi.status() reading.
    if (staConnected_ && !evStaDisconnected_) {
        if (staFlapHoldMs_ == 0) {
            staFlapHoldMs_ = nowMs;
        } else if (nowMs - staFlapHoldMs_ <= constants::WIFI_STATUS_FLAP_HOLD_MS) {
            return;
        }
    }
    staConnected_ = false;
    staFlapHoldMs_ = 0;

    // STA failing: track the failure window for the AP fallback.
    if (staFailSinceMs_ == 0) {
        staFailSinceMs_ = nowMs;
    }
    // Field-recovery fallback (prevents a wrong password from locking the
    // unit out of the web setup UI forever): after persistent STA failure
    // open the setup AP alongside and keep retrying in the background.
    if (!apMode_ && nowMs - staFailSinceMs_ >= constants::WIFI_AP_FALLBACK_MS) {
        Logger::error("wifi: STA failing - setup AP started");
        Serial.printf("wifi: STA keeps failing; AP '%s' started (re-enter "
                      "credentials at http://192.168.4.1)\n", apSsid_);
        startAp(true);
    }

    // STA not connected: re-attempt with the backoff sequence
    // (spec 3.10: 1000, 2000, 5000, 10000, 30000 ms). WiFi.begin() in the
    // core is non-blocking (rejoin handled by the Wi-Fi library context),
    // so the main loop stays responsive; the step advances only while
    // attempts keep failing.
    const uint32_t wait = backoffMs(backoffStep_);
    if (nowMs - lastAttemptMs_ >= wait) {
        lastAttemptMs_ = nowMs;
        if (backoffStep_ < 4) {
            backoffStep_++;
        }
        WiFi.disconnect();
        WiFi.begin(staSsid_, staPass_);
    }
}

bool WifiController::consumeStaConnectedEvent() {
    bool e = false;
    if (evStaConnected_) {
        evStaConnected_ = false;
        e = true;
    }
    return e;
}

bool WifiController::consumeStaDisconnectedEvent() {
    bool e = false;
    if (evStaDisconnected_) {
        evStaDisconnected_ = false;
        e = true;
    }
    return e;
}

const char *WifiController::modeName() const {
    if (staConnected_) {
        return "STA";
    }
    if (apMode_) {
        return staActive_ ? "AP+STA" : "AP";
    }
    return "OFF";
}

void WifiController::restartWithCredentials(const AppConfig &cfg) {
    // Full (re-)start of the radio with the given credentials.
    WiFi.mode(WIFI_OFF);
    const char *ssid = (cfg.wifiSsid[0] != '\0') ? cfg.wifiSsid : nullptr;
    if (ssid != nullptr) {
        startSta(cfg.wifiSsid, cfg.wifiPass);
    } else {
        startAp(false);
    }
}

void WifiController::startSta(const char *ssid, const char *pass) {
    apMode_ = false;
    staActive_ = true;
    staFailSinceMs_ = 0;
    snprintf(staSsid_, sizeof(staSsid_), "%s", (ssid != nullptr) ? ssid : "");
    snprintf(staPass_, sizeof(staPass_), "%s", (pass != nullptr) ? pass : "");
    staIpBuf_[0] = '\0';
    lastAttemptMs_ = millis();
    backoffStep_ = 0;
    WiFi.disconnect();
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("acid-doser");   // stable mDNS-friendly name
    WiFi.setSleep(true);              // save power on an idle dosing unit
    WiFi.begin(staSsid_, staPass_);   // non-blocking core rejoin
}

void WifiController::startAp(bool keepSta) {
    apMode_ = true;
    if (!keepSta) {
        staActive_ = false;
    }
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    // spec 3.10: SSID prefix "AcidDoser-" + last 4 hex chars of the MAC.
    snprintf(apSsid_, sizeof(apSsid_), "AcidDoser-%02X%02X", mac[4], mac[5]);

    if (keepSta) {
        // Field recovery: add the AP beside the existing STA radio state.
        WiFi.mode(WIFI_MODE_APSTA);
    } else {
        WiFi.disconnect();
        WiFi.mode(WIFI_AP);
    }
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                      IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    // Open network for initial setup (spec 3.10 / Step 8 requirement 1).
    WiFi.softAP(apSsid_);

    // Captive portal: every DNS query (wildcard "*") is answered with the
    // AP IP so unknown hosts land on http://192.168.4.1/ (spec 3.10,
    // Step 8 requirement 4).
    DNSServer &d = dns();
    d.setErrorReplyCode(DNSReplyCode::NoError);
    d.start(53, "*", IPAddress(192, 168, 4, 1));
}
