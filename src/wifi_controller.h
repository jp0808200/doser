// WifiController: AP/STA management, captive portal DNS redirect and
// non-blocking STA reconnect with exponential backoff.
//
// Per acid-dosing-fw-specification.md 3.10 and the Step 8 prompt:
//  - if a saved Wi-Fi SSID exists, connect as STA (non-blocking
//    WiFi.status() polling from the main loop; the Wi-Fi library event
//    callbacks only set flags, per spec 3.10 internal constraints);
//  - if no SSID is saved, start an open AP "AcidDoser-XXXX" (XXXX = last
//    4 hex characters of the MAC) at 192.168.4.1, with the core DNSServer
//    captive-portal redirect (any unknown host is answered with the AP IP
//    and the HTTP 307 redirect path, so phones/browsers land on the web UI);
//  - STA reconnect backoff: 1000, 2000, 5000, 10000, 30000 ms;
//  - no application tasks are created anywhere; the only non-loop contexts
//    involved are the Wi-Fi stack library contexts (spec 2.2).
#ifndef ACID_DOSE_WIFI_CONTROLLER_H
#define ACID_DOSE_WIFI_CONTROLLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <esp_event.h>

#include "types.h"

class DNSServer;

class WifiController {
public:
    // STA reconnect backoff sequence (spec 3.10).
    static uint32_t backoffMs(uint8_t step) {
        static const uint32_t table[5] = {1000, 2000, 5000, 10000, 30000};
        return table[step < 5 ? step : 4];
    }

    // Applies the given credentials: STA if a SSID is present, AP captive
    // portal otherwise. Non-blocking (core STA connect is backgrounded).
    void begin(const AppConfig &cfg);

    // Main-loop hook: captive portal DNS processing in AP mode, STA state
    // polling and backoff reconnection in STA mode. Call once per loop.
    void update(uint32_t nowMs);

    bool isApMode() const { return apMode_; }
    bool isStaConnected() const { return staConnected_; }

    // AP SSID ("AcidDoser-XXXX", NUL-terminated), empty in STA mode.
    const char *getApSsid() const { return apSsid_; }
    // Saved STA SSID (empty in AP mode) and current IP ("" when not up).
    const char *getStaSsid() const { return staSsid_; }
    const char *getStaIp() const { return staIpBuf_; }
    // Current connection mode as "AP", "STA", "AP+STA" or "OFF".
    const char *modeName() const;

    // Last STA disconnect reason code (esp_wifi reason enum); the core
    // also prints the reason name on serial. Auth/handshake codes mean
    // wrong password / incompatible security mode (e.g. WPA3-only router).
    uint8_t disconnectReason() const { return discReason_; }

    // Library-context setter for the last STA disconnect reason (only
    // stores state; spec 3.10 constraint).
    void noteDisconnectReason(uint8_t reason){ discReason_ = reason; }

    // Wi-Fi event flags raised by the core (library context) and consumed
    // by AppCore for logging (spec 3.10: "Wi-Fi event callbacks only set
    // flags"). Consume returns true exactly once per event.
    bool consumeStaConnectedEvent();
    bool consumeStaDisconnectedEvent();

    // Re-starts the connection with fresh credentials without rebooting
    // (spec 3.10 API; the Step 8 web flow uses a soft reboot instead,
    // which exercises the full begin() path on the next boot).
    // (intentionally unconnected - see note above / AI review n2.)
    void restartWithCredentials(const AppConfig &cfg);

    // Hook for the core Wi-Fi event callback (library context): only sets
    // the ev* flags (spec 3.10: "Wi-Fi event callbacks only set flags").
    void noteEvent(arduino_event_id_t event);

private:
    void startSta(const char *ssid, const char *pass);
    // keepSta=false: pure setup AP (no saved SSID). keepSta=true: field
    // recovery - open the setup AP *beside* a struggling STA so the
    // password can be re-entered via the web UI instead of flashing.
    void startAp(bool keepSta);
private:
    // Raw wifi event hook (library context): latches the disconnect reason
    // so AppCore can report it alongside the event log line.
    static void rawWifiEvtHandler(void *arg, esp_event_base_t base,
                                  int32_t id, void *data);

    bool apMode_ = false;
    bool staActive_ = false;   // STA retries enabled (true after startSta)
    volatile bool staConnected_ = false;
    uint32_t staFlapHoldMs_ = 0;   // modem-sleep status-flap hold anchor (0 = not holding)
    volatile uint8_t discReason_ = 0;  // last STA disconnect reason code
    char apSsid_[17] = {0};
    char staSsid_[32] = {0};
    char staPass_[64] = {0};
    char staIpBuf_[17] = {0};
    uint32_t lastAttemptMs_ = 0;
    uint32_t staFailSinceMs_ = 0;  // 0 = STA currently up / not yet failed
    uint8_t backoffStep_ = 0;

    // Wi-Fi event flags (set by the library event context).
    volatile bool evStaConnected_ = false;
    volatile bool evStaDisconnected_ = false;
};

#endif /* ACID_DOSE_WIFI_CONTROLLER_H */
