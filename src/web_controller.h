// WebController: async web server, captive-portal pages, HTTP API and the
// configuration-update bridge to the main loop (Step 8).
//
// Per acid-dosing-fw-specification.md 3.11 and the Step 8 prompt:
//  - transport: the existing public ESPAsyncWebServer library (AsyncTCP
//    socket servicing is a library context, spec 2.2 - no application
//    FreeRTOS tasks are created by this component);
//  - routes: status page, config page, calibration page, stats page,
//    /debug/log, /api/state, /api/stats plus the full POST API
//    (motor speed, manual dose, schedule, dose factor, Wi-Fi, web auth,
//    MQTT, time, calibration start/stop/confirm, ack, reboot);
//  - auth: none in AP mode (open setup network), HTTP Basic with the
//    stored webUser/webPass in STA mode;
//  - captive portal: in AP mode unknown routes redirect to "/" and the
//    DNS server (WifiController) redirects unknown hosts;
//  - request handlers only parse, validate and push a ConfigUpdate into
//    the thread-safe ConfigUpdateQueue (spec 3.11 / 4.8); AppCore applies
//    the updates (NVS writes, motor/calibration/dose actions, delayed
//    reboot) from the main loop. No NVS, no logging and no heavy work
//    happens inside request handlers;
//  - fixed-size buffers everywhere in request handlers (body max 1024
//    bytes, responses built into 2048-byte buffers, zero-heap JSON scan
//    helpers in web_json.h), per spec 3.11 internal constraints.
//
// The /api/state response carries the spec 3.11 required fields plus a
// few extra keys (ssid, schedule, flow/pump calibration progress) that
// the local UI uses to pre-fill forms.
#ifndef ACID_DOSE_WEB_CONTROLLER_H
#define ACID_DOSE_WEB_CONTROLLER_H

#include <Arduino.h>

#include <ESPAsyncWebServer.h>

#include "constants.h"
#include "types.h"

class AppCore;

class WebController {
public:
    // Registers all routes and starts the server. Call once from the main
    // loop after the AppCore components (and the Wi-Fi controller) exist.
    void begin(AppCore &app);

    // No periodic work is required (the AsyncTCP library context services
    // sockets); kept for interface parity with the other loop hooks.
    void update(uint32_t nowMs) {}

    // True once begin() successfully started the listener on port 80.
    bool running() const { return running_; }

    // Diagnostics (visible in the HIL console).
    uint32_t requestCount() const { return requests_; }
    uint32_t authRejectCount() const { return authRejects_; }

    // Body accumulation hook used by every POST route (registered as the
    // onBody callback). Appends library-delivered chunks to the per-request
    // fixed buffer; no allocation.
    static void onPostBody(AsyncWebServerRequest *req, uint8_t *data,
                           size_t len, size_t index, size_t total);

private:
    // Route handlers (run in the AsyncTCP library context; keep light).
    static void hStatusPage(AsyncWebServerRequest *req);
    static void hConfigPage(AsyncWebServerRequest *req);
    static void hCalibrationPage(AsyncWebServerRequest *req);
    static void hStatsPage(AsyncWebServerRequest *req);
    static void hDebugLog(AsyncWebServerRequest *req);
    static void hApiState(AsyncWebServerRequest *req);
    static void hApiStats(AsyncWebServerRequest *req);
    static void hApiConfig(AsyncWebServerRequest *req);
    static void hNotFound(AsyncWebServerRequest *req);

    // POST API handlers.
    static void hMotorSpeed(AsyncWebServerRequest *req);
    static void hManualDose(AsyncWebServerRequest *req);
    static void hSchedule(AsyncWebServerRequest *req);
    static void hDoseFactor(AsyncWebServerRequest *req);
    static void hWifi(AsyncWebServerRequest *req);
    static void hWebAuth(AsyncWebServerRequest *req);
    static void hMqtt(AsyncWebServerRequest *req);
    static void hTime(AsyncWebServerRequest *req);
    static void hCalFlowStart(AsyncWebServerRequest *req);
    static void hCalFlowStop(AsyncWebServerRequest *req);
    static void hCalPumpStart(AsyncWebServerRequest *req);
    static void hCalPumpConfirm(AsyncWebServerRequest *req);
    static void hCalCancel(AsyncWebServerRequest *req);
    static void hAck(AsyncWebServerRequest *req);
    static void hReboot(AsyncWebServerRequest *req);
    static void hResetDefaults(AsyncWebServerRequest *req);

    AppCore *app_ = nullptr;
    AsyncWebServer server_{80}; // spec 3.11: HTTP on port 80
    bool running_ = false;
    volatile uint32_t requests_ = 0;
    volatile uint32_t authRejects_ = 0;

public:
    // HTTP Basic auth (STA) / open (AP); used by the file-local needAuth().
    static bool authorized(AsyncWebServerRequest *req);
    // Internal counters bumped by the file-local request helpers.
    void incRequest() { requests_++; }
    void incAuthReject() { authRejects_++; }
};

extern WebController *sWebSelf; // single instance (set in begin())

#endif /* ACID_DOSE_WEB_CONTROLLER_H */
