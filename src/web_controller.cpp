// web_controller.cpp - Wi-Fi web UI + HTTP API (spec Step 8, section 3.11).
//
// Design notes (spec 3.11 internal constraints):
//  - No FreeRTOS tasks: this file only registers async callbacks (the
//    AsyncTCP socket task + arduino-esp32 WiFi task) and cooperative
//    update()/begin() calls that run from the application super-loop.
//  - No heap in request handlers: no String::concat / ESPJSON anywhere.
//    JSON request parsing uses the dependency-free scanner in web_json.
//    Responses are written into fixed char buffers (max
//    WEB_MAX_RESPONSE_BYTES) and sent with req->send().
//  - HTML pages are raw string literals in flash (static data); they are
//    served with zero per-request buffer building. POST requests from the
//    UI are JSON made with browser fetch() (the UI code is part of the page
//    text, executed on the browser, not on the device).
//  - POST bodies are accumulated in fixed portMUX-protected slots (max
//    WEB_MAX_BODY_BYTES) because ESPAsyncWebServer hands the body in
//    chunks via the onBody callback.
//  - Handlers only: parse, validate, enqueue (ConfigQueue) or answer from
//    read-only state; the main loop applies updates (AppCore::
//    processQueuedUpdates). Reboots are delayed main-loop restarts.
//  - Auth: HTTP Basic in STA mode (user/pass from config); open in AP mode.
//    (spec 3.11 step 8, authentication paragraph.)
//  - /api/config intentionally does not return the MQTT password (no
//    credential disclosure over a plain read endpoint).

#include "web_controller.h"

#include <esp_system.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

#include "app_core.h"
#include "config_update.h"
#include "hal.h"
#include "logger.h"
#include "names.h"
#include "web_json.h"
#include "wifi_controller.h"

WebController *sWebSelf = nullptr;

namespace {

// ---------------------------------------------------------------------------
// Fixed-size body accumulation (spec 3.11: max request body 1024 bytes, no
// heap in request handlers). The ESPAsyncWebServer library delivers POST
// body chunks through the onBody hook (context: AsyncTCP socket library
// task); the route handler runs once the full body has been received
// (library behavior), so each POST handler reads the accumulated buffer.
// -------------------------------------------------------------------------
struct PendingBody {
    char buf[constants::WEB_MAX_BODY_BYTES + 1];
    size_t len = 0;
    volatile bool complete = false;
    volatile bool inUse = false;
    volatile AsyncWebServerRequest *owner = nullptr;
};

constexpr uint8_t BODY_SLOTS = constants::WEB_BODY_SLOTS;
PendingBody gPending[BODY_SLOTS];
portMUX_TYPE gPendingMux = portMUX_INITIALIZER_UNLOCKED;

PendingBody *pendingFind(const AsyncWebServerRequest *req) {
    for (uint8_t i = 0; i < BODY_SLOTS; i++) {
        if (gPending[i].owner == req) {
            return &gPending[i];
        }
    }
    return nullptr;
}

PendingBody *pendingTake(AsyncWebServerRequest *req) {
    portENTER_CRITICAL(&gPendingMux);
    for (uint8_t i = 0; i < BODY_SLOTS; i++) {
        if (!gPending[i].inUse) {
            gPending[i].inUse = true;
            gPending[i].owner = req;
            gPending[i].len = 0;
            gPending[i].complete = false;
            portEXIT_CRITICAL(&gPendingMux);
            return &gPending[i];
        }
    }
    portEXIT_CRITICAL(&gPendingMux);
    return nullptr;
}

void pendingRelease(PendingBody *p) {
    if (p == nullptr) {
        return;
    }
    portENTER_CRITICAL(&gPendingMux);
    p->inUse = false;
    p->owner = nullptr;
    p->complete = false;
    p->len = 0;
    portEXIT_CRITICAL(&gPendingMux);
}

// RAII-style body access: resolves the accumulated body (or an empty body
// when the endpoint tolerates no body), reports errors and makes sure the
// temporary slot is always released.

size_t jadd(char *b, size_t cap, size_t n, const char *fmt, ...) {
    if (n + 1 >= cap || b == nullptr) {
        return n;
    }
    va_list ap;
    va_start(ap, fmt);
    const int written = vsnprintf(b + n, cap - n, fmt, ap);
    va_end(ap);
    if (written > 0) {
        n += (size_t)written;
    }
    if (n >= cap) {
        n = cap - 1;
    }
    return n;
}

void jsonEsc(char *dst, size_t cap, const char *src) {
    if (cap == 0) {
        return;
    }
    if (src == nullptr) {
        src = "";
    }
    size_t o = 0;
    for (size_t i = 0; src[i] != '\0' && o + 1 < cap; i++) {
        const char c = src[i];
        if (c == '"' || c == '\\') {
            if (o + 2 >= cap) {
                break;
            }
            dst[o++] = '\\';
        } else if ((unsigned char)c < 0x20) {
            continue; // drop control characters
        }
        dst[o++] = c;
    }
    dst[o] = '\0';
}

void sendJson(AsyncWebServerRequest *req, int code, const char *json) {
    req->send(code, "application/json", json);
}

void sendOk(AsyncWebServerRequest *req) {
    sendJson(req, 200, R"({"ok":true})");
}

void sendErr(AsyncWebServerRequest *req, int code, const char *error) {
    char buf[160];
    char esc[160];
    jsonEsc(esc, sizeof(esc), error);
    snprintf(buf, sizeof(buf), R"({"ok":false,"error":"%s"})", esc);
    sendJson(req, code, buf);
}


struct Body {
    bool ok = false;
    const char *data = nullptr;
    size_t len = 0;
    PendingBody *slot = nullptr;

    explicit Body(AsyncWebServerRequest *req) {
        if (req->contentLength() == 0) {
            data = ""; // {} or empty body
            len = 0;
            ok = true;
            return;
        }
        if (req->contentLength() > constants::WEB_MAX_BODY_BYTES) {
            sendErr(req, 413, "body_too_large");
            return;
        }
        slot = pendingFind(req);
        if (slot == nullptr || !slot->complete) {
            if (slot != nullptr) {
                pendingRelease(slot);
            }
            sendErr(req, 400, "body_incomplete");
            return;
        }
        data = slot->buf;
        len = slot->len;
        ok = true;
    }

    ~Body() {
        pendingRelease(slot);
    }
};

} // namespace
// ---------------------------------------------------------------------------
// HTTP Basic authentication (Step 8 requirement 3): open in AP mode,
// stored credentials in STA mode. A failed send issues the 401 challenge
// itself (library helper), so no response duplication is possible.
// ---------------------------------------------------------------------------
bool WebController::authorized(AsyncWebServerRequest *req) {
    if (sWebSelf == nullptr || sWebSelf->app_ == nullptr) {
        sendErr(req, 500, "not_ready");
        return false;
    }
    if (sWebSelf->app_->wifi().isApMode()) {
        return true; // AP: no authentication for setup (spec 3.11)
    }
    const AppConfig &c = sWebSelf->app_->configStore().getAppConfig();
    if (c.webUser[0] == '\0') {
        return true; // no credentials stored: allow (first-run recovery)
    }
    if (req->authenticate(c.webUser, c.webPass)) {
        return true;
    }
    sWebSelf->incAuthReject();
    req->requestAuthentication(AsyncAuthType::AUTH_BASIC, "Acid Doser");
    return false;
}

static void countRequest() {
    if (sWebSelf != nullptr) {
        sWebSelf->incRequest();
    }
}

static bool needAuth(AsyncWebServerRequest *req) {
    countRequest();
    return WebController::authorized(req);
}

// ---------------------------------------------------------------------------
// Pages (spec 3.11 endpoint table; self-contained, each <= 2048 bytes).
// ---------------------------------------------------------------------------
static const char PAGE_STATUS[] =
    R"HTML(<!doctype html><html><head><meta charset=utf-8><meta name=viewport
content="width=device-width,initial-scale=1"><title>Acid Doser</title><style>
body{font:14px sans-serif;margin:14px;background:#f6f7f9}
h1{font-size:17px}nav a{color:#155fa0;margin-right:10px}
table{border-collapse:collapse;background:#fff;border:1px solid #dde1e6}
td{padding:3px 9px 3px 1px;text-align:left}
.ok{color:#0b7a3b}.bad{color:#c02b20}
button{font-size:14px;padding:9px 15px;background:#155fa0;color:#fff;
border:0;border-radius:4px;margin-top:8px}button:disabled{opacity:.5}
#m{margin-left:10px;font-size:13px}</style></head><body><h1>Acid Doser</h1>
<nav><a href=/config>Config</a><a href=/calibration>Calibration</a>
<a href=/stats>Stats</a></nav><table>
<tr><td>State</td><td id=state>-</td></tr>
<tr><td>Time</td><td id=local_time>-</td></tr>
<tr><td>Schedule</td><td id=schedule>-</td></tr>
<tr><td>Motor speed</td><td id=motor_speed_hz>-</td></tr>
<tr><td>Water since dose (l)</td><td id=water_since_dose_l>-</td></tr>
<tr><td>Line present</td><td id=line_present>-</td></tr>
<tr><td>Tank empty</td><td id=tank_empty>-</td></tr>
<tr><td>Driver fault</td><td id=driver_fault>-</td></tr>
<tr><td>Fatal fault</td><td id=fatal_fault>-</td></tr>
<tr><td>Last dose (ml)</td><td id=last_dose_ml>-</td></tr>
<tr><td>Calibration</td><td id=calibration_state>-</td></tr>
<tr><td>Wi-Fi</td><td id=wifi_mode>-</td></tr>
<tr><td>IP</td><td id=ip>-</td></tr>
<tr><td>Boot count</td><td id=boot_count>-</td></tr>
<tr><td>Watchdog resets</td><td id=watchdog_reset_count>-</td></tr>
<tr><td>Loop max (ms)</td><td id=loop_max_ms>-</td></tr>
<tr><td>Loop avg (ms)</td><td id=loop_avg_ms>-</td></tr></table>
<button id=md onclick=md()>Manual dose</button><span id=m></span><script>
var japi=function(u){return fetch(u).then(function(r){
if(!r.ok){return r.text().then(function(x){
throw 'HTTP '+r.status+' '+((x||'').slice(0,60)||'no body')})}
return r.json()});};
function up(){japi('/api/state').then(function(s){
function p2(x){return(x<10?'0':'')+x}
var sc=(s.schedule||[]).map(function(x){return p2(x.hour)+':'+p2(x.minute)+
(x.enabled?'':' off')}).join(', ');
function set(k,v){var e=document.getElementById(k);if(!e)return;
if(v===true||v===false){e.textContent=v?'yes':'no';e.className=v?'bad':'ok';}
else e.textContent=v;}
if(sc)set('schedule',sc);
Object.keys(s).forEach(function(k){if(k!=='schedule'&&
typeof s[k]!=='object')set(k,s[k]);});},
function(e){var st=document.getElementById('state');if(st){st.className='bad';
st.textContent='API error: '+e;}});}
up();setInterval(up,2000);
function md(){var b=document.getElementById('md');b.disabled=true;
fetch('/api/manual_dose',{method:'POST',headers:{'Content-Type':'application/json'},
body:'{}'}).then(function(r){return r.json()}).then(function(d){
document.getElementById('m').textContent=JSON.stringify(d);b.disabled=false;});}
</script></body></html>)HTML";

static const char PAGE_CONFIG[] =
    R"HTML(<!doctype html><html><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1"><title>cfg</title>
<style>body{font:13px sans-serif;margin:10px}h1{font-size:14px}
h2{font-size:13px;margin:10px 0 2px;color:#1560a8}
input{margin:1px 2px 1px 0}.e{color:#c22;font-size:12px}</style></head><body>
<h1>Acid Doser &mdash; Config</h1><a href=/>status</a> | <a href=/calibration>cal</a> | <a href=/stats>stats</a>
<h2>Wi-Fi</h2>SSID <input id=a> Pass <input id=b type=password> <button onclick=W()>Save</button>
<h2>Web auth</h2>User <input id=c> Pass <input id=d type=password> <button onclick=A()>Save</button>
<h2>Dosing</h2>Hz <input id=e type=number> ml/L <input id=f type=number step=0.1> <button onclick=D()>Save</button>
<h2>Schedule (h:m, tick=on)</h2><div id=g></div> <button onclick=S()>Save</button>
<h2>MQTT</h2><input id=i type=checkbox>on host <input id=j> port <input id=k type=number> user <input id=l> pass <input id=m type=password> base <input id=nb> <button onclick=Mq()>Save</button>
<h2>Time (UTC offset, NTP, sync)</h2>TZ min <input id=o type=number> <input id=p type=checkbox>NTP <input id=q> epoch <input id=r type=number> <button onclick=T()>Save</button>
<div id=x class=e>ready</div><script>
var X=document.getElementById('x');
function v(i){return document.getElementById(i).value}
function c(i){return document.getElementById(i).checked}
function P(u,o){X.textContent=u+' ...';fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)}).then(function(r){return r.json()}).then(function(d){X.textContent=u+' '+JSON.stringify(d)},function(){X.textContent=u+' ERR'})}
function W(){P('/api/wifi',{ssid:v('a'),password:v('b')})}
function A(){P('/api/web_auth',{user:v('c'),password:v('d')})}
function D(){P('/api/motor_speed',{speed_hz:+v('e')});P('/api/dose_factor',{ml_per_l:+v('f')})}
function S(){var a=[];for(var i=0;i<4;i++)a.push({hour:+v('h'+i),minute:+v('n'+i),enabled:c('eb'+i)});P('/api/schedule',{slots:a})}
function Mq(){P('/api/mqtt',{enabled:c('i'),host:v('j'),port:+v('k'),user:v('l'),password:v('m'),base_topic:v('nb')})}
function T(){var o={timezone_offset_min:+v('o'),ntp_enabled:c('p'),ntp_server:v('q')};var e=+v('r');if(e>0)o.epoch=e;P('/api/time',o)}
function v2(i,x){document.getElementById(i).value=x}
function c2(i,x){document.getElementById(i).checked=!!x}
for(var i=0;i<4;i++)document.getElementById('g').innerHTML+='<br>'+(i+1)+' <input type=number size=2 id=h'+i+'>:<input type=number size=2 id=n'+i+'><input type=checkbox id=eb'+i+'>';
fetch('/api/config').then(function(r){return r.json()}).then(function(k){
v2('a',k.ssid);v2('e',k.speed_hz);v2('f',k.dose_factor_ml_per_l);
v2('c',k.web_user||'');
v2('j',k.mqtt_host||'');v2('k',k.mqtt_port);v2('l',k.mqtt_user||'');v2('nb',k.mqtt_base||'');c2('i',k.mqtt_enabled);
v2('o',k.timezone_offset_min);c2('p',k.ntp_enabled);v2('q',k.ntp_server||'');
for(var i=0;i<4;i++){s=k.schedule[i]||{hour:0,minute:0,enabled:0};v2('h'+i,s.hour);v2('n'+i,s.minute);c2('eb'+i,s.enabled)};
X.textContent='ok'}).catch(function(e){X.textContent='prefill failed: '
+(e&&e.message?e.message:'network error');});
</script></body></html>)HTML";

static const char PAGE_CALIBRATION[] =
    R"HTML(<!doctype html><html><head><meta charset=utf-8><meta name=viewport
content="width=device-width,initial-scale=1"><title>Calibration</title><style>
body{font:14px sans-serif;margin:14px;background:#f6f7f9}
h1{font-size:17px}a{color:#155fa0}
fieldset{margin:8px 0;padding:9px;background:#fff;min-width:max-content}
legend{font-weight:600}input{font-size:13px;padding:4px;margin:1px}
button{font-size:13px;padding:7px 12px;background:#155fa0;color:#fff;border:0}
#m{font-size:13px;margin:6px 0}</style></head><body>
<h1>Acid Doser - Calibration</h1><a href=/>&larr; status</a>
<p>State: <b id=calibration_state>-</b> &middot; Flow pulses:
<b id=flow_cal_pulses>-</b> &middot; Pump steps: <b id=pump_cal_steps>-</b></p>
<fieldset><legend>Flow meter</legend>
<button onclick=P('/api/cal_flow_start',{})>Start</button>
Volume L <input id=fl type=number size=7 min=0 step=0.1>
<button onclick=P('/api/cal_flow_stop',{volume_liters:+$('fl').value})>
Stop & Save</button></fieldset>
<fieldset><legend>Pump</legend>
<button onclick=P('/api/cal_pump_start',{})>Start</button>
Volume ml <input id=pm type=number size=7 min=0 step=1>
<button onclick=P('/api/cal_pump_confirm',{volume_ml:+$('pm').value})>
Confirm</button></fieldset>
<button onclick=P('/api/cal_cancel',{})>Cancel active session</button>
<span id=m></span><script>
function $(i){return document.getElementById(i)}
function P(u,o){$('m').textContent=u+' ...';
fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},
body:JSON.stringify(o)}).then(function(r){return r.json()}).then(function(d){
$('m').textContent=u+' '+JSON.stringify(d);up();},function(){$('m').
textContent=u+' ERR'});}
var japi=function(u){return fetch(u).then(function(r){
if(!r.ok){return r.text().then(function(x){
throw 'HTTP '+r.status+' '+((x||'').slice(0,60)||'no body')})}
return r.json()});};
function up(){japi('/api/state').then(function(s){
$('calibration_state').textContent=s.calibration_state;
$('flow_cal_pulses').textContent=s.flow_cal_pulses;
$('pump_cal_steps').textContent=s.pump_cal_steps;},
function(e){$('calibration_state').textContent='API error: '+e;});}
up();setInterval(up,2000);
</script></body></html>)HTML";

static const char PAGE_STATS[] =
    R"HTML(<!doctype html><html><head><meta charset=utf-8><meta name=viewport
content="width=device-width,initial-scale=1"><title>Stats</title><style>
body{font:14px sans-serif;margin:14px;background:#f6f7f9}
h1{font-size:17px}a{color:#155fa0}
table{border-collapse:collapse;background:#fff;border:1px solid #dde1e6}
td{padding:3px 9px 3px 1px}</style></head><body><h1>Acid Doser - Stats</h1>
<a href=/>&larr; status</a><table>
<tr><td>Total doses</td><td id=total_doses>-</td></tr>
<tr><td>Total water (l)</td><td id=total_water_l>-</td></tr>
<tr><td>Total acid (ml)</td><td id=total_acid_ml>-</td></tr>
<tr><td>Last dose water (l)</td><td id=last_dose_water_l>-</td></tr>
<tr><td>Last dose acid (ml)</td><td id=last_dose_acid_ml>-</td></tr>
<tr><td>Motor speed</td><td id=motor_speed_hz>-</td></tr>
<tr><td>Boot count</td><td id=boot_count>-</td></tr>
<tr><td>Watchdog resets</td><td id=watchdog_reset_count>-</td></tr>
<tr><td>Wi-Fi</td><td id=wifi_mode>-</td></tr>
<tr><td>IP</td><td id=ip>-</td></tr></table><script>
var japi=function(u){return fetch(u).then(function(r){
if(!r.ok){return r.text().then(function(x){
throw 'HTTP '+r.status+' '+((x||'').slice(0,60)||'no body')})}
return r.json()});};
function up(){Promise.all([japi('/api/stats'),japi('/api/state')]).then(
function(a){var s=a[0];s.wifi_mode=a[1].wifi_mode;s.ip=a[1].ip;
Object.keys(s).forEach(function(k){var e=document.getElementById(k);
if(e&&typeof s[k]!=='object')e.textContent=s[k];});},
function(e){var el=document.getElementById('total_doses');
if(el)el.textContent='API error: '+e;});}
up();setInterval(up,3000);
</script></body></html>)HTML";

void WebController::hStatusPage(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    req->send(200, "text/html", PAGE_STATUS);
}

void WebController::hConfigPage(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    req->send(200, "text/html", PAGE_CONFIG);
}

void WebController::hCalibrationPage(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    req->send(200, "text/html", PAGE_CALIBRATION);
}

void WebController::hStatsPage(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    req->send(200, "text/html", PAGE_STATS);
}

void WebController::hDebugLog(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    char buf[constants::WEB_MAX_RESPONSE_BYTES];
    const uint32_t epochSeconds = (sWebSelf->app_ != nullptr &&
                                   sWebSelf->app_->clock().isTimeKnown())
                                     ? sWebSelf->app_->clock().getEpoch()
                                     : 0;
    if (Logger::serializeRecentJson(buf, sizeof(buf),
                                    constants::WEB_LOG_MAX_ENTRIES,
                                    epochSeconds)) {
        sendJson(req, 200, buf);
    } else {
        sendErr(req, 500, "log_serialize_failed");
    }
}

void WebController::hApiState(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    AppCore &app = *sWebSelf->app_;
    const AppConfig &cfg = app.configStore().getAppConfig();
    const AppCore::FlowStatus &fs = app.getFlowStatus();
    const StatsData &st = app.configStore().getStatsData();

    char buf[constants::WEB_MAX_RESPONSE_BYTES];
    char local[32] = "unknown";
    char esc[64];
    size_t n = 0;
    n = jadd(buf, sizeof(buf), n, "{");
    n = jadd(buf, sizeof(buf), n, "\"device_id\":\"%s\",", app.deviceId());
    n = jadd(buf, sizeof(buf), n, "\"state\":\"%s\",",
         systemStateName(app.getState()));
    n = jadd(buf, sizeof(buf), n, "\"state_id\":%d,",
         (int)app.getState());
    n = jadd(buf, sizeof(buf), n, "\"boot_count\":%lu,",
         (unsigned long)app.configStore().getBootInfo().bootCount);
    n = jadd(buf, sizeof(buf), n, "\"watchdog_reset_count\":%lu,",
         (unsigned long)app.configStore().getBootInfo().watchdogResetCount);
    n = jadd(buf, sizeof(buf), n, "\"loop_max_ms\":%lu,",
         (unsigned long)app.getLoopMaxMs());
    n = jadd(buf, sizeof(buf), n, "\"loop_avg_ms\":%lu,",
         (unsigned long)app.getLoopAvgMs());
    n = jadd(buf, sizeof(buf), n, "\"calibration_state\":%d,",
         (int)app.calibration().getSubState());
    n = jadd(buf, sizeof(buf), n, "\"time_known\":%s,",
         app.clock().isTimeKnown() ? "true" : "false");
    n = jadd(buf, sizeof(buf), n, "\"epoch\":%lu,",
         (unsigned long)app.clock().getEpoch());
    app.clock().formatLocal(local, sizeof(local));
    n = jadd(buf, sizeof(buf), n, "\"local_time\":\"%s\",", local);
    n = jadd(buf, sizeof(buf), n, "\"motor_speed_hz\":%lu,",
         (unsigned long)cfg.motorSpeedHz);
    n = jadd(buf, sizeof(buf), n, "\"line_present\":%s,",
         app.safetyMonitor().isLinePresent() ? "true" : "false");
    n = jadd(buf, sizeof(buf), n, "\"tank_empty\":%s,",
         app.safetyMonitor().isTankEmpty() ? "true" : "false");
    n = jadd(buf, sizeof(buf), n, "\"driver_fault\":%s,",
         app.safetyMonitor().isDriverFault() ? "true" : "false");
    n = jadd(buf, sizeof(buf), n, "\"fatal_fault\":%s,",
         app.isFatalFaultActive() ? "true" : "false");
    n = jadd(buf, sizeof(buf), n, "\"warning_active\":%s,",
         app.isWarningActive() ? "true" : "false");
    n = jadd(buf, sizeof(buf), n, "\"fault_code\":%u,",
         (unsigned)app.getFaultCode());
    n = jadd(buf, sizeof(buf), n, "\"fault_message\":\"%s\",",
         faultCodeName(app.getFaultCode()));
    n = jadd(buf, sizeof(buf), n, "\"water_since_dose_ml\":%.1f,",
         (double)fs.waterMlSinceDose);
    n = jadd(buf, sizeof(buf), n, "\"water_since_dose_l\":%.1f,",
         (double)fs.waterLitersSinceDose);
    n = jadd(buf, sizeof(buf), n, "\"required_dose_ml\":%.2f,",
         (double)app.dosing().getRequiredDoseMilliliters());
    n = jadd(buf, sizeof(buf), n, "\"required_steps\":%lu,",
         (unsigned long)app.dosing().getRequiredSteps());
    n = jadd(buf, sizeof(buf), n, "\"completed_metered_steps\":%lu,",
         (unsigned long)app.dosing().getCompletedMeteredSteps());
    n = jadd(buf, sizeof(buf), n, "\"total_doses\":%lu,",
         (unsigned long)st.totalDoses);
    n = jadd(buf, sizeof(buf), n, "\"total_water_l\":%.1f,",
         (double)(st.totalWaterLiters));
    n = jadd(buf, sizeof(buf), n, "\"total_acid_ml\":%.1f,",
         (double)st.totalAcidMilliliters);
    n = jadd(buf, sizeof(buf), n, "\"last_dose_ml\":%.1f,",
         (double)st.lastDoseAcidMilliliters);
    n = jadd(buf, sizeof(buf), n, "\"last_dose_water_l\":%.1f,",
         (double)(st.lastDoseWaterLiters));
    n = jadd(buf, sizeof(buf), n, "\"flow_calibration_ppl\":%.1f,",
         (double)cfg.pulsesPerLiter);
    n = jadd(buf, sizeof(buf), n,
         "\"pump_calibration_ml_per_step\":%.4f,", (double)cfg.pumpMlPerStep);
    n = jadd(buf, sizeof(buf), n, "\"dose_factor_ml_per_l\":%.3f,",
         (double)cfg.doseFactorMlPerLiter);
    n = jadd(buf, sizeof(buf), n, "\"mqtt_connected\":%s,",
         app.mqtt().isConnected() ? "true" : "false");
    n = jadd(buf, sizeof(buf), n, "\"wifi_mode\":\"%s\",", app.wifi().modeName());
    if (app.wifi().isApMode()) {
        n = jadd(buf, sizeof(buf), n, "\"ip\":\"192.168.4.1\",");
    } else {
        n = jadd(buf, sizeof(buf), n, "\"ip\":\"%s\",", app.wifi().getStaIp());
    }
    // Extras consumed by the local UI (schedule display, flow calibration
    // progress, pump calibration progress, current SSID).
    jsonEsc(esc, sizeof(esc), app.wifi().getStaSsid());
    n = jadd(buf, sizeof(buf), n, "\"ssid\":\"%s\",", esc);
    n = jadd(buf, sizeof(buf), n, "\"flow_cal_pulses\":%lu,",
         (unsigned long)(fs.calibrationActive ? fs.calPulsesSinceStart : 0));
    n = jadd(buf, sizeof(buf), n, "\"pump_cal_steps\":%lu,",
         (unsigned long)app.calibration().getPumpStepsCompleted());
    n = jadd(buf, sizeof(buf), n, "\"schedule\":[");
    for (uint8_t i = 0; i < constants::DEFAULT_SCHEDULE_COUNT; i++) {
        const ScheduleSlot &sl = cfg.schedule[i];
        n = jadd(buf, sizeof(buf), n,
             "%s{\"hour\":%u,\"minute\":%u,\"enabled\":%s}", (i ? "," : ""),
             (unsigned)sl.hour, (unsigned)sl.minute, sl.enabled ? "true" : "false");
    }
    n = jadd(buf, sizeof(buf), n, "]}");
    sendJson(req, 200, buf);
}

void WebController::hApiStats(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    AppCore &app = *sWebSelf->app_;
    const StatsData &st = app.configStore().getStatsData();
    const BootInfo &bi = app.configStore().getBootInfo();
    char buf[512];
    snprintf(buf, sizeof(buf),
             "{\"total_doses\":%lu,"
             "\"total_water_l\":%.1f,"
             "\"total_acid_ml\":%.1f,"
             "\"last_dose_water_l\":%.1f,"
             "\"last_dose_acid_ml\":%.1f,"
             "\"motor_speed_hz\":%lu,"
             "\"boot_count\":%lu,"
             "\"watchdog_reset_count\":%lu,"
             "\"last_dose_boot_ms\":%lu,"
             "\"last_dose_epoch\":%lu}",
             (unsigned long)st.totalDoses,
             (double)st.totalWaterLiters,
             (double)st.totalAcidMilliliters,
             (double)st.lastDoseWaterLiters,
             (double)st.lastDoseAcidMilliliters,
             (unsigned long)app.configStore().getAppConfig().motorSpeedHz,
             (unsigned long)bi.bootCount,
             (unsigned long)bi.watchdogResetCount,
             (unsigned long)st.lastDoseBootMs,
             (unsigned long)st.lastDoseEpoch);
    sendJson(req, 200, buf);
}

void WebController::hApiConfig(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    const AppConfig &c = sWebSelf->app_->configStore().getAppConfig();
    char buf[1024];
    char a[64], b[64], c2[32], d[32];
    jsonEsc(a, sizeof(a), c.wifiSsid);
    jsonEsc(b, sizeof(b), c.mqttHost);
    jsonEsc(c2, sizeof(c2), c.mqttUser);
    jsonEsc(d, sizeof(d), c.mqttBaseTopic);
    char wu[32];
    jsonEsc(wu, sizeof(wu), c.webUser);
    // NOTE: the MQTT password is intentionally NOT part of this response
    // (no credential disclosure through GET; the UI only needs to
    // re-enter it to change it).
    char ntp[96];
    jsonEsc(ntp, sizeof(ntp), c.ntpServer);
    char sched[160];
    int o = 0;
    for (uint8_t i = 0;
         i < constants::DEFAULT_SCHEDULE_COUNT && o + 40 < (int)sizeof(sched);
         i++) {
        o += snprintf(sched + o, sizeof(sched) - o,
                      "%s{\"hour\":%u,\"minute\":%u,\"enabled\":%s}",
                      (i ? "," : ""), (unsigned)c.schedule[i].hour,
                      (unsigned)c.schedule[i].minute,
                      c.schedule[i].enabled ? "true" : "false");
    }
    snprintf(buf, sizeof(buf),
             "{\"ssid\":\"%s\",\"speed_hz\":%lu,"
             "\"dose_factor_ml_per_l\":%.3f,"
             "\"schedule\":[%s],"
             "\"mqtt_enabled\":%s,\"mqtt_host\":\"%s\",\"mqtt_port\":%u,"
             "\"mqtt_user\":\"%s\",\"mqtt_base\":\"%s\","
             "\"timezone_offset_min\":%d,"
             "\"ntp_enabled\":%s,\"ntp_server\":\"%s\",\"web_user\":\"%s\"}",
             a, (unsigned long)c.motorSpeedHz, (double)c.doseFactorMlPerLiter,
             sched, c.mqttEnabled ? "true" : "false", b,
             (unsigned)c.mqttPort, c2, d,
             (int)c.timezoneOffsetMinutes,
             c.ntpEnabled ? "true" : "false", ntp, wu);
    sendJson(req, 200, buf);
}

void WebController::hNotFound(AsyncWebServerRequest *req) {
    countRequest();
    if (sWebSelf == nullptr || sWebSelf->app_ == nullptr) {
        req->send(500, "text/plain", "not ready");
        return;
    }
    // Captive portal (Step 8 requirement 4): in AP mode any unknown route
    // redirects to the status page.
    if (sWebSelf->app_->wifi().isApMode()) {
        req->redirect("/", 302);
    } else {
        req->send(404, "text/plain", "Not found");
    }
}

// ---------------------------------------------------------------------------
// POST API handlers: validate -> queue -> respond. The AppCore main loop
// applies the update (NVS / state machine) in its deterministic order
// (spec 3.13 order item 5).
// ---------------------------------------------------------------------------
void WebController::hMotorSpeed(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    size_t vlen = 0;
    const char *v = wj::findValue(b.data, b.len, "speed_hz", vlen);
    long hz = 0;
    if (v != nullptr && !wj::toInt(v, vlen, hz)) {
        float f = 0.0f;
        if (!wj::toFloat(v, vlen, f) || f != f) {
            sendErr(req, 400, "invalid_speed");
            return;
        }
        hz = (long)f;
    }
    if (v == nullptr || hz <= 0) {
        sendErr(req, 400, "invalid_speed");
        return;
    }
    // Spec 3.6: sanitize out-of-range values (100..800 Hz).
    uint32_t out = (uint32_t)hz;
    if (out < constants::MIN_MOTOR_SPEED_HZ) {
        out = constants::MIN_MOTOR_SPEED_HZ;
    }
    if (out > constants::MAX_MOTOR_SPEED_HZ) {
        out = constants::MAX_MOTOR_SPEED_HZ;
    }
    AppCore &app = *sWebSelf->app_;
    const bool busy = app.stepperEngine().isBusy() ||
                      app.dosing().isDoseActive() ||
                      app.calibration().isActive();
    if (busy) {
        char buf[96];
        snprintf(buf, sizeof(buf),
                 R"({"ok":false,"error":"motor_busy","applied_hz":%lu})",
                 (unsigned long)app.configStore().getAppConfig().motorSpeedHz);
        sendJson(req, 400, buf);
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::MOTOR_SPEED;
    u.u32 = out;
    ConfigQueue::instance().push(u);
    char buf[96];
    snprintf(buf, sizeof(buf), R"({"ok":true,"applied_hz":%lu})", (unsigned)out);
    sendJson(req, 200, buf);
}

void WebController::hManualDose(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    AppCore &app = *sWebSelf->app_;
    if (app.dosing().isDoseActive() || app.dosing().isDosePending()) {
        sendErr(req, 409, "dose_active");
        return;
    }
    if (app.isCalibrationActive()) {
        sendErr(req, 409, "calibration_active");
        return;
    }
    if (app.isFatalFaultActive()) {
        sendErr(req, 500, "fatal_fault");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::MANUAL_DOSE;
    ConfigQueue::instance().push(u);
    sendOk(req);
}

void WebController::hSchedule(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::SCHEDULE;
    size_t vlen = 0;
    const char *v = wj::findValue(b.data, b.len, "slots", vlen);
    if (v != nullptr) {
        wj::Elem e[constants::DEFAULT_SCHEDULE_COUNT];
        const uint8_t n =
            wj::arrayObjs(v, vlen, e, constants::DEFAULT_SCHEDULE_COUNT);
        if (n == 0) {
            sendErr(req, 400, "invalid_schedule");
            return;
        }
        u.schedCount = n;
        u.b = true; // batch form: apply entries [0..schedCount) at once
        for (uint8_t i = 0; i < n; i++) {
            size_t kl = 0;
            const char *kv = wj::findValue(e[i].p, e[i].len, "hour", kl);
            long hour = 0;
            if (kv == nullptr || !wj::toInt(kv, kl, hour) || hour < 0 ||
                hour > 23) {
                sendErr(req, 400, "invalid_schedule");
                return;
            }
            const char *mv = wj::findValue(e[i].p, e[i].len, "minute", kl);
            long minute = 0;
            if (mv == nullptr || !wj::toInt(mv, kl, minute) || minute < 0 ||
                minute > 59) {
                sendErr(req, 400, "invalid_schedule");
                return;
            }
            bool en = false;
            const char *ev = wj::findValue(e[i].p, e[i].len, "enabled", kl);
            if (ev != nullptr && !wj::toBool(ev, kl, en)) {
                sendErr(req, 400, "invalid_schedule");
                return;
            }
            u.schedHour[i] = (uint8_t)hour;
            u.schedMinute[i] = (uint8_t)minute;
            u.schedEn[i] = en;
        }
    } else {
        // Single-slot form: {"slot":N,"hour":H,"minute":M,"enabled":E}.
        long slot = 0, hour = 0, minute = 0;
        bool en = false;
        size_t kl = 0;
        const char *kv = wj::findValue(b.data, b.len, "slot", kl);
        if (kv != nullptr && (!wj::toInt(kv, kl, slot) || slot < 0 || slot > 3)) {
            sendErr(req, 400, "invalid_schedule");
            return;
        }
        const char *hv = wj::findValue(b.data, b.len, "hour", kl);
        if (hv == nullptr || !wj::toInt(hv, kl, hour) || hour < 0 || hour > 23) {
            sendErr(req, 400, "invalid_schedule");
            return;
        }
        const char *mv = wj::findValue(b.data, b.len, "minute", kl);
        if (mv == nullptr || !wj::toInt(mv, kl, minute) || minute < 0 ||
            minute > 59) {
            sendErr(req, 400, "invalid_schedule");
            return;
        }
        const char *ev = wj::findValue(b.data, b.len, "enabled", kl);
        if (ev != nullptr && !wj::toBool(ev, kl, en)) {
            sendErr(req, 400, "invalid_schedule");
            return;
        }
        // Single-slot form: requested slot index in u.u32, values in slot 0,
        // u.b = false (distinguishes from the batch form in AppCore).
        u.b = false;
        u.schedCount = 1;
        u.u32 = (uint32_t)slot;
        u.schedHour[0] = (uint8_t)hour;
        u.schedMinute[0] = (uint8_t)minute;
        u.schedEn[0] = en;
    }
    ConfigQueue::instance().push(u);
    char buf[96];
    snprintf(buf, sizeof(buf), R"({"ok":true,"slots_applied":%u})",
             (unsigned)(u.b ? u.schedCount : 1));
    sendJson(req, 200, buf);
}

void WebController::hDoseFactor(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    size_t vlen = 0;
    const char *v = wj::findValue(b.data, b.len, "ml_per_l", vlen);
    if (v == nullptr) {
        v = wj::findValue(b.data, b.len, "dose_factor", vlen);
    }
    float f = 0.0f;
    if (v == nullptr || !wj::toFloat(v, vlen, f) || f != f ||
        f < constants::MIN_DOSE_FACTOR_ML_PER_L ||
        f > constants::MAX_DOSE_FACTOR_ML_PER_L) {
        sendErr(req, 400, "invalid_range");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::DOSE_FACTOR;
    u.f = f;
    ConfigQueue::instance().push(u);
    char buf[96];
    snprintf(buf, sizeof(buf), R"({"ok":true,"applied_ml_per_l":%.3f})",
             (double)f);
    sendJson(req, 200, buf);
}

void WebController::hWifi(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    char ssid[64] = {0};
    char pass[64] = {0};
    size_t vlen = 0;
    bool haveSsid = false;
    const char *v = wj::findValue(b.data, b.len, "ssid", vlen);
    if (v == nullptr) {
        sendErr(req, 400, "missing_ssid");
        return;
    }
    if (!wj::toStr(v, vlen, ssid, sizeof(ssid))) {
        sendErr(req, 400, "invalid_ssid");
        return;
    }
    haveSsid = (ssid[0] != '\0');
    v = wj::findValue(b.data, b.len, "password", vlen);
    if (v == nullptr) {
        v = wj::findValue(b.data, b.len, "pass", vlen);
    }
    if (v != nullptr && !wj::toStr(v, vlen, pass, sizeof(pass))) {
        sendErr(req, 400, "invalid_pass");
        return;
    }
    if (!haveSsid && pass[0] != '\0') {
        sendErr(req, 400, "missing_ssid");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::WIFI;
    snprintf(u.strA, sizeof(u.strA), "%s", ssid);
    snprintf(u.strB, sizeof(u.strB), "%s", pass);
    u.rebootAfter = true;
    ConfigQueue::instance().push(u);
    sendJson(req, 200, R"({"ok":true,"reboot":true})");
}

void WebController::hWebAuth(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    char user[32] = {0};
    char pass[64] = {0};
    size_t vlen = 0;
    const char *v = wj::findValue(b.data, b.len, "user", vlen);
    if (v == nullptr || !wj::toStr(v, vlen, user, sizeof(user)) ||
        user[0] == '\0') {
        sendErr(req, 400, "invalid_credentials");
        return;
    }
    v = wj::findValue(b.data, b.len, "password", vlen);
    if (v == nullptr || !wj::toStr(v, vlen, pass, sizeof(pass)) ||
        pass[0] == '\0') {
        sendErr(req, 400, "invalid_credentials");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::WEB_AUTH;
    snprintf(u.strA, sizeof(u.strA), "%s", user);
    snprintf(u.strB, sizeof(u.strB), "%s", pass);
    ConfigQueue::instance().push(u);
    sendOk(req);
}

void WebController::hMqtt(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    size_t vlen = 0;
    bool enabled = false;
    const char *v = wj::findValue(b.data, b.len, "enabled", vlen);
    if (v == nullptr || !wj::toBool(v, vlen, enabled)) {
        sendErr(req, 400, "missing_enabled");
        return;
    }
    char host[64] = {0};
    char user[33] = {0};
    char pass[64] = {0};
    char base[33] = {0};
    long port = constants::DEFAULT_MQTT_PORT;
    if (enabled) {
        v = wj::findValue(b.data, b.len, "host", vlen);
        if (v == nullptr || !wj::toStr(v, vlen, host, sizeof(host)) ||
            host[0] == '\0') {
            sendErr(req, 400, "invalid_host");
            return;
        }
        v = wj::findValue(b.data, b.len, "port", vlen);
        if (v != nullptr) {
            if (!wj::toInt(v, vlen, port) || port < 1 || port > 65535) {
                sendErr(req, 400, "invalid_port");
                return;
            }
        }
    } else {
        // Disabled: capture what was provided (host may be cleared).
        v = wj::findValue(b.data, b.len, "host", vlen);
        if (v != nullptr && !wj::toStr(v, vlen, host, sizeof(host))) {
            sendErr(req, 400, "invalid_host");
            return;
        }
    }
    v = wj::findValue(b.data, b.len, "user", vlen);
    if (v != nullptr && !wj::toStr(v, vlen, user, sizeof(user))) {
        sendErr(req, 400, "invalid_user");
        return;
    }
    v = wj::findValue(b.data, b.len, "password", vlen);
    if (v != nullptr && !wj::toStr(v, vlen, pass, sizeof(pass))) {
        sendErr(req, 400, "invalid_pass");
        return;
    }
    v = wj::findValue(b.data, b.len, "base_topic", vlen);
    if (v != nullptr && !wj::toStr(v, vlen, base, sizeof(base))) {
        sendErr(req, 400, "invalid_topic");
        return;
    }
    if (base[0] == '\0') {
        snprintf(base, sizeof(base), "%s", constants::DEFAULT_BASE_TOPIC);
    }
    ConfigUpdate u;
    u.kind = UpdateKind::MQTT;
    u.b = enabled;
    snprintf(u.strA, sizeof(u.strA), "%s", host);
    snprintf(u.strB, sizeof(u.strB), "%s", pass);
    snprintf(u.strC, sizeof(u.strC), "%s", user);
    snprintf(u.strD, sizeof(u.strD), "%s", base);
    u.u32 = (uint32_t)port;
    ConfigQueue::instance().push(u);
    sendJson(req, 200, R"({"ok":true,"saved":true})");
}

void WebController::hTime(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    size_t vlen = 0;
    long epoch = 0;
    const char *v = wj::findValue(b.data, b.len, "epoch", vlen);
    bool pushed = false;
    if (v != nullptr) {
        if (!wj::toInt(v, vlen, epoch) || epoch < 1) {
            sendErr(req, 400, "invalid_epoch");
            return;
        }
        ConfigUpdate u;
        u.kind = UpdateKind::TIME_EPOCH;
        u.u32 = (uint32_t)epoch;
        ConfigQueue::instance().push(u);
        pushed = true;
    }
    long tz = 0;
    v = wj::findValue(b.data, b.len, "timezone_offset_min", vlen);
    if (v == nullptr) {
        v = wj::findValue(b.data, b.len, "timezone", vlen);
    }
    if (v != nullptr) {
        if (!wj::toInt(v, vlen, tz) || tz < -780 || tz > 780) {
            sendErr(req, 400, "invalid_timezone");
            return;
        }
        ConfigUpdate u;
        u.kind = UpdateKind::TIMEZONE;
        u.i16 = (int16_t)tz;
        ConfigQueue::instance().push(u);
        pushed = true;
    }
    bool ntp = false;
    v = wj::findValue(b.data, b.len, "ntp_enabled", vlen);
    if (v != nullptr) {
        if (!wj::toBool(v, vlen, ntp)) {
            sendErr(req, 400, "invalid_ntp");
            return;
        }
        char server[64] = {0};
        const char *sv = wj::findValue(b.data, b.len, "ntp_server", vlen);
        if (sv != nullptr && !wj::toStr(sv, vlen, server, sizeof(server))) {
            sendErr(req, 400, "invalid_ntp");
            return;
        }
        ConfigUpdate u;
        u.kind = UpdateKind::NTP;
        u.b = ntp;
        snprintf(u.strA, sizeof(u.strA), "%s", server);
        ConfigQueue::instance().push(u);
        pushed = true;
    }
    if (!pushed) {
        sendErr(req, 400, "nothing_to_set");
        return;
    }
    sendOk(req);
}

void WebController::hCalFlowStart(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    AppCore &app = *sWebSelf->app_;
    if (app.isCalibrationActive()) {
        sendErr(req, 409, "calibration_active");
        return;
    }
    if (app.dosing().isDoseActive() || app.dosing().isDosePending()) {
        sendErr(req, 409, "dose_active");
        return;
    }
    if (app.isFatalFaultActive()) {
        sendErr(req, 500, "fatal_fault");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::CAL_FLOW_START;
    ConfigQueue::instance().push(u);
    sendOk(req);
}

void WebController::hCalFlowStop(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    if (!sWebSelf->app_->isCalibrationActive() ||
        !sWebSelf->app_->calibration().isFlowCalibrationActive()) {
        sendErr(req, 409, "not_active");
        return;
    }
    size_t vlen = 0;
    float f = 0.0f;
    const char *v = wj::findValue(b.data, b.len, "volume_liters", vlen);
    if (v == nullptr || !wj::toFloat(v, vlen, f) || f != f ||
        f <= 0.0f || f > constants::MAX_FLOW_CAL_VOLUME_LITER) {
        sendErr(req, 400, "invalid_volume");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::CAL_FLOW_STOP;
    u.f = f;
    ConfigQueue::instance().push(u);
    char buf[96];
    snprintf(buf, sizeof(buf), R"({"ok":true,"volume_liters":%.3f})",
             (double)f);
    sendJson(req, 200, buf);
}

void WebController::hCalPumpStart(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    AppCore &app = *sWebSelf->app_;
    if (app.isCalibrationActive()) {
        sendErr(req, 409, "calibration_active");
        return;
    }
    if (app.dosing().isDoseActive() || app.dosing().isDosePending()) {
        sendErr(req, 409, "dose_active");
        return;
    }
    if (app.isFatalFaultActive()) {
        sendErr(req, 500, "fatal_fault");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::CAL_PUMP_START;
    ConfigQueue::instance().push(u);
    sendOk(req);
}

void WebController::hCalPumpConfirm(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    AppCore &app = *sWebSelf->app_;
    if (!app.calibration().isPumpCalibrationActive()) {
        sendErr(req, 409, "not_active");
        return;
    }
    if (app.calibration().getSubState() !=
        CalibrationManager::SubState::PUMP_WAIT_VOLUME) {
        sendErr(req, 409, "not_ready");
        return;
    }
    Body b(req);
    if (!b.ok) {
        return;
    }
    size_t vlen = 0;
    float f = 0.0f;
    const char *v = wj::findValue(b.data, b.len, "volume_ml", vlen);
    if (v == nullptr || !wj::toFloat(v, vlen, f) || f != f ||
        f <= 0.0f || f > 1000.0f) {
        sendErr(req, 400, "invalid_volume");
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::CAL_PUMP_CONFIRM;
    u.f = f;
    ConfigQueue::instance().push(u);
    char buf[96];
    snprintf(buf, sizeof(buf), R"({"ok":true,"volume_ml":%.3f})", (double)f);
    sendJson(req, 200, buf);
}

void WebController::hCalCancel(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::CAL_CANCEL;
    ConfigQueue::instance().push(u);
    sendOk(req);
}

void WebController::hAck(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    const bool had = sWebSelf->app_->isWarningActive();
    ConfigUpdate u;
    u.kind = UpdateKind::ACK;
    ConfigQueue::instance().push(u);
    char buf[96];
    snprintf(buf, sizeof(buf), R"({"ok":true,"cleared":%s})",
             had ? "true" : "false");
    sendJson(req, 200, buf);
}

void WebController::hReboot(AsyncWebServerRequest *req) {
    if (!needAuth(req)) {
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::REBOOT;
    ConfigQueue::instance().push(u);
    sendJson(req, 200, R"({"ok":true,"rebooting":true})");
}

void WebController::hResetDefaults(AsyncWebServerRequest *req) {
    // Field recovery (AI review n8): restore all configuration and
    // statistics to factory defaults (persisted to NVS in the main loop),
    // then soft reboot so Wi-Fi/web/MQTT credentials reload. Auth-gated
    // like every other mutating endpoint; safe at any state - the reboot
    // path halts the motor via the normal power-cycle path.
    if (!needAuth(req)) {
        return;
    }
    ConfigUpdate u;
    u.kind = UpdateKind::RESET_DEFAULTS;
    ConfigQueue::instance().push(u);
    sendJson(req, 200, R"({"ok":true,"resetting":true,"rebooting":true})");
}

void WebController::onPostBody(AsyncWebServerRequest *req, uint8_t *data,
                               size_t len, size_t index, size_t total) {
    PendingBody *p = pendingFind(req);
    if (p == nullptr) {
        p = pendingTake(req);
        if (p == nullptr) {
            return; // no slots free: the handler will report the error
        }
    }
    size_t room = sizeof(p->buf) - 1 - p->len;
    if (len > room) {
        len = room;
    }
    if (len > 0 && p->len + 1 < sizeof(p->buf)) {
        memcpy(p->buf + p->len, data, len);
        p->len += len;
        p->buf[p->len] = '\0';
    }
    if (index + len >= total) {
        p->complete = true;
    }
}

void WebController::begin(AppCore &app) {
    app_ = &app;
    sWebSelf = this;

    // Pages (spec 3.11 endpoint table).
    server_.on("/", HTTP_GET, &WebController::hStatusPage);
    server_.on("/config", HTTP_GET, &WebController::hConfigPage);
    server_.on("/calibration", HTTP_GET, &WebController::hCalibrationPage);
    server_.on("/stats", HTTP_GET, &WebController::hStatsPage);
    server_.on("/debug/log", HTTP_GET, &WebController::hDebugLog);
    server_.on("/api/state", HTTP_GET, &WebController::hApiState);
    server_.on("/api/stats", HTTP_GET, &WebController::hApiStats);
    server_.on("/api/config", HTTP_GET, &WebController::hApiConfig);

    // Captive-portal keep-alive endpoints commonly probed by phones.
    server_.on("/generate_204", HTTP_GET,
               [](AsyncWebServerRequest *req) { req->send(204); });
    server_.on("/success.txt", HTTP_GET,
               [](AsyncWebServerRequest *req) {
                   req->send(200, "text/plain", "OK");
               });
    server_.on("/hotspot-detect.html", HTTP_GET,
               [](AsyncWebServerRequest *req) { req->redirect("/", 302); });

    // POST API (body accumulation via onPostBody for every endpoint).
    server_.on("/api/motor_speed", HTTP_POST, &WebController::hMotorSpeed,
               nullptr, &WebController::onPostBody);
    server_.on("/api/manual_dose", HTTP_POST, &WebController::hManualDose,
               nullptr, &WebController::onPostBody);
    server_.on("/api/schedule", HTTP_POST, &WebController::hSchedule, nullptr,
               &WebController::onPostBody);
    server_.on("/api/dose_factor", HTTP_POST, &WebController::hDoseFactor,
               nullptr, &WebController::onPostBody);
    server_.on("/api/wifi", HTTP_POST, &WebController::hWifi, nullptr,
               &WebController::onPostBody);
    server_.on("/api/web_auth", HTTP_POST, &WebController::hWebAuth, nullptr,
               &WebController::onPostBody);
    server_.on("/api/mqtt", HTTP_POST, &WebController::hMqtt, nullptr,
               &WebController::onPostBody);
    server_.on("/api/time", HTTP_POST, &WebController::hTime, nullptr,
               &WebController::onPostBody);
    server_.on("/api/cal_flow_start", HTTP_POST,
               &WebController::hCalFlowStart, nullptr,
               &WebController::onPostBody);
    server_.on("/api/cal_flow_stop", HTTP_POST, &WebController::hCalFlowStop,
               nullptr, &WebController::onPostBody);
    server_.on("/api/cal_pump_start", HTTP_POST,
               &WebController::hCalPumpStart, nullptr,
               &WebController::onPostBody);
    server_.on("/api/cal_pump_confirm", HTTP_POST,
               &WebController::hCalPumpConfirm, nullptr,
               &WebController::onPostBody);
    server_.on("/api/cal_cancel", HTTP_POST, &WebController::hCalCancel,
               nullptr, &WebController::onPostBody);
    server_.on("/api/ack", HTTP_POST, &WebController::hAck, nullptr,
               &WebController::onPostBody);
    server_.on("/api/reboot", HTTP_POST, &WebController::hReboot, nullptr,
               &WebController::onPostBody);
    server_.on("/api/reset_defaults", HTTP_POST, &WebController::hResetDefaults,
               nullptr, &WebController::onPostBody);

    // Unknown-route handling (captive redirect in AP mode, 404 in STA).
    server_.onNotFound(&WebController::hNotFound);

    server_.begin(); // listens on port 80 (library context, no new task)
    running_ = true;
}
