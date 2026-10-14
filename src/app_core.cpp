#include "app_core.h"

#include <math.h>

#include <esp_efuse.h>

#include "constants.h"
#include "hal.h"
#include "logger.h"
#include "types.h"

// Flow is considered "stopped" when no pulse arrived for this long.
static constexpr uint32_t FLOW_FRESH_MAX_AGE_MS = 1000;

// Throttle for the near-timeout warning so a persistent stall does not
// flood the log buffer, per specification 3.13.
static constexpr uint32_t NEAR_TIMEOUT_LOG_THROTTLE_MS = 5000;

#ifdef ENABLE_TEST_INJECTION
static constexpr size_t TEST_LINE_MAX = 64;
#endif

bool AppCore::setup() {
    // Watchdog first: 5000 ms task watchdog on the loop task, added before
    // the rest of the (slower) initialization (Step 4 requirement 9).
    if (!watchdog_.begin()) {
        // The task watchdog init/add is non-fatal to boot, but the absence
        // of loop protection must be visible in the boot log.
        Serial.println("watchdog           : INIT FAILED - loopTask unprotected!");
    }
    watchdog_.feed(); // keep it happy across setup()/init.

    // Logger FIRST so config-store messages emitted during load()
    // (e.g. "config repaired") survive Logger::begin()'s buffer reset
    // (AI review M5 - this matters on corrupt/first-NVS boots).
    Logger::begin();

    const bool nvsHadData = configStore_.load();

    const ResetCause cause = Logger::getResetCause();
    Logger::recordResetCause(cause, configStore_.getWatchdogResetCount());
    if (!nvsHadData) {
        Logger::info("first boot, defaults seeded");
    }
    if (configStore_.isConfigRepaired()) {
        Logger::event(EVENT_CONFIG_LOAD_FAILED, "invalid NVS values replaced with defaults");
    }

    // Reset post-mortem (Step 10 req. 4): more than 3 consecutive watchdog
    // resets (each of those boots died before it could clear the streak)
    // means the loop never stabilizes - enter ERROR, then SAFE_SHUTDOWN and
    // deep sleep; only a power cycle or reset releases the unit.
    if (configStore_.getWatchdogStreak() > constants::WDT_STORM_MAX_COUNT) {
        Serial.println("watchdog reset storm: >3 consecutive WDT resets - safe shutdown");
        Logger::event(EVENT_SAFE_SHUTDOWN, "watchdog reset storm (\u003e3 consecutive WDT resets)");
        safeShutdownEnter(FaultCode::WATCHDOG_RESET, "watchdog reset storm");
        // not reached
    }

    const AppConfig &cfg = configStore_.getAppConfig();
    Serial.printf("=== Acid-Dosing Unit FW (ESP32-C3, Step 10 final) ===\n");
    Serial.printf("motorSpeedHz         : %lu (range %lu..%lu)\n",
                  (unsigned long)cfg.motorSpeedHz,
                  (unsigned long)constants::MIN_MOTOR_SPEED_HZ,
                  (unsigned long)constants::MAX_MOTOR_SPEED_HZ);
    Serial.printf("pulsesPerLiter       : %.1f\n", (double)cfg.pulsesPerLiter);
    Serial.printf("pumpMlPerStep        : %.3f\n", (double)cfg.pumpMlPerStep);
    Serial.printf("doseFactorMlPerLiter : %.3f\n", (double)cfg.doseFactorMlPerLiter);

    // Clock (Step 5): NTP is the time source (core SNTP client via
    // configTime, non-blocking), timezone applied, manual epoch supported.
    clock_.begin(cfg);

    Serial.print("schedule             :");
    for (uint8_t i = 0; i < constants::DEFAULT_SCHEDULE_COUNT; ++i) {
        Serial.printf(" [%u: %02u:%02u%s]",
                      (unsigned)i,
                      (unsigned)cfg.schedule[i].hour,
                      (unsigned)cfg.schedule[i].minute,
                      cfg.schedule[i].enabled ? "" : ".off");
    }
    Serial.printf("  (time known: %s)\n", clock_.isTimeKnown() ? "yes" : "no");

    // Flow meter: attach the GPIO4 falling-edge interrupt.
    flowMeter_.setCalibration(cfg.pulsesPerLiter);
    flowMeter_.begin();
    Serial.printf("flow meter           : interrupt active on GPIO%d (falling edge)\n",
                  PIN_FLOW_PULSE);

    // Stepper engine: arm the step generator, then adopt the single
    // user speed parameter from configuration.
    if (stepperEngine_.begin()) {
        stepperEngine_.setSpeedHz(cfg.motorSpeedHz);
        Serial.printf("stepper engine       : one-shot step generator armed, %lu Hz single speed (range %lu..%lu)\n",
                      (unsigned long)stepperEngine_.getSpeedHz(),
                      (unsigned long)constants::MIN_MOTOR_SPEED_HZ,
                      (unsigned long)constants::MAX_MOTOR_SPEED_HZ);
    } else {
        // Non-fatal for this step: motor operations stay unavailable.
        Serial.println("stepper engine       : INIT FAILED - motor operations unavailable");
        Logger::error("stepper engine init failed");
    }

    // Alert queue (bounded, thread-safe) for later MQTT/web reporting.
    Alerts::queue().begin();

    // Safety monitor: tank/line/driver-fault polling with debounce.
    safetyMonitor_.begin();
    Serial.printf("safety monitor       : line/tank/fault polling armed "
                  "(line %lu ms, tank %lu ms, fault %lu ms, fault confirm %lu ms)\n",
                  (unsigned long)SafetyMonitor::LINE_POLL_MS,
                  (unsigned long)SafetyMonitor::TANK_POLL_MS,
                  (unsigned long)SafetyMonitor::DRV_FAULT_POLL_MS,
                  (unsigned long)SafetyMonitor::FAULT_CONFIRM_MS);

    // Dose scheduler (4 daily HH:MM slots) + dosing controller (dose
    // calculation + state machine) + calibration manager (Step 7: flow
    // and pump calibration). The guard keeps dosing out of ERROR states,
    // out while calibration or a dose is active.
    scheduler_.begin(clock_, configStore_, &AppCore::dosingGuard, this);
    dosing_.begin(flowMeter_, configStore_, safetyMonitor_,
                  stepperEngine_, scheduler_, calibration_);
    calibration_.begin(flowMeter_, configStore_, safetyMonitor_,
                       stepperEngine_, dosing_);
    Serial.println("scheduler            : 4 daily slots, once-per-second check (local time)");
    Serial.println("calibration          : flow + pump calibration ready (1600-step pump run)");

    // Step 8: Wi-Fi (STA or AP captive portal) + web server/API. The web
    // routes call into this AppCore; start the server last so all facades
    // (state, config, motor, calibration, manual dose) are live.
    wifi_.begin(configStore_.getAppConfig());
    if (wifi_.isApMode()) {
        Serial.printf("wifi                 : AP captive portal '%s' (open, DNS redirect to 192.168.4.1)\n",
                      wifi_.getApSsid());
    } else {
        Serial.printf("wifi                 : STA '%s'\n", wifi_.getStaSsid());
    }
    web_.begin(*this);
    Serial.printf("web                  : async server on port 80 (Basic auth in STA mode, open in AP)\n");

    // Step 9: optional MQTT reporter (status / alerts / stats). Connects
    // only once the STA link is up and mqttEnabled; otherwise idle.
    mqttReporter_.begin(*this);

    // Device id for the web/MQTT status payloads (spec 4.7).
    {
        uint8_t mac[6] = {0};
        esp_efuse_mac_get_default(mac);
        snprintf(deviceId_, sizeof(deviceId_), "ESP32C3_%02X%02X%02X%02X",
                 mac[2], mac[3], mac[4], mac[5]);
        Serial.printf("device id            : %s\n", deviceId_);
    }

    // Zero the web->main-loop update queue before any traffic.
    ConfigQueue::instance().begin();

    // Healthy at boot.
    fatalHandled_ = false;
    return nvsHadData;
}

void AppCore::loop() {
    const uint32_t nowMs = millis();

    // 1. Main-loop cycle time check + watchdog feed (spec 3.13, Step 4 req. 9)
    //    and Step 10 loop diagnostics (loop_max_ms / loop_avg_ms).
    updateLoopDiagnostics(nowMs);
    watchdog_.feed();

    // 2. Wi-Fi (Step 8, spec 3.13 order item 2): captive DNS in AP mode,
    //    STA state polling + backoff reconnect in STA mode; consume the
    //    library-context event flags and log them from the main loop.
    wifi_.update(nowMs);
    if (wifi_.consumeStaConnectedEvent()) {
        Logger::event(EVENT_WIFI_CONNECTED,
                      "STA connected");
        Serial.printf("wifi: STA connected (ip %s)\n", wifi_.getStaIp());
    }
    if (wifi_.consumeStaDisconnectedEvent()) {
        // Record *why* the STA link is down: auth/4-way-handshake codes
        // mean wrong password or an incompatible security mode (e.g. a
        // WPA3-only router); NO_AP_FOUND means the SSID isn't visible
        // (name/range). The core also prints "Reason: N - ..." on serial.
        const uint8_t reason = wifi_.disconnectReason();
        Logger::event(EVENT_WIFI_DISCONNECTED, "STA disconnected");
        Serial.printf("wifi: STA disconnected (reason %u: %s)\n",
                      (unsigned)reason,
                      WiFi.disconnectReasonName((wifi_err_reason_t)reason));
    }

    // 3. MQTT (Step 9, spec 3.13 order item 3): connection phase only
    //    (start/stop, backoff, link-state logging). The status/alert/stats
    //    publishes happen in publishIfDue() after the scheduler/dosing/
    //    calibration updates (spec 3.13 order item 10, Step 10 req. 2).
    //    Never blocks and never affects the dosing path.
    mqttReporter_.update(nowMs);

    // 3. Clock: latch NTP/manual time (Step 5 req. 4, spec 3.7 time handling).
    clock_.update(nowMs);

    // 3. ConfigStore: dirty-stat wear-reduced flush + config persist.
    configStore_.update(nowMs);

    // 3. Logger: periodic heartbeat entry.
    Logger::tick(nowMs);

    // 4. Flow: volume/freshness bookkeeping once per main loop.
    flowMeter_.update(nowMs);

    // 5. Safety: tank/line/driver-fault polling, debounce, alert pushes.
    //    (Step 4 requirement 11: called exactly once per loop.)
    safetyMonitor_.update(nowMs);

    // 6. Fatal fault: apply the safe stop (motor off) exactly once and hold
    //    the ERROR state; the alert was already queued by the monitor.
    handleFatalFault();

    // 6a. Step 10 (spec 3.13 + Step 10 req. 3): the fatal fault has held for
    //     2000 ms - publish the final alert, then ERROR -> SAFE_SHUTDOWN ->
    //     deep sleep. enterSafeShutdown never returns.
    if (fatalHandled_ && (nowMs - fatalAtMs_) >= constants::SAFE_SHUTDOWN_HOLD_MS) {
        safeShutdownEnter(safetyMonitor_.getFaultCode(),
                          "fatal fault held 2000 ms");
    }

    // 6b. Step 8 (spec 3.13 order item 5): apply the config updates queued
    //     by the web handlers (NVS writes / motor / calibration / dose /
    //     reboot) exactly once per loop, before the scheduler/dosing run.
    processQueuedUpdates(nowMs);

    // 6c. Delayed soft reboot ("/api/reboot", "/api/wifi"), scheduled so the
    //     HTTP response has flushed. Spec Step 8 requirement 9.
    if (rebootDue_ && ((int32_t)(nowMs - rebootAtMs_) >= 0)) {
        Serial.println("reboot requested - restarting");
        Logger::event(EVENT_BOOT, "soft reboot requested by operator");
        Hal::restart();
    }

    // 7. Dose scheduler: once-per-second local-time slot check (spec 3.13
    //    order item 6 after safety update).
    scheduler_.update(nowMs);

    // 8. Dosing controller: advances the dosing state machine (spec 3.13
    //    order item 7), consuming any scheduler/manual request.
    dosing_.update(nowMs);

    // 8b. Calibration manager (Step 7): advances the flow/pump calibration
    //     sub-state machine (spec 3.9 sequence); runs after dosing so the
    //     motor is only taken over when a dose is not in progress.
    calibration_.update(nowMs);

    // 8c. Step 9 (spec 3.12): successful doses reach POST_DOSE after the
    //     reverse purge; flag an immediate stats publish for the publish
    //     phase below.
    if (dosing_.getState() == SystemState::POST_DOSE) {
        mqttReporter_.requestStatsPublish();
    }

    // 7. Flow status for the web/MQTT status structure.
    updateFlowStatus();

    // 8d. Step 10 (spec 3.13 order item 10): publish the MQTT status,
    //     alerts and stats if due, after all same-loop events have landed.
    mqttReporter_.publishIfDue(nowMs);

#ifdef ENABLE_TEST_INJECTION
    // 8. Serial test commands (fixed 64-byte line buffer, no allocation).
    if (Serial) {
        static char line[TEST_LINE_MAX];
        static size_t len = 0;
        while (Serial.available() > 0) {
            const char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                if (len > 0) {
                    line[len] = '\0';
                    len = 0;
                    handleTestCommand(line);
                }
                continue;
            }
            if (len < TEST_LINE_MAX - 1) {
                line[len++] = c;
            }
        }
    }
#endif

    // 9. Status LED: solid on fatal fault (spec 3.13 item 11), blink otherwise
    //    (Step 1 boot behavior).
    statusLedUpdate(nowMs);
}

// ---------------------------------------------------------------------------
// Step 8: config update processing (spec 3.11/4.8 web->main-loop bridge).
// The web handlers only validate and enqueue; ALL NVS writes, motor speed
// application, dose/calibration actions and the delayed reboot happen here,
// in the deterministic main-loop order. Re-checks at apply time keep the
// behavior safe even if the state raced between request and apply.
// ---------------------------------------------------------------------------
void AppCore::processQueuedUpdates(uint32_t nowMs) {
    ConfigUpdate u;
    while (ConfigQueue::instance().pop(u)) {
        applyConfigUpdate(u, nowMs);
    }
}

void AppCore::applyConfigUpdate(const ConfigUpdate &u, uint32_t nowMs) {
    switch (u.kind) {
        case UpdateKind::MOTOR_SPEED: {
            // Spec 3.6: while the motor is busy a speed change is rejected
            // (the web layer already answered motor_busy; re-check here in
            // case the motor started between the request and this pass).
            if (stepperEngine_.isBusy() || dosing_.isDoseActive() ||
                calibration_.isActive()) {
                Serial.println("config: motor speed change dropped (motor busy)");
                break;
            }
            if (configStore_.setMotorSpeedHz(u.u32)) {
                stepperEngine_.setSpeedHz(u.u32);
                Logger::event(EVENT_CONFIG_SAVED, "motor speed Hz");
                Serial.printf("config: motor speed -> %lu Hz\n", (unsigned)u.u32);
            }
            break;
        }
        case UpdateKind::DOSE_FACTOR:
            if (configStore_.setDoseFactor(u.f)) {
                Logger::event(EVENT_CONFIG_SAVED, "dose factor ml/L");
                Serial.printf("config: dose factor -> %.3f ml/L\n", (double)u.f);
            }
            break;

        case UpdateKind::SCHEDULE: {
            const bool batch = u.b && u.schedCount > 0;
            const uint8_t first = batch ? 0 : (uint8_t)u.u32;
            const uint8_t count = batch ? u.schedCount : 1;
            bool ok = true;
            for (uint8_t i = 0; i < count; i++) {
                const uint8_t slot = first + i;
                const uint8_t j = batch ? i : 0; // single-slot values sit at [0]
                if (!configStore_.setScheduleSlot(slot, u.schedHour[j],
                                                   u.schedMinute[j],
                                                   u.schedEn[j])) {
                    ok = false;
                }
                scheduler_.setSlot(slot, u.schedHour[j],
                                   u.schedMinute[j], u.schedEn[j]);
            }
            if (ok) {
                Logger::event(EVENT_CONFIG_SAVED, "schedule");
                Serial.printf("config: schedule updated (%u slots)\n", count);
            }
            break;
        }

        case UpdateKind::WIFI: {
            // Persist synchronously before scheduling the reboot. The
            // dirty-flag flush in ConfigStore::update() happens on the
            // *next* loop pass, so the fixed-delay reboot could in theory
            // outrun the NVS commit; a lost commit means the credentials
            // vanish on the next boot and the unit comes back in AP mode.
            if (!configStore_.setWifiCredentials(u.strA, u.strB)) {
                // Silent-failure guard: the web layer already answered
                // ok:true, so make the rejection (SSID > 31 chars or pass
                // > 63 chars exceeds the config buffers) visible, and do
                // NOT reboot (there is nothing new to apply).
                Logger::error("wifi credentials rejected: exceeds config buffer limits");
                Serial.println("config: wifi credentials REJECTED (SSID max 31 chars, pass max 63 chars)");
                break;
            }
            if (u.rebootAfter && !configStore_.save()) {
                // NVS commit failed: the loop keeps retrying (dirty flag
                // stays set); stay in the current mode instead of rebooting
                // into AP mode without the credentials.
                Logger::error("wifi save: NVS commit failed; reboot aborted");
                Serial.println("config: wifi NVS commit FAILED - NOT rebooting (save will be retried)");
                break;
            }
            Logger::event(EVENT_CONFIG_SAVED, "wifi credentials");
            Serial.println(u.rebootAfter
                               ? "config: wifi credentials saved to NVS; rebooting to apply"
                               : "config: wifi credentials saved (live)");
            if (u.rebootAfter) {
                scheduleReboot(nowMs, constants::WIFI_REBOOT_DELAY_MS);
            }
            break;
        }

        case UpdateKind::WEB_AUTH:
            if (configStore_.setWebCredentials(u.strA, u.strB)) {
                Logger::event(EVENT_CONFIG_SAVED, "web auth");
                Serial.println("config: web user/password updated (live)");
            }
            break;

        case UpdateKind::MQTT:
            if (configStore_.setMqttSettings(u.b, u.strA, (uint16_t)u.u32,
                                             u.strC, u.strB, u.strD)) {
                Logger::event(EVENT_CONFIG_SAVED,
                              u.b ? "mqtt enabled" : "mqtt disabled");
                Serial.printf("config: mqtt %s (%s:%lu)\n", u.b ? "on" : "off",
                              u.strA, (unsigned long)u.u32);
            }
            break;

        case UpdateKind::TIME_EPOCH: {
            const bool ok = clock_.setEpoch(u.u32);
            Serial.printf("config: epoch -> %lu (%s)\n", (unsigned)u.u32,
                          ok ? "applied" : "rejected");
            break;
        }

        case UpdateKind::TIMEZONE:
            configStore_.setTimezoneOffsetMinutes(u.i16);
            scheduler_.setTimezoneOffsetMinutes(u.i16); // forwards to ClockManager
            Serial.printf("config: timezone -> %d min\n", (int)u.i16);
            break;

        case UpdateKind::NTP: {
            const char *server = (u.strA[0] != '\0')
                                     ? u.strA
                                     : configStore_.getAppConfig().ntpServer;
            clock_.setNtp(u.b, server);
            configStore_.setNtpEnabled(u.b);
            if (u.strA[0] != '\0') {
                configStore_.setNtpServer(server);
            }
            Serial.printf("config: ntp %s (%s)\n", u.b ? "on" : "off", server);
            break;
        }

        case UpdateKind::MANUAL_DOSE: {
            const bool ok = dosing_.requestManualDose();
            Serial.printf("web: manual dose request -> %s\n",
                          ok ? "accepted" : "rejected");
            break;
        }

        case UpdateKind::CAL_FLOW_START:
            calibration_.startFlowCalibration();
            break;
        case UpdateKind::CAL_FLOW_STOP:
            calibration_.stopFlowCalibration(u.f);
            break;
        case UpdateKind::CAL_PUMP_START:
            calibration_.startPumpCalibration();
            break;
        case UpdateKind::CAL_PUMP_CONFIRM:
            calibration_.confirmPumpCalibration(u.f);
            break;
        case UpdateKind::CAL_CANCEL:
            if (calibration_.isFlowCalibrationActive()) {
                calibration_.cancelFlowCalibration();
            } else if (calibration_.isActive()) {
                calibration_.cancelPumpCalibration();
            }
            break;

        case UpdateKind::ACK: {
            const bool had = safetyMonitor_.isWarningActive();
            safetyMonitor_.clearWarning();
            if (had) {
                Logger::info("warning acknowledged and cleared");
                Serial.println("web: warning acknowledged");
            }
            break;
        }

        case UpdateKind::REBOOT:
            Serial.println("web: reboot requested; applying after response flush");
            scheduleReboot(nowMs, constants::REBOOT_DELAY_MS);
            break;

        case UpdateKind::RESET_DEFAULTS:
            // Field recovery: restore defaults + stats to factory values,
            // persist them, then reboot so Wi-Fi/web/MQTT credentials are
            // reloaded from the restored config (soft-bricked-config path,
            // AI review n8). Runs in the main loop only.
            if (configStore_.resetToDefaults()) {
                Logger::event(EVENT_CONFIG_LOAD_FAILED,
                              "config + stats reset to defaults (operator action)");
                Serial.println("web: config + stats reset to defaults; rebooting");
                scheduleReboot(nowMs, constants::REBOOT_DELAY_MS);
            } else {
                Logger::error("reset-to-defaults NVS write failed");
                Serial.println("web: reset-to-defaults FAILED (NVS write error)");
            }
            break;

        case UpdateKind::NONE:
        default:
            break; // ignore unknown kinds (forward compatibility)
    }
}

void AppCore::scheduleReboot(uint32_t nowMs, uint32_t delayMs) {
    rebootAtMs_ = nowMs + delayMs;
    if (!rebootDue_) {
        rebootDue_ = true;
        Logger::info("soft reboot scheduled");
    }
}

void AppCore::updateLoopDiagnostics(uint32_t nowMs) {
    const uint32_t elapsed = (lastLoopStartMs_ != 0) ? (nowMs - lastLoopStartMs_) : 0;
    if (lastLoopStartMs_ != 0) {
        if (elapsed > loopMaxMs_) {
            loopMaxMs_ = elapsed;
        }
        if (elapsed > constants::LOOP_CYCLE_WARN_MS) {
            // spec 3.13: "if a loop cycle time exceeds 500 ms, log
            // EVENT_WATCHDOG_NEAR_TIMEOUT". Throttled to avoid flooding the
            // fixed 4096-byte log buffer during a stuck loop.
            if (nowMs - lastNearTimeoutLogMs_ >= NEAR_TIMEOUT_LOG_THROTTLE_MS) {
                lastNearTimeoutLogMs_ = nowMs;
                Logger::event(EVENT_WATCHDOG_NEAR_TIMEOUT,
                              "main loop cycle exceeded 500 ms");
            }
        }
        // Accumulate into the rolling 1000 ms window (Step 10 req. 6):
        // when the window closes, the just-closed one becomes the previous
        // window and the average spans at most two window lengths.
        if ((uint32_t)(nowMs - loopWinStartMs_) >= constants::LOOP_AVG_WINDOW_MS) {
            loopWinPrev_ = loopWinCur_;
            loopWinCur_.sumMs = 0;
            loopWinCur_.count = 0;
            loopWinStartMs_ = nowMs;
        }
        loopWinCur_.sumMs += elapsed;
        if (loopWinCur_.count < 0xFFFFu) {
            loopWinCur_.count++;
        }
    }
    lastLoopStartMs_ = nowMs;
}

// Step 10 req. 6: average loop cycle time over the last 1000 ms window
// (the currently open window plus the last closed one).
uint32_t AppCore::getLoopAvgMs() const {
    const uint32_t now = millis();
    // The current window is already entirely outside the last-1000 ms
    // range: use the most recently closed window, falling back to the
    // current one if no window has closed yet.
    if ((uint32_t)(now - loopWinStartMs_) > constants::LOOP_AVG_WINDOW_MS) {
        if (loopWinPrev_.count > 0) {
            return loopWinPrev_.sumMs / loopWinPrev_.count;
        }
        return (loopWinCur_.count > 0)
                   ? (loopWinCur_.sumMs / loopWinCur_.count)
                   : 0;
    }
    const uint32_t sum = loopWinCur_.sumMs + loopWinPrev_.sumMs;
    const uint32_t count = (uint32_t)loopWinCur_.count + loopWinPrev_.count;
    return (count > 0) ? (sum / count) : 0;
}

// ---------------------------------------------------------------------------
// Step 10: safe shutdown. Called when the fatal fault has held for 2000 ms
// (spec 3.13) or the reset post-mortem detects a watchdog reset storm
// (Step 10 req. 4). Sequence: ERROR -> final alert -> SAFE_SHUTDOWN output
// state -> deep sleep without wake sources (power cycle / reset only).
// ---------------------------------------------------------------------------
void AppCore::safeShutdownEnter(FaultCode code, const char *reason) {
    safeShutdown_ = true;
    // Hold the ERROR state first (spec: "enter ERROR then SAFE_SHUTDOWN").
    fatalHandled_ = true;

    // Final alert: queue it and make one last attempt to publish it while
    // the broker link may still be alive. A failed publish is tolerated -
    // the queue is bounded and the unit sleeps immediately after.
    mqttReporter_.enqueueAlert(AlertSeverity::ERROR, code, reason);
    mqttReporter_.drainAlertNow(millis());

    Logger::event(EVENT_SAFE_SHUTDOWN, reason);
    Logger::event(EVENT_DEEP_SLEEP_ENTERED, "DEEP_SLEEP_ENTERED");
    Serial.printf("SAFE SHUTDOWN: %s -- entering deep sleep (power cycle to release)\n", reason);

    // Let the USB CDC host drain the pending bytes (bounded spin, no delay).
    uint32_t drainStart = millis();
    while (Serial && (millis() - drainStart) < 300) {
    }

    // SAFE_SHUTDOWN output state (Step 10 req. 3): motor ENABLE HIGH
    // (disabled), STEP low, DIR forward (low where safe), status LED off.
    stepperEngine_.stopNow();
    Hal::disableMotor();
    Hal::setStep(false);
    Hal::setDir(false);
    Hal::setStatusLed(false);

    // Deep sleep with NO wake source: only a power cycle or hardware reset
    // reboots the unit (Step 10 constraint). esp_deep_sleep does not return.
    Hal::enterSafeShutdown();
    for (;;) {
        // defensive: esp_deep_sleep never returns
    }
}

void AppCore::handleFatalFault() {
    if (!safetyMonitor_.safeStopRequested() || fatalHandled_) {
        return;
    }
    fatalHandled_ = true;
    fatalAtMs_ = millis();

    // Safe stop (Step 4 requirement 10): immediately disable the motor and
    // hold the ERROR state (derived in getState()). Since Step 10 the ERROR
    // hold is time-limited: if the fault is still latched after 2000 ms the
    // main loop escalates to SAFE_SHUTDOWN and deep sleep (spec 3.13).
    stepperEngine_.stopNow();
    Hal::disableMotor();
    Logger::event(EVENT_SAFE_SHUTDOWN, "fatal fault: safe stop, motor disabled");
    Serial.println("FATAL FAULT: safe stop applied (motor disabled), state=ERROR; deep sleep in 2000 ms");
}

SystemState AppCore::getState() {
    // High-level state (Step 5 requirement 2), derived so the scheduler
    // guard and the web/MQTT layers always agree on the current state.
    if (safeShutdown_) {
        return SystemState::SAFE_SHUTDOWN;
    }
    if (fatalHandled_) {
        return SystemState::ERROR;
    }
    if (dosing_.isDoseActive()) {
        return dosing_.getState();
    }
    if (calibration_.isActive()) {
        return SystemState::CALIBRATION;
    }
    if (flowMeter_.getWaterVolumeLitersSinceDose() > 0.0f) {
        return SystemState::COUNTING;
    }
    return SystemState::IDLE;
}

// DoseScheduler guard (spec 3.7 firing rules): a slot may fire only when the
// system is IDLE or COUNTING. getState() already excludes ERROR,
// CALIBRATION, active doses and fatal faults, so those rules are enforced
// here. Cheap flag checks only - the once-per-second scheduler stays within
// its 100 microsecond budget (Step 5 constraint).
bool AppCore::dosingGuard(void *context) {
    AppCore *self = static_cast<AppCore *>(context);
    if (self == nullptr) {
        return false;
    }
    const SystemState s = self->getState();
    return (s == SystemState::IDLE || s == SystemState::COUNTING);
}

void AppCore::statusLedUpdate(uint32_t nowMs) {
    if (safetyMonitor_.isFatalFaultActive()) {
        // Fatal fault: solid indicator so the failure is visible at a glance.
        if (!ledOn_) {
            ledOn_ = true;
            Hal::setStatusLed(true);
        }
        return;
    }
    if (nowMs - lastLedToggleMs_ >= constants::STATUS_LED_BLINK_PERIOD_MS) {
        lastLedToggleMs_ = nowMs;
        ledOn_ = !ledOn_;
        Hal::setStatusLed(ledOn_);
    }
}

void AppCore::updateFlowStatus() {
    FlowStatus &s = flowStatus_;
    s.calibrationActive = flowMeter_.isCalibrationActive();
    s.calPulsesSinceStart =
        s.calibrationActive ? flowMeter_.getCalibrationPulsesSinceStart() : 0;
    s.pulsesPerLiter = flowMeter_.getPulsesPerLiter();
    s.lastCalibratedPulsesPerLiter = flowMeter_.getLastCalibratedPulsesPerLiter();
    s.waterLitersSinceDose = flowMeter_.getWaterVolumeLitersSinceDose();
    s.waterMlSinceDose = flowMeter_.getEstimatedWaterMillilitersSinceDose();
    s.flowFresh = flowMeter_.isFresh(FLOW_FRESH_MAX_AGE_MS);
}

#ifdef ENABLE_TEST_INJECTION
// HIL test-injection commands (specification 5.2 + Step 4).
void AppCore::handleTestCommand(const char *line) {
    if (strncmp(line, "flowinject", 10) == 0) {
        const char *p = line + 10;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p != '\0') {
            unsigned long pulses = strtoul(p, nullptr, 10);
            flowMeter_.testInject((uint32_t)pulses);
            Serial.printf("flowinject: +%lu, total %lu, water_since_dose %.3f L\n",
                          pulses,
                          (unsigned long)flowMeter_.getPulseCount(),
                          (double)flowMeter_.getWaterVolumeLitersSinceDose());
        } else {
            Serial.println("usage: flowinject N");
        }
        return;
    }

    if (strncmp(line, "line", 4) == 0) {
        const char *p = line + 4;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p == '1') {
            safetyMonitor_.injectLine(true);
        } else if (*p == '0') {
            safetyMonitor_.injectLine(false);
        } else if (*p == '\0') {
            Serial.printf("line: present=%d\n", (int)safetyMonitor_.isLinePresent());
        } else {
            Serial.println("usage: line [1|0]");
        }
        safetyMonitor_.report();
        return;
    }

    if (strncmp(line, "tank", 4) == 0) {
        const char *p = line + 4;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p == '1') {
            safetyMonitor_.injectTankEmpty(true);
        } else if (*p == '0') {
            safetyMonitor_.injectTankEmpty(false);
        } else if (*p == '\0') {
            Serial.printf("tank: empty=%d\n", (int)safetyMonitor_.isTankEmpty());
        } else {
            Serial.println("usage: tank [1|0]");
        }
        safetyMonitor_.report();
        return;
    }

    if (strncmp(line, "fault", 5) == 0) {
        const char *p = line + 5;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p == '1') {
            safetyMonitor_.injectDriverFault(true);
        } else if (*p == '0') {
            safetyMonitor_.injectDriverFault(false);
        } else if (*p == '\0') {
            Serial.printf("fault: active=%d\n", (int)safetyMonitor_.isDriverFault());
        } else {
            Serial.println("usage: fault [1|0]");
        }
        safetyMonitor_.report();
        return;
    }

    if (strncmp(line, "safety", 6) == 0) {
        safetyMonitor_.report();
        return;
    }

    if (strncmp(line, "alerts", 6) == 0) {
        AlertQueue &aq = Alerts::queue();
        const unsigned q = (unsigned)aq.size();
        Serial.printf("alerts: queued=%u dropped=%u\n",
                      q, (unsigned)aq.droppedCount());
        for (unsigned i = 0; i < q; ++i) {
            AlertQueue::Entry e;
            if (aq.at(i, e)) {
                Serial.printf("  [%u] sev=%u code=%u ts=%lu \"%s\"\n",
                              (unsigned)i,
                              (unsigned)e.severity,
                              (unsigned)e.code,
                              (unsigned long)e.epochOrBootMs,
                              e.message);
            }
        }
        return;
    }

    if (strncmp(line, "state", 5) == 0) {
        Serial.printf("state: %u fatal=%d warning=%d code=%u\n",
                      (unsigned)getState(),
                      (int)safetyMonitor_.isFatalFaultActive(),
                      (int)safetyMonitor_.isWarningActive(),
                      (unsigned)safetyMonitor_.getFaultCode());
        Serial.printf("water_since_dose=%.3f L pulses=%lu\n",
                      (double)flowMeter_.getWaterVolumeLitersSinceDose(),
                      (unsigned long)flowMeter_.getPulseCount());
        // Step 6: dosing sequence inspection (phase + exact step counters).
        Serial.printf("dosing: controller=%u phase=%u required_steps=%lu "
                      "completed_steps=%lu\n",
                      (unsigned)dosing_.getState(),
                      (unsigned)dosing_.getPhase(),
                      (unsigned long)dosing_.getRequiredSteps(),
                      (unsigned long)dosing_.getCompletedMeteredSteps());
        Serial.printf("engine: mode=%u busy=%d steps_done=%lu steps_target=%lu\n",
                      (unsigned)stepperEngine_.getMode(),
                      (int)stepperEngine_.isBusy(),
                      (unsigned long)stepperEngine_.getStepsDone(),
                      (unsigned long)stepperEngine_.getTargetSteps());
        // Step 7: calibration manager inspection.
        Serial.printf("cal: active=%d sub=%u flow_pulses_since_start=%lu pump_steps_done=%lu\n",
                      (int)calibration_.isActive(),
                      (unsigned)calibration_.getSubState(),
                      (unsigned long)calibration_.getFlowPulsesSinceStart(),
                      (unsigned long)calibration_.getPumpStepsCompleted());
        return;
    }

    // ---- Step 10: diagnostics (stats / log) ------------------
    if (strncmp(line, "stats", 5) == 0) {
        const StatsData &st = configStore_.getStatsData();
        const BootInfo &bi = configStore_.getBootInfo();
        Serial.printf("stats: doses=%lu water_total=%.1f ml acid_total=%.1f ml "
                      "last_dose=%.2f ml (water %.1f ml)\n",
                      (unsigned long)st.totalDoses,
                      (double)(st.totalWaterLiters * 1000.0f),
                      (double)st.totalAcidMilliliters,
                      (double)st.lastDoseAcidMilliliters,
                      (double)(st.lastDoseWaterLiters * 1000.0f));
        Serial.printf("boot: count=%lu wdt_resets=%lu wdt_streak=%lu last_cause=%lu\n",
                      (unsigned long)bi.bootCount,
                      (unsigned long)bi.watchdogResetCount,
                      (unsigned long)bi.watchdogStreak,
                      (unsigned long)bi.lastResetCause);
        Serial.printf("loop: max=%lu ms avg(last 1000ms)=%lu ms\n",
                      (unsigned long)getLoopMaxMs(), (unsigned long)getLoopAvgMs());
        return;
    }

    if (strncmp(line, "log", 3) == 0) {
        char logbuf[4096];
        const uint32_t epochSeconds = clock_.isTimeKnown() ? clock_.getEpoch() : 0;
        if (Logger::serializeRecentJson(logbuf, sizeof(logbuf), 32,
                                        epochSeconds)) {
            Serial.printf("log: %s\n", logbuf);
        } else {
            Serial.println("log: serialize failed");
        }
        return;
    }

    // Step 3 motor commands remain available.
    if (strncmp(line, "motor_forward", 13) == 0) {
        stepperEngine_.startForwardPrime(constants::MAX_FORWARD_PRIME_MS);
        return;
    }
    if (strncmp(line, "motor_reverse", 13) == 0) {
        stepperEngine_.startReversePurge(constants::MAX_PURGE_MS);
        return;
    }
    if (strncmp(line, "motor_meter", 11) == 0) {
        const char *p = line + 11;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p != '\0') {
            unsigned long steps = strtoul(p, nullptr, 10);
            if (steps < 1) steps = 1;
            if (steps > 1000000UL) steps = 1000000UL;
            stepperEngine_.startForwardMeter((uint32_t)steps);
        } else {
            Serial.println("usage: motor_meter N");
        }
        return;
    }
    if (strncmp(line, "motor_speed", 11) == 0) {
        const char *p = line + 11;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p != '\0') {
            stepperEngine_.setSpeedHz((uint32_t)strtoul(p, nullptr, 10));
        } else {
            Serial.println("usage: motor_speed HZ");
        }
        return;
    }
    if (strncmp(line, "motor_stop", 10) == 0) {
        stepperEngine_.stopNow();
        return;
    }

    // ---- Step 5: clock, schedule, dose -----------------------------
    if (strncmp(line, "now", 3) == 0) {
        char buf[32];
        Serial.printf("now: time_known=%d epoch=%lu local='%s'\n",
                      (int)clock_.isTimeKnown(),
                      (unsigned long)clock_.getEpoch(),
                      clock_.formatLocal(buf, sizeof(buf)) ? buf : "<unknown>");
        return;
    }

    if (strncmp(line, "settime", 7) == 0) {
        const char *p = line + 7;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p != '\0') {
            const unsigned long epoch = strtoul(p, nullptr, 10);
            Serial.printf("settime: epoch=%lu -> %s\n", epoch,
                          (clock_.setEpoch((uint32_t)epoch)) ? "OK" : "REJECTED");
        } else {
            Serial.println("usage: settime EPOCH");
        }
        return;
    }

    if (strncmp(line, "setzone", 7) == 0) {
        const char *p = line + 7;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p != '\0') {
            const int minutes = (int)strtol(p, nullptr, 10);
            Serial.printf("setzone: %d min -> %s\n", minutes,
                          (clock_.updateTimezone((int16_t)minutes)) ? "OK" : "REJECTED");
        } else {
            Serial.println("usage: setzone MINUTES");
        }
        return;
    }

    if (strncmp(line, "slots", 5) == 0) {
        const AppConfig &c = configStore_.getAppConfig();
        for (uint8_t i = 0; i < constants::DEFAULT_SCHEDULE_COUNT; ++i) {
            Serial.printf("  slot %u: %02u:%02u %s last_doy=%u\n",
                          (unsigned)i, (unsigned)c.schedule[i].hour,
                          (unsigned)c.schedule[i].minute,
                          c.schedule[i].enabled ? "on" : "off",
                          (unsigned)c.schedule[i].lastDayOfYear);
        }
        return;
    }

    if (strncmp(line, "setslot", 7) == 0) {
        // setslot N HH MM [0|1]   (enable defaults to 1)
        int slot, hh, mm, en = 1;
        if (sscanf(line + 7, "%d %d %d %d", &slot, &hh, &mm, &en) >= 3
            && slot >= 0 && slot < constants::DEFAULT_SCHEDULE_COUNT) {
            const bool ok = scheduler_.setSlot((uint8_t)slot, (uint8_t)hh,
                                               (uint8_t)mm, en != 0);
            Serial.printf("setslot %d %02d:%02d %s -> %s\n", slot, hh, mm,
                          en ? "on" : "off", ok ? "OK" : "REJECTED");
        } else {
            Serial.println("usage: setslot N HH MM [0|1]");
        }
        return;
    }

    if (strncmp(line, "dose", 4) == 0) {
        // Live recompute only when no dose is active, so inspection never
        // clobbers the bookkeeping of an in-progress sequence (Step 6).
        if (!dosing_.isDoseActive()) {
            dosing_.computeDose(); // live recompute: values reflect current water/calibration
        }
        Serial.printf("dose: state=%d phase=%d pending=%d water_since_dose=%.1f ml "
                      "required=%.2f ml steps=%lu completed=%lu canDose=%d\n",
                      (unsigned)getState(),
                      (unsigned)dosing_.getPhase(),
                      (int)dosing_.isDosePending(),
                      (double)dosing_.getWaterSinceDoseMilliliters(),
                      (double)dosing_.getRequiredDoseMilliliters(),
                      (unsigned long)dosing_.getRequiredSteps(),
                      (unsigned long)dosing_.getCompletedMeteredSteps(),
                      (int)dosing_.canDose());
        return;
    }

    if (strncmp(line, "manual_dose", 11) == 0) {
        Serial.printf("manual_dose -> %s\n",
                      (dosing_.requestManualDose()) ? "accepted" : "rejected");
        return;
    }

    // ---- Step 7: calibration --------------------------------- 
    if (strncmp(line, "flowcal_start", 13) == 0) {
        Serial.printf("flowcal_start -> %s\n",
                      (calibration_.startFlowCalibration()) ? "started" : "rejected");
        return;
    }
    if (strncmp(line, "flowcal_stop", 12) == 0) {
        const char *p = line + 12;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p != '\0') {
            const float liters = (float)strtod(p, nullptr);
            Serial.printf("flowcal_stop %.6f L -> %s\n", (double)liters,
                          (calibration_.stopFlowCalibration(liters)) ? "saved" : "rejected");
        } else {
            Serial.println("usage: flowcal_stop LITERS");
        }
        return;
    }
    if (strncmp(line, "flowcal_cancel", 14) == 0) {
        calibration_.cancelFlowCalibration();
        return;
    }
    if (strncmp(line, "pumpcal_start", 13) == 0) {
        Serial.printf("pumpcal_start -> %s\n",
                      (calibration_.startPumpCalibration()) ? "started" : "rejected");
        return;
    }
    if (strncmp(line, "pumpcal_confirm", 15) == 0) {
        const char *p = line + 15;
        while (*p == ' ' || *p == '\t') { p++; }
        if (*p != '\0') {
            const float ml = (float)strtod(p, nullptr);
            Serial.printf("pumpcal_confirm %.6f ml -> %s\n", (double)ml,
                          (calibration_.confirmPumpCalibration(ml)) ? "saved" : "rejected");
        } else {
            Serial.println("usage: pumpcal_confirm ML");
        }
        return;
    }
    if (strncmp(line, "pumpcal_cancel", 14) == 0) {
        calibration_.cancelPumpCalibration();
        return;
    }
    if (strncmp(line, "cal", 3) == 0) {
        Serial.printf("cal: active=%d sub=%u flow_active=%d pump_active=%d\n",
                      (int)calibration_.isActive(), (unsigned)calibration_.getSubState(),
                      (int)calibration_.isFlowCalibrationActive(),
                      (int)calibration_.isPumpCalibrationActive());
        Serial.printf("  flow: pulses_since_start=%lu last_delta_pulses=%.1f ppl=%.1f\n",
                      (unsigned long)calibration_.getFlowPulsesSinceStart(),
                      (double)calibration_.getFlowLastDeltaPulses(),
                      (double)flowMeter_.getPulsesPerLiter());
        Serial.printf("  pump: steps_done=%lu last_volume_ml=%.2f ml/step=%.4f\n",
                      (unsigned long)calibration_.getPumpStepsCompleted(),
                      (double)calibration_.getPumpCalibrationVolumeMl(),
                      (double)configStore_.getAppConfig().pumpMlPerStep);
        return;
    }

    // ---- Step 8: wifi / web inspection -------------------------
    if (strncmp(line, "wifi", 4) == 0) {
        const uint8_t reason = wifi_.disconnectReason();
        Serial.printf("wifi: mode=%s sta_connected=%d sta_ssid='%s' sta_ip='%s' ap_ssid='%s' dis_reason=%u (%s)\n",
                      wifi_.modeName(), (int)wifi_.isStaConnected(),
                      wifi_.getStaSsid(), wifi_.getStaIp(), wifi_.getApSsid(),
                      (unsigned)reason,
                      WiFi.disconnectReasonName((wifi_err_reason_t)reason));;
        Serial.printf("web: running=%d requests=%lu auth_rejects=%lu %s\n",
                      (int)web_.running(), (unsigned long)web_.requestCount(),
                      (unsigned long)web_.authRejectCount(),
                      wifi_.isApMode() ? "auth open (AP)" : "basic auth (STA)");
        return;
    }
    if (strncmp(line, "queue", 5) == 0) {
        Serial.printf("config queue: pending=%u dropped=%u\n",
                      (unsigned)ConfigQueue::instance().size(),
                      (unsigned)ConfigQueue::instance().droppedCount());
        return;
    }

    // ---- Step 9: mqtt --------------------------------------------
    if (strncmp(line, "mqtt_status", 11) == 0) {
        Serial.printf("mqtt_status -> %s\n",
                      mqttReporter_.publishStatusNow(millis())
                          ? "PUBLISHED" : "FAILED (not connected? or bad broker)");
        return;
    }
    if (strncmp(line, "mqtt_stats", 10) == 0) {
        Serial.printf("mqtt_stats -> %s\n",
                      mqttReporter_.publishStatsNow(millis())
                          ? "PUBLISHED" : "FAILED (not connected? or bad broker)");
        return;
    }
    if (strncmp(line, "mqtt_alert", 10) == 0) {
        // mqtt_alert CODE MESSAGE...  (queued; drained on the next
        // connected update pass exactly like a real alert)
        const char *p = line + 10;
        while (*p == ' ' || *p == '\t') { p++; }
        char *end = nullptr;
        const long code = strtol(p, &end, 10);
        if (end == p) {
            Serial.println("usage: mqtt_alert CODE MESSAGE");
            return;
        }
        const char *msg = end;
        while (*msg == ' ' || *msg == '\t') { msg++; }
        if (*msg == '\0') {
            msg = "TEST_ALERT";
        }
        mqttReporter_.enqueueAlert(AlertSeverity::WARNING,
                                   (FaultCode)(unsigned)code, msg);
        Serial.printf("mqtt_alert: queued code=%lu \"%s\" (drained when connected)\n",
                      (unsigned long)code, msg);
        return;
    }
    if (strncmp(line, "mqtt", 4) == 0) {
        const AppConfig &c = configStore_.getAppConfig();
        Serial.printf("mqtt: enabled=%d host='%s' port=%u sta_up=%d running=%d connected=%d\n",
                      (int)c.mqttEnabled, c.mqttHost, (unsigned)c.mqttPort,
                      (int)wifi_.isStaConnected(), (int)mqttReporter_.isRunning(),
                      (int)mqttReporter_.isConnected());
        Serial.printf("  topics: %s | %s | %s\n",
                      mqttReporter_.topicStatus(), mqttReporter_.topicAlert(),
                      mqttReporter_.topicStats());
        Serial.printf("  published status=%lu alerts=%lu stats=%lu fails=%lu alerts_queued=%u\n",
                      (unsigned long)mqttReporter_.statusPublishedCount(),
                      (unsigned long)mqttReporter_.alertPublishedCount(),
                      (unsigned long)mqttReporter_.statsPublishedCount(),
                      (unsigned long)mqttReporter_.publishFailures(),
                      (unsigned)Alerts::queue().size());
        return;
    }
    if (strncmp(line, "reboot", 6) == 0) {
        Serial.println("reboot: restarting now");
        // Let the USB CDC host drain the pending bytes (bounded spin, no delay).
        uint32_t t = millis();
        while (Serial && (millis() - t) < 200) {
        }
        Hal::restart();
        return; // not reached
    }

    Serial.printf("unknown command: '%s'\n", line);
}
#endif
