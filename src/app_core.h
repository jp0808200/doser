// AppCore: coordinates components in a deterministic main-loop order.
// Step 1-8 scope: ConfigStore + FlowMeter + StepperEngine + SafetyMonitor
// + AlertQueue + Watchdog + ClockManager (NTP/manual time) + DoseScheduler
// + DosingController (Step 6: full prime/meter/purge dosing sequence)
// + CalibrationManager (Step 7: flow and pump calibration)
// + WifiController/WebController (Step 8: Wi-Fi, captive portal, web UI,
//   HTTP API and the config-update queue bridge), flow status exposure,
// fatal-fault safe stop and ERROR state, and main-loop cycle-time /
// watchdog monitoring (acid-dosing-fw-specification.md 3.13 and Step
// 4/5/8 prompts). The only application context is the Arduino loopTask;
// no tasks are created.
#ifndef ACID_DOSE_APP_CORE_H
#define ACID_DOSE_APP_CORE_H

#include <Arduino.h>

#include "config_store.h"
#include "flow_meter.h"
#include "alert_queue.h"
#include "calibration_manager.h"
#include "safety_monitor.h"
#include "stepper_engine.h"
#include "clock_manager.h"
#include "config_update.h"
#include "dose_scheduler.h"
#include "dosing_controller.h"
#include "wifi_controller.h"
#include "web_controller.h"
#include "mqtt_reporter.h"
#include "types.h"
#include "watchdog.h"

class AppCore {
public:
    // Non-volatile view of the flow subsystem for the web/MQTT status
    // structure (later steps read this instead of touching FlowMeter).
    struct FlowStatus {
        bool calibrationActive = false;
        uint32_t calPulsesSinceStart = 0;
        float pulsesPerLiter = 0.0f;
        float lastCalibratedPulsesPerLiter = 0.0f;
        float waterLitersSinceDose = 0.0f;
        float waterMlSinceDose = 0.0f;
        bool flowFresh = false;
    };

    // Initializes ConfigStore, ClockManager, FlowMeter, StepperEngine,
    // AlertQueue, SafetyMonitor, Watchdog, DoseScheduler and
    // DosingController. Must be called after Hal::setup().
    // Returns true if pre-existing NVS data was present.
    bool setup();

    // One main-loop pass in the deterministic order (spec 3.13, Step 5).
    void loop();

    ConfigStore &configStore() { return configStore_; }
    FlowMeter &flowMeter() { return flowMeter_; }
    StepperEngine &stepperEngine() { return stepperEngine_; }
    SafetyMonitor &safetyMonitor() { return safetyMonitor_; }
    AlertQueue &alerts() { return Alerts::queue(); }
    Watchdog &watchdog() { return watchdog_; }
    ClockManager &clock() { return clock_; }
    DoseScheduler &scheduler() { return scheduler_; }
    DosingController &dosing() { return dosing_; }
    CalibrationManager &calibration() { return calibration_; }
    WifiController &wifi() { return wifi_; }
    WebController &web() { return web_; }
    MqttReporter &mqtt() { return mqttReporter_; }

    // Device identifier for the status payloads ("ESP32C3_XXXXXXXX",
    // last 4 MAC bytes; spec 4.7 status schema).
    const char *deviceId() const { return deviceId_; }

    // Calibration facades (Step 7; logging/serial output happens inside
    // CalibrationManager, the facades are thin pass-throughs so the Step 8
    // web endpoints can call either level).
    bool startFlowCalibration() { return calibration_.startFlowCalibration(); }
    bool completeFlowCalibration(float measuredVolumeLiters) {
        return calibration_.stopFlowCalibration(measuredVolumeLiters);
    }
    void cancelFlowCalibration() { calibration_.cancelFlowCalibration(); }
    bool startPumpCalibration() { return calibration_.startPumpCalibration(); }
    bool confirmPumpCalibration(float volumeMilliliters) {
        return calibration_.confirmPumpCalibration(volumeMilliliters);
    }
    void cancelPumpCalibration() { calibration_.cancelPumpCalibration(); }

    const FlowStatus &getFlowStatus() const { return flowStatus_; }

    // High-level state (Step 5 requirement 2): POWER_ON/IDLE/COUNTING/
    // DOSE_READY/DOSING/POST_DOSE/CALIBRATION/ERROR/SAFE_SHUTDOWN.
    // ERROR is latched when a fatal fault is handled; while a dose is
    // active the DosingController's state is reported; COUNTING reflects
    // accumulated water since the last dose.
    SystemState getState();
    bool isFatalFaultActive() const { return safetyMonitor_.isFatalFaultActive(); }
    bool isWarningActive() const { return safetyMonitor_.isWarningActive(); }
    FaultCode getFaultCode() const { return safetyMonitor_.getFaultCode(); }
    bool isSafeStopActive() const { return safetyMonitor_.safeStopRequested(); }
    // Any calibration session (flow or pump, Step 7) keeps the system out
    // of normal dosing.
    bool isCalibrationActive() const { return calibration_.isActive(); }

    // Manual dose request facade (Step 5; web endpoint arrives in Step 8).
    bool requestManualDose() { return dosing_.requestManualDose(); }
    bool isDoseActive() const { return dosing_.isDoseActive(); }
    bool isDosePending() const { return dosing_.isDosePending(); }

    // Step 10 final loop diagnostics (spec 3.13 / Step 10 req. 6):
    // maximum loop cycle time since boot, and the rolling average loop
    // time over the last 1000 ms window.
    uint32_t getLoopMaxMs() const { return loopMaxMs_; }
    uint32_t getLoopAvgMs() const;
    // True once the 2000 ms fatal-fault hold expired (deep sleep imminent).
    bool isInSafeShutdown() const { return safeShutdown_; }

    // Step 8: applies one queued update from the web layer. Split into an
    // individual function per UpdateKind so the switch stays flat.
    void applyConfigUpdate(const ConfigUpdate &u, uint32_t nowMs);
    void scheduleReboot(uint32_t nowMs, uint32_t delayMs);

private:
    void updateFlowStatus();
    void updateLoopDiagnostics(uint32_t nowMs);
    void statusLedUpdate(uint32_t nowMs);
    void handleFatalFault();
    // Step 10: ERROR -> SAFE_SHUTDOWN -> deep sleep (motor disabled, LED off,
    // non-essential outputs low, no wake sources). Never returns.
    [[noreturn]] void safeShutdownEnter(FaultCode code, const char *reason);
    void processQueuedUpdates(uint32_t nowMs);
#ifdef ENABLE_TEST_INJECTION
    void handleTestCommand(const char *line);
#endif

    // DoseScheduler guard callback (static; the scheduler stores it as a
    // plain function pointer with this AppCore as its context).
    static bool dosingGuard(void *context);

    ConfigStore configStore_;
    FlowMeter flowMeter_;
    StepperEngine stepperEngine_;
    SafetyMonitor safetyMonitor_;
    Watchdog watchdog_;
    ClockManager clock_;
    DoseScheduler scheduler_;
    DosingController dosing_;
    CalibrationManager calibration_;
    FlowStatus flowStatus_;
    WifiController wifi_;
    WebController web_;   // must out-live nothing; routes point into it
    MqttReporter mqttReporter_;   // optional MQTT (Step 9; begin() needs *this)
    char deviceId_[17] = "ESP32C3_00000000";

    // Step 8: soft reboot scheduled after the HTTP response has flushed
    // ("/api/reboot" and "/api/wifi" with rebootAfter; spec Step 8 req. 9).
    volatile bool rebootDue_ = false;
    uint32_t rebootAtMs_ = 0;

    bool fatalHandled_ = false;   // latch: fatal fault -> ERROR held
    uint32_t fatalAtMs_ = 0;     // latch time (2000 ms SAFE_SHUTDOWN hold)
    bool safeShutdown_ = false;  // SAFE_SHUTDOWN in progress (deep sleep next)

    // Step 10 loop diagnostics: max cycle since boot + rolling average over
    // a 1000 ms window (two half-window accumulators, no per-loop storage).
    uint32_t loopMaxMs_ = 0;
    struct LoopWindow { uint32_t sumMs = 0; uint16_t count = 0; };
    LoopWindow loopWinCur_ = {};
    LoopWindow loopWinPrev_ = {};
    uint32_t loopWinStartMs_ = 0;

    uint32_t lastLoopStartMs_ = 0;
    uint32_t lastNearTimeoutLogMs_ = 0;
    uint32_t lastLedToggleMs_ = 0;
    bool ledOn_ = false;
};

#endif /* ACID_DOSE_APP_CORE_H */
