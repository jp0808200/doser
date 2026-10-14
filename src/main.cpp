// Acid-Dosing Unit Firmware - application entry point.
// Cooperative super-loop only: the single Arduino loopTask is the only
// application context. No FreeRTOS tasks are created.
#include <Arduino.h>

#include "app_core.h"
#include "constants.h"
#include "hal.h"
#include "logger.h"
#include "pins.h"
#include "types.h"

static AppCore appCore;

void setup() {
    Serial.begin(115200);

    // Wait up to 3 s for a USB CDC host to attach. Non-blocking.
    uint32_t waitStart = millis();
    while (!Serial && (millis() - waitStart) < 3000) {
        // Poll USB CDC availability without delay().
    }

    // Hardware: configure GPIOs, leave the motor disabled.
    Hal::setup();

    // AppCore: NVS load, logger, reset-cause recording, flow meter start.
    appCore.setup();

    const ConfigStore &cfg = appCore.configStore();
    Serial.printf("boot count           : %u\n", cfg.getBootCounter());
    Serial.printf("watchdog reset count : %u\n", cfg.getWatchdogResetCount());
    Serial.println("boot complete (Step 10 final). Main loop running.");
#ifdef ENABLE_TEST_INJECTION
    Serial.println("test injection enabled (serial: 'flowinject N', 'line [1|0]', 'tank [1|0]', 'fault [1|0]',");
    Serial.println("  'safety', 'alerts', 'state', 'stats', 'log', 'now', 'settime EPOCH', 'setzone MIN', 'slots',");
    Serial.println("  'setslot N HH MM [0|1]', 'dose', 'manual_dose', 'motor_forward', 'motor_reverse',");
    Serial.println("  'motor_meter N', 'motor_speed HZ', 'motor_stop', 'reboot',");
    Serial.println("  'flowcal_start', 'flowcal_stop LITERS', 'flowcal_cancel',");
    Serial.println("  'pumpcal_start', 'pumpcal_confirm ML', 'pumpcal_cancel', 'cal', 'wifi', 'queue', 'mqtt*')");
#endif
}

void loop() {
    appCore.loop();
}
