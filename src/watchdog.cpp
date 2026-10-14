#include "watchdog.h"

#include <Arduino.h>
#include <esp_err.h>
#include <esp_task_wdt.h>

// The Arduino-ESP32 framework ships the classic ESP-IDF task watchdog API:
//   esp_task_wdt_init(timeout_s, panic) / esp_task_wdt_add(handle) /
//   esp_task_wdt_reset() / esp_task_wdt_delete(handle).
// - esp_task_wdt_init() takes the timeout in SECONDS (see header:
//   "Timeout period of TWDT in seconds") and updates an already-initialized
//   TWDT instead of failing, so init here is idempotent.
// - esp_task_wdt_add() fails (ESP_ERR_INVALID_ARG per the header) if the
//   task is already subscribed - the Arduino core may already have added
//   the loopTask, so a pre-check via add-state probing is not available in
//   this SDK; instead the duplicate-subscription error is swallowed as
//   "already registered" (Step 4 requirement 9: "if Arduino already added
//   loop task to watchdog, avoid duplicate add errors").
// - Feed is esp_task_wdt_reset() for the currently running task (loopTask).
// - On a feed timeout the chip resets with reason ESP_RST_TASK_WDT, which
//   Logger maps to ResetCause::WDT_TASK and ConfigStore increments the NVS
//   watchdog reset counter (Step 4 verification: "reset count logic
//   increments in NVS"). The reset cause is NOT a panic trap; panic=false.

bool Watchdog::begin() {
    // Initialize the task watchdog with the 5000 ms timeout, no panic trap.
    const esp_err_t initErr = esp_task_wdt_init(TIMEOUT_MS / 1000u, false /*panic*/);
    if (initErr != ESP_OK) {
        Serial.printf("watchdog init failed: %s\n", esp_err_to_name(initErr));
        return false;
    }

    // Subscribe the current (loop) task. NULL = current task.
    const esp_err_t addErr = esp_task_wdt_add(NULL);
    if (addErr != ESP_OK &&
        addErr != ESP_ERR_INVALID_ARG &&    // already subscribed (Step 4 req. 9)
        addErr != ESP_ERR_INVALID_STATE) { // already subscribed (newer-IDF alias)
        Serial.printf("watchdog add loop task failed: %s\n", esp_err_to_name(addErr));
        return false;
    }

    Serial.printf("watchdog: task watchdog %lu ms active, loopTask registered\n",
                  (unsigned long)TIMEOUT_MS);
    return true;
}

void Watchdog::feed() {
    // "Feed" in this API: reset the TWDT on behalf of the running task.
    // Cheap, non-blocking, safe from the loop task context only.
    esp_task_wdt_reset();
}
