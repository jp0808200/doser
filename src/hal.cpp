#include <Arduino.h>

#include "hal.h"

#include <esp_system.h>

namespace Hal {

void setup() {
    // Motor control outputs. Configured before the motor can be enabled.
    pinMode(PIN_STEP, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);

    // Sensor inputs: external pull-ups are assumed by the hardware,
    // so plain INPUT mode is used (no internal pulls added).
    pinMode(PIN_FLOW_PULSE, INPUT);
    pinMode(PIN_ACID_LINE_DET, INPUT);
    pinMode(PIN_TANK_LEVEL, INPUT);
    pinMode(PIN_DRV_FAULT, INPUT);

    // Status LED.
    pinMode(PIN_STATUS_LED, OUTPUT);

    // Safe initial state: no step pulses, forward direction,
    // motor disabled (ENABLE HIGH), LED off. No interrupts attached here.
    setDir(false);
    setStep(false);
    disableMotor();
    setStatusLed(false);
}

void setStep(bool high) {
    digitalWrite(PIN_STEP, high ? HIGH : LOW);
}

void setDir(bool reverse) {
    digitalWrite(PIN_DIR, reverse ? HAL_DIR_REVERSE : HAL_DIR_FORWARD);
}

void enableMotor() {
    digitalWrite(PIN_ENABLE, HAL_ENABLE_LEVEL_ENABLED);
}

void disableMotor() {
    digitalWrite(PIN_ENABLE, HAL_ENABLE_LEVEL_DISABLED);
}

// Sensor input polarities per specification 3.5 (active-high line and tank
// inputs, active-low DRV_FAULT; pull-ups are provided by the hardware/
// driver).
bool readAcidLine() {
    return digitalRead(PIN_ACID_LINE_DET) == HAL_SENSOR_ACTIVE_HIGH_LEVEL;
}

bool readTankEmpty() {
    return digitalRead(PIN_TANK_LEVEL) == HAL_SENSOR_ACTIVE_HIGH_LEVEL;
}

bool readDriverFault() {
    return digitalRead(PIN_DRV_FAULT) == HAL_SENSOR_ACTIVE_LOW_LEVEL;
}

void setStatusLed(bool on) {
    digitalWrite(PIN_STATUS_LED, on ? HIGH : LOW);
}

void enterSafeShutdown() {
    // Disable motor first, then enter deep sleep.
    disableMotor();
    esp_deep_sleep(0);
}

void restart() {
    esp_restart();
}

} // namespace Hal
