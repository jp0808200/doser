// Hardware abstraction layer for the fixed acid-dosing hardware.
// Per acid-dosing-fw-specification.md 3.1.
#ifndef ACID_DOSE_HAL_H
#define ACID_DOSE_HAL_H

#include <stdbool.h>

#include "pins.h"

// Driver-level GPIO polarity constants.
// DRV8825 ENABLE is active low: HIGH = motor disabled, LOW = motor enabled.
enum {
    HAL_ENABLE_LEVEL_DISABLED = HIGH,
    HAL_ENABLE_LEVEL_ENABLED  = LOW
};

// DRV8825 direction polarity, centralized in these constants.
enum {
    HAL_DIR_FORWARD = LOW,
    HAL_DIR_REVERSE = HIGH
};

// Sensor input interpretation (specification 3.5 behaviour / Step 4 req. 4):
//   ACID_LINE_DET high = liquid present, TANK_LEVEL high = tank empty
//   (active high), DRV_FAULT low = fault asserted (active low, internal
//   pull-up on the DRV8825 side).
enum {
    HAL_SENSOR_ACTIVE_HIGH_LEVEL = HIGH,
    HAL_SENSOR_ACTIVE_LOW_LEVEL  = LOW
};

namespace Hal {

void setup();

void setStep(bool high);
void setDir(bool reverse);
void enableMotor();
void disableMotor();

bool readAcidLine();   // true when liquid is detected in the acid line (input high)
bool readTankEmpty();  // true when the tank-empty switch is asserted (input high)
bool readDriverFault();// true when the DRV8825 reports a fault (input low)

void setStatusLed(bool on);

// Disables the motor first, then enters deep sleep. Not for use from ISR.
void enterSafeShutdown();

// Full chip restart. Not for use from ISR.
void restart();

} // namespace Hal

#endif /* ACID_DOSE_HAL_H */
