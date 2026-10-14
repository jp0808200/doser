// Fixed hardware pin map.
// Do not change: the hardware is fixed per acid-dosing-fw-specification.md 3.1.
#ifndef ACID_DOSE_PINS_H
#define ACID_DOSE_PINS_H

#include <stdint.h>

constexpr uint8_t PIN_STEP          = 0;
constexpr uint8_t PIN_DIR           = 1;
constexpr uint8_t PIN_ENABLE        = 3;
constexpr uint8_t PIN_FLOW_PULSE    = 4;
constexpr uint8_t PIN_ACID_LINE_DET = 5;
constexpr uint8_t PIN_TANK_LEVEL    = 6;
constexpr uint8_t PIN_DRV_FAULT     = 7;
constexpr uint8_t PIN_STATUS_LED    = 10;

// Reserved pins that must never be used for functional control.
constexpr uint8_t UNUSED_RESERVED_2  = 2;
constexpr uint8_t UNUSED_RESERVED_8  = 8;
constexpr uint8_t UNUSED_RESERVED_9  = 9;
constexpr uint8_t UNUSED_RESERVED_20 = 20;
constexpr uint8_t UNUSED_RESERVED_21 = 21;

#endif /* ACID_DOSE_PINS_H */
