// ConfigUpdateQueue: bounded, thread-safe queue between the web request
// handlers (AsyncTCP library context) and the main loop (AppCore).
//
// Per acid-dosing-fw-specification.md 3.11 internal constraints and 4.8
// ("Web handler -> ConfigStore: Config update queue, queue protected by
// mutex"): request handlers only validate input and push a ConfigUpdate;
// AppCore processes the queue once per main loop and performs the NVS
// writes / state machine actions there. This step (Step 8) adds manual
// dose, calibration, acknowledge and reboot requests to the same queue
// (spec 4.8: "Web handler -> DosingController: manual dose request flag,
// flag read/cleared by the main loop").
//
// - Fixed capacity (CONFIG_UPDATE_QUEUE_SIZE), static storage, no heap.
// - push/pop under a portMUX critical section (spec 4.5 rules: short,
//   never yield/heap/NVS/logging inside).
// - When full the OLDEST entry is overwritten and a dropped counter is
//   incremented (same policy as the alert queue, spec 3.12).
#ifndef ACID_DOSE_CONFIG_UPDATE_H
#define ACID_DOSE_CONFIG_UPDATE_H

#include <stddef.h>
#include <stdint.h>

#include "constants.h"

enum class UpdateKind : uint8_t {
    NONE = 0,
    MOTOR_SPEED,      // u32 = Hz (validated 100..800 by the web layer)
    DOSE_FACTOR,      // f   = ml per liter
    SCHEDULE,         // schedCount + schedHour[] / schedMinute[] / schedEn[]
    WIFI,             // strA = ssid, strB = pass, rebootAfter = true
    WEB_AUTH,         // strA = user, strB = pass
    MQTT,             // b = enabled, strA = host, u32 = port,
                      //    strB = pass, strC = user, strD = base topic
    TIME_EPOCH,       // u32 = epoch seconds
    TIMEZONE,         // i16 = offset minutes
    NTP,              // b = enabled, strA = server
    MANUAL_DOSE,
    CAL_FLOW_START,
    CAL_FLOW_STOP,    // f = measured volume liters
    CAL_PUMP_START,
    CAL_PUMP_CONFIRM, // f = dispensed volume ml
    CAL_CANCEL,
    ACK,              // clear the warning latch
    REBOOT,           // soft reboot after the response has been flushed
    RESET_DEFAULTS    // config + stats -> defaults (persisted), then reboot
};

struct ConfigUpdate {
    UpdateKind kind = UpdateKind::NONE;

    // Scalar payload (meaning depends on kind).
    uint32_t u32 = 0;
    int16_t i16 = 0;
    float f = 0.0f;
    bool b = false;

    // String payload (NUL-terminated, fixed-size, no heap).
    char strA[64] = {0};
    char strB[64] = {0};
    char strC[33] = {0};
    char strD[33] = {0};

    // Schedule batch (up to 4 slots).
    uint8_t schedCount = 0;
    uint8_t schedHour[4] = {0, 0, 0, 0};
    uint8_t schedMinute[4] = {0, 0, 0, 0};
    bool schedEn[4] = {false, false, false, false};

    // Set for /api/wifi: the system must reboot afterwards to apply the
    // new credentials (Step 8 requirement 9).
    bool rebootAfter = false;
};

class ConfigUpdateQueue {
public:
    static const size_t CAPACITY = constants::CONFIG_UPDATE_QUEUE_SIZE;

    // Zeroes queue state. Must be called once from the main loop before use.
    void begin();

    // Thread-safe append (web handler context). Overwrites the oldest
    // entry when full and increments the dropped counter.
    void push(const ConfigUpdate &u);

    // Thread-safe pop of the oldest entry into `out`. Main loop context.
    bool pop(ConfigUpdate &out);

    size_t size() const;
    uint32_t droppedCount() const;

private:
    ConfigUpdate entries_[CAPACITY];
    size_t head_ = 0;   // index of oldest entry
    size_t count_ = 0;
    uint32_t dropped_ = 0;
};

// Process-wide queue instance (spec 4.8 inter-module mechanism).
namespace ConfigQueue {
ConfigUpdateQueue &instance();
}

#endif /* ACID_DOSE_CONFIG_UPDATE_H */
