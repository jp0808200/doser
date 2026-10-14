// Watchdog helper: ESP32 task watchdog protecting the Arduino loopTask
// (the only application thread).
// Per acid-dosing-fw-specification.md 3.13 (watchdog feed, loop cycle time
// threshold) and the Step 4 prompt requirement 9.
//
// - Timeout: 5000 ms (Step 4 requirement 9).
// - The loop task is added to the task watchdog; if it is already
//   registered (Arduino core or a previous call), the duplicate add error is
//   swallowed rather than treated as a failure.
// - Feed is called from AppCore once per main loop.
// - No tasks are created; the task watchdog is an SoC feature.
#ifndef ACID_DOSE_WATCHDOG_H
#define ACID_DOSE_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

class Watchdog {
public:
    // Timeout applied to the loopTask watchdog (Step 4 requirement 9).
    static constexpr uint32_t TIMEOUT_MS = 5000;

    // Loop cycle time above which EVENT_WATCHDOG_NEAR_TIMEOUT is logged
    // (specification 3.13). 500 ms.
    static constexpr uint32_t LOOP_CYCLE_WARN_MS = 500;

    // Initialize the task watchdog (5000 ms) and add the current
    // (loop) task. Safe to call once during setup(); duplicate add errors
    // are ignored per Step 4 requirement 9 ("avoid duplicate add errors").
    // Returns true when the watchdog is active (or was already active).
    bool begin();

    // Feeds the watchdog for the calling task. Must be called from the
    // loop task at least once per main loop.
    void feed();

    // True when a loop cycle took longer than LOOP_CYCLE_WARN_MS.
    // Convenience for AppCore: compare millis() deltas.
    static bool loopCycleExceeded(uint32_t elapsedMs) {
        return elapsedMs > LOOP_CYCLE_WARN_MS;
    }
};

#endif /* ACID_DOSE_WATCHDOG_H */
