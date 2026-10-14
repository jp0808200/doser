// Alert queue: bounded, thread-safe FIFO of safety/alert records for later
// MQTT (Step 9) and web (Step 8) reporting.
// Per acid-dosing-fw-specification.md 3.12 (Alert Queue), 4.5 (gAlertMux)
// and the Step 4 prompt requirement 7.
//
// - Fixed capacity of ALERT_QUEUE_SIZE (8) entries, no dynamic allocation.
// - Push/pop use a portMUX critical section (short, no blocking inside).
// - When full, a push overwrites the OLDEST entry and increments a
//   dropped-alert counter (specification 3.12).
// - Failed alert messages remain queued until consumed by the reporter.
#ifndef ACID_DOSE_ALERT_QUEUE_H
#define ACID_DOSE_ALERT_QUEUE_H

#include <stddef.h>
#include <stdint.h>

#include "constants.h"
#include "types.h"

class AlertQueue {
public:
    // One queued alert record (specification 3.12 AlertMsg layout).
    struct Entry {
        uint32_t epochOrBootMs;
        AlertSeverity severity;
        FaultCode code;
        char message[64];
    };

    static const size_t CAPACITY = constants::ALERT_QUEUE_SIZE;

    // Zeroes the queue state. Called once from the main loop context.
    void begin();

    // Thread-safe append. Overwrites the oldest entry when full and
    // increments the dropped counter. message is truncated to 63 chars.
    // epochOrBootMs is the caller-supplied timestamp (millis while the
    // system clock is not known).
    void push(AlertSeverity severity, FaultCode code,
              const char *message, uint32_t epochOrBootMs);

    // Thread-safe pop of the oldest entry into `out`.
    // Returns true when an entry was available.
    bool pop(Entry &out);

    bool empty() const;
    size_t size() const;

    // Inspect the entry at `index` (0 = oldest) without removing it.
    bool at(size_t index, Entry &out) const;

    // Number of pushes that overwrote an older entry because the queue was full.
    uint32_t droppedCount() const;

private:
    Entry entries_[CAPACITY];
    size_t head_ = 0;   // index of oldest entry
    size_t size_ = 0;
    uint32_t dropped_ = 0;
};

// Process-wide alert record ring (specification 3.12 alert queue). A single
// shared instance is used by SafetyMonitor (producer, Step 4) and later the
// MQTT reporter (Step 9) and the web debug endpoint (Step 8).
namespace Alerts {

// The shared queue instance.
AlertQueue &queue();

} // namespace Alerts

#endif /* ACID_DOSE_ALERT_QUEUE_H */
