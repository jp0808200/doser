// Logger / diagnostics circular buffer (4096 bytes, 64 x 64-byte entries).
// Per acid-dosing-fw-specification.md 3.3 and the Step 1 prompt.
#ifndef ACID_DOSE_LOGGER_H
#define ACID_DOSE_LOGGER_H

#include <stddef.h>
#include <stdint.h>

#include "constants.h"
#include "types.h"

// Synthesized event IDs for untyped info/warning/error messages.
constexpr uint16_t LOGGER_EVENT_INFO    = 128;
constexpr uint16_t LOGGER_EVENT_WARNING = 129;
constexpr uint16_t LOGGER_EVENT_ERROR   = 130;

namespace Logger {

void begin();

void event(EventId id, const char *message);
void info(const char *message);
void warning(const char *message);
void error(const char *message);

// Serializes up to maxEntries most recent log entries as a JSON array into
// buffer (Step 10 req. 5 field names: ts_boot_ms / event_id / message).
// When epochSeconds is non-zero (clock time known) each entry also carries
// the common "epoch" seconds. Never writes more than bufferSize bytes.
// Returns true on success.
bool serializeRecentJson(char *buffer, size_t bufferSize, uint8_t maxEntries,
                         uint32_t epochSeconds);

// Maps the ESP32 reset-reason API to the ResetCause enum of the firmware.
ResetCause getResetCause();

// Human-readable name of a reset cause (never NULL).
const char *resetCauseName(ResetCause cause);

// Records the boot reset cause and watchdog reset count, and logs EVENT_BOOT.
// Must be called after begin().
void recordResetCause(ResetCause cause, uint32_t watchdogResetCount);

uint8_t getRecordedResetCause();
uint32_t getRecordedWatchdogResetCount();

// Main-loop hook: periodic non-blocking heartbeat entry.
void tick(uint32_t nowMs);

} // namespace Logger

#endif /* ACID_DOSE_LOGGER_H */
