#include "logger.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_system.h>
#include <string.h>

#include "types.h"

namespace {

struct LogEntry {
    uint32_t tsMs;
    uint16_t eventId;
    uint16_t reserved;
    uint8_t messageLen;
    char message[constants::LOG_MESSAGE_MAX_CHARS];
};

// 4 + 2 + 2 + 1 + 55 = 64 bytes exactly.
static_assert(sizeof(LogEntry) == constants::LOG_ENTRY_SIZE_BYTES,
              "LogEntry must be exactly 64 bytes");

LogEntry s_buffer[constants::LOG_MAX_ENTRIES];
uint16_t s_head = 0;
uint16_t s_entryCount = 0;

uint8_t s_recordedCause = static_cast<uint8_t>(ResetCause::UNKNOWN);
bool s_recorded = false;
uint32_t s_recordedWdtCount = 0;

const char *kResetCauseNames[] = {
    "UNKNOWN",          // 0
    "POWER_ON",         // 1
    "EXTERNAL",         // 2
    "SOFT_RESET",       // 3
    "WDT_CPU",          // 4
    "WDT_BROWNOUT",     // 5
    "WDT_SUPER",        // 6
    "WDT_TASK",         // 7
    "BROWNOUT"          // 8
};

} // namespace

namespace Logger {

void begin() {
    memset(s_buffer, 0, sizeof(s_buffer));
    s_head = 0;
    s_entryCount = 0;
    s_recorded = false;
}

void event(EventId id, const char *message) {
    LogEntry &e = s_buffer[s_head];
    e.tsMs = static_cast<uint32_t>(millis());
    e.eventId = static_cast<uint16_t>(id);
    e.reserved = 0;

    const char *text = message ? message : "";
    size_t len = strlen(text);
    const size_t maxLen = constants::LOG_MESSAGE_MAX_CHARS - 1;
    if (len > maxLen) {
        len = maxLen;
    }
    memcpy(e.message, text, len);
    e.message[len] = '\0';
    e.messageLen = static_cast<uint8_t>(len);
    //send to serial output
    Serial.print("Logger: ");
    Serial.println(text);
    s_head = static_cast<uint16_t>((s_head + 1) & (constants::LOG_MAX_ENTRIES - 1));
    if (s_entryCount < constants::LOG_MAX_ENTRIES) {
        s_entryCount++;
    }
}

void info(const char *message) {
    event(static_cast<EventId>(LOGGER_EVENT_INFO), message);
}

void warning(const char *message) {
    event(static_cast<EventId>(LOGGER_EVENT_WARNING), message);
}

void error(const char *message) {
    event(static_cast<EventId>(LOGGER_EVENT_ERROR), message);
}

bool serializeRecentJson(char *buffer, size_t bufferSize, uint8_t maxEntries,
                         uint32_t epochSeconds) {
    if (buffer == nullptr || bufferSize == 0) {
        return false;
    }

    JsonDocument doc;
    JsonArray arr = doc.to<JsonArray>();

    uint16_t n = (maxEntries > s_entryCount) ? s_entryCount : maxEntries;
    const uint16_t base = static_cast<uint16_t>(
        (s_head + constants::LOG_MAX_ENTRIES - n) & (constants::LOG_MAX_ENTRIES - 1));

    for (uint16_t i = 0; i < n; i++) {
        const LogEntry &e = s_buffer[(base + i) & (constants::LOG_MAX_ENTRIES - 1)];
        JsonObject o = arr.add<JsonObject>();
        o["ts_boot_ms"] = e.tsMs;
        o["event_id"] = e.eventId;
        o["message"] = e.message;
        // Step 10 req. 5: include the epoch in seconds when the time is known.
        if (epochSeconds != 0) {
            o["epoch"] = epochSeconds;
        }
    }

    const size_t len = serializeJson(doc, buffer, bufferSize);
    return len > 0;
}

ResetCause getResetCause() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:
            return ResetCause::POWER_ON;
        case ESP_RST_EXT:
            // ResetCause::EXTERNAL (2): name is unusable here because
            // Arduino.h defines EXTERNAL as a pin-mode macro.
            return static_cast<ResetCause>(2u);
        case ESP_RST_SW:
            return ResetCause::SOFT_RESET;
        case ESP_RST_INT_WDT:
            return ResetCause::WDT_CPU;
        case ESP_RST_TASK_WDT:
            return ResetCause::WDT_TASK;
        case ESP_RST_WDT:
            return ResetCause::WDT_SUPER;
        case ESP_RST_DEEPSLEEP: // resuming from deep sleep is a power-on event
        case ESP_RST_BROWNOUT:
            return ResetCause::BROWNOUT;
        default:
            return ResetCause::UNKNOWN;
    }
}

const char *resetCauseName(ResetCause cause) {
    const uint8_t idx = static_cast<uint8_t>(cause);
    if (idx < sizeof(kResetCauseNames) / sizeof(kResetCauseNames[0])) {
        return kResetCauseNames[idx];
    }
    return "UNKNOWN";
}

void recordResetCause(ResetCause cause, uint32_t watchdogResetCount) {
    s_recordedCause = static_cast<uint8_t>(cause);
    s_recordedWdtCount = watchdogResetCount;
    s_recorded = true;

    char msg[constants::LOG_MESSAGE_MAX_CHARS];
    snprintf(msg, sizeof(msg), "boot: cause=%u wdt=%u",
             static_cast<unsigned>(cause), (unsigned)watchdogResetCount);
    event(EVENT_BOOT, msg);
}

uint8_t getRecordedResetCause() {
    return s_recordedCause;
}

uint32_t getRecordedWatchdogResetCount() {
    return s_recordedWdtCount;
}

void tick(uint32_t nowMs) {
    static uint32_t s_lastHeartbeatMs = 0;
    if (nowMs - s_lastHeartbeatMs >= constants::LOGGER_HEARTBEAT_PERIOD_MS) {
        s_lastHeartbeatMs = nowMs;
        info("heartbeat");
    }
}

} // namespace Logger
