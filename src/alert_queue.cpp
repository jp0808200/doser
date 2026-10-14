#include "alert_queue.h"

#include <string.h>

#include <Arduino.h>

// Alert queue critical section (specification 4.5: gAlertMux).
static portMUX_TYPE gAlertMux = portMUX_INITIALIZER_UNLOCKED;

namespace Alerts {

AlertQueue &queue() {
    static AlertQueue s_queue;
    return s_queue;
}

} // namespace Alerts

void AlertQueue::begin() {
    memset(entries_, 0, sizeof(entries_));
    portENTER_CRITICAL(&gAlertMux);
    head_ = 0;
    size_ = 0;
    dropped_ = 0;
    portEXIT_CRITICAL(&gAlertMux);
}

void AlertQueue::push(AlertSeverity severity, FaultCode code,
                      const char *message, uint32_t epochOrBootMs) {
    Entry entry;
    entry.epochOrBootMs = epochOrBootMs;
    entry.severity = severity;
    entry.code = code;
    snprintf(entry.message, sizeof(entry.message), "%s",
             (message != nullptr) ? message : "");

    portENTER_CRITICAL(&gAlertMux);
    if (size_ == CAPACITY) {
        // Queue full: overwrite the oldest entry and count the drop.
        dropped_++;
        head_ = (head_ + 1) % CAPACITY;
        const size_t slot = head_;
        entries_[slot] = entry;
    } else {
        const size_t slot = (head_ + size_) % CAPACITY;
        entries_[slot] = entry;
        size_++;
    }
    portEXIT_CRITICAL(&gAlertMux);
}

bool AlertQueue::pop(Entry &out) {
    portENTER_CRITICAL(&gAlertMux);
    if (size_ == 0) {
        portEXIT_CRITICAL(&gAlertMux);
        return false;
    }
    out = entries_[head_];
    head_ = (head_ + 1) % CAPACITY;
    size_--;
    portEXIT_CRITICAL(&gAlertMux);
    return true;
}

bool AlertQueue::empty() const {
    portENTER_CRITICAL(&gAlertMux);
    const bool e = (size_ == 0);
    portEXIT_CRITICAL(&gAlertMux);
    return e;
}

bool AlertQueue::at(size_t index, Entry &out) const {
    portENTER_CRITICAL(&gAlertMux);
    if (index >= size_) {
        portEXIT_CRITICAL(&gAlertMux);
        return false;
    }
    out = entries_[(head_ + index) % CAPACITY];
    portEXIT_CRITICAL(&gAlertMux);
    return true;
}

size_t AlertQueue::size() const {
    portENTER_CRITICAL(&gAlertMux);
    const size_t s = size_;
    portEXIT_CRITICAL(&gAlertMux);
    return s;
}

uint32_t AlertQueue::droppedCount() const {
    portENTER_CRITICAL(&gAlertMux);
    const uint32_t d = dropped_;
    portEXIT_CRITICAL(&gAlertMux);
    return d;
}
