#include "config_update.h"

#include <string.h>

#include <Arduino.h>

// Config update queue critical section (specification 4.5 style portMUX;
// short sections only, no yield/heap/NVS/logging inside).
static portMUX_TYPE gConfigUpdMux = portMUX_INITIALIZER_UNLOCKED;

ConfigUpdateQueue &ConfigQueue::instance() {
    static ConfigUpdateQueue queue;
    return queue;
}

void ConfigUpdateQueue::begin() {
    memset(entries_, 0, sizeof(entries_));
    portENTER_CRITICAL(&gConfigUpdMux);
    head_ = 0;
    count_ = 0;
    dropped_ = 0;
    portEXIT_CRITICAL(&gConfigUpdMux);
}

void ConfigUpdateQueue::push(const ConfigUpdate &u) {
    portENTER_CRITICAL(&gConfigUpdMux);
    if (count_ == CAPACITY) {
        // Queue full: overwrite the oldest entry and count the drop
        // (same policy as the alert queue, spec 3.12).
        dropped_++;
        head_ = (head_ + 1) % CAPACITY;
        entries_[head_] = u;
    } else {
        entries_[(head_ + count_) % CAPACITY] = u;
        count_++;
    }
    portEXIT_CRITICAL(&gConfigUpdMux);
}

bool ConfigUpdateQueue::pop(ConfigUpdate &out) {
    portENTER_CRITICAL(&gConfigUpdMux);
    if (count_ == 0) {
        portEXIT_CRITICAL(&gConfigUpdMux);
        return false;
    }
    out = entries_[head_];
    head_ = (head_ + 1) % CAPACITY;
    count_--;
    portEXIT_CRITICAL(&gConfigUpdMux);
    return true;
}

size_t ConfigUpdateQueue::size() const {
    portENTER_CRITICAL(&gConfigUpdMux);
    const size_t s = count_;
    portEXIT_CRITICAL(&gConfigUpdMux);
    return s;
}

uint32_t ConfigUpdateQueue::droppedCount() const {
    portENTER_CRITICAL(&gConfigUpdMux);
    const uint32_t d = dropped_;
    portEXIT_CRITICAL(&gConfigUpdMux);
    return d;
}
