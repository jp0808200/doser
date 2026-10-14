// Human-readable names for the core enums, used by the web UI (Step 8),
// the MQTT payloads (Step 9) and the HIL console. Plain lookup tables in
// headers: no allocation, no tasks.
#ifndef ACID_DOSE_NAMES_H
#define ACID_DOSE_NAMES_H

#include "types.h"

inline const char *systemStateName(SystemState s) {
    switch (s) {
        case SystemState::POWER_ON:      return "POWER_ON";
        case SystemState::IDLE:          return "IDLE";
        case SystemState::COUNTING:      return "COUNTING";
        case SystemState::DOSE_READY:    return "DOSE_READY";
        case SystemState::DOSING:        return "DOSING";
        case SystemState::POST_DOSE:     return "POST_DOSE";
        case SystemState::CALIBRATION:   return "CALIBRATION";
        case SystemState::ERROR:         return "ERROR";
        case SystemState::SAFE_SHUTDOWN: return "SAFE_SHUTDOWN";
        default:                         return "UNKNOWN";
    }
}

inline const char *faultCodeName(FaultCode c) {
    switch (c) {
        case FaultCode::NONE:              return "NONE";
        case FaultCode::TANK_EMPTY:        return "TANK_EMPTY";
        case FaultCode::DRIVER_FAULT:      return "DRIVER_FAULT";
        case FaultCode::LINE_NO_ACID:      return "LINE_NO_ACID";
        case FaultCode::LINE_NOT_EMPTY:    return "LINE_NOT_EMPTY";
        case FaultCode::DOSE_TIMEOUT:      return "DOSE_TIMEOUT";
        case FaultCode::FLOW_NO_CAL:       return "FLOW_NO_CAL";
        case FaultCode::PUMP_NO_CAL:       return "PUMP_NO_CAL";
        case FaultCode::NVS_WRITE_ERROR:   return "NVS_WRITE_ERROR";
        case FaultCode::WATCHDOG_RESET:    return "WATCHDOG_RESET";
        case FaultCode::CAL_INVALID_INPUT: return "CAL_INVALID_INPUT";
        case FaultCode::MAX_DOSE_EXCEEDED: return "MAX_DOSE_EXCEEDED";
        case FaultCode::UNKNOWN:           return "UNKNOWN";
        default:                           return "UNKNOWN";
    }
}

inline const char *severityName(AlertSeverity s) {
    switch (s) {
        case AlertSeverity::INFO:    return "info";
        case AlertSeverity::WARNING: return "warning";
        case AlertSeverity::ERROR:   return "error";
        default:                     return "info";
    }
}

#endif /* ACID_DOSE_NAMES_H */
