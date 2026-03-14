#pragma once

#include "ShuttleProtocol.h"

class AlertManager {
private:
    uint16_t _warningCode = 0;
    uint16_t _errorCode = 0;
    uint32_t _warningTimeouts[16] = { 0 };

public:
    void setFault(ShuttleFault fault) { _errorCode |= (uint16_t)fault; }
    void clearFault(ShuttleFault fault) { _errorCode &= ~(uint16_t)fault; }
    void clearAllFaults() { _errorCode = 0; }
    uint16_t getErrorCode() const { return _errorCode; }

    void setWarning(ShuttleWarning warn, uint32_t timeoutMs, uint32_t currentMillis) {
        uint16_t warnBits = (uint16_t)warn;
        _warningCode |= warnBits;

        for (uint8_t i = 0; i < 16; i++) {
            if (warnBits & (1U << i)) {
                _warningTimeouts[i] = (timeoutMs > 0) ? (currentMillis + timeoutMs) : 0;
            }
        }
    }

    void clearWarning(ShuttleWarning warn) {
        uint16_t warnBits = (uint16_t)warn;
        _warningCode &= ~warnBits;

        for (uint8_t i = 0; i < 16; i++) {
            if (warnBits & (1U << i)) {
                _warningTimeouts[i] = 0;
            }
        }
    }

    void clearAllWarnings() {
        _warningCode = 0;

        for (uint8_t i = 0; i < 16; i++) {
            _warningTimeouts[i] = 0;
        }
    }

    void processTimeouts(uint32_t currentMillis) {
        if (_warningCode == 0) return;

        for (uint8_t i = 0; i < 16; i++) {
            uint16_t bit = (1U << i);
            if ((_warningCode & bit) && _warningTimeouts[i] > 0) {
                if ((int32_t)(currentMillis - _warningTimeouts[i]) >= 0) {
                    _warningCode &= ~bit;
                    _warningTimeouts[i] = 0;
                }
            }
        }
    }

    uint16_t getWarningCode() const { return _warningCode; }
};
