#pragma once

#include "As5600HealthMonitor.h"
#include "ShuttleProtocol.h"
#include <Arduino.h>
#include <Wire.h>

class As5600Sensor
{
  public:
    using BusReadyCallback = bool (*)(uint32_t nowMs);

    static constexpr uint32_t kServiceIntervalMs       = 250U;
    static constexpr uint32_t kHealthPublishIntervalMs = 5000U;
    static constexpr uint32_t kFailureLogIntervalMs    = 30000U;

    As5600Sensor(TwoWire &wire, BusReadyCallback busReady);

    bool begin(uint32_t nowMs);
    void service(uint32_t nowMs);

    bool readAngle(uint16_t *rawAngle);
    bool verifyRecovery(uint32_t nowMs);
    uint8_t probeAddress(uint32_t nowMs);
    bool probeForBusVote();

    bool     shouldDeclareFault(uint32_t nowMs) const;
    void     forceFault(uint8_t i2cStatus);
    bool     angleValid(uint32_t nowMs) const;
    uint16_t telemetryAngle(uint32_t nowMs) const;
    uint8_t  reportedState() const;

    uint32_t totalReadFailures() const;
    uint8_t  consecutiveFailures() const;
    uint8_t  lastI2cStatus() const;
    uint32_t lastGoodSampleAgeMs(uint32_t nowMs) const;
    uint16_t lastGoodAngleRaw() const;
    bool     hasLastGoodAngle() const;

    void populateHealthPacket(As5600HealthPacket *packet, uint32_t nowMs, bool faultLatched) const;

  private:
    static constexpr uint8_t kI2cAddress           = 0x36U;
    static constexpr uint8_t kDirectionPin         = 4U;
    static constexpr uint8_t kStatusReg            = 0x0BU;
    static constexpr uint8_t kAngleHighReg         = 0x0EU;
    static constexpr uint8_t kAgcReg               = 0x1AU;
    static constexpr uint8_t kMagnitudeHighReg     = 0x1BU;
    static constexpr uint8_t kAngleReadLength      = 2U;
    static constexpr uint8_t kMotionReadAttempts   = 2U;
    static constexpr uint8_t kStatusMagnetHigh     = 0x08U;
    static constexpr uint8_t kStatusMagnetLow      = 0x10U;
    static constexpr uint8_t kStatusMagnetDetected = 0x20U;

    enum LocalI2cStatus : uint8_t
    {
        I2C_INVALID_ARGUMENT = 0xF0U,
        I2C_BUS_UNAVAILABLE  = 0xF1U,
        I2C_POINTER_WRITE    = 0xF2U,
        I2C_SHORT_READ       = 0xF3U
    };

    bool ensureBusReady(uint32_t nowMs) const;
    bool readRegisters(uint8_t reg, uint8_t *data, uint8_t length, uint8_t *i2cStatus);
    bool sampleHealth(uint32_t nowMs);
    void recordReadFailure(uint8_t i2cStatus);
    bool readAngleOnce(uint16_t *rawAngle, uint8_t *i2cStatus);

    TwoWire             &_wire;
    BusReadyCallback     _busReady;
    As5600HealthMonitor  _health;
    uint32_t             _lastServiceMs;
    uint16_t             _lastGoodAngleRaw;
    uint16_t             _magnitude;
    uint8_t              _statusRegister;
    uint8_t              _agc;
    bool                 _lastGoodAngleValid;
    bool                 _responding;
    bool                 _diagnosticsValid;
};
