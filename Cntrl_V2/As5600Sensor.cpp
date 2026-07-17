#include "As5600Sensor.h"

As5600Sensor::As5600Sensor(TwoWire &wire, BusReadyCallback busReady)
    : _wire(wire), _busReady(busReady), _lastServiceMs(0U), _lastGoodAngleRaw(0U), _magnitude(0U),
      _statusRegister(0U), _agc(0U), _lastGoodAngleValid(false), _responding(false),
      _diagnosticsValid(false)
{
}

bool As5600Sensor::begin(uint32_t nowMs)
{
    pinMode(kDirectionPin, OUTPUT);
    digitalWrite(kDirectionPin, LOW);
    _health.reset(nowMs);
    _lastServiceMs       = 0U;
    _lastGoodAngleRaw    = 0U;
    _magnitude           = 0U;
    _statusRegister      = 0U;
    _agc                 = 0U;
    _lastGoodAngleValid  = false;
    _responding          = false;
    _diagnosticsValid    = false;
    return sampleHealth(nowMs);
}

void As5600Sensor::service(uint32_t nowMs)
{
    if (_lastServiceMs != 0U && nowMs - _lastServiceMs < kServiceIntervalMs)
    {
        return;
    }
    _lastServiceMs = (nowMs == 0U) ? 1U : nowMs;
    (void)sampleHealth(nowMs);
}

bool As5600Sensor::ensureBusReady(uint32_t nowMs) const
{
    return _busReady == nullptr || _busReady(nowMs);
}

bool As5600Sensor::readRegisters(uint8_t reg, uint8_t *data, uint8_t length, uint8_t *i2cStatus)
{
    if (data == nullptr || length == 0U)
    {
        if (i2cStatus != nullptr)
            *i2cStatus = I2C_INVALID_ARGUMENT;
        return false;
    }
    _wire.beginTransmission(kI2cAddress);
    if (_wire.write(reg) != 1U)
    {
        (void)_wire.endTransmission(true);
        if (i2cStatus != nullptr)
            *i2cStatus = I2C_POINTER_WRITE;
        return false;
    }

    const uint8_t txStatus = _wire.endTransmission(true);
    if (txStatus != 0U)
    {
        if (i2cStatus != nullptr)
            *i2cStatus = txStatus;
        return false;
    }

    const uint8_t requested = _wire.requestFrom(kI2cAddress, length);
    uint8_t       received  = 0U;
    while (_wire.available() > 0 && received < length)
    {
        data[received++] = (uint8_t)_wire.read();
    }
    while (_wire.available() > 0)
    {
        (void)_wire.read();
    }
    if (requested != length || received != length)
    {
        if (i2cStatus != nullptr)
            *i2cStatus = I2C_SHORT_READ;
        return false;
    }

    if (i2cStatus != nullptr)
        *i2cStatus = 0U;
    return true;
}

void As5600Sensor::recordReadFailure(uint8_t i2cStatus)
{
    _responding = false;
    _health.noteFailure(i2cStatus);
}

bool As5600Sensor::sampleHealth(uint32_t nowMs)
{
    uint8_t angleBytes[2]     = { 0U, 0U };
    uint8_t statusValue       = 0U;
    uint8_t agcValue          = 0U;
    uint8_t magnitudeBytes[2] = { 0U, 0U };
    uint8_t i2cStatus         = 0U;

    // Qualify the shared bus once for the complete sample. Checking the physical
    // line state between consecutive STOP/START operations can observe SDA while
    // it is still settling and incorrectly reject the remaining register reads.
    if (!ensureBusReady(nowMs))
    {
        recordReadFailure(I2C_BUS_UNAVAILABLE);
        return false;
    }

    if (!readRegisters(kAngleHighReg, angleBytes, 2U, &i2cStatus))
    {
        recordReadFailure(i2cStatus);
        return false;
    }

    _lastGoodAngleRaw =
        (uint16_t)((((uint16_t)angleBytes[0] << 8) | angleBytes[1]) & 0x0FFFU);
    _lastGoodAngleValid = true;
    _responding         = true;

    // STATUS, AGC and MAGNITUDE are useful diagnostics, but angle availability
    // is the operational requirement. Do not fault motion because an optional
    // diagnostic read failed or because reserved diagnostic bits vary by device.
    uint8_t diagnosticI2cStatus = 0U;
    _diagnosticsValid = readRegisters(kStatusReg, &statusValue, 1U, &diagnosticI2cStatus) &&
                        readRegisters(kAgcReg, &agcValue, 1U, &diagnosticI2cStatus) &&
                        readRegisters(kMagnitudeHighReg, magnitudeBytes, 2U, &diagnosticI2cStatus);
    if (_diagnosticsValid)
    {
        _statusRegister = statusValue;
        _agc            = agcValue;
        _magnitude      =
            (uint16_t)((((uint16_t)magnitudeBytes[0] << 8) | magnitudeBytes[1]) & 0x0FFFU);
    }
    else
    {
        _statusRegister = 0U;
        _agc            = 0U;
        _magnitude      = 0U;
    }

    _health.noteSuccess(nowMs);
    return true;
}

bool As5600Sensor::readAngleOnce(uint16_t *rawAngle, uint8_t *i2cStatus)
{
    uint8_t bytes[2] = { 0U, 0U };
    if (!readRegisters(kAngleHighReg, bytes, kAngleReadLength, i2cStatus))
    {
        return false;
    }

    *rawAngle = (uint16_t)((((uint16_t)bytes[0] << 8) | bytes[1]) & 0x0FFFU);
    _lastGoodAngleRaw      = *rawAngle;
    _lastGoodAngleValid    = true;
    _responding            = true;
    return true;
}

bool As5600Sensor::readAngle(uint16_t *rawAngle)
{
    if (rawAngle == nullptr)
    {
        return false;
    }
    if (!ensureBusReady(millis()))
    {
        recordReadFailure(I2C_BUS_UNAVAILABLE);
        return false;
    }

    for (uint8_t attempt = 0U; attempt < kMotionReadAttempts; ++attempt)
    {
        uint8_t i2cStatus = 0U;
        if (readAngleOnce(rawAngle, &i2cStatus))
        {
            return true;
        }
        recordReadFailure(i2cStatus);
    }
    return false;
}

bool As5600Sensor::verifyRecovery(uint32_t nowMs)
{
    for (uint8_t i = 0U; i < As5600HealthMonitor::kRecoverySuccesses; ++i)
    {
        if (!sampleHealth(nowMs + i))
        {
            return false;
        }
        delay(2);
    }
    return _health.outputValid(millis());
}

uint8_t As5600Sensor::probeAddress(uint32_t nowMs)
{
    if (!ensureBusReady(nowMs))
    {
        return I2C_BUS_UNAVAILABLE;
    }

    _wire.beginTransmission(kI2cAddress);
    return _wire.endTransmission(true);
}

bool As5600Sensor::probeForBusVote()
{
    if (!ensureBusReady(millis()))
    {
        return false;
    }

    uint16_t angle     = 0U;
    uint8_t  i2cStatus = 0U;
    return readAngleOnce(&angle, &i2cStatus);
}

bool As5600Sensor::shouldDeclareFault(uint32_t nowMs) const
{
    return _health.shouldDeclareFault(nowMs);
}

void As5600Sensor::forceFault(uint8_t i2cStatus)
{
    _health.forceFault(i2cStatus);
}

bool As5600Sensor::angleValid(uint32_t nowMs) const
{
    return _health.outputValid(nowMs) && _lastGoodAngleValid;
}

uint16_t As5600Sensor::telemetryAngle(uint32_t nowMs) const
{
    return angleValid(nowMs) ? _lastGoodAngleRaw : 0U;
}

uint8_t As5600Sensor::reportedState() const
{
    return (uint8_t)_health.state();
}

uint32_t As5600Sensor::totalReadFailures() const
{
    return _health.totalFailures();
}

uint32_t As5600Sensor::totalRecoveries() const
{
    return _health.totalRecoveries();
}

uint8_t As5600Sensor::consecutiveFailures() const
{
    return _health.consecutiveFailures();
}

uint8_t As5600Sensor::consecutiveSuccesses() const
{
    return _health.consecutiveSuccesses();
}

uint8_t As5600Sensor::lastI2cStatus() const
{
    return _health.lastI2cStatus();
}

uint32_t As5600Sensor::lastGoodSampleAgeMs(uint32_t nowMs) const
{
    return _health.lastGoodSampleAgeMs(nowMs);
}

uint16_t As5600Sensor::lastGoodAngleRaw() const
{
    return _lastGoodAngleRaw;
}

bool As5600Sensor::hasLastGoodAngle() const
{
    return _lastGoodAngleValid;
}

void As5600Sensor::populateHealthPacket(As5600HealthPacket *packet, uint32_t nowMs, bool faultLatched) const
{
    if (packet == nullptr)
        return;

    packet->lastGoodSampleAgeMs = _health.lastGoodSampleAgeMs(nowMs);
    packet->totalReadFailures   = _health.totalFailures();
    packet->angleRaw            = _lastGoodAngleRaw;
    packet->magnitude           = _magnitude;
    packet->recoveryCount       =
        (_health.totalRecoveries() > 0xFFFFUL) ? 0xFFFFU : (uint16_t)_health.totalRecoveries();
    packet->state               = (uint8_t)_health.state();
    packet->flags               = 0U;
    packet->statusRegister      = _statusRegister;
    packet->agc                 = _agc;
    packet->consecutiveFailures  = _health.consecutiveFailures();
    packet->consecutiveSuccesses = _health.consecutiveSuccesses();
    packet->lastI2cStatus       = _health.lastI2cStatus();

    if (_responding)
        packet->flags |= AS5600_FLAG_RESPONDING;
    if (angleValid(nowMs))
        packet->flags |= AS5600_FLAG_ANGLE_VALID;
    if (_diagnosticsValid && (_statusRegister & kStatusMagnetDetected) != 0U)
        packet->flags |= AS5600_FLAG_MAGNET_DETECTED;
    if (_diagnosticsValid && (_statusRegister & kStatusMagnetLow) != 0U)
        packet->flags |= AS5600_FLAG_MAGNET_TOO_WEAK;
    if (_diagnosticsValid && (_statusRegister & kStatusMagnetHigh) != 0U)
        packet->flags |= AS5600_FLAG_MAGNET_TOO_STRONG;
    if (faultLatched)
        packet->flags |= AS5600_FLAG_FAULT_LATCHED;
}
