#pragma once

#include <stdint.h>

enum class As5600HealthState : uint8_t
{
    Starting = 0,
    Healthy,
    Degraded,
    Faulted,
    Recovering
};

class As5600HealthMonitor
{
  public:
    static constexpr uint8_t  kFailureThreshold       = 3U;
    static constexpr uint8_t  kRecoverySuccesses      = 3U;
    static constexpr uint32_t kFreshSampleTimeoutMs   = 1000U;

    As5600HealthMonitor()
    {
        reset(0U);
    }

    void reset(uint32_t nowMs)
    {
        _state                = As5600HealthState::Starting;
        _startedAtMs          = nowMs;
        _lastGoodSampleMs     = 0U;
        _hasGoodSample        = false;
        _recoveringFromFault  = false;
        _consecutiveFailures  = 0U;
        _consecutiveSuccesses = 0U;
        _totalFailures        = 0U;
        _totalRecoveries      = 0U;
        _lastI2cStatus        = 0U;
    }

    void noteSuccess(uint32_t nowMs)
    {
        _lastGoodSampleMs = nowMs;
        _hasGoodSample    = true;
        _lastI2cStatus    = 0U;
        _consecutiveFailures = 0U;
        if (_consecutiveSuccesses < 0xFFU)
        {
            _consecutiveSuccesses++;
        }

        if (_recoveringFromFault)
        {
            if (_consecutiveSuccesses >= kRecoverySuccesses)
            {
                _state = As5600HealthState::Healthy;
                _recoveringFromFault = false;
                _totalRecoveries++;
            }
            else
            {
                _state = As5600HealthState::Recovering;
            }
            return;
        }

        _state = As5600HealthState::Healthy;
    }

    void noteFailure(uint8_t i2cStatus)
    {
        const bool wasFaulted = _recoveringFromFault || _state == As5600HealthState::Faulted ||
                                _state == As5600HealthState::Recovering;
        _lastI2cStatus        = i2cStatus;
        _consecutiveSuccesses = 0U;
        if (_consecutiveFailures < 0xFFU)
        {
            _consecutiveFailures++;
        }
        if (_totalFailures < 0xFFFFFFFFUL)
        {
            _totalFailures++;
        }

        if (wasFaulted || _consecutiveFailures >= kFailureThreshold)
        {
            _recoveringFromFault = true;
            _state = As5600HealthState::Faulted;
        }
        else
        {
            _state = As5600HealthState::Degraded;
        }
    }

    void forceFault(uint8_t i2cStatus)
    {
        _lastI2cStatus        = i2cStatus;
        _consecutiveSuccesses = 0U;
        if (_consecutiveFailures < kFailureThreshold)
        {
            _consecutiveFailures = kFailureThreshold;
        }
        _state = As5600HealthState::Faulted;
        _recoveringFromFault = true;
    }

    bool shouldDeclareFault(uint32_t nowMs) const
    {
        if (_state == As5600HealthState::Faulted)
        {
            return true;
        }
        return _hasGoodSample && (nowMs - _lastGoodSampleMs >= kFreshSampleTimeoutMs);
    }

    bool outputValid(uint32_t nowMs) const
    {
        return _state == As5600HealthState::Healthy && _hasGoodSample &&
               (nowMs - _lastGoodSampleMs < kFreshSampleTimeoutMs);
    }

    As5600HealthState state() const { return _state; }
    bool hasGoodSample() const { return _hasGoodSample; }
    uint8_t consecutiveFailures() const { return _consecutiveFailures; }
    uint8_t consecutiveSuccesses() const { return _consecutiveSuccesses; }
    uint32_t totalFailures() const { return _totalFailures; }
    uint32_t totalRecoveries() const { return _totalRecoveries; }
    uint8_t lastI2cStatus() const { return _lastI2cStatus; }

    uint32_t lastGoodSampleAgeMs(uint32_t nowMs) const
    {
        return _hasGoodSample ? nowMs - _lastGoodSampleMs : 0xFFFFFFFFUL;
    }

  private:
    As5600HealthState _state;
    uint32_t          _startedAtMs;
    uint32_t          _lastGoodSampleMs;
    bool              _hasGoodSample;
    bool              _recoveringFromFault;
    uint8_t           _consecutiveFailures;
    uint8_t           _consecutiveSuccesses;
    uint32_t          _totalFailures;
    uint32_t          _totalRecoveries;
    uint8_t           _lastI2cStatus;
};
