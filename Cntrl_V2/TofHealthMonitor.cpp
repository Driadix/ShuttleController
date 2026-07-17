#include "TofHealthMonitor.h"

TofHealthMonitor::TofHealthMonitor()
{
    reset(0U);
}

void TofHealthMonitor::reset(uint32_t nowMs)
{
    _state                    = TofHealthState::Starting;
    _startedAtMs              = nowMs;
    _lastFreshFrameMs         = nowMs;
    _lastUsableSampleMs       = nowMs;
    _lastSensorTimeMs         = 0U;
    _hasFreshFrame            = false;
    _hasUsableSample          = false;
    _hasSensorTime            = false;
    _faultLatched             = false;
    _faultCause               = TofFaultCause::None;
    _lastMeasurementStatus    = 0U;
    _lastSignalStrength       = 0U;
    _totalPolls               = 0U;
    _totalTransportFailures   = 0U;
    _totalInvalidMeasurements = 0U;
    _totalSensorRestarts      = 0U;
    _totalRecoveries          = 0U;
    _pollsAtFault               = 0U;
    _transportFailuresAtFault   = 0U;
    _invalidMeasurementsAtFault = 0U;
    _sensorRestartsAtFault      = 0U;
    clearWindow();
    clearFilter();
}

void TofHealthMonitor::excludePausedTime(uint32_t pausedMs)
{
    _startedAtMs += pausedMs;
    if (_hasFreshFrame)
    {
        _lastFreshFrameMs += pausedMs;
    }
    if (_hasUsableSample)
    {
        _lastUsableSampleMs += pausedMs;
    }
}

void TofHealthMonitor::noteTransportFailure(uint32_t nowMs)
{
    (void)nowMs;
    ++_totalPolls;
    ++_totalTransportFailures;
    pushWindow(false, false);
    updateState();
}

TofSampleResult
TofHealthMonitor::noteMeasurement(
    uint32_t nowMs,
    uint32_t sensorTimeMs,
    uint16_t status,
    uint32_t distanceMm,
    uint16_t signalStrength)
{
    TofSampleResult result = {};
    ++_totalPolls;

    if (!_hasSensorTime)
    {
        result.freshFrame = true;
        _hasSensorTime    = true;
    }
    else if (sensorTimeMs != _lastSensorTimeMs)
    {
        if (sensorTimeMs < _lastSensorTimeMs && _lastSensorTimeMs - sensorTimeMs < 0x80000000UL)
        {
            result.sensorRestarted = true;
            ++_totalSensorRestarts;
            clearWindow();
            clearFilter();
            _startedAtMs     = nowMs;
            _state           = _faultLatched ? TofHealthState::Recovering : TofHealthState::Starting;
            _hasFreshFrame   = false;
            _hasUsableSample = false;
        }
        result.freshFrame = true;
    }

    _lastSensorTimeMs      = sensorTimeMs;
    _lastMeasurementStatus = status;
    _lastSignalStrength    = signalStrength;
    if (result.freshFrame)
    {
        _lastFreshFrameMs = nowMs;
        _hasFreshFrame    = true;
    }

    result.measurementValid = status == 1U && distanceMm > 0U;
    result.usable           = result.freshFrame && result.measurementValid;
    if (result.freshFrame && !result.measurementValid)
    {
        ++_totalInvalidMeasurements;
    }

    if (result.usable)
    {
        const uint16_t boundedDistance =
            distanceMm > kMaximumDistanceMm ? kMaximumDistanceMm : static_cast<uint16_t>(distanceMm);
        result.filteredDistanceMm = updateFilter(boundedDistance);
        result.filterUpdated      = true;
        _lastUsableSampleMs       = nowMs;
        _hasUsableSample          = true;
    }
    else
    {
        result.filteredDistanceMm = _filteredDistanceMm;
    }

    pushWindow(result.freshFrame, result.usable);
    if (!result.freshFrame || status > 1U)
    {
        _plausibleStreak = 0U;
    }
    else if (_plausibleStreak < 0xFFU)
    {
        ++_plausibleStreak;
    }
    updateState();
    return result;
}

bool TofHealthMonitor::shouldDeclareFault(uint32_t nowMs) const
{
    if (_faultLatched)
    {
        return false;
    }
    if (!_hasFreshFrame)
    {
        return nowMs - _startedAtMs >= kStartupGraceMs;
    }
    return nowMs - _lastFreshFrameMs >= kFreshFrameTimeoutMs;
}

void TofHealthMonitor::markFaulted(uint32_t nowMs, TofFaultCause cause)
{
    if (!_faultLatched)
    {
        _pollsAtFault               = _totalPolls;
        _transportFailuresAtFault   = _totalTransportFailures;
        _invalidMeasurementsAtFault = _totalInvalidMeasurements;
        _sensorRestartsAtFault      = _totalSensorRestarts;
    }
    _faultLatched = true;
    _faultCause   = cause;
    _state        = TofHealthState::Faulted;
    _startedAtMs  = nowMs;
    clearWindow();
    clearFilter();
    _hasUsableSample = false;
}

bool TofHealthMonitor::recoveryQualified(uint32_t nowMs) const
{
    return recoveryReason(nowMs) != TofRecoveryReason::None;
}

TofRecoveryReason TofHealthMonitor::recoveryReason(uint32_t nowMs) const
{
    if (!_faultLatched)
    {
        return TofRecoveryReason::None;
    }

    const bool measurementQualityReady =
        _windowSamples >= kWindowSize && _hasUsableSample &&
        populationCount(_freshWindow) >= kRecoveryFreshMinimum &&
        populationCount(_usableWindow) >= kRecoveryUsableMinimum && _usableStreak >= kRecoveryUsableStreak &&
        nowMs - _lastUsableSampleMs <= kRecoveryUsableMaxAgeMs;
    if (measurementQualityReady)
    {
        return TofRecoveryReason::MeasurementQuality;
    }

    const bool transportAlive = _faultCause == TofFaultCause::Transport &&
                                _freshStreak >= kLivenessFreshStreak &&
                                _plausibleStreak >= kLivenessFreshStreak && _hasFreshFrame &&
                                nowMs - _lastFreshFrameMs <= kRecoveryUsableMaxAgeMs;
    return transportAlive ? TofRecoveryReason::TransportLiveness : TofRecoveryReason::None;
}

void TofHealthMonitor::confirmRecovery()
{
    if (_faultLatched)
    {
        ++_totalRecoveries;
    }
    _faultLatched = false;
    _faultCause   = TofFaultCause::None;
    updateState();
}

bool TofHealthMonitor::outputValid(uint32_t nowMs) const
{
    return !_faultLatched && _filterPrimed && _hasUsableSample && nowMs - _lastUsableSampleMs <= kOutputFreshTimeoutMs;
}

bool TofHealthMonitor::measurementReady(uint32_t nowMs) const
{
    return outputValid(nowMs) && _windowSamples >= kWindowSize &&
           populationCount(_freshWindow) >= kRecoveryFreshMinimum &&
           populationCount(_usableWindow) >= kRecoveryUsableMinimum && _usableStreak >= kRecoveryUsableStreak;
}

uint16_t TofHealthMonitor::filteredDistanceMm() const
{
    return _filteredDistanceMm;
}

TofHealthState TofHealthMonitor::state() const
{
    return _state;
}

TofFaultCause TofHealthMonitor::faultCause() const
{
    return _faultCause;
}

uint8_t TofHealthMonitor::freshRatePercent() const
{
    return ratePercent(_freshWindow, _windowSamples);
}

uint8_t TofHealthMonitor::usableRatePercent() const
{
    return ratePercent(_usableWindow, _windowSamples);
}

uint8_t TofHealthMonitor::usableStreak() const
{
    return _usableStreak;
}

uint8_t TofHealthMonitor::freshStreak() const
{
    return _freshStreak;
}

uint32_t TofHealthMonitor::freshFrameAgeMs(uint32_t nowMs) const
{
    return _hasFreshFrame ? nowMs - _lastFreshFrameMs : nowMs - _startedAtMs;
}

uint32_t TofHealthMonitor::usableSampleAgeMs(uint32_t nowMs) const
{
    return _hasUsableSample ? nowMs - _lastUsableSampleMs : nowMs - _startedAtMs;
}

uint16_t TofHealthMonitor::lastMeasurementStatus() const
{
    return _lastMeasurementStatus;
}

uint16_t TofHealthMonitor::lastSignalStrength() const
{
    return _lastSignalStrength;
}

uint32_t TofHealthMonitor::totalPolls() const
{
    return _totalPolls;
}

uint32_t TofHealthMonitor::totalTransportFailures() const
{
    return _totalTransportFailures;
}

uint32_t TofHealthMonitor::totalInvalidMeasurements() const
{
    return _totalInvalidMeasurements;
}

uint32_t TofHealthMonitor::totalSensorRestarts() const
{
    return _totalSensorRestarts;
}

uint32_t TofHealthMonitor::totalRecoveries() const
{
    return _totalRecoveries;
}

uint32_t TofHealthMonitor::pollsSinceFault() const
{
    return _totalPolls - _pollsAtFault;
}

uint32_t TofHealthMonitor::transportFailuresSinceFault() const
{
    return _totalTransportFailures - _transportFailuresAtFault;
}

uint32_t TofHealthMonitor::invalidMeasurementsSinceFault() const
{
    return _totalInvalidMeasurements - _invalidMeasurementsAtFault;
}

uint32_t TofHealthMonitor::sensorRestartsSinceFault() const
{
    return _totalSensorRestarts - _sensorRestartsAtFault;
}

uint8_t TofHealthMonitor::populationCount(uint16_t value)
{
    uint8_t count = 0U;
    while (value != 0U)
    {
        count += static_cast<uint8_t>(value & 1U);
        value >>= 1U;
    }
    return count;
}

uint8_t TofHealthMonitor::ratePercent(uint16_t window, uint8_t samples)
{
    return samples == 0U ? 0U : static_cast<uint8_t>((populationCount(window) * 100U) / samples);
}

void TofHealthMonitor::clearWindow()
{
    _freshWindow   = 0U;
    _usableWindow  = 0U;
    _windowSamples = 0U;
    _usableStreak  = 0U;
    _freshStreak     = 0U;
    _plausibleStreak = 0U;
}

void TofHealthMonitor::clearFilter()
{
    for (uint8_t i = 0U; i < kFilterSize; ++i)
    {
        _filter[i] = 0U;
    }
    _filterIndex        = 0U;
    _filterPrimed       = false;
    _filteredDistanceMm = 0U;
}

void TofHealthMonitor::pushWindow(bool fresh, bool usable)
{
    _freshWindow  = static_cast<uint16_t>((_freshWindow << 1U) | (fresh ? 1U : 0U));
    _usableWindow = static_cast<uint16_t>((_usableWindow << 1U) | (usable ? 1U : 0U));
    if (_windowSamples < kWindowSize)
    {
        ++_windowSamples;
    }
    if (!usable)
    {
        _usableStreak = 0U;
    }
    else if (_usableStreak < 0xFFU)
    {
        ++_usableStreak;
    }
    if (!fresh)
    {
        _freshStreak     = 0U;
        _plausibleStreak = 0U;
    }
    else if (_freshStreak < 0xFFU)
    {
        ++_freshStreak;
    }
}

void TofHealthMonitor::updateState()
{
    if (_faultLatched)
    {
        _state = (_freshWindow != 0U || _usableWindow != 0U) ? TofHealthState::Recovering : TofHealthState::Faulted;
        return;
    }
    if (_windowSamples < kWindowSize)
    {
        _state = TofHealthState::Starting;
        return;
    }
    _state =
        populationCount(_freshWindow) >= kRecoveryFreshMinimum && populationCount(_usableWindow) >= kRecoveryUsableMinimum
            ? TofHealthState::Healthy
            : TofHealthState::Degraded;
}

uint16_t TofHealthMonitor::updateFilter(uint16_t distanceMm)
{
    if (!_filterPrimed)
    {
        for (uint8_t i = 0U; i < kFilterSize; ++i)
        {
            _filter[i] = distanceMm;
        }
        _filterPrimed = true;
        _filterIndex  = 0U;
    }
    else
    {
        _filter[_filterIndex] = distanceMm;
        _filterIndex          = static_cast<uint8_t>((_filterIndex + 1U) % kFilterSize);
    }

    uint16_t minimum = _filter[0];
    uint16_t maximum = _filter[0];
    uint32_t sum     = 0U;
    for (uint8_t i = 0U; i < kFilterSize; ++i)
    {
        if (_filter[i] < minimum)
        {
            minimum = _filter[i];
        }
        if (_filter[i] > maximum)
        {
            maximum = _filter[i];
        }
        sum += _filter[i];
    }
    _filteredDistanceMm = static_cast<uint16_t>((sum - minimum - maximum) / (kFilterSize - 2U));
    return _filteredDistanceMm;
}
