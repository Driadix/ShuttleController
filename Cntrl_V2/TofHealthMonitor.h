#pragma once

#include <stdint.h>

enum class TofHealthState : uint8_t
{
    Starting,
    Healthy,
    Degraded,
    Faulted,
    Recovering
};

struct TofSampleResult
{
    bool     freshFrame;
    bool     measurementValid;
    bool     usable;
    bool     sensorRestarted;
    bool     filterUpdated;
    uint16_t filteredDistanceMm;
};

class TofHealthMonitor
{
  public:
    static constexpr uint8_t  kWindowSize             = 16U;
    static constexpr uint8_t  kFilterSize             = 5U;
    static constexpr uint16_t kMaximumDistanceMm      = 1500U;
    static constexpr uint32_t kStartupGraceMs         = 1000U;
    static constexpr uint32_t kFreshFrameTimeoutMs    = 300U;
    static constexpr uint32_t kOutputFreshTimeoutMs   = 300U;
    static constexpr uint32_t kRecoveryUsableMaxAgeMs = 100U;
    static constexpr uint8_t  kRecoveryFreshMinimum   = 15U;
    static constexpr uint8_t  kRecoveryUsableMinimum  = 12U;
    static constexpr uint8_t  kRecoveryUsableStreak   = 6U;

    TofHealthMonitor();

    void            reset(uint32_t nowMs);
    void            excludePausedTime(uint32_t pausedMs);
    void            noteTransportFailure(uint32_t nowMs);
    TofSampleResult noteMeasurement(uint32_t nowMs, uint32_t sensorTimeMs, uint16_t status, uint32_t distanceMm);

    bool shouldDeclareFault(uint32_t nowMs) const;
    void markFaulted(uint32_t nowMs);
    bool recoveryQualified(uint32_t nowMs) const;
    void confirmRecovery();

    bool           outputValid(uint32_t nowMs) const;
    uint16_t       filteredDistanceMm() const;
    TofHealthState state() const;
    uint8_t        freshRatePercent() const;
    uint8_t        usableRatePercent() const;
    uint8_t        usableStreak() const;
    uint32_t       freshFrameAgeMs(uint32_t nowMs) const;
    uint32_t       usableSampleAgeMs(uint32_t nowMs) const;
    uint32_t       totalPolls() const;
    uint32_t       totalTransportFailures() const;
    uint32_t       totalInvalidMeasurements() const;
    uint32_t       totalSensorRestarts() const;
    uint32_t       totalRecoveries() const;

  private:
    static uint8_t populationCount(uint16_t value);
    static uint8_t ratePercent(uint16_t window, uint8_t samples);

    void     clearWindow();
    void     clearFilter();
    void     pushWindow(bool fresh, bool usable);
    void     updateState();
    uint16_t updateFilter(uint16_t distanceMm);

    TofHealthState _state;
    uint32_t       _startedAtMs;
    uint32_t       _lastFreshFrameMs;
    uint32_t       _lastUsableSampleMs;
    uint32_t       _lastSensorTimeMs;
    bool           _hasFreshFrame;
    bool           _hasUsableSample;
    bool           _hasSensorTime;
    bool           _faultLatched;
    uint16_t       _freshWindow;
    uint16_t       _usableWindow;
    uint8_t        _windowSamples;
    uint8_t        _usableStreak;
    uint16_t       _filter[kFilterSize];
    uint8_t        _filterIndex;
    bool           _filterPrimed;
    uint16_t       _filteredDistanceMm;
    uint32_t       _totalPolls;
    uint32_t       _totalTransportFailures;
    uint32_t       _totalInvalidMeasurements;
    uint32_t       _totalSensorRestarts;
    uint32_t       _totalRecoveries;
};
