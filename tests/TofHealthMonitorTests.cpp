#include "../Cntrl_V2/TofHealthMonitor.h"

#include <assert.h>
#include <stdint.h>

static void testStartupAndFreshnessTimeouts()
{
    TofHealthMonitor health;
    health.reset(100U);
    assert(!health.shouldDeclareFault(1099U));
    assert(health.shouldDeclareFault(1100U));

    health.reset(0U);
    TofSampleResult sample = health.noteMeasurement(10U, 50U, 1U, 600U);
    assert(sample.freshFrame);
    assert(sample.usable);
    assert(sample.filterUpdated);
    assert(sample.filteredDistanceMm == 600U);
    assert(!health.shouldDeclareFault(309U));
    assert(health.shouldDeclareFault(310U));
}

static void testFrozenFrameIsNotFresh()
{
    TofHealthMonitor health;
    health.reset(0U);
    const TofSampleResult first = health.noteMeasurement(0U, 1234U, 1U, 500U);
    assert(first.freshFrame);
    const TofSampleResult repeated = health.noteMeasurement(100U, 1234U, 1U, 500U);
    assert(!repeated.freshFrame);
    assert(!repeated.usable);
    assert(!health.shouldDeclareFault(299U));
    assert(health.shouldDeclareFault(300U));
}

static void testInvalidMeasurementsDoNotPolluteFilter()
{
    TofHealthMonitor health;
    health.reset(0U);
    const TofSampleResult first = health.noteMeasurement(0U, 10U, 1U, 700U);
    assert(first.filteredDistanceMm == 700U);

    const TofSampleResult invalid = health.noteMeasurement(32U, 42U, 0U, 1U);
    assert(invalid.freshFrame);
    assert(!invalid.measurementValid);
    assert(!invalid.filterUpdated);
    assert(health.filteredDistanceMm() == 700U);
    assert(health.totalInvalidMeasurements() == 1U);
    health.noteMeasurement(301U, 311U, 0U, 0U);
    assert(!health.outputValid(301U));

    TofHealthMonitor distant;
    distant.reset(0U);
    const TofSampleResult saturated = distant.noteMeasurement(0U, 1U, 1U, 8000U);
    assert(saturated.usable);
    assert(saturated.filteredDistanceMm == TofHealthMonitor::kMaximumDistanceMm);
}

static void testSensorRestartResetsEvidenceAndFilter()
{
    TofHealthMonitor health;
    health.reset(0U);
    const TofSampleResult first = health.noteMeasurement(0U, 1000U, 1U, 400U);
    assert(first.filteredDistanceMm == 400U);

    const TofSampleResult restarted = health.noteMeasurement(32U, 10U, 1U, 900U);
    assert(restarted.sensorRestarted);
    assert(restarted.freshFrame);
    assert(restarted.filteredDistanceMm == 900U);
    assert(health.totalSensorRestarts() == 1U);
    assert(health.freshRatePercent() == 100U);
    assert(health.usableRatePercent() == 100U);
}

static void testSensorTimeWrapIsNotRestart()
{
    TofHealthMonitor health;
    health.reset(0U);
    health.noteMeasurement(0U, 0xFFFFFFF0UL, 1U, 500U);
    const TofSampleResult wrapped = health.noteMeasurement(32U, 5U, 1U, 500U);
    assert(wrapped.freshFrame);
    assert(!wrapped.sensorRestarted);
}

static void testRecoveryRequiresFullHealthyWindow()
{
    TofHealthMonitor health;
    health.reset(0U);
    health.markFaulted(1000U);

    uint32_t now = 1000U;
    for (uint8_t i = 0U; i < TofHealthMonitor::kWindowSize - 1U; ++i)
    {
        now += 32U;
        health.noteMeasurement(now, static_cast<uint32_t>(i + 1U) * 10U, 1U, 500U);
        assert(!health.recoveryQualified(now));
    }
    now += 32U;
    health.noteMeasurement(now, 160U, 1U, 500U);
    assert(health.recoveryQualified(now));
    assert(!health.outputValid(now));

    health.confirmRecovery();
    assert(health.outputValid(now));
    assert(health.totalRecoveries() == 1U);
}

static void testRecoveryRatesAllowLimitedInvalidReadings()
{
    TofHealthMonitor health;
    health.reset(0U);
    health.markFaulted(100U);

    uint32_t now = 100U;
    for (uint8_t i = 0U; i < TofHealthMonitor::kWindowSize; ++i)
    {
        now += 32U;
        const bool usable = i >= 4U;
        health.noteMeasurement(now, static_cast<uint32_t>(i + 1U) * 10U, usable ? 1U : 0U, usable ? 500U : 0U);
    }
    assert(health.freshRatePercent() == 100U);
    assert(health.usableRatePercent() == 75U);
    assert(health.usableStreak() == 12U);
    assert(health.recoveryQualified(now));

    TofHealthMonitor weak;
    weak.reset(0U);
    weak.markFaulted(100U);
    now = 100U;
    for (uint8_t i = 0U; i < TofHealthMonitor::kWindowSize; ++i)
    {
        now += 32U;
        const bool usable = i >= 5U;
        weak.noteMeasurement(now, static_cast<uint32_t>(i + 1U) * 10U, usable ? 1U : 0U, usable ? 500U : 0U);
    }
    assert(weak.usableRatePercent() == 68U);
    assert(!weak.recoveryQualified(now));
}

static void testRecoveryFreshRateRejectsRepeatedTransportLosses()
{
    TofHealthMonitor health;
    health.reset(0U);
    health.markFaulted(100U);

    uint32_t now = 100U;
    health.noteTransportFailure(now);
    for (uint8_t i = 1U; i < TofHealthMonitor::kWindowSize; ++i)
    {
        now += 32U;
        health.noteMeasurement(now, static_cast<uint32_t>(i) * 10U, 1U, 500U);
    }
    assert(health.freshRatePercent() == 93U);
    assert(health.recoveryQualified(now));

    TofHealthMonitor weak;
    weak.reset(0U);
    weak.markFaulted(100U);
    now = 100U;
    weak.noteTransportFailure(now);
    weak.noteTransportFailure(now + 1U);
    for (uint8_t i = 2U; i < TofHealthMonitor::kWindowSize; ++i)
    {
        now += 32U;
        weak.noteMeasurement(now, static_cast<uint32_t>(i) * 10U, 1U, 500U);
    }
    assert(weak.freshRatePercent() == 87U);
    assert(!weak.recoveryQualified(now));
}

static void testFreshInvalidFramesDegradeWithoutFalseSensorFault()
{
    TofHealthMonitor health;
    health.reset(0U);
    uint32_t now = 0U;
    for (uint8_t i = 0U; i < 20U; ++i)
    {
        now += 32U;
        health.noteMeasurement(now, static_cast<uint32_t>(i + 1U) * 10U, 0U, 0U);
    }
    assert(health.state() == TofHealthState::Degraded);
    assert(!health.shouldDeclareFault(now));
    assert(!health.outputValid(now));
}

static void testPausedTimeIsExcluded()
{
    TofHealthMonitor health;
    health.reset(0U);
    health.noteMeasurement(100U, 10U, 1U, 500U);
    health.excludePausedTime(1000U);
    assert(!health.shouldDeclareFault(1399U));
    assert(health.shouldDeclareFault(1400U));
}

int main()
{
    testStartupAndFreshnessTimeouts();
    testFrozenFrameIsNotFresh();
    testInvalidMeasurementsDoNotPolluteFilter();
    testSensorRestartResetsEvidenceAndFilter();
    testSensorTimeWrapIsNotRestart();
    testRecoveryRequiresFullHealthyWindow();
    testRecoveryRatesAllowLimitedInvalidReadings();
    testRecoveryFreshRateRejectsRepeatedTransportLosses();
    testFreshInvalidFramesDegradeWithoutFalseSensorFault();
    testPausedTimeIsExcluded();
    return 0;
}
