#include "../Cntrl_V2/As5600HealthMonitor.h"

#include <assert.h>

static void testFailureDebounceAndRecoveryQualification()
{
    As5600HealthMonitor health;
    health.reset(0U);
    health.noteSuccess(10U);
    assert(health.state() == As5600HealthState::Healthy);

    health.noteFailure(2U);
    health.noteFailure(2U);
    assert(health.state() == As5600HealthState::Degraded);
    assert(!health.shouldDeclareFault(20U));

    health.noteFailure(2U);
    assert(health.state() == As5600HealthState::Faulted);
    assert(health.shouldDeclareFault(30U));

    health.noteSuccess(40U);
    health.noteSuccess(50U);
    assert(health.state() == As5600HealthState::Recovering);
    health.noteFailure(2U);
    assert(health.state() == As5600HealthState::Faulted);
    health.noteSuccess(51U);
    health.noteSuccess(52U);
    assert(health.state() == As5600HealthState::Recovering);
    health.noteSuccess(60U);
    assert(health.state() == As5600HealthState::Healthy);
    assert(health.totalRecoveries() == 1U);
}

static void testFreshnessAndForcedMotionFault()
{
    As5600HealthMonitor health;
    health.reset(100U);
    health.noteSuccess(200U);
    assert(health.outputValid(1199U));
    assert(!health.outputValid(1200U));
    assert(health.shouldDeclareFault(1200U));

    health.forceFault(0xFEU);
    assert(health.state() == As5600HealthState::Faulted);
    assert(health.consecutiveFailures() == As5600HealthMonitor::kFailureThreshold);
    assert(health.lastI2cStatus() == 0xFEU);

    health.noteSuccess(1201U);
    health.noteSuccess(1202U);
    assert(health.state() == As5600HealthState::Recovering);
    assert(!health.outputValid(1202U));
    health.noteSuccess(1203U);
    assert(health.state() == As5600HealthState::Healthy);
    assert(health.outputValid(1203U));
    assert(health.totalRecoveries() == 1U);
}

int main()
{
    testFailureDebounceAndRecoveryQualification();
    testFreshnessAndForcedMotionFault();
    return 0;
}
