#include "../Cntrl_V2/BmsDdA5.hpp"

#include <assert.h>
#include <deque>
#include <stdint.h>
#include <utility>
#include <vector>

struct FakeTransport
{
    uint32_t nowMs = 0U;
    bool driverEnabled = false;
    std::deque<uint8_t> rx;
    std::vector<std::pair<uint32_t, BmsDdA5::RequestKind> > writes;
};

static size_t fakeWrite(const uint8_t *data, size_t len, void *user)
{
    FakeTransport *fake = static_cast<FakeTransport *>(user);
    assert(fake != 0);
    assert(len == 7U);
    fake->writes.push_back(
        std::make_pair(fake->nowMs, static_cast<BmsDdA5::RequestKind>(data[2])));
    return len;
}

static int fakeRead(void *user)
{
    FakeTransport *fake = static_cast<FakeTransport *>(user);
    if (fake == 0 || fake->rx.empty())
        return -1;
    const uint8_t value = fake->rx.front();
    fake->rx.pop_front();
    return value;
}

static void fakeSetDriverEnable(bool enabled, void *user)
{
    FakeTransport *fake = static_cast<FakeTransport *>(user);
    assert(fake != 0);
    fake->driverEnabled = enabled;
}

static BmsDdA5::Hooks makeHooks(FakeTransport *fake)
{
    BmsDdA5::Hooks hooks;
    hooks.write = fakeWrite;
    hooks.readByte = fakeRead;
    hooks.setDriverEnable = fakeSetDriverEnable;
    hooks.user = fake;
    return hooks;
}

static std::vector<uint8_t> makeResponse(BmsDdA5::RequestKind kind)
{
    uint8_t payloadLen = 1U;
    if (kind == BmsDdA5::RequestKind::BasicInfo)
        payloadLen = 20U;
    else if (kind == BmsDdA5::RequestKind::CellInfo)
        payloadLen = 2U;

    std::vector<uint8_t> frame(static_cast<size_t>(payloadLen) + 7U, 0U);
    frame[0] = 0xDDU;
    frame[1] = static_cast<uint8_t>(kind);
    frame[2] = 0x00U;
    frame[3] = payloadLen;
    if (kind == BmsDdA5::RequestKind::BasicInfo)
        frame[4U + 19U] = 75U;
    else if (kind == BmsDdA5::RequestKind::CellInfo)
    {
        frame[4] = 0x0CU;
        frame[5] = 0xE4U;
    }
    else
        frame[4] = 'B';

    uint16_t checksum = 0U;
    for (uint8_t i = 1U; i < static_cast<uint8_t>(4U + payloadLen); ++i)
        checksum = static_cast<uint16_t>(checksum + frame[i]);
    checksum = static_cast<uint16_t>(~checksum + 1U);
    frame[4U + payloadLen] = static_cast<uint8_t>(checksum >> 8);
    frame[5U + payloadLen] = static_cast<uint8_t>(checksum & 0xFFU);
    frame[6U + payloadLen] = 0x77U;
    return frame;
}

static uint32_t tick(
    BmsDdA5::Client &client, FakeTransport &fake, uint32_t nowMs, BmsDdA5::ActivityHint activity)
{
    fake.nowMs = nowMs;
    return client.tick(nowMs, activity);
}

static void queueResponse(FakeTransport &fake, BmsDdA5::RequestKind kind)
{
    const std::vector<uint8_t> frame = makeResponse(kind);
    fake.rx.insert(fake.rx.end(), frame.begin(), frame.end());
}

static void establishBasic(BmsDdA5::Client &client, FakeTransport &fake)
{
    client.begin(0U);
    assert(tick(client, fake, 999U, BmsDdA5::ActivityHint::Idle) == 0U);
    assert(fake.writes.empty());
    (void)tick(client, fake, 1000U, BmsDdA5::ActivityHint::Idle);
    assert(fake.writes.size() == 1U);
    assert(fake.driverEnabled);
    queueResponse(fake, BmsDdA5::RequestKind::BasicInfo);
    const uint32_t events = tick(client, fake, 1012U, BmsDdA5::ActivityHint::Idle);
    assert(BmsDdA5::hasEvent(events, BmsDdA5::Event::BasicUpdated));
    assert(!fake.driverEnabled);
    assert(client.snapshot().hasBasic);
}

static void testStartupRetry()
{
    FakeTransport fake;
    BmsDdA5::Client client(makeHooks(&fake));
    client.begin(0U);
    (void)tick(client, fake, 999U, BmsDdA5::ActivityHint::Idle);
    assert(fake.writes.empty());
    (void)tick(client, fake, 1000U, BmsDdA5::ActivityHint::Idle);
    assert(fake.writes.size() == 1U);
    (void)tick(client, fake, 1012U, BmsDdA5::ActivityHint::Idle);
    (void)tick(client, fake, 1140U, BmsDdA5::ActivityHint::Idle);
    (void)tick(client, fake, 1999U, BmsDdA5::ActivityHint::Idle);
    assert(fake.writes.size() == 1U);
    (void)tick(client, fake, 2000U, BmsDdA5::ActivityHint::Idle);
    assert(fake.writes.size() == 2U);
}

static void testBasicIntervals()
{
    const BmsDdA5::ActivityHint activities[] = {
        BmsDdA5::ActivityHint::Idle,
        BmsDdA5::ActivityHint::Active,
        BmsDdA5::ActivityHint::HighLoad,
        BmsDdA5::ActivityHint::LowBatteryFault
    };
    const uint32_t dueAt[] = { 6000U, 16000U, 61000U, 6000U };

    for (size_t i = 0U; i < 4U; ++i)
    {
        FakeTransport fake;
        BmsDdA5::Client client(makeHooks(&fake));
        establishBasic(client, fake);
        (void)tick(client, fake, dueAt[i] - 1U, activities[i]);
        assert(fake.writes.size() == 1U);
        (void)tick(client, fake, dueAt[i], activities[i]);
        assert(fake.writes.size() == 2U);
        assert(fake.writes.back().second == BmsDdA5::RequestKind::BasicInfo);
    }
}

static void testIdleAuxiliarySchedule()
{
    FakeTransport fake;
    BmsDdA5::Client client(makeHooks(&fake));
    client.begin(0U);

    bool responsePending = false;
    uint32_t responseAt = 0U;
    BmsDdA5::RequestKind pendingKind = BmsDdA5::RequestKind::BasicInfo;
    uint32_t firstCellMs = 0U;
    uint32_t firstDeviceMs = 0U;
    size_t observedWrites = 0U;

    for (uint32_t now = 0U; now <= 300040U; ++now)
    {
        if (responsePending && now == responseAt)
        {
            queueResponse(fake, pendingKind);
            responsePending = false;
        }
        (void)tick(client, fake, now, BmsDdA5::ActivityHint::Idle);

        if (fake.writes.size() != observedWrites)
        {
            observedWrites = fake.writes.size();
            pendingKind = fake.writes.back().second;
            responseAt = now + 12U;
            responsePending = true;
            if (pendingKind == BmsDdA5::RequestKind::CellInfo && firstCellMs == 0U)
                firstCellMs = now;
            if (pendingKind == BmsDdA5::RequestKind::DeviceInfo && firstDeviceMs == 0U)
                firstDeviceMs = now;
        }
    }

    assert(firstCellMs == 60000U);
    assert(firstDeviceMs >= 300000U && firstDeviceMs <= 300020U);
    assert(!fake.driverEnabled);
}

int main()
{
    testStartupRetry();
    testBasicIntervals();
    testIdleAuxiliarySchedule();
    return 0;
}
