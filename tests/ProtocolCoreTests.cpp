#include "../Cntrl_V2/AlertManager.h"
#include "../Cntrl_V2/ShuttleProtocol.h"

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <vector>

static std::vector<uint8_t> makeSimpleCommandFrame(uint8_t sequence, uint8_t command)
{
    std::vector<uint8_t> frame(sizeof(FrameHeader) + sizeof(SimpleCmdPacket) + PROTOCOL_CRC_SIZE, 0U);

    FrameHeader header;
    header.sync1    = PROTOCOL_SYNC_1_V2;
    header.sync2    = PROTOCOL_SYNC_2_V2;
    header.msgID    = MSG_CMD_SIMPLE;
    header.targetID = 7U;
    header.seq      = sequence;
    header.length   = sizeof(SimpleCmdPacket);

    const SimpleCmdPacket payload = { command };
    memcpy(&frame[0], &header, sizeof(header));
    memcpy(&frame[sizeof(header)], &payload, sizeof(payload));
    ProtocolUtils::appendCRC(&frame[0], static_cast<uint16_t>(sizeof(header) + sizeof(payload)));
    return frame;
}

static FrameHeader *feedFrame(ProtocolParser &parser, const std::vector<uint8_t> &frame, uint32_t startMs)
{
    FrameHeader *parsed = 0;
    for (size_t i = 0U; i < frame.size(); ++i)
    {
        FrameHeader *candidate = parser.feed(frame[i], startMs + static_cast<uint32_t>(i));
        if (candidate != 0)
            parsed = candidate;
    }
    return parsed;
}

static void testCrcReferenceVector()
{
    static const uint8_t reference[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    assert(ProtocolUtils::calcCRC16(reference, sizeof(reference)) == 0x29B1U);
}

static void testParserAcceptsValidFrameAfterNoise()
{
    ProtocolParser parser;
    assert(parser.feed(0x00U, 0U) == 0);
    assert(parser.feed(PROTOCOL_SYNC_1_V2, 1U) == 0);
    assert(parser.feed(0x01U, 2U) == 0);

    const std::vector<uint8_t> frame = makeSimpleCommandFrame(42U, CMD_LIFT_UP);
    FrameHeader *header = feedFrame(parser, frame, 3U);
    assert(header != 0);
    assert(header->seq == 42U);
    assert(header->msgID == MSG_CMD_SIMPLE);
    assert(header->length == sizeof(SimpleCmdPacket));

    const SimpleCmdPacket *payload =
        reinterpret_cast<const SimpleCmdPacket *>(reinterpret_cast<const uint8_t *>(header) + sizeof(FrameHeader));
    assert(payload->cmdType == CMD_LIFT_UP);
    assert(!parser.crcError);
}

static void testParserRejectsBadCrcAndRecovers()
{
    ProtocolParser parser;
    std::vector<uint8_t> badFrame = makeSimpleCommandFrame(5U, CMD_STOP);
    badFrame[sizeof(FrameHeader)] ^= 0x01U;
    assert(feedFrame(parser, badFrame, 0U) == 0);
    assert(parser.crcError);

    const std::vector<uint8_t> goodFrame = makeSimpleCommandFrame(6U, CMD_STOP_MANUAL);
    FrameHeader *header = feedFrame(parser, goodFrame, 100U);
    assert(header != 0);
    assert(header->seq == 6U);
    assert(!parser.crcError);
}

static void testParserDropsPartialFrameAfterTimeout()
{
    ProtocolParser parser;
    assert(parser.feed(PROTOCOL_SYNC_1_V2, 0U) == 0);
    assert(parser.feed(PROTOCOL_SYNC_2_V2, 300U) == 0);

    const std::vector<uint8_t> frame = makeSimpleCommandFrame(9U, CMD_HOME);
    FrameHeader *header = feedFrame(parser, frame, 301U);
    assert(header != 0);
    assert(header->seq == 9U);
}

static void testAlertTimeoutsAndFaultLatching()
{
    AlertManager alerts;
    alerts.setFault(FAULT_BUMPER_F1);
    alerts.setFault(FAULT_MOTOR_STALL);
    assert(AlertUtils::hasFault(alerts.getErrorCode(), FAULT_BUMPER_F1));
    assert(AlertUtils::hasFault(alerts.getErrorCode(), FAULT_MOTOR_STALL));
    alerts.clearFault(FAULT_BUMPER_F1);
    assert(!AlertUtils::hasFault(alerts.getErrorCode(), FAULT_BUMPER_F1));

    alerts.setWarning(WARN_NOT_IN_CHANNEL, 10U, 100U);
    alerts.setWarning(WARN_OBSTACLE_AHEAD, 0U, 100U);
    alerts.processTimeouts(109U);
    assert(AlertUtils::hasWarning(alerts.getWarningCode(), WARN_NOT_IN_CHANNEL));
    alerts.processTimeouts(110U);
    assert(!AlertUtils::hasWarning(alerts.getWarningCode(), WARN_NOT_IN_CHANNEL));
    assert(AlertUtils::hasWarning(alerts.getWarningCode(), WARN_OBSTACLE_AHEAD));
}

static void testAlertTimeoutAcrossMillisWrap()
{
    AlertManager alerts;
    alerts.setWarning(WARN_MANUAL_TIMEOUT, 10U, 0xFFFFFFFAUL);
    alerts.processTimeouts(3U);
    assert(AlertUtils::hasWarning(alerts.getWarningCode(), WARN_MANUAL_TIMEOUT));
    alerts.processTimeouts(4U);
    assert(!AlertUtils::hasWarning(alerts.getWarningCode(), WARN_MANUAL_TIMEOUT));
}

int main()
{
    testCrcReferenceVector();
    testParserAcceptsValidFrameAfterNoise();
    testParserRejectsBadCrcAndRecovers();
    testParserDropsPartialFrameAfterTimeout();
    testAlertTimeoutsAndFaultLatching();
    testAlertTimeoutAcrossMillisWrap();
    return 0;
}
