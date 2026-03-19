#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

constexpr uint8_t PROTOCOL_SYNC_1_V2 = 0xBB;
constexpr uint8_t PROTOCOL_SYNC_2_V2 = 0xCC;
constexpr uint8_t PROTOCOL_VER       = 2;

constexpr uint8_t TARGET_ID_NONE      = 0x00; // Direct UART line
constexpr uint8_t TARGET_ID_BROADCAST = 0xFF; // Global command

constexpr uint8_t MAX_LOG_STRING_LEN      = 55;
constexpr uint8_t LOG_MAX_PRINTABLE_CHARS = MAX_LOG_STRING_LEN - 1;

#pragma pack(push, 1)

typedef struct
{
    uint8_t sync1;    // Always 0xBB
    uint8_t sync2;    // Always 0xCC
    uint8_t msgID;    // Identifies the Payload struct (MsgID enum)
    uint8_t targetID; // Routing identifier
    uint8_t seq;      // Rolling sequence counter (0-255)
    uint8_t length;   // Length of Payload ONLY (excludes header and CRC)
} FrameHeader;

enum MsgID : uint8_t
{
    // Routine Telemetry (Push/Pull)
    MSG_HEARTBEAT     = 0x01, // Pushed ONLY on request
    MSG_SENSORS       = 0x02, // Pushed ONLY on request
    MSG_STATS         = 0x03, // Pushed ONLY on request
    MSG_REQ_HEARTBEAT = 0x04, // Pult -> Shuttle: Request Heartbeat (Keep-Alive)
    MSG_REQ_SENSORS   = 0x05, // Pult -> Shuttle: Request Sensors
    MSG_REQ_STATS     = 0x06, // Pult -> Shuttle: Request Stats

    // Asynchronous
    MSG_LOG = 0x10, // Shuttle -> Display: Truncated vsnprintf string

    // Configuration
    MSG_CONFIG_SET       = 0x20, // Pult/Display -> Shuttle: Set single EEPROM param
    MSG_CONFIG_GET       = 0x21, // Pult/Display -> Shuttle: Request single param
    MSG_CONFIG_REP       = 0x22, // Shuttle -> Pult/Display: Reply with single param
    MSG_CONFIG_SYNC_REQ  = 0x23, // Pult/Display -> Shuttle: Request FullConfigPacket
    MSG_CONFIG_SYNC_PUSH = 0x24, // Pult/Display -> Shuttle: Send FullConfigPacket to save
    MSG_CONFIG_SYNC_REP  = 0x25, // Shuttle -> Pult/Display: Reply with FullConfigPacket

    // Action Commands (Split for bandwidth efficiency)
    MSG_CMD_SIMPLE   = 0x30, // Pult/Display -> Shuttle: 1-byte payload (No arguments)
    MSG_CMD_WITH_ARG = 0x31, // Pult/Display -> Shuttle: 5-byte payload (Cmd + int32_t arg)
    MSG_SET_DATETIME = 0x32, // Display -> Shuttle: RTC Sync (DateTimePacket)
    MSG_ACK          = 0x33, // Shuttle -> Pult/Display: Command acknowledgment
    MSG_BMS_EXT      = 0x34, // Reserved for future extended battery telemetry
    MSG_ACK_TELEM    = 0x35  // Shuttle -> Pult: Compound ACK + Telemetry
};

// Flag placed in the MSB of msgID to suppress ACKs for volatile commands
constexpr uint8_t MSG_FLAG_NO_ACK = 0x80;
// Flag to instruct the Shuttle to append telemetry to the ACK
constexpr uint8_t MSG_FLAG_REQ_TELEM = 0x40;
// Mask to extract the real MsgID (masks out both top flag bits)
constexpr uint8_t MSG_ID_MASK = 0x3F;

enum LogLevel : uint8_t
{
    LOG_INFO  = 0,
    LOG_WARN  = 1,
    LOG_ERROR = 2,
    LOG_DEBUG = 3
};

enum CmdType : uint8_t
{
    // -- 0x00 Block: Lifecycle & State --
    CMD_STOP         = 0x00,
    CMD_STOP_MANUAL  = 0x01,
    CMD_SYSTEM_RESET = 0x02,
    CMD_RESET_ERROR  = 0x03,
    CMD_MANUAL_MODE  = 0x04,
    CMD_LOG_MODE     = 0x05,
    CMD_DEMO         = 0x06,
    CMD_HOME         = 0x07,

    // -- 0x10 Block: Core Movement --
    CMD_MOVE_RIGHT_MAN = 0x10,
    CMD_MOVE_LEFT_MAN  = 0x11,
    CMD_MOVE_DIST_R    = 0x12, // Requires MSG_CMD_WITH_ARG
    CMD_MOVE_DIST_F    = 0x13, // Requires MSG_CMD_WITH_ARG
    CMD_LIFT_UP        = 0x14,
    CMD_LIFT_DOWN      = 0x15,
    CMD_CALIBRATE      = 0x16,

    // -- 0x20 Block: Auto Operations --
    CMD_LOAD            = 0x20,
    CMD_UNLOAD          = 0x21,
    CMD_LONG_LOAD       = 0x22,
    CMD_LONG_UNLOAD     = 0x23,
    CMD_LONG_UNLOAD_QTY = 0x24, // Requires MSG_CMD_WITH_ARG
    CMD_COMPACT_F       = 0x25,
    CMD_COMPACT_R       = 0x26,
    CMD_COUNT_PALLETS   = 0x27,
    CMD_EVACUATE_ON     = 0x28,

    // -- 0x30 Block: Configuration Updates --
    CMD_SAVE_EEPROM     = 0x30,
    CMD_GET_CONFIG      = 0x31,
    CMD_FIRMWARE_UPDATE = 0x32
};

enum ConfigParamID : uint8_t
{
    CFG_SHUTTLE_NUM  = 1,
    CFG_INTER_PALLET = 2,
    CFG_SHUTTLE_LEN  = 3,
    CFG_MAX_SPEED    = 4,
    CFG_MIN_BATT     = 5,
    CFG_WAIT_TIME    = 6,
    CFG_MPR_OFFSET   = 7,
    CFG_CHNL_OFFSET  = 8,
    CFG_FIFO_LIFO    = 9,
    CFG_REVERSE_MODE = 10
};

enum AckResult : uint8_t
{
    ACK_OK              = 0, // Command accepted and will execute
    ACK_REJECTED        = 1, // Generic rejection (reserved)
    ACK_BUSY            = 2, // Shuttle is already executing another operation
    ACK_BAD_ENVIRONMENT = 3, // Shuttle not in valid physical state (e.g., not in channel)
    ACK_ERROR_STATE     = 4  // Shuttle is in error mode, only overrides accepted
};

enum ShuttleState : uint8_t
{
    STATE_IDLE            = 0,
    STATE_MANUAL          = 1,
    STATE_LOAD            = 2,
    STATE_UNLOAD          = 3,
    STATE_COMPACT         = 4,
    STATE_EVACUATE        = 5,
    STATE_DEMO            = 6,
    STATE_COUNT_PALLETS   = 7,
    STATE_ERROR           = 8,
    STATE_WAITING         = 9,
    STATE_LONG_LOAD       = 10,
    STATE_LONG_UNLOAD     = 11,
    STATE_LONG_UNLOAD_QTY = 12,
    STATE_MOVE_FWD        = 13, // Move by distance forward
    STATE_MOVE_REV        = 14, // Move by distance reverse
    STATE_LIFT_UP         = 15,
    STATE_LIFT_DOWN       = 16,
    STATE_HOME            = 17,
    STATE_CALIBRATE       = 18
};

enum ShuttleFault : uint16_t
{
    FAULT_NONE           = 0x0000,
    FAULT_TOF_CH_F       = (1 << 1),
    FAULT_TOF_CH_R       = (1 << 2),
    FAULT_TOF_PAL_F      = (1 << 3),
    FAULT_TOF_PAL_R      = (1 << 4),
    FAULT_LIFTER_TIMEOUT = (1 << 9),
    FAULT_MOTOR_STALL    = (1 << 10),
    FAULT_LOW_BATTERY    = (1 << 11),
    FAULT_CRASH_BUMPER   = (1 << 12),
    FAULT_MOVE_TIMEOUT   = (1 << 13)
};

enum ShuttleWarning : uint16_t
{
    WARN_NONE              = 0x0000,
    WARN_PALLET_NOT_FOUND  = (1 << 0),
    WARN_CHANNEL_FULL      = (1 << 1),
    WARN_NOT_IN_CHANNEL    = (1 << 2),
    WARN_PALLET_SIZE_ERROR = (1 << 3),
    WARN_END_OF_CHANNEL    = (1 << 4),
    WARN_MANUAL_TIMEOUT    = (1 << 5),
    WARN_I2C_RECOVERY      = (1 << 6),
    WARN_OBSTACLE_AHEAD    = (1 << 7)
};

struct TelemetryPacket
{
    uint16_t     errorCode;
    uint16_t     warningCode;
    uint16_t     currentPosition; // mm
    uint16_t     speed;
    uint16_t     batteryVoltage_mV; // 12500 = 12.5V
    uint16_t     stateFlags;        // Bit 0: lifterUp, 1: motorStart, 2: reverse, 3: inv, 4: inChnl, 5: fifoLifo
    ShuttleState shuttleStatus;     // Current high-level operation
    uint8_t      batteryCharge;     // %
    uint8_t      shuttleNumber;
    uint8_t      palleteCount;
};

// Reserved payload for future battery diagnostics stream (not used yet).
struct BmsExtPacket
{
    uint16_t packVoltage_mV;
    int16_t  packCurrent_cA;
    uint16_t remainCapacity_cAh;
    uint16_t nominalCapacity_cAh;
    uint16_t cycleCount;
    uint16_t protectionFlags;
    uint8_t  fetStatus;
    uint8_t  socPercent;
    uint8_t  cellCount;
    uint16_t cellMin_mV;
    uint16_t cellMax_mV;
    uint16_t cellDelta_mV;
    uint8_t  ntcCount;
    int16_t  ntcTemp_dC[4];
};

struct SensorPacket
{
    uint16_t distanceF;
    uint16_t distanceR;
    uint16_t distancePltF;
    uint16_t distancePltR;
    uint16_t angle;
    int16_t  lifterCurrent;
    int16_t  temperature_dC; // 255 = 25.5C
    uint16_t hardwareFlags;  // Bitmask for discretes
};

struct StatsPacket
{
    uint32_t totalDist;
    uint32_t loadCounter;
    uint32_t unloadCounter;
    uint32_t compactCounter;
    uint32_t liftUpCounter;
    uint32_t liftDownCounter;
    uint32_t lifetimePalletsDetected;
    uint32_t totalUptimeMinutes;
    uint16_t motorStallCount;
    uint16_t lifterOverloadCount;
    uint16_t crashCount;
    uint16_t watchdogResets;
    uint16_t lowBatteryEvents;
};

struct FullConfigPacket
{
    uint16_t interPallet;
    uint16_t shuttleLen;
    uint16_t maxSpeed;
    uint16_t waitTime;
    int16_t  mprOffset;
    int16_t  chnlOffset;
    uint8_t  shuttleNumber;
    uint8_t  minBatt;
    uint8_t  fifoLifo;
    uint8_t  reverseMode;
};

// Used with MSG_CONFIG_SET / MSG_CONFIG_GET / MSG_CONFIG_REP (5 bytes)
struct ConfigPacket
{
    int32_t value;
    uint8_t paramID;
};

// Used with MSG_CMD_SIMPLE (1 byte)
struct SimpleCmdPacket
{
    uint8_t cmdType;
};

// Used with MSG_CMD_WITH_ARG (5 bytes)
struct ParamCmdPacket
{
    int32_t arg;
    uint8_t cmdType;
};

// Used with MSG_SET_DATETIME (6 bytes)
struct DateTimePacket
{
    uint8_t year; // Offset from 2000
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

// Used with MSG_LOG (Variable length up to MAX_LOG_STRING_LEN + 1)
struct LogPacket
{
    uint8_t logLevel;
    char    message[MAX_LOG_STRING_LEN]; // Null terminated via vsnprintf
};

// Used with MSG_ACK (2 bytes)
struct AckPacket
{
    uint8_t   refSeq; // Sequence number of the command being ACK'd
    AckResult result; // Reason code for the ACK response
};

// Used with MSG_ACK_TELEM (18 bytes total: 2 AckPacket + 16 TelemetryPacket)
struct AckTelemPacket
{
    AckPacket       ack;       // 2 bytes
    TelemetryPacket telemetry; // 16 bytes
};

#pragma pack(pop)

class AlertUtils
{
  public:
    static inline bool hasFault(uint16_t currentCode, ShuttleFault fault)
    {
        return (currentCode & static_cast<uint16_t>(fault)) != 0;
    }

    static inline bool hasWarning(uint16_t currentCode, ShuttleWarning warning)
    {
        return (currentCode & static_cast<uint16_t>(warning)) != 0;
    }

    template<typename AlertEnum> static uint8_t extractActiveAlerts(uint16_t bitmask, AlertEnum *outArray, uint8_t maxItems)
    {
        uint8_t count = 0;
        for (uint8_t i = 0; i < 16 && count < maxItems; i++)
        {
            uint16_t currentBit = (1U << i);
            if (bitmask & currentBit)
            {
                outArray[count++] = static_cast<AlertEnum>(currentBit);
            }
        }
        return count;
    }
};

class ProtocolUtils
{
  public:
    static inline uint16_t updateCRC16(uint16_t crc, uint8_t byte)
    {
        crc ^= (uint16_t)byte << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
        return crc;
    }

    static inline uint16_t calcCRC16(const uint8_t *data, uint16_t length)
    {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < length; i++)
        {
            crc = updateCRC16(crc, data[i]);
        }
        return crc;
    }

    static inline void appendCRC(uint8_t *buffer, uint16_t lengthWithoutCRC)
    {
        uint16_t crc                 = calcCRC16(buffer, lengthWithoutCRC);
        buffer[lengthWithoutCRC]     = (uint8_t)(crc & 0xFF);        // LSB
        buffer[lengthWithoutCRC + 1] = (uint8_t)((crc >> 8) & 0xFF); // MSB
    }
};

class ProtocolParser
{
  public:
    enum State
    {
        STATE_WAIT_SYNC1,
        STATE_WAIT_SYNC2,
        STATE_READ_HEADER,
        STATE_READ_PAYLOAD,
        STATE_READ_CRC
    };

    bool crcError;

    ProtocolParser()
    {
        reset();
    }

    inline void reset()
    {
        state      = STATE_WAIT_SYNC1;
        rxIndex    = 0;
        payloadLen = 0;
        crcError   = false;
    }

    // millis() tick to prevent deadlocks on dropped bytes
    inline FrameHeader *feed(uint8_t byte, uint32_t currentMillis)
    {
        if (state != STATE_WAIT_SYNC1 && (currentMillis - lastRxTime > RX_TIMEOUT_MS))
        {
            reset();
        }
        lastRxTime = currentMillis;

        switch (state)
        {
        case STATE_WAIT_SYNC1:
            crcError = false;
            if (byte == PROTOCOL_SYNC_1_V2)
            {
                rxBuffer[0] = byte;
                state       = STATE_WAIT_SYNC2;
            }
            break;

        case STATE_WAIT_SYNC2:
            if (byte == PROTOCOL_SYNC_2_V2)
            {
                rxBuffer[1] = byte;
                rxIndex     = 2;
                state       = STATE_READ_HEADER;
            }
            else if (byte == PROTOCOL_SYNC_1_V2)
            {
                state = STATE_WAIT_SYNC2;
            }
            else
            {
                state = STATE_WAIT_SYNC1;
            }
            break;

        case STATE_READ_HEADER:
            rxBuffer[rxIndex++] = byte;
            if (rxIndex >= sizeof(FrameHeader))
            {
                FrameHeader *header = (FrameHeader *)rxBuffer;
                payloadLen          = header->length;

                if (payloadLen > sizeof(rxBuffer) - sizeof(FrameHeader) - 2)
                {
                    state   = STATE_WAIT_SYNC1;
                    rxIndex = 0;
                }
                else if (payloadLen == 0)
                {
                    state = STATE_READ_CRC;
                }
                else
                {
                    state = STATE_READ_PAYLOAD;
                }
            }
            break;

        case STATE_READ_PAYLOAD:
            rxBuffer[rxIndex++] = byte;
            if (rxIndex >= sizeof(FrameHeader) + payloadLen)
            {
                state = STATE_READ_CRC;
            }
            break;

        case STATE_READ_CRC:
            rxBuffer[rxIndex++] = byte;
            if (rxIndex >= sizeof(FrameHeader) + payloadLen + 2)
            {
                uint16_t totalLen = sizeof(FrameHeader) + payloadLen;

                uint16_t receivedCRC   = rxBuffer[totalLen] | (rxBuffer[totalLen + 1] << 8);
                uint16_t calculatedCRC = ProtocolUtils::calcCRC16(rxBuffer, totalLen);

                state = STATE_WAIT_SYNC1;

                if (receivedCRC == calculatedCRC)
                {
                    return (FrameHeader *)rxBuffer;
                }
                else
                {
                    crcError = true;
                    return nullptr;
                }
            }
            break;
        }
        return nullptr;
    }

  private:
    State                     state;
    uint32_t                  lastRxTime    = 0;
    static constexpr uint32_t RX_TIMEOUT_MS = 250;

    uint8_t  rxBuffer[64];
    uint16_t rxIndex;
    uint8_t  payloadLen;
};
