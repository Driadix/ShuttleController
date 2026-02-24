#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// --- Transport Layer Constants ---
#define PROTOCOL_SYNC_1           0xAA
#define PROTOCOL_SYNC_2           0x55
#define PROTOCOL_VER              1

#define TARGET_ID_NONE        0x00  // Direct UART line, target ID doesn't matter
#define TARGET_ID_BROADCAST       0xFF  // Global command to all shuttles listening (use carefully)

#pragma pack(push, 1)

typedef struct {
    uint8_t  sync1;      // Always 0xAA
    uint8_t  sync2;      // Always 0x55
    uint16_t length;     // Length of Payload ONLY (excludes header and CRC)
    uint8_t  targetID;   // 0x00 = Display, 0x01-0x20 = Specific Shuttle, 0xFF = Broadcast
    uint8_t  seq;        // Rolling sequence counter (0-255)
    uint8_t  msgID;      // Identifies the Payload struct
} FrameHeader;

enum MsgID : uint8_t {
    MSG_HEARTBEAT      = 0x01, // Pushed by Shuttle: Position, Speed, State
    MSG_SENSORS        = 0x02, // Pushed by Shuttle: TOF, Encoders, Pallet sensors
    MSG_STATS          = 0x03, // Pushed by Shuttle: Odometry, Cycles
    MSG_REQ_HEARTBEAT  = 0x04, // Pult -> Shuttle: Request Telemetry (Active Focus Keep-Alive)
    MSG_REQ_SENSORS    = 0x05, // Pult -> Shuttle: Request Sensors
    MSG_REQ_STATS      = 0x06, // Pult -> Shuttle: Request Stats
    MSG_LOG            = 0x10, // Async: Human readable strings with levels
    MSG_CONFIG_SET     = 0x20, // Display/Pult -> Shuttle: Set EEPROM param
    MSG_CONFIG_GET     = 0x21, // Display/Pult -> Shuttle: Request param
    MSG_CONFIG_REP     = 0x22, // Shuttle -> Display/Pult: Reply with param
    MSG_COMMAND        = 0x30, // Display/Pult -> Shuttle: Action command
    MSG_ACK            = 0x31  // Shuttle -> Display/Pult: Command acknowledgment
};

enum LogLevel : uint8_t {
    LOG_INFO = 0, LOG_WARN = 1, LOG_ERROR = 2, LOG_DEBUG = 3
};

enum CmdType : uint8_t {
    CMD_STOP            = 5,
    CMD_STOP_MANUAL     = 55,
    CMD_MOVE_RIGHT_MAN  = 1,
    CMD_MOVE_LEFT_MAN   = 2,
    CMD_LIFT_UP         = 3,
    CMD_LIFT_DOWN       = 4,
    CMD_LOAD            = 6,
    CMD_UNLOAD          = 7,
    CMD_MOVE_DIST_R     = 8,
    CMD_MOVE_DIST_F     = 9,
    CMD_CALIBRATE       = 10,
    CMD_DEMO            = 11,
    CMD_COUNT_PALLETS   = 12,
    CMD_SAVE_EEPROM     = 13,
    CMD_COMPACT_F       = 14,
    CMD_COMPACT_R       = 15,
    CMD_GET_CONFIG      = 16,
    CMD_EVACUATE_ON     = 20,
    CMD_LONG_LOAD       = 21,
    CMD_LONG_UNLOAD     = 22,
    CMD_LONG_UNLOAD_QTY = 23,
    CMD_RESET_ERROR     = 24,
    CMD_MANUAL_MODE     = 25,
    CMD_LOG_MODE        = 26,
    CMD_HOME            = 27,
    CMD_PING            = 100,
    CMD_FIRMWARE_UPDATE = 200,
    CMD_SYSTEM_RESET    = 201,
    CMD_SET_DATETIME    = 202
};

enum ConfigParamID : uint8_t {
    CFG_SHUTTLE_NUM     = 1,   // "dNN"
    CFG_INTER_PALLET    = 2,   // "dDm"
    CFG_SHUTTLE_LEN     = 3,   // "dSl"
    CFG_MAX_SPEED       = 4,   // "dSp"
    CFG_MIN_BATT        = 5,   // "dBc"
    CFG_WAIT_TIME       = 6,   // "dWt"
    CFG_MPR_OFFSET      = 7,   // "dMo"
    CFG_CHNL_OFFSET     = 8,   // "dMc"
    CFG_FIFO_LIFO       = 9,   // "dFIFO_" / "dLIFO_"
    CFG_REVERSE_MODE    = 10   // "dRevOn" / "dReOff"
};

// --- 4. Payloads ---
struct TelemetryPacket {
    uint32_t timestamp;        // millis()
    uint16_t errorCode;
    uint8_t  shuttleStatus;    // Current status (0-27 mapping)
    uint16_t currentPosition;  // mm
    uint16_t speed;            // Current speed %
    uint8_t  batteryCharge;    // %
    float    batteryVoltage;   // Volts
    uint16_t stateFlags;       // Bit 0: lifterUp, 1: motorStart, 2: reverse, 3: inv, 4: inChnl, 5: fifoLifo
    uint8_t  shuttleNumber;
    uint8_t  palleteCount;     // Runtime pallet count
};

struct SensorPacket {
    uint16_t distanceF;
    uint16_t distanceR;
    uint16_t distancePltF;
    uint16_t distancePltR;
    uint16_t angle;            // as5600.readAngle()
    int16_t  lifterCurrent;
    float    temperature;      // Chip temp
    uint8_t  hardwareFlags;    // Discrete sensors bitmask
};

struct StatsPacket {
    uint32_t totalDist;                // Odometer (mm)
    uint32_t loadCounter;              // Number of loads
    uint32_t unloadCounter;            // Number of unloads
    uint32_t compactCounter;           // Number of compactions
    uint32_t liftUpCounter;            // Number of lift ups
    uint32_t liftDownCounter;          // Number of lift downs
    uint32_t lifetimePalletsDetected;  // Total pallets detected (never resets)
    uint32_t totalUptimeMinutes;       // Total system uptime
    uint32_t motorStallCount;          // Number of motor stalls (Error 10)
    uint32_t lifterOverloadCount;      // Number of lifter overloads
    uint16_t crashCount;               // Number of crashes
    uint16_t watchdogResets;           // Number of WDT/HardFault resets
    uint16_t lowBatteryEvents;         // Number of low battery events (Error 11)
};

struct ConfigPacket {
    uint8_t paramID;           // ConfigParamID enum
    int32_t value;             // Value to set / reported value
};

struct CommandPacket {
    uint8_t cmdType;           // CmdType enum
    int32_t arg1;              // Used for Distances (dMr, dMf), Qty (dQt)
    int32_t arg2;              // Unused currently, reserved for future (e.g., specific speed)
};

struct AckPacket {
    uint8_t refSeq;            // Sequence number of the command being ACK'd
    uint8_t result;            // 0 = Success/Accepted, 1 = Error, 2 = Busy
};

#pragma pack(pop)

class ProtocolUtils {
public:
    static inline uint16_t calcCRC16(const uint8_t* data, uint16_t length) {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < length; i++) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t j = 0; j < 8; j++) {
                if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
                else crc <<= 1;
            }
        }
        return crc;
    }

    // Safely appends the CRC to the end of a prepared buffer in Little-Endian format
    static inline void appendCRC(uint8_t* buffer, uint16_t lengthWithoutCRC) {
        uint16_t crc = calcCRC16(buffer, lengthWithoutCRC);
        buffer[lengthWithoutCRC]     = (uint8_t)(crc & 0xFF);         // LSB
        buffer[lengthWithoutCRC + 1] = (uint8_t)((crc >> 8) & 0xFF);  // MSB
    }
};

class ProtocolParser {
public:
    enum State {
        STATE_WAIT_SYNC1,
        STATE_WAIT_SYNC2,
        STATE_READ_HEADER,
        STATE_READ_PAYLOAD,
        STATE_READ_CRC
    };

    bool crcError;

    ProtocolParser() { reset(); }

    inline void reset() {
        state = STATE_WAIT_SYNC1;
        rxIndex = 0;
        payloadLen = 0;
        crcError = false;
    }

    // Feeds a byte. Returns pointer to FrameHeader ONLY if perfect CRC match.
    inline FrameHeader* feed(uint8_t byte) {
        switch (state) {
            case STATE_WAIT_SYNC1:
                crcError = false;
                if (byte == PROTOCOL_SYNC_1) {
                    rxBuffer[0] = byte;
                    state = STATE_WAIT_SYNC2;
                }
                break;

            case STATE_WAIT_SYNC2:
                if (byte == PROTOCOL_SYNC_2) {
                    rxBuffer[1] = byte;
                    rxIndex = 2;
                    state = STATE_READ_HEADER;
                } else {
                    state = STATE_WAIT_SYNC1;
                }
                break;

            case STATE_READ_HEADER:
                rxBuffer[rxIndex++] = byte;
                if (rxIndex >= sizeof(FrameHeader)) {
                    FrameHeader* header = (FrameHeader*)rxBuffer;
                    payloadLen = header->length;

                    if (payloadLen > sizeof(rxBuffer) - sizeof(FrameHeader) - 2) {
                        state = STATE_WAIT_SYNC1;
                        rxIndex = 0;
                    } else if (payloadLen == 0) {
                        state = STATE_READ_CRC;
                    } else {
                        state = STATE_READ_PAYLOAD;
                    }
                }
                break;

            case STATE_READ_PAYLOAD:
                rxBuffer[rxIndex++] = byte;
                if (rxIndex >= sizeof(FrameHeader) + payloadLen) {
                    state = STATE_READ_CRC;
                }
                break;

            case STATE_READ_CRC:
                rxBuffer[rxIndex++] = byte;
                if (rxIndex >= sizeof(FrameHeader) + payloadLen + 2) {
                    uint16_t totalLen = sizeof(FrameHeader) + payloadLen;

                    uint16_t receivedCRC = rxBuffer[totalLen] | (rxBuffer[totalLen+1] << 8);
                    uint16_t calculatedCRC = ProtocolUtils::calcCRC16(rxBuffer, totalLen);

                    state = STATE_WAIT_SYNC1;

                    if (receivedCRC == calculatedCRC) {
                        return (FrameHeader*)rxBuffer;
                    } else {
                        crcError = true;
                        return nullptr;
                    }
                }
                break;
        }
        return nullptr;
    }

private:
    State state;
    uint8_t rxBuffer[256];
    uint16_t rxIndex;
    uint16_t payloadLen;
};