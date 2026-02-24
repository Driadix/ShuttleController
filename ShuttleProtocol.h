#pragma once
#include <stdint.h>
#include <stddef.h>

// --- Transport Layer: Frame Definition ---
#define PROTOCOL_SYNC_1    0xAA
#define PROTOCOL_SYNC_2    0x55
#define PROTOCOL_VER       1

#pragma pack(push, 1)

typedef struct {
    uint8_t  sync1;      // Always 0xAA
    uint8_t  sync2;      // Always 0x55
    uint16_t length;     // Length of Payload ONLY (excludes header and CRC)
    uint8_t  targetID;   // 0x00 = Display (Direct), 0x01-0x20 = Specific Shuttle
    uint8_t  seq;        // Rolling sequence counter (0-255)
    uint8_t  msgID;      // Identifies the Payload struct
} FrameHeader;

// --- Message IDs ---
enum MsgID : uint8_t {
    MSG_HEARTBEAT      = 0x01, // High freq: Position, Speed, State
    MSG_SENSORS        = 0x02, // Med freq: TOF, Encoders, Pallet sensors
    MSG_STATS          = 0x03, // Low freq: Odometry, Cycles
    MSG_REQ_HEARTBEAT  = 0x04, // Pult -> Shuttle: Request Telemetry
    MSG_REQ_SENSORS    = 0x05, // Pult -> Shuttle: Request Sensors (Only on Sensor Page)
    MSG_REQ_STATS      = 0x06, // Pult -> Shuttle: Request Stats (Only on Stats Page)
    MSG_LOG            = 0x10, // Async: Human readable strings with levels
    MSG_CONFIG_SET     = 0x20, // Display -> Shuttle: Set EEPROM param
    MSG_CONFIG_GET     = 0x21, // Display -> Shuttle: Request param
    MSG_CONFIG_REP     = 0x22, // Shuttle -> Display: Reply with param
    MSG_COMMAND        = 0x30, // Display -> Shuttle: Action command
    MSG_ACK            = 0x31  // Shuttle -> Display: Command acknowledgment
};

// --- Enums ---
enum LogLevel : uint8_t {
    LOG_INFO = 0, LOG_WARN = 1, LOG_ERROR = 2, LOG_DEBUG = 3
};

// Mapped from Cntrl_V1_2_WS.ino get_Cmd() / run_Cmd() strings
enum CmdType : uint8_t {
    CMD_STOP            = 5,   // "dStop_"
    CMD_STOP_MANUAL     = 55,  // "dStopM"
    CMD_MOVE_RIGHT_MAN  = 1,   // "dRight"
    CMD_MOVE_LEFT_MAN   = 2,   // "dLeft_"
    CMD_LIFT_UP         = 3,   // "dUp___"
    CMD_LIFT_DOWN       = 4,   // "dDown_"
    CMD_LOAD            = 6,   // "dLoad_"
    CMD_UNLOAD          = 7,   // "dUnld_"
    CMD_MOVE_DIST_R     = 8,   // "dMr"
    CMD_MOVE_DIST_F     = 9,   // "dMf"
    CMD_CALIBRATE       = 10,  // "dClbr_"
    CMD_DEMO            = 11,  // "dDemo_"
    CMD_COUNT_PALLETS   = 12,  // "dGetQu"
    CMD_SAVE_EEPROM     = 13,  // "dSaveC"
    CMD_COMPACT_F       = 14,  // "dComFo"
    CMD_COMPACT_R       = 15,  // "dComBa"
    CMD_GET_CONFIG      = 16,  // "dSGet_" / "dSpGet"
    //CMD_TEST_SENSORS    = 17,  // "dDataP"
    //CMD_ERROR_REQ       = 19,  // "tError"
    CMD_EVACUATE_ON     = 20,  // "dEvOn_"
    //CMD_EVACUATE_OFF    = 28,  // "dEvOff"
    CMD_LONG_LOAD       = 21,  // "dLLoad"
    CMD_LONG_UNLOAD     = 22,  // "dLUnld"
    CMD_LONG_UNLOAD_QTY = 23,  // "dQt"
    CMD_RESET_ERROR     = 24,  // "dReset"
    CMD_MANUAL_MODE     = 25,  // "dManua"
    CMD_LOG_MODE        = 26,  // "dGetLg"
    CMD_HOME            = 27,  // "dHome_"
    CMD_PING            = 100, // "ngPing"
    CMD_FIRMWARE_UPDATE = 200, // "Firmware"
    CMD_SYSTEM_RESET    = 201, // "Reboot__"
    CMD_SET_DATETIME    = 202  // "DT"
};

// Config Parameter IDs for MSG_CONFIG_SET / GET
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

// --- Payloads ---

// 0x01: Heartbeat (High Frequency - e.g., 10Hz)
struct TelemetryPacket {
    uint32_t timestamp;        // millis()
    uint16_t errorCode;        // Replaces 16-byte errorStatus array
    uint8_t  shuttleStatus;    // Current status (0-27 mapping)
    uint16_t currentPosition;  // mm
    uint16_t speed;            // Current speed %
    uint8_t  batteryCharge;    // %
    float    batteryVoltage;   // Volts

    // Bitmask for boolean states to save bandwidth
    // Bit 0: lifterUp, Bit 1: motorStart, Bit 2: motorReverse,
    // Bit 3: inverse, Bit 4: inChannel, Bit 5: fifoLifo
    uint16_t stateFlags;
    uint8_t shuttleNumber; // Shuttle number
    uint8_t palleteCount;  // Runtime pallet count (reset per op)
};

// 0x02: Sensors (Medium Frequency - e.g., 2Hz)
struct SensorPacket {
    uint16_t distanceF;        // distance[1]
    uint16_t distanceR;        // distance[0]
    uint16_t distancePltF;     // distance[3]
    uint16_t distancePltR;     // distance[2]
    uint16_t angle;            // as5600.readAngle()
    int16_t  lifterCurrent;    //
    float    temperature;      // Chip temp

    // Bitmask for discrete hardware sensors
    // Bit 0: detectPalleteF1, Bit 1: F2, Bit 2: R1, Bit 3: R2
    // Bit 4: BUMPER_F, Bit 5: BUMPER_R, Bit 6: DL_UP, Bit 7: DL_DOWN
    uint8_t hardwareFlags;
};

// 0x03: Stats (Low Frequency - e.g., 0.1Hz or on change)
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

// 0x10: Log (Asynchronous)
struct LogPacket {
    uint8_t level;             // LogLevel enum
    // char text[];            // Implicit payload data. Length = FrameHeader.length - 1
};

// 0x20 & 0x21 & 0x22: Configuration Set/Get/Report
struct ConfigPacket {
    uint8_t paramID;           // ConfigParamID enum
    int32_t value;             // Value to set / reported value
};

// 0x30: Command
struct CommandPacket {
    uint8_t cmdType;           // CmdType enum
    int32_t arg1;              // Used for Distances (dMr, dMf), Qty (dQt)
    int32_t arg2;              // Unused currently, reserved for future (e.g., specific speed)
};

// 0x31: ACK
struct AckPacket {
    uint8_t refSeq;            // Sequence number of the command being ACK'd
    uint8_t result;            // 0 = Success/Accepted, 1 = Error, 2 = Busy
};

#pragma pack(pop)

class ShuttleProtocol {
public:
    static uint16_t calcCRC16(const uint8_t* data, uint16_t length) {
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

    void init() {
        reset();
    }

    void reset() {
        state = STATE_WAIT_SYNC1;
        rxIndex = 0;
        payloadLen = 0;
        crcError = false;
    }

    FrameHeader* feed(uint8_t byte) {
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
                    uint16_t calculatedCRC = ShuttleProtocol::calcCRC16(rxBuffer, totalLen);

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

    uint8_t* getBuffer() { return rxBuffer; }
    uint16_t getTotalLength() { return sizeof(FrameHeader) + payloadLen; }

private:
    State state;
    uint8_t rxBuffer[512];
    uint16_t rxIndex;
    uint16_t payloadLen;
};