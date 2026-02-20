#pragma once
#include <stdint.h>

// --- Transport Layer: Frame Definition ---
#define PROTOCOL_SYNC_1    0xAA
#define PROTOCOL_SYNC_2    0x55
#define PROTOCOL_VER       1

#pragma pack(push, 1)

typedef struct {
    uint8_t  sync1;      // Always 0xAA
    uint8_t  sync2;      // Always 0x55
    uint16_t length;     // Length of Payload ONLY (excludes header and CRC)
    uint8_t  seq;        // Rolling sequence counter (0-255)
    uint8_t  msgID;      // Identifies the Payload struct
} FrameHeader;

// --- Message IDs ---
enum MsgID : uint8_t {
    MSG_HEARTBEAT      = 0x01, // High freq: Position, Speed, State
    MSG_SENSORS        = 0x02, // Med freq: TOF, Encoders, Pallet sensors
    MSG_STATS          = 0x03, // Low freq: Odometry, Cycles
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
    CMD_TEST_SENSORS    = 17,  // "dDataP"
    CMD_ERROR_REQ       = 19,  // "tError"
    CMD_EVACUATE_ON     = 20,  // "dEvOn_"
    CMD_EVACUATE_OFF    = 28,  // "dEvOff"
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
    uint32_t totalDist;        //
    uint32_t loadCounter;      //
    uint32_t unloadCounter;    //
    uint32_t compactCounter;   //
    uint32_t liftUpCounter;    //
    uint32_t liftDownCounter;  //
    uint8_t  palleteCount;     //
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