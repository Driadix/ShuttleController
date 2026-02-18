#pragma once
#include <stdint.h>

#pragma pack(push, 1) // Ensure no padding bytes

// Message IDs
enum MsgID : uint8_t {
    MSG_HEARTBEAT = 0x01,
    MSG_SENSORS   = 0x02,
    MSG_LOG       = 0x10,
    MSG_CONFIG_SET = 0x20,
    MSG_CONFIG_GET = 0x21,
    MSG_COMMAND   = 0x30,
    MSG_ACK       = 0x31
};

// Log Severity
enum LogLevel : uint8_t {
    LOG_INFO  = 0,
    LOG_WARN  = 1,
    LOG_ERROR = 2,
    LOG_DEBUG = 3
};

// 0x01: Heartbeat Payload
struct TelemetryPacket {
    uint32_t timestamp;
    uint8_t  shuttleState; // Idle, Moving, Error
    uint8_t  batteryPct;
    int32_t  position;     // mm
    int16_t  velocity;
    uint16_t activeErrors; // Bitmask
};

// 0x02: Sensor Payload
struct SensorPacket {
    uint16_t tof_dist[4];
    uint16_t lift_current;
    int16_t temperature;
};

// 0x10: Log Payload
struct LogPacket {
    uint8_t level;
    char    text[64]; // Fixed size buffer
};

// 0x20/0x21: Config Payload
struct ConfigPacket {
    uint8_t paramID;
    int32_t value;
};

// 0x30: Command Payload
struct CommandPacket {
    uint8_t cmdType; // 1=Move, 2=Lift, etc.
    int32_t arg1;    // e.g., Distance
    int32_t arg2;    // e.g., Speed
};

#pragma pack(pop)
