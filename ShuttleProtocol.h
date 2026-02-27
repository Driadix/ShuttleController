#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// --- Transport Layer Constants ---
constexpr uint8_t PROTOCOL_SYNC_1_V2 = 0xBB;
constexpr uint8_t PROTOCOL_SYNC_2_V2 = 0xCC;
constexpr uint8_t PROTOCOL_VER = 2;

constexpr uint8_t TARGET_ID_NONE = 0x00;       // Direct UART line
constexpr uint8_t TARGET_ID_BROADCAST = 0xFF;  // Global command

#pragma pack(push, 1)

typedef struct {
    uint8_t  sync1;      // Always 0xBB
    uint8_t  sync2;      // Always 0xCC
    uint8_t  msgID;      // Identifies the Payload struct
    uint8_t  targetID;   // Routing identifier
    uint8_t  seq;        // Rolling sequence counter (0-255)
    uint8_t  length;     // Length of Payload ONLY (excludes header and CRC)
} FrameHeader;

enum MsgID : uint8_t {
    // Routine Telemetry
    MSG_HEARTBEAT      = 0x01, 
    MSG_SENSORS        = 0x02, 
    MSG_STATS          = 0x03, 
    MSG_REQ_HEARTBEAT  = 0x04, 
    MSG_REQ_SENSORS    = 0x05, 
    MSG_REQ_STATS      = 0x06, 
    
    // Asynchronous
    MSG_LOG            = 0x10, 
    
    // Configuration
    MSG_CONFIG_SET     = 0x20, 
    MSG_CONFIG_GET     = 0x21, 
    MSG_CONFIG_REP     = 0x22, 
    MSG_CONFIG_SYNC_REQ  = 0x23, 
    MSG_CONFIG_SYNC_PUSH = 0x24, 
    MSG_CONFIG_SYNC_REP  = 0x25, 

    // Action
    MSG_COMMAND        = 0x30, 
    MSG_ACK            = 0x31  
};

enum LogLevel : uint8_t {
    LOG_INFO = 0, LOG_WARN = 1, LOG_ERROR = 2, LOG_DEBUG = 3
};

enum CmdType : uint8_t {
    // -- 0x00 Block: Lifecycle & State --
    CMD_STOP            = 0x00,
    CMD_STOP_MANUAL     = 0x01,
    CMD_SYSTEM_RESET    = 0x02,
    CMD_RESET_ERROR     = 0x03,
    CMD_MANUAL_MODE     = 0x04,
    CMD_LOG_MODE        = 0x05,
    CMD_DEMO            = 0x06,
    CMD_HOME            = 0x07,

    // -- 0x10 Block: Core Movement --
    CMD_MOVE_RIGHT_MAN  = 0x10,
    CMD_MOVE_LEFT_MAN   = 0x11,
    CMD_MOVE_DIST_R     = 0x12,
    CMD_MOVE_DIST_F     = 0x13,
    CMD_LIFT_UP         = 0x14,
    CMD_LIFT_DOWN       = 0x15,
    CMD_CALIBRATE       = 0x16,

    // -- 0x20 Block: Auto Operations --
    CMD_LOAD            = 0x20,
    CMD_UNLOAD          = 0x21,
    CMD_LONG_LOAD       = 0x22,
    CMD_LONG_UNLOAD     = 0x23,
    CMD_LONG_UNLOAD_QTY = 0x24,
    CMD_COMPACT_F       = 0x25,
    CMD_COMPACT_R       = 0x26,
    CMD_COUNT_PALLETS   = 0x27,
    CMD_EVACUATE_ON     = 0x28,

    // -- 0x30 Block: Configuration Updates --
    CMD_SAVE_EEPROM     = 0x30,
    CMD_GET_CONFIG      = 0x31,
    CMD_FIRMWARE_UPDATE = 0x32,
    CMD_SET_DATETIME    = 0x33
};

enum ConfigParamID : uint8_t {
    CFG_SHUTTLE_NUM     = 1,
    CFG_INTER_PALLET    = 2,
    CFG_SHUTTLE_LEN     = 3,
    CFG_MAX_SPEED       = 4,
    CFG_MIN_BATT        = 5,
    CFG_WAIT_TIME       = 6,
    CFG_MPR_OFFSET      = 7,
    CFG_CHNL_OFFSET     = 8,
    CFG_FIFO_LIFO       = 9,
    CFG_REVERSE_MODE    = 10
};

// --- 4. Payloads

struct TelemetryPacket {
    uint32_t timestamp;        // 4 bytes
    uint16_t errorCode;        // 2 bytes
    uint16_t currentPosition;  // 2 bytes
    uint16_t speed;            // 2 bytes
    uint16_t batteryVoltage_mV;// 2 bytes (12500 = 12.5V)
    uint16_t stateFlags;       // 2 bytes
    uint8_t  shuttleStatus;    // 1 byte
    uint8_t  batteryCharge;    // 1 byte
    uint8_t  shuttleNumber;    // 1 byte
    uint8_t  palleteCount;     // 1 byte
};

struct SensorPacket {
    uint16_t distanceF;        // 2 bytes
    uint16_t distanceR;        // 2 bytes
    uint16_t distancePltF;     // 2 bytes
    uint16_t distancePltR;     // 2 bytes
    uint16_t angle;            // 2 bytes
    int16_t  lifterCurrent;    // 2 bytes
    int16_t  temperature_dC;   // 2 bytes (255 = 25.5C)
    uint16_t hardwareFlags;    // 2 bytes
};

struct StatsPacket {
    uint32_t totalDist;                
    uint32_t loadCounter;              
    uint32_t unloadCounter;            
    uint32_t compactCounter;           
    uint32_t liftUpCounter;            
    uint32_t liftDownCounter;          
    uint32_t lifetimePalletsDetected;  
    uint32_t totalUptimeMinutes;       
    uint32_t motorStallCount;          
    uint32_t lifterOverloadCount;      
    uint16_t crashCount;               
    uint16_t watchdogResets;           
    uint16_t lowBatteryEvents;         
};

struct ConfigPacket {
    int32_t value;             // 4 bytes
    uint8_t paramID;           // 1 byte
};

struct FullConfigPacket {
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

struct CommandPacket {
    int32_t arg1;              // 4 bytes
    int32_t arg2;              // 4 bytes
    uint8_t cmdType;           // 1 byte
};

struct AckPacket {
    uint8_t refSeq;            
    uint8_t result;            // 0 = Success, 1 = Error, 2 = Busy
};

#pragma pack(pop)

class ProtocolUtils {
public:
    static inline uint16_t updateCRC16(uint16_t crc, uint8_t byte) {
        crc ^= (uint16_t)byte << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
        return crc;
    }

    static inline uint16_t calcCRC16(const uint8_t* data, uint16_t length) {
        uint16_t crc = 0xFFFF;
        for (uint16_t i = 0; i < length; i++) {
            crc = updateCRC16(crc, data[i]);
        }
        return crc;
    }

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

    inline FrameHeader* feed(uint8_t byte, uint32_t currentMillis) {
        if (state != STATE_WAIT_SYNC1 && (currentMillis - lastRxTime > RX_TIMEOUT_MS)) {
            reset();
        }
        lastRxTime = currentMillis;

        switch (state) {
            case STATE_WAIT_SYNC1:
                crcError = false;
                if (byte == PROTOCOL_SYNC_1_V2) {
                    rxBuffer[0] = byte;
                    state = STATE_WAIT_SYNC2;
                }
                break;

            case STATE_WAIT_SYNC2:
                if (byte == PROTOCOL_SYNC_2_V2) {
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
    uint32_t lastRxTime = 0;
    static constexpr uint32_t RX_TIMEOUT_MS = 100;
    
    uint8_t rxBuffer[270]; 
    uint16_t rxIndex;
    uint8_t payloadLen; 
};