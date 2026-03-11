#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace E32Radio {

constexpr uint8_t kAddressHighDefault = 0x00;
constexpr uint8_t kAddressLowUnassigned = 0x00;
constexpr uint8_t kAddressNodeMin = 1;
constexpr uint8_t kAddressNodeMax = 254;
constexpr uint8_t kDefaultChannel440 = 30;
constexpr uint8_t kDefaultEnsureAttempts = 3;

struct Address {
    uint8_t addh;
    uint8_t addl;
};

enum class ConfigModeLevel : uint8_t {
    ConfigLow = 0,
    ConfigHigh = 1
};

enum class UartParity : uint8_t {
    U8N1 = 0,
    U8O1 = 1,
    U8E1 = 2,
    U8N1_ALT = 3
};

enum class UartBaud : uint8_t {
    B1200 = 0,
    B2400 = 1,
    B4800 = 2,
    B9600 = 3,
    B19200 = 4,
    B38400 = 5,
    B57600 = 6,
    B115200 = 7
};

enum class AirDataRate : uint8_t {
    B300 = 0,
    B1200 = 1,
    B2400 = 2,
    B4800 = 3,
    B9600 = 4,
    B19200 = 5,
    B19200_ALT1 = 6,
    B19200_ALT2 = 7
};

enum class TransmissionMode : uint8_t {
    Transparent = 0,
    Fixed = 1
};

enum class IoDriveMode : uint8_t {
    OpenCollector = 0,
    PushPull = 1
};

enum class WakeupTime : uint8_t {
    Ms250 = 0,
    Ms500 = 1,
    Ms750 = 2,
    Ms1000 = 3,
    Ms1250 = 4,
    Ms1500 = 5,
    Ms1750 = 6,
    Ms2000 = 7
};

enum class Fec : uint8_t {
    Off = 0,
    On = 1
};

enum class TxPower : uint8_t {
    Dbm30 = 0,
    Dbm27 = 1,
    Dbm24 = 2,
    Dbm21 = 3
};

struct RawConfig {
    uint8_t addh;
    uint8_t addl;
    uint8_t sped;
    uint8_t chan;
    uint8_t option;
};

struct LogicalConfig {
    Address address;
    uint8_t channel;
    UartParity parity;
    UartBaud uartBaud;
    AirDataRate airDataRate;
    TransmissionMode transmissionMode;
    IoDriveMode ioDriveMode;
    WakeupTime wakeupTime;
    Fec fec;
    TxPower txPower;
};

struct EnsureOptions {
    uint8_t maxAttempts = kDefaultEnsureAttempts;
    uint32_t modeSettleMs = 150;
    uint32_t ioTimeoutMs = 250;
    uint32_t readCmdSettleMs = 100;
    uint32_t writeSettleMs = 100;
    uint32_t baudSwitchSettleMs = 30;
    uint32_t postModeGuardMs = 20;
    uint32_t configCommandBaud = 9600;
};

enum class EnsureStatus : uint8_t {
    OkAlreadyMatched = 0,
    OkWrittenAndVerified,
    Timeout,
    WriteFail,
    VerifyMismatch,
    InvalidConfig,
    NotInitialized
};

struct EnsureResult {
    EnsureStatus status = EnsureStatus::NotInitialized;
    uint8_t attemptsUsed = 0;
    bool changed = false;
    bool hasFinalConfig = false;
    RawConfig finalConfig = {0, 0, 0, 0, 0};

    bool ok() const {
        return status == EnsureStatus::OkAlreadyMatched ||
               status == EnsureStatus::OkWrittenAndVerified;
    }
};

inline const char* statusToString(EnsureStatus status) {
    switch (status) {
        case EnsureStatus::OkAlreadyMatched: return "ok_already_matched";
        case EnsureStatus::OkWrittenAndVerified: return "ok_written_verified";
        case EnsureStatus::Timeout: return "timeout";
        case EnsureStatus::WriteFail: return "write_fail";
        case EnsureStatus::VerifyMismatch: return "verify_mismatch";
        case EnsureStatus::InvalidConfig: return "invalid_config";
        case EnsureStatus::NotInitialized: return "not_initialized";
        default: return "unknown";
    }
}

inline Address addressFromNodeId(uint8_t nodeId, uint8_t addh = kAddressHighDefault) {
    Address addr = {addh, nodeId};
    return addr;
}

inline bool isValidNodeId(uint8_t nodeId) {
    return nodeId >= kAddressNodeMin && nodeId <= kAddressNodeMax;
}

inline bool isValidChannel(uint8_t channel) {
    return channel <= 31;
}

inline uint16_t channelFrequencyMHz(uint8_t channel) {
    return static_cast<uint16_t>(410u + channel);
}

inline uint32_t uartBaudValue(UartBaud baud) {
    static const uint32_t kBaudMap[8] = {1200u, 2400u, 4800u, 9600u, 19200u, 38400u, 57600u, 115200u};
    return kBaudMap[static_cast<uint8_t>(baud) & 0x07u];
}

inline uint32_t airDataRateBps(AirDataRate rate) {
    static const uint32_t kAirRateMap[8] = {300u, 1200u, 2400u, 4800u, 9600u, 19200u, 19200u, 19200u};
    return kAirRateMap[static_cast<uint8_t>(rate) & 0x07u];
}

inline uint16_t wakeupTimeMs(WakeupTime wakeupTime) {
    static const uint16_t kWakeupMap[8] = {250u, 500u, 750u, 1000u, 1250u, 1500u, 1750u, 2000u};
    return kWakeupMap[static_cast<uint8_t>(wakeupTime) & 0x07u];
}

inline uint8_t txPowerDbm(TxPower power) {
    static const uint8_t kTxPowerMap[4] = {30u, 27u, 24u, 21u};
    return kTxPowerMap[static_cast<uint8_t>(power) & 0x03u];
}

inline RawConfig encode(const LogicalConfig& logical) {
    RawConfig raw = {};
    raw.addh = logical.address.addh;
    raw.addl = logical.address.addl;

    raw.sped = static_cast<uint8_t>(
        ((static_cast<uint8_t>(logical.parity) & 0x03u) << 6) |
        ((static_cast<uint8_t>(logical.uartBaud) & 0x07u) << 3) |
        (static_cast<uint8_t>(logical.airDataRate) & 0x07u));

    raw.chan = logical.channel;

    raw.option = static_cast<uint8_t>(
        ((static_cast<uint8_t>(logical.transmissionMode) & 0x01u) << 7) |
        ((static_cast<uint8_t>(logical.ioDriveMode) & 0x01u) << 6) |
        ((static_cast<uint8_t>(logical.wakeupTime) & 0x07u) << 3) |
        ((static_cast<uint8_t>(logical.fec) & 0x01u) << 2) |
        (static_cast<uint8_t>(logical.txPower) & 0x03u));
    return raw;
}

inline LogicalConfig decode(const RawConfig& raw) {
    LogicalConfig logical = {};
    logical.address = {raw.addh, raw.addl};
    logical.channel = raw.chan;
    logical.parity = static_cast<UartParity>((raw.sped >> 6) & 0x03u);
    logical.uartBaud = static_cast<UartBaud>((raw.sped >> 3) & 0x07u);
    logical.airDataRate = static_cast<AirDataRate>(raw.sped & 0x07u);
    logical.transmissionMode = static_cast<TransmissionMode>((raw.option >> 7) & 0x01u);
    logical.ioDriveMode = static_cast<IoDriveMode>((raw.option >> 6) & 0x01u);
    logical.wakeupTime = static_cast<WakeupTime>((raw.option >> 3) & 0x07u);
    logical.fec = static_cast<Fec>((raw.option >> 2) & 0x01u);
    logical.txPower = static_cast<TxPower>(raw.option & 0x03u);
    return logical;
}

inline LogicalConfig makeTransparentConfig(Address address,
                                           uint8_t channel,
                                           UartBaud uartBaud,
                                           AirDataRate airDataRate,
                                           TxPower txPower,
                                           UartParity parity = UartParity::U8N1,
                                           IoDriveMode ioDrive = IoDriveMode::PushPull,
                                           WakeupTime wakeupTime = WakeupTime::Ms250,
                                           Fec fec = Fec::On) {
    LogicalConfig cfg = {};
    cfg.address = address;
    cfg.channel = channel;
    cfg.parity = parity;
    cfg.uartBaud = uartBaud;
    cfg.airDataRate = airDataRate;
    cfg.transmissionMode = TransmissionMode::Transparent;
    cfg.ioDriveMode = ioDrive;
    cfg.wakeupTime = wakeupTime;
    cfg.fec = fec;
    cfg.txPower = txPower;
    return cfg;
}

inline LogicalConfig makeFixedConfig(Address address,
                                     uint8_t channel,
                                     UartBaud uartBaud,
                                     AirDataRate airDataRate,
                                     TxPower txPower,
                                     UartParity parity = UartParity::U8N1,
                                     IoDriveMode ioDrive = IoDriveMode::PushPull,
                                     WakeupTime wakeupTime = WakeupTime::Ms250,
                                     Fec fec = Fec::On) {
    LogicalConfig cfg = makeTransparentConfig(address, channel, uartBaud, airDataRate, txPower, parity, ioDrive, wakeupTime, fec);
    cfg.transmissionMode = TransmissionMode::Fixed;
    return cfg;
}

class Radio {
public:
    Radio()
        : _serial(nullptr),
          _modePin(0),
          _runtimeBaud(9600),
          _configModeLevelHigh(false),
          _initialized(false),
          _logger(nullptr) {}

    void init(HardwareSerial* serial,
              uint8_t modePin,
              uint32_t runtimeBaud,
              ConfigModeLevel configModeLevel,
              Print* logger = nullptr) {
        init(serial, modePin, runtimeBaud, configModeLevel == ConfigModeLevel::ConfigHigh, logger);
    }

    void init(HardwareSerial* serial, uint8_t modePin, uint32_t runtimeBaud, bool configModeLevelHigh, Print* logger = nullptr) {
        _serial = serial;
        _modePin = modePin;
        _runtimeBaud = runtimeBaud;
        _configModeLevelHigh = configModeLevelHigh;
        _logger = logger;
        _initialized = (_serial != nullptr);
        if (!_initialized) {
            return;
        }

        pinMode(_modePin, OUTPUT);
        setModeLevel(!_configModeLevelHigh, EnsureOptions{});
    }

    void setLogger(Print* logger) {
        _logger = logger;
    }

    bool isInitialized() const {
        return _initialized && (_serial != nullptr);
    }

    EnsureResult ensureConfig(const LogicalConfig& desired, const EnsureOptions& options = EnsureOptions{}) {
        return ensureRawConfig(encode(desired), options);
    }

    EnsureResult ensureRawConfig(const RawConfig& desired, const EnsureOptions& options = EnsureOptions{}) {
        EnsureResult result = {};
        if (!isInitialized()) {
            result.status = EnsureStatus::NotInitialized;
            return result;
        }
        if (!isValidRawConfig(desired)) {
            result.status = EnsureStatus::InvalidConfig;
            return result;
        }

        const uint8_t attempts = options.maxAttempts == 0 ? 1 : options.maxAttempts;
        setHostBaud(options.configCommandBaud, options);
        enterConfigMode(options);

        EnsureStatus lastFail = EnsureStatus::Timeout;
        bool configured = false;
        bool changed = false;

        for (uint8_t attempt = 0; attempt < attempts; ++attempt) {
            result.attemptsUsed = static_cast<uint8_t>(attempt + 1);
            setHostBaud(options.configCommandBaud, options);

            RawConfig current = {};
            const bool readOk = readRawConfigInternal(current, options);
            if (readOk) {
                result.finalConfig = current;
                result.hasFinalConfig = true;
                if (configsEqual(current, desired)) {
                    configured = true;
                    lastFail = EnsureStatus::OkAlreadyMatched;
                    break;
                }
            } else {
                logf("W", "config read timeout (%u/%u)", static_cast<unsigned>(attempt + 1), static_cast<unsigned>(attempts));
            }

            if (!writePersistentConfig(desired, options)) {
                lastFail = EnsureStatus::WriteFail;
                logf("W", "config write failed (%u/%u)", static_cast<unsigned>(attempt + 1), static_cast<unsigned>(attempts));
                continue;
            }

            RawConfig verify = {};
            if (!readRawConfigInternal(verify, options)) {
                lastFail = EnsureStatus::Timeout;
                logf("W", "verify read timeout (%u/%u)", static_cast<unsigned>(attempt + 1), static_cast<unsigned>(attempts));
                continue;
            }

            result.finalConfig = verify;
            result.hasFinalConfig = true;
            if (configsEqual(verify, desired)) {
                configured = true;
                changed = true;
                lastFail = EnsureStatus::OkWrittenAndVerified;
                break;
            }

            lastFail = EnsureStatus::VerifyMismatch;
            logf("W", "verify mismatch (%u/%u)", static_cast<unsigned>(attempt + 1), static_cast<unsigned>(attempts));
        }

        enterNormalMode(options);
        if (configured) {
            setHostBaud(uartBaudFromSped(desired.sped), options);
        } else {
            setHostBaud(_runtimeBaud, options);
        }

        result.status = lastFail;
        result.changed = changed;
        if (!result.hasFinalConfig && configured) {
            result.finalConfig = desired;
            result.hasFinalConfig = true;
        }
        return result;
    }

    EnsureStatus readRawConfig(RawConfig& outConfig, const EnsureOptions& options = EnsureOptions{}) {
        if (!isInitialized()) {
            return EnsureStatus::NotInitialized;
        }

        setHostBaud(options.configCommandBaud, options);
        enterConfigMode(options);
        const bool ok = readRawConfigInternal(outConfig, options);
        enterNormalMode(options);
        setHostBaud(_runtimeBaud, options);
        return ok ? EnsureStatus::OkAlreadyMatched : EnsureStatus::Timeout;
    }

private:
    static bool isValidConfigHeader(uint8_t value) {
        return value == 0xC0 || value == 0xC1 || value == 0xC2;
    }

    static bool isValidRawConfig(const RawConfig& cfg) {
        return isValidChannel(cfg.chan);
    }

    static bool configsEqual(const RawConfig& lhs, const RawConfig& rhs) {
        return lhs.addh == rhs.addh &&
               lhs.addl == rhs.addl &&
               lhs.sped == rhs.sped &&
               lhs.chan == rhs.chan &&
               lhs.option == rhs.option;
    }

    static uint32_t uartBaudFromSped(uint8_t sped) {
        static const uint32_t kBaudMap[8] = {1200u, 2400u, 4800u, 9600u, 19200u, 38400u, 57600u, 115200u};
        const uint8_t code = static_cast<uint8_t>((sped >> 3) & 0x07u);
        return kBaudMap[code];
    }

    bool setHostBaud(uint32_t baud, const EnsureOptions& options) {
        if (_serial == nullptr) {
            return false;
        }
        _serial->begin(baud);
        delay(options.baudSwitchSettleMs);
        flushInput();
        return true;
    }

    bool setModeLevel(bool high, const EnsureOptions& options) {
        if (!_initialized) {
            return false;
        }
        digitalWrite(_modePin, high ? HIGH : LOW);
        delay(options.modeSettleMs);
        if (options.postModeGuardMs > 0) {
            delay(options.postModeGuardMs);
        }
        flushInput();
        return true;
    }

    bool enterConfigMode(const EnsureOptions& options) {
        return setModeLevel(_configModeLevelHigh, options);
    }

    bool enterNormalMode(const EnsureOptions& options) {
        return setModeLevel(!_configModeLevelHigh, options);
    }

    void flushInput() {
        if (_serial == nullptr) {
            return;
        }
        _serial->flush();
        while (_serial->available()) {
            _serial->read();
        }
    }

    bool readConfigFrame(uint8_t* out6, const EnsureOptions& options) {
        if (_serial == nullptr || out6 == nullptr) {
            return false;
        }

        uint8_t window[6] = {0, 0, 0, 0, 0, 0};
        uint8_t count = 0;
        const uint32_t start = millis();

        while (millis() - start < options.ioTimeoutMs) {
            while (_serial->available()) {
                const uint8_t b = static_cast<uint8_t>(_serial->read());
                if (count < sizeof(window)) {
                    window[count++] = b;
                } else {
                    memmove(window, window + 1, sizeof(window) - 1);
                    window[sizeof(window) - 1] = b;
                }

                if (count == sizeof(window) && isValidConfigHeader(window[0])) {
                    memcpy(out6, window, sizeof(window));
                    return true;
                }
            }
            delay(1);
        }
        return false;
    }

    bool readRawConfigInternal(RawConfig& outConfig, const EnsureOptions& options) {
        if (_serial == nullptr) {
            return false;
        }

        const uint8_t readCmd[3] = {0xC1, 0xC1, 0xC1};
        flushInput();
        _serial->write(readCmd, sizeof(readCmd));
        _serial->flush();
        delay(options.readCmdSettleMs);

        uint8_t raw[6] = {0, 0, 0, 0, 0, 0};
        if (!readConfigFrame(raw, options)) {
            return false;
        }

        outConfig.addh = raw[1];
        outConfig.addl = raw[2];
        outConfig.sped = raw[3];
        outConfig.chan = raw[4];
        outConfig.option = raw[5];
        return true;
    }

    bool writePersistentConfig(const RawConfig& config, const EnsureOptions& options) {
        if (_serial == nullptr) {
            return false;
        }

        const uint8_t frame[6] = {0xC0, config.addh, config.addl, config.sped, config.chan, config.option};
        flushInput();
        const size_t written = _serial->write(frame, sizeof(frame));
        _serial->flush();
        delay(options.writeSettleMs);
        return written == sizeof(frame);
    }

    void logf(const char* level, const char* fmt, ...) const {
        if (_logger == nullptr || level == nullptr || fmt == nullptr) {
            return;
        }
        char buffer[160];
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        _logger->print("[E32][");
        _logger->print(level);
        _logger->print("] ");
        _logger->println(buffer);
    }

    HardwareSerial* _serial;
    uint8_t _modePin;
    uint32_t _runtimeBaud;
    bool _configModeLevelHigh;
    bool _initialized;
    Print* _logger;
};

}  // namespace E32Radio
