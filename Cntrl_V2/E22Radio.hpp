#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace E22Radio
{
    constexpr uint8_t  kAddressHighDefault        = 0x00;
    constexpr uint8_t  kAddressHighShuttle        = 0x01;
    constexpr uint8_t  kAddressHighRemote         = 0x02;
    constexpr uint8_t  kAddressLowUnassigned      = 0x00;
    constexpr uint8_t  kAddressNodeMin            = 1;
    constexpr uint8_t  kAddressNodeMax            = 32;
    constexpr uint8_t  kDefaultChannel440         = 30;
    constexpr uint8_t  kMaxChannel                = 83;
    constexpr uint8_t  kDefaultNetId              = 0;
    constexpr uint16_t kDefaultCryptKey           = 0;
    constexpr uint8_t  kDefaultEnsureAttempts     = 3;
    constexpr uint32_t kDefaultPacketRssiTimeoutMs = 20;

    struct ControlPins
    {
        int m0;
        int m1;
        int aux;
    };

    struct Address
    {
        uint8_t addh;
        uint8_t addl;
    };

    struct FixedRoute
    {
        uint8_t addh;
        uint8_t addl;
        uint8_t channel;
    };

    enum class ModuleMode : uint8_t
    {
        Normal = 0,
        Wor    = 1,
        Config = 2,
        Sleep  = 3
    };

    enum class UartParity : uint8_t
    {
        U8N1     = 0,
        U8O1     = 1,
        U8E1     = 2,
        U8N1_ALT = 3
    };

    enum class UartBaud : uint8_t
    {
        B1200   = 0,
        B2400   = 1,
        B4800   = 2,
        B9600   = 3,
        B19200  = 4,
        B38400  = 5,
        B57600  = 6,
        B115200 = 7
    };

    enum class AirDataRate : uint8_t
    {
        B300   = 0,
        B1200  = 1,
        B2400  = 2,
        B4800  = 3,
        B9600  = 4,
        B19200 = 5,
        B38400 = 6,
        B62500 = 7
    };

    enum class SubPacketSize : uint8_t
    {
        Bytes240 = 0,
        Bytes128 = 1,
        Bytes64  = 2,
        Bytes32  = 3
    };

    enum class TransmissionMode : uint8_t
    {
        Transparent = 0,
        Fixed       = 1
    };

    enum class WorRole : uint8_t
    {
        Receiver    = 0,
        Transmitter = 1
    };

    enum class WorCycle : uint8_t
    {
        Ms500  = 0,
        Ms1000 = 1,
        Ms1500 = 2,
        Ms2000 = 3,
        Ms2500 = 4,
        Ms3000 = 5,
        Ms3500 = 6,
        Ms4000 = 7
    };

    enum class TxPower : uint8_t
    {
        Dbm30 = 0,
        Dbm27 = 1,
        Dbm24 = 2,
        Dbm21 = 3
    };

    struct RawConfig
    {
        uint8_t addh;
        uint8_t addl;
        uint8_t netid;
        uint8_t reg0;
        uint8_t reg1;
        uint8_t reg2;
        uint8_t reg3;
        uint8_t crypth;
        uint8_t cryptl;
    };

    struct LogicalConfig
    {
        Address          address;
        uint8_t          netId;
        uint8_t          channel;
        UartParity       parity;
        UartBaud         uartBaud;
        AirDataRate      airDataRate;
        SubPacketSize    subPacketSize;
        bool             ambientRssiEnabled;
        TxPower          txPower;
        bool             appendRssiEnabled;
        TransmissionMode transmissionMode;
        bool             repeaterEnabled;
        bool             lbtEnabled;
        WorRole          worRole;
        WorCycle         worCycle;
        uint16_t         cryptKey;
    };

    struct EnsureOptions
    {
        uint8_t  maxAttempts             = kDefaultEnsureAttempts;
        uint32_t modeSettleMs            = 100;
        uint32_t auxTimeoutMs            = 1000;
        uint32_t ioTimeoutMs             = 500;
        uint32_t readCmdSettleMs         = 20;
        uint32_t writeSettleMs           = 100;
        uint32_t baudSwitchSettleMs      = 30;
        uint32_t postModeGuardMs         = 20;
        uint32_t configCommandBaud       = 9600;
        uint32_t packetRssiReadTimeoutMs = kDefaultPacketRssiTimeoutMs;
    };

    enum class EnsureStatus : uint8_t
    {
        OkAlreadyMatched = 0,
        OkWrittenAndVerified,
        Timeout,
        WriteFail,
        VerifyMismatch,
        InvalidConfig,
        NotInitialized
    };

    struct EnsureResult
    {
        EnsureStatus status         = EnsureStatus::NotInitialized;
        uint8_t      attemptsUsed   = 0;
        bool         changed        = false;
        bool         hasFinalConfig = false;
        RawConfig    finalConfig    = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        bool ok() const
        {
            return status == EnsureStatus::OkAlreadyMatched || status == EnsureStatus::OkWrittenAndVerified;
        }
    };

    inline const char *statusToString(EnsureStatus status)
    {
        switch (status)
        {
        case EnsureStatus::OkAlreadyMatched:
            return "ok_already_matched";
        case EnsureStatus::OkWrittenAndVerified:
            return "ok_written_verified";
        case EnsureStatus::Timeout:
            return "timeout";
        case EnsureStatus::WriteFail:
            return "write_fail";
        case EnsureStatus::VerifyMismatch:
            return "verify_mismatch";
        case EnsureStatus::InvalidConfig:
            return "invalid_config";
        case EnsureStatus::NotInitialized:
            return "not_initialized";
        default:
            return "unknown";
        }
    }

    inline Address addressFromNodeId(uint8_t nodeId, uint8_t addh = kAddressHighDefault)
    {
        Address addr = { addh, nodeId };
        return addr;
    }

    inline bool isValidNodeId(uint8_t nodeId)
    {
        return nodeId >= kAddressNodeMin && nodeId <= kAddressNodeMax;
    }

    inline bool isUnassignedOrValidNodeId(uint8_t nodeId)
    {
        return nodeId == kAddressLowUnassigned || isValidNodeId(nodeId);
    }

    inline Address shuttleAddressFromNodeId(uint8_t nodeId)
    {
        return addressFromNodeId(nodeId, kAddressHighShuttle);
    }

    inline Address remoteAddressFromNodeId(uint8_t nodeId)
    {
        return addressFromNodeId(nodeId, kAddressHighRemote);
    }

    inline FixedRoute fixedRouteFromAddress(Address address, uint8_t channel)
    {
        FixedRoute route = { address.addh, address.addl, channel };
        return route;
    }

    inline FixedRoute fixedRouteToShuttle(uint8_t shuttleId, uint8_t channel = kDefaultChannel440)
    {
        return fixedRouteFromAddress(shuttleAddressFromNodeId(shuttleId), channel);
    }

    inline FixedRoute fixedRouteToRemote(uint8_t shuttleId, uint8_t channel = kDefaultChannel440)
    {
        return fixedRouteFromAddress(remoteAddressFromNodeId(shuttleId), channel);
    }

    inline bool isValidChannel(uint8_t channel)
    {
        return channel <= kMaxChannel;
    }

    inline uint32_t channelFrequencyKhz(uint8_t channel)
    {
        return 410125UL + (static_cast<uint32_t>(channel) * 1000UL);
    }

    inline uint16_t channelFrequencyMHz(uint8_t channel)
    {
        return static_cast<uint16_t>(channelFrequencyKhz(channel) / 1000UL);
    }

    inline uint32_t uartBaudValue(UartBaud baud)
    {
        static const uint32_t kBaudMap[8] = { 1200u, 2400u, 4800u, 9600u, 19200u, 38400u, 57600u, 115200u };
        return kBaudMap[static_cast<uint8_t>(baud) & 0x07u];
    }

    inline uint32_t airDataRateBps(AirDataRate rate)
    {
        static const uint32_t kAirRateMap[8] = { 300u, 1200u, 2400u, 4800u, 9600u, 19200u, 38400u, 62500u };
        return kAirRateMap[static_cast<uint8_t>(rate) & 0x07u];
    }

    inline uint16_t subPacketSizeBytes(SubPacketSize size)
    {
        static const uint16_t kSubPacketMap[4] = { 240u, 128u, 64u, 32u };
        return kSubPacketMap[static_cast<uint8_t>(size) & 0x03u];
    }

    inline uint16_t worCycleMs(WorCycle cycle)
    {
        static const uint16_t kWorCycleMap[8] = { 500u, 1000u, 1500u, 2000u, 2500u, 3000u, 3500u, 4000u };
        return kWorCycleMap[static_cast<uint8_t>(cycle) & 0x07u];
    }

    inline uint8_t txPowerDbm(TxPower power)
    {
        static const uint8_t kTxPowerMap[4] = { 30u, 27u, 24u, 21u };
        return kTxPowerMap[static_cast<uint8_t>(power) & 0x03u];
    }

    inline int16_t packetRssiRawToDbm(uint8_t raw)
    {
        return static_cast<int16_t>(static_cast<int8_t>(raw));
    }

    inline int16_t ambientRssiRawToDbm(uint8_t raw)
    {
        return -static_cast<int16_t>(raw) / 2;
    }

    inline RawConfig encode(const LogicalConfig &logical)
    {
        RawConfig raw = {};
        raw.addh      = logical.address.addh;
        raw.addl      = logical.address.addl;
        raw.netid     = logical.netId;
        raw.reg0      = static_cast<uint8_t>(
            ((static_cast<uint8_t>(logical.uartBaud) & 0x07u) << 5) |
            ((static_cast<uint8_t>(logical.parity) & 0x03u) << 3) | (static_cast<uint8_t>(logical.airDataRate) & 0x07u));
        raw.reg1 = static_cast<uint8_t>(
            ((static_cast<uint8_t>(logical.subPacketSize) & 0x03u) << 6) |
            (logical.ambientRssiEnabled ? 0x20u : 0x00u) | (static_cast<uint8_t>(logical.txPower) & 0x03u));
        raw.reg2   = logical.channel;
        raw.reg3   = static_cast<uint8_t>(
            (logical.appendRssiEnabled ? 0x80u : 0x00u) |
            ((static_cast<uint8_t>(logical.transmissionMode) & 0x01u) << 6) |
            (logical.repeaterEnabled ? 0x20u : 0x00u) | (logical.lbtEnabled ? 0x10u : 0x00u) |
            ((static_cast<uint8_t>(logical.worRole) & 0x01u) << 3) | (static_cast<uint8_t>(logical.worCycle) & 0x07u));
        raw.crypth = static_cast<uint8_t>((logical.cryptKey >> 8) & 0xFFu);
        raw.cryptl = static_cast<uint8_t>(logical.cryptKey & 0xFFu);
        return raw;
    }

    inline LogicalConfig decode(const RawConfig &raw)
    {
        LogicalConfig logical          = {};
        logical.address                = { raw.addh, raw.addl };
        logical.netId                  = raw.netid;
        logical.uartBaud               = static_cast<UartBaud>((raw.reg0 >> 5) & 0x07u);
        logical.parity                 = static_cast<UartParity>((raw.reg0 >> 3) & 0x03u);
        logical.airDataRate            = static_cast<AirDataRate>(raw.reg0 & 0x07u);
        logical.subPacketSize          = static_cast<SubPacketSize>((raw.reg1 >> 6) & 0x03u);
        logical.ambientRssiEnabled     = (raw.reg1 & 0x20u) != 0;
        logical.txPower                = static_cast<TxPower>(raw.reg1 & 0x03u);
        logical.channel                = raw.reg2;
        logical.appendRssiEnabled      = (raw.reg3 & 0x80u) != 0;
        logical.transmissionMode       = static_cast<TransmissionMode>((raw.reg3 >> 6) & 0x01u);
        logical.repeaterEnabled        = (raw.reg3 & 0x20u) != 0;
        logical.lbtEnabled             = (raw.reg3 & 0x10u) != 0;
        logical.worRole                = static_cast<WorRole>((raw.reg3 >> 3) & 0x01u);
        logical.worCycle               = static_cast<WorCycle>(raw.reg3 & 0x07u);
        logical.cryptKey               = (static_cast<uint16_t>(raw.crypth) << 8) | raw.cryptl;
        return logical;
    }

    inline LogicalConfig makeTransparentConfig(
        Address       address,
        uint8_t       channel,
        UartBaud      uartBaud,
        AirDataRate   airDataRate,
        TxPower       txPower,
        UartParity    parity               = UartParity::U8N1,
        uint8_t       netId                = kDefaultNetId,
        SubPacketSize subPacketSize        = SubPacketSize::Bytes240,
        bool          ambientRssiEnabled   = true,
        bool          appendRssiEnabled    = true,
        bool          repeaterEnabled      = false,
        bool          lbtEnabled           = false,
        WorRole       worRole              = WorRole::Receiver,
        WorCycle      worCycle             = WorCycle::Ms2000,
        uint16_t      cryptKey             = kDefaultCryptKey)
    {
        LogicalConfig cfg       = {};
        cfg.address             = address;
        cfg.netId               = netId;
        cfg.channel             = channel;
        cfg.parity              = parity;
        cfg.uartBaud            = uartBaud;
        cfg.airDataRate         = airDataRate;
        cfg.subPacketSize       = subPacketSize;
        cfg.ambientRssiEnabled  = ambientRssiEnabled;
        cfg.txPower             = txPower;
        cfg.appendRssiEnabled   = appendRssiEnabled;
        cfg.transmissionMode    = TransmissionMode::Transparent;
        cfg.repeaterEnabled     = repeaterEnabled;
        cfg.lbtEnabled          = lbtEnabled;
        cfg.worRole             = worRole;
        cfg.worCycle            = worCycle;
        cfg.cryptKey            = cryptKey;
        return cfg;
    }

    inline LogicalConfig makeFixedConfig(
        Address       address,
        uint8_t       channel,
        UartBaud      uartBaud,
        AirDataRate   airDataRate,
        TxPower       txPower,
        UartParity    parity             = UartParity::U8N1,
        uint8_t       netId              = kDefaultNetId,
        SubPacketSize subPacketSize      = SubPacketSize::Bytes240,
        bool          ambientRssiEnabled = true,
        bool          appendRssiEnabled  = true,
        uint16_t      cryptKey           = kDefaultCryptKey)
    {
        LogicalConfig cfg = makeTransparentConfig(
            address,
            channel,
            uartBaud,
            airDataRate,
            txPower,
            parity,
            netId,
            subPacketSize,
            ambientRssiEnabled,
            appendRssiEnabled,
            false,
            false,
            WorRole::Receiver,
            WorCycle::Ms2000,
            cryptKey);
        cfg.transmissionMode = TransmissionMode::Fixed;
        return cfg;
    }

    class Radio
    {
      public:
        Radio()
            : _serial(nullptr), _pins{ -1, -1, -1 }, _runtimeBaud(9600), _initialized(false), _logger(nullptr),
              _lastPacketRssiRaw(0), _hasLastPacketRssi(false), _lastPacketRssiAtMs(0), _lastAmbientRssiRaw(0),
              _hasLastAmbientRssi(false), _lastAmbientRssiAtMs(0)
        {
        }

        void init(HardwareSerial *serial, ControlPins pins, uint32_t runtimeBaud, Print *logger = nullptr)
        {
            _serial      = serial;
            _pins        = pins;
            _runtimeBaud = runtimeBaud;
            _logger      = logger;
            _initialized = (_serial != nullptr && _pins.m0 >= 0 && _pins.m1 >= 0);
            if (!_initialized)
            {
                return;
            }

            pinMode(_pins.m0, OUTPUT);
            pinMode(_pins.m1, OUTPUT);
            if (_pins.aux >= 0)
            {
                pinMode(_pins.aux, INPUT_PULLUP);
            }
            setMode(ModuleMode::Normal, EnsureOptions{});
        }

        void setLogger(Print *logger)
        {
            _logger = logger;
        }

        bool isInitialized() const
        {
            return _initialized && (_serial != nullptr);
        }

        bool hasAuxPin() const
        {
            return _pins.aux >= 0;
        }

        bool isAuxHigh() const
        {
            return !hasAuxPin() || digitalRead(_pins.aux) == HIGH;
        }

        bool setMode(ModuleMode mode, const EnsureOptions &options = EnsureOptions{})
        {
            if (!isInitialized())
            {
                return false;
            }
            switch (mode)
            {
            case ModuleMode::Normal:
                digitalWrite(_pins.m1, LOW);
                digitalWrite(_pins.m0, LOW);
                break;
            case ModuleMode::Wor:
                digitalWrite(_pins.m1, LOW);
                digitalWrite(_pins.m0, HIGH);
                break;
            case ModuleMode::Config:
                digitalWrite(_pins.m1, HIGH);
                digitalWrite(_pins.m0, LOW);
                break;
            case ModuleMode::Sleep:
                digitalWrite(_pins.m1, HIGH);
                digitalWrite(_pins.m0, HIGH);
                break;
            }
            delay(options.modeSettleMs);
            waitAuxHigh(options.auxTimeoutMs);
            if (options.postModeGuardMs > 0)
            {
                delay(options.postModeGuardMs);
            }
            flushInput();
            return true;
        }

        bool waitAuxHigh(uint32_t timeoutMs) const
        {
            if (_pins.aux < 0)
            {
                return true;
            }
            const uint32_t start = millis();
            while (digitalRead(_pins.aux) == LOW)
            {
                if (millis() - start >= timeoutMs)
                {
                    return false;
                }
                delay(1);
            }
            return true;
        }

        EnsureResult ensureConfig(const LogicalConfig &desired, const EnsureOptions &options = EnsureOptions{})
        {
            return ensureRawConfig(encode(desired), options);
        }

        EnsureResult ensureRawConfig(const RawConfig &desired, const EnsureOptions &options = EnsureOptions{})
        {
            EnsureResult result = {};
            if (!isInitialized())
            {
                result.status = EnsureStatus::NotInitialized;
                return result;
            }
            if (!isValidRawConfig(desired))
            {
                result.status = EnsureStatus::InvalidConfig;
                return result;
            }

            const uint8_t attempts = options.maxAttempts == 0 ? 1 : options.maxAttempts;
            uint32_t      bauds[5];
            const uint8_t baudCount = buildBaudCandidates(bauds, sizeof(bauds) / sizeof(bauds[0]), options, desired.reg0);

            EnsureStatus lastFail   = EnsureStatus::Timeout;
            bool         configured = false;
            bool         changed    = false;

            for (uint8_t attempt = 0; attempt < attempts && !configured; ++attempt)
            {
                result.attemptsUsed = static_cast<uint8_t>(attempt + 1);
                for (uint8_t baudIndex = 0; baudIndex < baudCount && !configured; ++baudIndex)
                {
                    setHostBaud(bauds[baudIndex], options);
                    setMode(ModuleMode::Config, options);

                    RawConfig current = {};
                    const bool readOk = readRawConfigInternal(current, options);
                    if (!readOk)
                    {
                        continue;
                    }

                    result.finalConfig    = current;
                    result.hasFinalConfig = true;
                    if (configsEqual(current, desired))
                    {
                        configured = true;
                        lastFail   = EnsureStatus::OkAlreadyMatched;
                        break;
                    }

                    if (!writePersistentConfig(desired, options))
                    {
                        lastFail = EnsureStatus::WriteFail;
                        logf(
                            "W",
                            "config write failed (%u/%u)",
                            static_cast<unsigned>(attempt + 1),
                            static_cast<unsigned>(attempts));
                        continue;
                    }

                    const uint32_t desiredBaud = uartBaudFromReg0(desired.reg0);
                    setHostBaud(desiredBaud, options);
                    setMode(ModuleMode::Config, options);

                    RawConfig verify = {};
                    if (!readRawConfigInternal(verify, options))
                    {
                        lastFail = EnsureStatus::Timeout;
                        logf(
                            "W",
                            "verify read timeout (%u/%u)",
                            static_cast<unsigned>(attempt + 1),
                            static_cast<unsigned>(attempts));
                        continue;
                    }

                    result.finalConfig    = verify;
                    result.hasFinalConfig = true;
                    if (configsEqual(verify, desired))
                    {
                        configured = true;
                        changed    = true;
                        lastFail   = EnsureStatus::OkWrittenAndVerified;
                        break;
                    }

                    lastFail = EnsureStatus::VerifyMismatch;
                    logf("W", "verify mismatch (%u/%u)", static_cast<unsigned>(attempt + 1), static_cast<unsigned>(attempts));
                }
            }

            setMode(ModuleMode::Normal, options);
            if (configured)
            {
                setHostBaud(uartBaudFromReg0(desired.reg0), options);
            }
            else
            {
                setHostBaud(_runtimeBaud, options);
            }

            result.status  = lastFail;
            result.changed = changed;
            if (!result.hasFinalConfig && configured)
            {
                result.finalConfig    = desired;
                result.hasFinalConfig = true;
            }
            return result;
        }

        EnsureStatus readRawConfig(RawConfig &outConfig, const EnsureOptions &options = EnsureOptions{})
        {
            uint8_t registers[9] = {};
            EnsureStatus status  = readRegisters(0x00, registers, sizeof(registers), options);
            if (status != EnsureStatus::OkAlreadyMatched)
            {
                return status;
            }
            outConfig = rawConfigFromBytes(registers);
            return status;
        }

        EnsureStatus readRegisters(uint8_t startAddress, uint8_t *outRegisters, uint8_t length, const EnsureOptions &options = EnsureOptions{})
        {
            if (!isInitialized())
            {
                return EnsureStatus::NotInitialized;
            }
            if (outRegisters == nullptr || length == 0 || startAddress + length > 0x09)
            {
                return EnsureStatus::InvalidConfig;
            }

            uint32_t      bauds[5];
            const uint8_t baudCount = buildBaudCandidates(bauds, sizeof(bauds) / sizeof(bauds[0]), options, 0);
            for (uint8_t i = 0; i < baudCount; ++i)
            {
                setHostBaud(bauds[i], options);
                setMode(ModuleMode::Config, options);
                if (readRegistersInternal(startAddress, outRegisters, length, options))
                {
                    setMode(ModuleMode::Normal, options);
                    setHostBaud(_runtimeBaud, options);
                    return EnsureStatus::OkAlreadyMatched;
                }
            }

            setMode(ModuleMode::Normal, options);
            setHostBaud(_runtimeBaud, options);
            return EnsureStatus::Timeout;
        }

        EnsureStatus writeRegisters(
            uint8_t startAddress,
            const uint8_t *registers,
            uint8_t length,
            bool persistent = true,
            const EnsureOptions &options = EnsureOptions{})
        {
            if (!isInitialized())
            {
                return EnsureStatus::NotInitialized;
            }
            if (registers == nullptr || length == 0 || startAddress + length > 0x09)
            {
                return EnsureStatus::InvalidConfig;
            }

            uint32_t      bauds[5];
            const uint8_t baudCount = buildBaudCandidates(bauds, sizeof(bauds) / sizeof(bauds[0]), options, 0);
            bool          ok        = false;
            for (uint8_t i = 0; i < baudCount && !ok; ++i)
            {
                setHostBaud(bauds[i], options);
                setMode(ModuleMode::Config, options);
                ok = writeRegistersInternal(startAddress, registers, length, persistent ? 0xC0 : 0xC2, options);
            }
            setMode(ModuleMode::Normal, options);
            setHostBaud(_runtimeBaud, options);
            return ok ? EnsureStatus::OkWrittenAndVerified : EnsureStatus::WriteFail;
        }

        bool readAmbientRssiRaw(uint8_t &outRaw, const EnsureOptions &options = EnsureOptions{})
        {
            if (!isInitialized())
            {
                return false;
            }
            setHostBaud(_runtimeBaud, options);
            setMode(ModuleMode::Normal, options);
            flushInput();
            const uint8_t cmd[3] = { 0xC0, 0xC1, 0xC2 };
            if (!waitAuxHigh(options.auxTimeoutMs))
            {
                return false;
            }
            _serial->write(cmd, sizeof(cmd));
            _serial->flush();

            uint8_t raw = 0;
            if (!readByteWithTimeout(raw, options.ioTimeoutMs))
            {
                return false;
            }
            _lastAmbientRssiRaw  = raw;
            _hasLastAmbientRssi  = true;
            _lastAmbientRssiAtMs = millis();
            outRaw               = raw;
            return true;
        }

        bool readLastPacketRssiRaw(uint8_t &outRaw, uint32_t timeoutMs = kDefaultPacketRssiTimeoutMs)
        {
            if (!isInitialized())
            {
                return false;
            }
            uint8_t raw = 0;
            if (!readByteWithTimeout(raw, timeoutMs))
            {
                return false;
            }
            _lastPacketRssiRaw  = raw;
            _hasLastPacketRssi  = true;
            _lastPacketRssiAtMs = millis();
            outRaw              = raw;
            return true;
        }

        bool hasLastPacketRssi() const
        {
            return _hasLastPacketRssi;
        }

        uint8_t lastPacketRssiRaw() const
        {
            return _lastPacketRssiRaw;
        }

        int16_t lastPacketRssiDbm() const
        {
            return packetRssiRawToDbm(_lastPacketRssiRaw);
        }

        uint32_t lastPacketRssiAtMs() const
        {
            return _lastPacketRssiAtMs;
        }

        bool hasLastAmbientRssi() const
        {
            return _hasLastAmbientRssi;
        }

        uint8_t lastAmbientRssiRaw() const
        {
            return _lastAmbientRssiRaw;
        }

        int16_t lastAmbientRssiDbm() const
        {
            return ambientRssiRawToDbm(_lastAmbientRssiRaw);
        }

        uint32_t lastAmbientRssiAtMs() const
        {
            return _lastAmbientRssiAtMs;
        }

      private:
        static RawConfig rawConfigFromBytes(const uint8_t *bytes)
        {
            RawConfig raw = {};
            raw.addh      = bytes[0];
            raw.addl      = bytes[1];
            raw.netid     = bytes[2];
            raw.reg0      = bytes[3];
            raw.reg1      = bytes[4];
            raw.reg2      = bytes[5];
            raw.reg3      = bytes[6];
            raw.crypth    = bytes[7];
            raw.cryptl    = bytes[8];
            return raw;
        }

        static void rawConfigToBytes(const RawConfig &config, uint8_t *outBytes)
        {
            outBytes[0] = config.addh;
            outBytes[1] = config.addl;
            outBytes[2] = config.netid;
            outBytes[3] = config.reg0;
            outBytes[4] = config.reg1;
            outBytes[5] = config.reg2;
            outBytes[6] = config.reg3;
            outBytes[7] = config.crypth;
            outBytes[8] = config.cryptl;
        }

        static bool isValidRawConfig(const RawConfig &cfg)
        {
            return isValidChannel(cfg.reg2);
        }

        static bool configsEqual(const RawConfig &lhs, const RawConfig &rhs)
        {
            return lhs.addh == rhs.addh && lhs.addl == rhs.addl && lhs.netid == rhs.netid && lhs.reg0 == rhs.reg0 &&
                   lhs.reg1 == rhs.reg1 && lhs.reg2 == rhs.reg2 && lhs.reg3 == rhs.reg3 && lhs.crypth == rhs.crypth &&
                   lhs.cryptl == rhs.cryptl;
        }

        static uint32_t uartBaudFromReg0(uint8_t reg0)
        {
            return uartBaudValue(static_cast<UartBaud>((reg0 >> 5) & 0x07u));
        }

        static void addBaudCandidate(uint32_t *bauds, uint8_t maxCount, uint8_t &count, uint32_t baud)
        {
            if (baud == 0 || count >= maxCount)
            {
                return;
            }
            for (uint8_t i = 0; i < count; ++i)
            {
                if (bauds[i] == baud)
                {
                    return;
                }
            }
            bauds[count++] = baud;
        }

        uint8_t buildBaudCandidates(uint32_t *bauds, uint8_t maxCount, const EnsureOptions &options, uint8_t desiredReg0) const
        {
            uint8_t count = 0;
            addBaudCandidate(bauds, maxCount, count, options.configCommandBaud);
            addBaudCandidate(bauds, maxCount, count, _runtimeBaud);
            if (desiredReg0 != 0)
            {
                addBaudCandidate(bauds, maxCount, count, uartBaudFromReg0(desiredReg0));
            }
            addBaudCandidate(bauds, maxCount, count, 9600);
            addBaudCandidate(bauds, maxCount, count, 115200);
            return count;
        }

        bool setHostBaud(uint32_t baud, const EnsureOptions &options)
        {
            if (_serial == nullptr || baud == 0)
            {
                return false;
            }
#if defined(ARDUINO_ARCH_ESP32)
            _serial->updateBaudRate(baud);
#else
            _serial->begin(baud);
#endif
            delay(options.baudSwitchSettleMs);
            flushInput();
            return true;
        }

        void flushInput()
        {
            if (_serial == nullptr)
            {
                return;
            }
            _serial->flush();
            while (_serial->available())
            {
                _serial->read();
            }
        }

        bool readByteWithTimeout(uint8_t &outByte, uint32_t timeoutMs)
        {
            if (_serial == nullptr)
            {
                return false;
            }
            const uint32_t start = millis();
            while (millis() - start < timeoutMs)
            {
                if (_serial->available())
                {
                    outByte = static_cast<uint8_t>(_serial->read());
                    return true;
                }
                delay(1);
            }
            return false;
        }

        bool readRegistersInternal(uint8_t startAddress, uint8_t *outRegisters, uint8_t length, const EnsureOptions &options)
        {
            if (_serial == nullptr || outRegisters == nullptr || length == 0)
            {
                return false;
            }

            const uint8_t readCmd[3] = { 0xC1, startAddress, length };
            flushInput();
            if (!waitAuxHigh(options.auxTimeoutMs))
            {
                return false;
            }
            _serial->write(readCmd, sizeof(readCmd));
            _serial->flush();
            delay(options.readCmdSettleMs);

            uint8_t        header[3]    = { 0, 0, 0 };
            uint8_t        headerIndex  = 0;
            uint8_t        payloadIndex = 0;
            const uint32_t start        = millis();

            while (millis() - start < options.ioTimeoutMs)
            {
                while (_serial->available())
                {
                    const uint8_t b = static_cast<uint8_t>(_serial->read());
                    if (headerIndex < sizeof(header))
                    {
                        header[headerIndex++] = b;
                        if (headerIndex == sizeof(header))
                        {
                            if (header[0] != 0xC1 || header[1] != startAddress || header[2] != length)
                            {
                                header[0]   = header[1];
                                header[1]   = header[2];
                                headerIndex = 2;
                            }
                        }
                        continue;
                    }

                    outRegisters[payloadIndex++] = b;
                    if (payloadIndex >= length)
                    {
                        return true;
                    }
                }
                delay(1);
            }
            return false;
        }

        bool readRawConfigInternal(RawConfig &outConfig, const EnsureOptions &options)
        {
            uint8_t registers[9] = {};
            if (!readRegistersInternal(0x00, registers, sizeof(registers), options))
            {
                return false;
            }
            outConfig = rawConfigFromBytes(registers);
            return true;
        }

        bool writeRegistersInternal(
            uint8_t startAddress,
            const uint8_t *registers,
            uint8_t length,
            uint8_t command,
            const EnsureOptions &options)
        {
            if (_serial == nullptr || registers == nullptr || length == 0)
            {
                return false;
            }

            uint8_t frame[12] = {};
            frame[0]          = command;
            frame[1]          = startAddress;
            frame[2]          = length;
            memcpy(frame + 3, registers, length);

            flushInput();
            if (!waitAuxHigh(options.auxTimeoutMs))
            {
                return false;
            }
            const size_t written = _serial->write(frame, static_cast<size_t>(length + 3));
            _serial->flush();
            delay(options.writeSettleMs);
            waitAuxHigh(options.auxTimeoutMs);
            return written == static_cast<size_t>(length + 3);
        }

        bool writePersistentConfig(const RawConfig &config, const EnsureOptions &options)
        {
            uint8_t registers[9] = {};
            rawConfigToBytes(config, registers);
            return writeRegistersInternal(0x00, registers, sizeof(registers), 0xC0, options);
        }

        void logf(const char *level, const char *fmt, ...) const
        {
            if (_logger == nullptr || level == nullptr || fmt == nullptr)
            {
                return;
            }
            char    buffer[160];
            va_list args;
            va_start(args, fmt);
            vsnprintf(buffer, sizeof(buffer), fmt, args);
            va_end(args);
            _logger->print("[E22][");
            _logger->print(level);
            _logger->print("] ");
            _logger->println(buffer);
        }

        HardwareSerial *_serial;
        ControlPins     _pins;
        uint32_t        _runtimeBaud;
        bool            _initialized;
        Print          *_logger;
        uint8_t         _lastPacketRssiRaw;
        bool            _hasLastPacketRssi;
        uint32_t        _lastPacketRssiAtMs;
        uint8_t         _lastAmbientRssiRaw;
        bool            _hasLastAmbientRssi;
        uint32_t        _lastAmbientRssiAtMs;
    };

} // namespace E22Radio
