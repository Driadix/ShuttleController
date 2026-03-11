#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace BmsDdA5 {

enum class ActivityHint : uint8_t {
    Idle = 0,
    Active,
    HighLoad
};

enum class RequestKind : uint8_t {
    BasicInfo = 0x03,
    CellInfo = 0x04,
    DeviceInfo = 0x05
};

enum class Event : uint32_t {
    None             = 0,
    BasicUpdated     = 1UL << 0,
    CellUpdated      = 1UL << 1,
    DeviceUpdated    = 1UL << 2,
    FreshnessLost    = 1UL << 3,
    FreshnessRestored= 1UL << 4,
    Timeout          = 1UL << 5,
    Overflow         = 1UL << 6,
    BadFrame         = 1UL << 7,
    BadChecksum      = 1UL << 8,
    BadStatus        = 1UL << 9,
    CommandMismatch  = 1UL << 10,
    LengthError      = 1UL << 11,
    TxShort          = 1UL << 12
};

inline uint32_t eventMask(Event eventValue) {
    return static_cast<uint32_t>(eventValue);
}

inline bool hasEvent(uint32_t events, Event eventValue) {
    return (events & eventMask(eventValue)) != 0U;
}

struct Hooks {
    size_t (*write)(const uint8_t* data, size_t len, void* user);
    int (*readByte)(void* user);
    void (*setDriverEnable)(bool enabled, void* user);
    void* user;

    Hooks()
        : write(0), readByte(0), setDriverEnable(0), user(0) {}
};

struct Config {
    uint32_t txHoldMs = 12U;
    uint32_t rxTimeoutMs = 140U;
    uint32_t basicIdleMs = 1000U;
    uint32_t basicActiveMs = 3000U;
    uint32_t basicHighLoadMs = 60000U;
    uint32_t cellIdleMs = 30000U;
    uint32_t deviceIdleMs = 120000U;
    uint32_t staleWarnMs = 15000U;
    uint32_t staleRepeatMs = 60000U;
    uint8_t maxStoredCells = 24U;
    uint8_t maxStoredNtc = 4U;
    uint8_t maxDeviceInfoLen = 24U;
};

struct Snapshot {
    bool hasBasic = false;
    bool hasCell = false;
    bool hasDevice = false;

    uint16_t packVoltage_mV = 0;
    int16_t packCurrent_cA = 0;
    uint16_t remainCapacity_cAh = 0;
    uint16_t nominalCapacity_cAh = 0;
    uint16_t cycleCount = 0;
    uint16_t protectionFlags = 0;
    uint8_t fetStatus = 0;
    uint8_t socPercent = 0;
    uint8_t seriesCellCount = 0;
    uint8_t ntcCount = 0;
    int16_t ntcTemp_dC[4] = {0, 0, 0, 0};

    uint8_t cellCount = 0;
    uint16_t cellMv[24] = {0};
    uint16_t cellMin_mV = 0;
    uint16_t cellMax_mV = 0;
    uint16_t cellDelta_mV = 0;
    uint32_t cellSum_mV = 0;

    char deviceInfo[24] = {0};

    uint32_t lastValidFrameMs = 0;
    uint32_t lastBasicUpdateMs = 0;
    uint32_t lastCellUpdateMs = 0;
    uint32_t lastDeviceUpdateMs = 0;
};

struct Diagnostics {
    uint32_t timeouts = 0;
    uint32_t crcErrors = 0;
    uint32_t badStatus = 0;
    uint32_t cmdMismatch = 0;
    uint32_t badFrames = 0;
    uint32_t lengthErrors = 0;
    uint32_t rxOverflows = 0;
    uint32_t txShorts = 0;
    uint32_t staleTransitions = 0;
    Event lastErrorEvent = Event::None;
    RequestKind lastRequested = RequestKind::BasicInfo;
};

class Client {
public:
    explicit Client(const Hooks& hooks, const Config& config = Config())
        : hooks_(hooks), config_(sanitizeConfig(config)) {
        initializeRuntime(0U, false);
    }

    void begin(uint32_t nowMs) {
        initializeRuntime(nowMs, true);
    }

    uint32_t tick(uint32_t nowMs, ActivityHint activity) {
        uint32_t events = 0U;

        if (state_ == TransportState::TxSent) {
            if (timeReached(nowMs, txReleaseAtMs_)) {
                setDriverEnable(false);
                state_ = TransportState::RxWait;
            }
        }

        if (state_ == TransportState::RxWait) {
            while (true) {
                const int rawByte = readByte();
                if (rawByte < 0) break;

                const uint8_t rx = static_cast<uint8_t>(rawByte);
                if (rxLen_ == 0U && rx != 0xDDU) continue;

                if (rxLen_ >= kRxFrameCapacity) {
                    setError(Event::Overflow, &events);
                    resetTransaction();
                    break;
                }

                rxFrame_[rxLen_++] = rx;

                if (rxLen_ >= 4U && expectedLen_ == 0U) {
                    expectedLen_ = static_cast<uint8_t>(rxFrame_[3] + 7U);
                    if (expectedLen_ > kRxFrameCapacity) {
                        setError(Event::LengthError, &events);
                        resetTransaction();
                        break;
                    }
                }

                if (expectedLen_ > 0U && rxLen_ >= expectedLen_) {
                    (void)processFrame(nowMs, &events);
                    resetTransaction();
                    break;
                }
            }

            if (state_ == TransportState::RxWait && timeReached(nowMs, rxTimeoutAtMs_)) {
                setError(Event::Timeout, &events);
                resetTransaction();
            }

            events |= updateFreshness(nowMs);
            return events;
        }

        if (state_ != TransportState::Idle) {
            events |= updateFreshness(nowMs);
            return events;
        }

        if (activity != ActivityHint::HighLoad) {
            const uint32_t basicInterval =
                (activity == ActivityHint::Idle) ? config_.basicIdleMs : config_.basicActiveMs;

            if (elapsedSince(nowMs, lastBasicPollMs_) >= basicInterval) {
                startRequest(RequestKind::BasicInfo, nowMs, &events);
                events |= updateFreshness(nowMs);
                return events;
            }
        }

        if (activity == ActivityHint::Idle) {
            if (elapsedSince(nowMs, lastCellPollMs_) >= config_.cellIdleMs) {
                startRequest(RequestKind::CellInfo, nowMs, &events);
                events |= updateFreshness(nowMs);
                return events;
            }

            if (deviceInfoBootPending_ || elapsedSince(nowMs, lastDevicePollMs_) >= config_.deviceIdleMs) {
                startRequest(RequestKind::DeviceInfo, nowMs, &events);
                events |= updateFreshness(nowMs);
                return events;
            }
        }

        events |= updateFreshness(nowMs);
        return events;
    }

    const Snapshot& snapshot() const {
        return snapshot_;
    }

    const Diagnostics& diagnostics() const {
        return diagnostics_;
    }

    const Config& config() const {
        return config_;
    }

    bool isFresh(uint32_t nowMs) const {
        if (config_.staleWarnMs == 0U) return true;
        if (snapshot_.lastValidFrameMs == 0U) {
            return nowMs < config_.staleWarnMs;
        }
        return elapsedSince(nowMs, snapshot_.lastValidFrameMs) < config_.staleWarnMs;
    }

    bool isTransactionActive() const {
        return state_ != TransportState::Idle;
    }

    RequestKind activeRequest() const {
        return activeRequest_;
    }

    void reset(uint32_t nowMs) {
        initializeRuntime(nowMs, true);
    }

    bool requestNow(RequestKind kind, uint32_t nowMs) {
        if (state_ != TransportState::Idle) return false;
        startRequest(kind, nowMs, 0);
        return true;
    }

private:
    enum class TransportState : uint8_t {
        Idle = 0,
        TxSent,
        RxWait
    };

    static const uint8_t kTxFrameLen = 7U;
    static const uint8_t kRxFrameCapacity = 64U;
    static const uint8_t kStoredCellCapacity = 24U;
    static const uint8_t kStoredNtcCapacity = 4U;
    static const uint8_t kStoredDeviceInfoCapacity = 24U;

    Hooks hooks_;
    Config config_;
    Snapshot snapshot_;
    Diagnostics diagnostics_;

    TransportState state_;
    RequestKind activeRequest_;

    uint8_t txFrame_[kTxFrameLen];
    uint8_t rxFrame_[kRxFrameCapacity];
    uint8_t rxLen_;
    uint8_t expectedLen_;

    uint32_t txReleaseAtMs_;
    uint32_t rxTimeoutAtMs_;
    uint32_t lastBasicPollMs_;
    uint32_t lastCellPollMs_;
    uint32_t lastDevicePollMs_;

    bool deviceInfoBootPending_;
    bool freshState_;

    static Config sanitizeConfig(const Config& input) {
        Config output = input;

        if (output.maxStoredCells == 0U || output.maxStoredCells > kStoredCellCapacity) {
            output.maxStoredCells = kStoredCellCapacity;
        }
        if (output.maxStoredNtc > kStoredNtcCapacity) {
            output.maxStoredNtc = kStoredNtcCapacity;
        }
        if (output.maxDeviceInfoLen == 0U || output.maxDeviceInfoLen > kStoredDeviceInfoCapacity) {
            output.maxDeviceInfoLen = kStoredDeviceInfoCapacity;
        }

        return output;
    }

    static uint32_t elapsedSince(uint32_t nowMs, uint32_t thenMs) {
        return static_cast<uint32_t>(nowMs - thenMs);
    }

    static bool timeReached(uint32_t nowMs, uint32_t targetMs) {
        return static_cast<int32_t>(nowMs - targetMs) >= 0;
    }

    static uint16_t readU16BE(const uint8_t* src) {
        return static_cast<uint16_t>((static_cast<uint16_t>(src[0]) << 8) | src[1]);
    }

    void initializeRuntime(uint32_t nowMs, bool applyDriverState) {
        snapshot_ = Snapshot();
        diagnostics_ = Diagnostics();
        memset(txFrame_, 0, sizeof(txFrame_));
        memset(rxFrame_, 0, sizeof(rxFrame_));

        state_ = TransportState::Idle;
        activeRequest_ = RequestKind::BasicInfo;
        diagnostics_.lastRequested = RequestKind::BasicInfo;
        diagnostics_.lastErrorEvent = Event::None;

        rxLen_ = 0U;
        expectedLen_ = 0U;
        txReleaseAtMs_ = 0U;
        rxTimeoutAtMs_ = 0U;

        lastBasicPollMs_ = nowMs - config_.basicIdleMs;
        lastCellPollMs_ = nowMs - config_.cellIdleMs;
        lastDevicePollMs_ = nowMs - config_.deviceIdleMs;

        deviceInfoBootPending_ = true;
        snapshot_.deviceInfo[0] = '\0';

        if (applyDriverState) {
            setDriverEnable(false);
        }
        freshState_ = isFresh(nowMs);
    }

    size_t writeData(const uint8_t* data, size_t len) {
        if (!hooks_.write) return 0U;
        return hooks_.write(data, len, hooks_.user);
    }

    int readByte() {
        if (!hooks_.readByte) return -1;
        return hooks_.readByte(hooks_.user);
    }

    void setDriverEnable(bool enabled) {
        if (hooks_.setDriverEnable) {
            hooks_.setDriverEnable(enabled, hooks_.user);
        }
    }

    void drainInput() {
        while (readByte() >= 0) {
        }
    }

    void buildRequestFrame(RequestKind kind, uint8_t* outFrame) {
        outFrame[0] = 0xDDU;
        outFrame[1] = 0xA5U;
        outFrame[2] = static_cast<uint8_t>(kind);
        outFrame[3] = 0x00U;
        const uint16_t checksum = static_cast<uint16_t>(~static_cast<uint16_t>(outFrame[2]) + 1U);
        outFrame[4] = static_cast<uint8_t>(checksum >> 8);
        outFrame[5] = static_cast<uint8_t>(checksum & 0xFFU);
        outFrame[6] = 0x77U;
    }

    void resetTransaction() {
        setDriverEnable(false);
        state_ = TransportState::Idle;
        rxLen_ = 0U;
        expectedLen_ = 0U;
    }

    void recordCounter(Event eventValue) {
        switch (eventValue) {
            case Event::Timeout:
                diagnostics_.timeouts++;
                break;
            case Event::Overflow:
                diagnostics_.rxOverflows++;
                break;
            case Event::BadFrame:
                diagnostics_.badFrames++;
                break;
            case Event::BadChecksum:
                diagnostics_.crcErrors++;
                break;
            case Event::BadStatus:
                diagnostics_.badStatus++;
                break;
            case Event::CommandMismatch:
                diagnostics_.cmdMismatch++;
                break;
            case Event::LengthError:
                diagnostics_.lengthErrors++;
                break;
            case Event::TxShort:
                diagnostics_.txShorts++;
                break;
            default:
                break;
        }
    }

    void setError(Event eventValue, uint32_t* events) {
        recordCounter(eventValue);
        diagnostics_.lastErrorEvent = eventValue;
        if (events) {
            *events |= eventMask(eventValue);
        }
    }

    uint32_t updateFreshness(uint32_t nowMs) {
        const bool fresh = isFresh(nowMs);
        if (fresh == freshState_) return 0U;

        freshState_ = fresh;
        diagnostics_.staleTransitions++;
        return eventMask(fresh ? Event::FreshnessRestored : Event::FreshnessLost);
    }

    void startRequest(RequestKind kind, uint32_t nowMs, uint32_t* events) {
        drainInput();

        buildRequestFrame(kind, txFrame_);
        activeRequest_ = kind;
        diagnostics_.lastRequested = kind;
        rxLen_ = 0U;
        expectedLen_ = 0U;

        setDriverEnable(true);
        const size_t written = writeData(txFrame_, sizeof(txFrame_));
        if (kind == RequestKind::BasicInfo) {
            lastBasicPollMs_ = nowMs;
        } else if (kind == RequestKind::CellInfo) {
            lastCellPollMs_ = nowMs;
        } else if (kind == RequestKind::DeviceInfo) {
            lastDevicePollMs_ = nowMs;
            deviceInfoBootPending_ = false;
        }

        if (written != sizeof(txFrame_)) {
            setError(Event::TxShort, events);
            resetTransaction();
            return;
        }

        txReleaseAtMs_ = nowMs + config_.txHoldMs;
        rxTimeoutAtMs_ = nowMs + config_.rxTimeoutMs;
        state_ = TransportState::TxSent;
    }

    bool checksumOk(uint8_t payloadLen) const {
        const uint16_t crcRx = readU16BE(&rxFrame_[4U + payloadLen]);

        uint16_t calc1 = 0U;
        for (uint8_t i = 1U; i < static_cast<uint8_t>(4U + payloadLen); i++) calc1 += rxFrame_[i];
        calc1 = static_cast<uint16_t>(~calc1 + 1U);

        uint16_t calc2 = 0U;
        for (uint8_t i = 2U; i < static_cast<uint8_t>(4U + payloadLen); i++) calc2 += rxFrame_[i];
        calc2 = static_cast<uint16_t>(~calc2 + 1U);

        uint16_t calc3 = 0U;
        for (uint8_t i = 3U; i < static_cast<uint8_t>(4U + payloadLen); i++) calc3 += rxFrame_[i];
        calc3 = static_cast<uint16_t>(~calc3 + 1U);

        return (crcRx == calc1 || crcRx == calc2 || crcRx == calc3);
    }

    bool parseBasicInfo(uint8_t payloadLen) {
        if (payloadLen < 20U) return false;

        const uint8_t* p = &rxFrame_[4];
        snapshot_.packVoltage_mV = static_cast<uint16_t>(readU16BE(&p[0]) * 10U);
        snapshot_.packCurrent_cA = static_cast<int16_t>(readU16BE(&p[2]));
        snapshot_.remainCapacity_cAh = readU16BE(&p[4]);
        snapshot_.nominalCapacity_cAh = readU16BE(&p[6]);
        snapshot_.cycleCount = readU16BE(&p[8]);
        snapshot_.protectionFlags = (payloadLen >= 18U) ? readU16BE(&p[16]) : 0U;

        const uint8_t soc = p[19];
        if (soc <= 100U) {
            snapshot_.socPercent = soc;
        }

        snapshot_.fetStatus = (payloadLen >= 21U) ? p[20] : 0U;
        snapshot_.seriesCellCount = (payloadLen >= 22U) ? p[21] : 0U;
        snapshot_.ntcCount = (payloadLen >= 23U) ? p[22] : 0U;

        for (uint8_t i = 0U; i < kStoredNtcCapacity; i++) snapshot_.ntcTemp_dC[i] = 0;

        uint8_t ntcToStore = snapshot_.ntcCount;
        if (ntcToStore > config_.maxStoredNtc) ntcToStore = config_.maxStoredNtc;
        if (ntcToStore > kStoredNtcCapacity) ntcToStore = kStoredNtcCapacity;

        for (uint8_t i = 0U; i < ntcToStore; i++) {
            const uint8_t idx = static_cast<uint8_t>(23U + i * 2U);
            if (static_cast<uint8_t>(idx + 1U) >= payloadLen) break;
            const int16_t tRaw = static_cast<int16_t>(readU16BE(&p[idx]));
            snapshot_.ntcTemp_dC[i] = static_cast<int16_t>(tRaw - 2731);
        }

        return true;
    }

    bool parseCellInfo(uint8_t payloadLen) {
        if ((payloadLen & 0x01U) != 0U) return false;

        const uint8_t* p = &rxFrame_[4];
        uint8_t cells = static_cast<uint8_t>(payloadLen / 2U);
        if (cells > config_.maxStoredCells) cells = config_.maxStoredCells;
        if (cells > kStoredCellCapacity) cells = kStoredCellCapacity;

        snapshot_.cellCount = cells;

        uint16_t vMin = 0xFFFFU;
        uint16_t vMax = 0U;
        uint32_t vSum = 0U;

        for (uint8_t i = 0U; i < cells; i++) {
            const uint16_t mv = readU16BE(&p[i * 2U]);
            snapshot_.cellMv[i] = mv;
            if (mv < vMin) vMin = mv;
            if (mv > vMax) vMax = mv;
            vSum += mv;
        }

        for (uint8_t i = cells; i < kStoredCellCapacity; i++) snapshot_.cellMv[i] = 0U;

        if (cells == 0U) {
            snapshot_.cellMin_mV = 0U;
            snapshot_.cellMax_mV = 0U;
            snapshot_.cellDelta_mV = 0U;
            snapshot_.cellSum_mV = 0U;
        } else {
            snapshot_.cellMin_mV = vMin;
            snapshot_.cellMax_mV = vMax;
            snapshot_.cellDelta_mV = static_cast<uint16_t>(vMax - vMin);
            snapshot_.cellSum_mV = vSum;
        }

        return true;
    }

    bool parseDeviceInfo(uint8_t payloadLen) {
        const uint8_t* p = &rxFrame_[4];
        const uint8_t maxOut = static_cast<uint8_t>(config_.maxDeviceInfoLen - 1U);
        uint8_t outIdx = 0U;

        for (uint8_t i = 0U; i < payloadLen && outIdx < maxOut; i++) {
            char ch = static_cast<char>(p[i]);
            if (ch < 32 || ch > 126) ch = '.';
            snapshot_.deviceInfo[outIdx++] = ch;
        }

        snapshot_.deviceInfo[outIdx] = '\0';
        if (outIdx == 0U) {
            snapshot_.deviceInfo[0] = 'n';
            snapshot_.deviceInfo[1] = '/';
            snapshot_.deviceInfo[2] = 'a';
            snapshot_.deviceInfo[3] = '\0';
        }

        return true;
    }

    bool processFrame(uint32_t nowMs, uint32_t* events) {
        if (rxLen_ < 7U) {
            setError(Event::BadFrame, events);
            return false;
        }

        if (rxFrame_[0] != 0xDDU || rxFrame_[rxLen_ - 1U] != 0x77U) {
            setError(Event::BadFrame, events);
            return false;
        }

        const uint8_t payloadLen = rxFrame_[3];
        const uint8_t frameLen = static_cast<uint8_t>(payloadLen + 7U);
        if (frameLen > rxLen_ || frameLen > kRxFrameCapacity) {
            setError(Event::LengthError, events);
            return false;
        }

        const uint8_t cmd = rxFrame_[1];
        const uint8_t statusCode = rxFrame_[2];

        if (cmd != static_cast<uint8_t>(activeRequest_)) {
            setError(Event::CommandMismatch, events);
            return false;
        }

        if (statusCode != 0x00U) {
            setError(Event::BadStatus, events);
            return false;
        }

        if (!checksumOk(payloadLen)) {
            setError(Event::BadChecksum, events);
            return false;
        }

        bool parsed = false;
        if (cmd == static_cast<uint8_t>(RequestKind::BasicInfo)) {
            parsed = parseBasicInfo(payloadLen);
            if (parsed) {
                snapshot_.hasBasic = true;
                snapshot_.lastBasicUpdateMs = nowMs;
                if (events) *events |= eventMask(Event::BasicUpdated);
            }
        } else if (cmd == static_cast<uint8_t>(RequestKind::CellInfo)) {
            parsed = parseCellInfo(payloadLen);
            if (parsed) {
                snapshot_.hasCell = true;
                snapshot_.lastCellUpdateMs = nowMs;
                if (events) *events |= eventMask(Event::CellUpdated);
            }
        } else if (cmd == static_cast<uint8_t>(RequestKind::DeviceInfo)) {
            parsed = parseDeviceInfo(payloadLen);
            if (parsed) {
                snapshot_.hasDevice = true;
                snapshot_.lastDeviceUpdateMs = nowMs;
                if (events) *events |= eventMask(Event::DeviceUpdated);
            }
        } else {
            setError(Event::CommandMismatch, events);
            return false;
        }

        if (!parsed) {
            setError(Event::LengthError, events);
            return false;
        }

        snapshot_.lastValidFrameMs = nowMs;
        return true;
    }
};

}  // namespace BmsDdA5
