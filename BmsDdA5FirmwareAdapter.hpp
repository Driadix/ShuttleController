#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "ShuttleProtocol.h"
#include "BmsDdA5.hpp"

namespace BmsDdA5Firmware {

struct WarnConfig {
  uint32_t countMismatchMs = 5000U;
  uint32_t protectionMs = 60000U;
  uint32_t sumMismatchMs = 5000U;
  uint32_t cellDeltaMs = 60000U;
  uint32_t cellLowMs = 60000U;
  uint32_t transportMs = 2000U;
  uint16_t sumMismatchAbs_mV = 500U;
  uint16_t cellDeltaWarn_mV = 300U;
  uint16_t cellLowWarn_mV = 2500U;
};

class Adapter {
public:
  typedef void (*LogFn)(LogLevel level, const char* format, ...);

  explicit Adapter(Stream& port,
                   uint8_t dePin,
                   LogFn logFn,
                   const BmsDdA5::Config& bmsConfig = BmsDdA5::Config(),
                   const WarnConfig& warnConfig = WarnConfig())
      : port_(&port),
        dePin_(dePin),
        logFn_(logFn),
        hooks_(makeHooks(this)),
        client_(hooks_, bmsConfig),
        warnConfig_(warnConfig) {
    resetWarnState();
  }

  void begin(uint32_t nowMs) {
    client_.begin(nowMs);
    resetWarnState();
  }

  void tick(uint32_t nowMs, BmsDdA5::ActivityHint activity) {
    const uint32_t events = client_.tick(nowMs, activity);
    logWarnEvents(nowMs, events);
    logStaleWarning(nowMs);
  }

  bool requestNow(BmsDdA5::RequestKind kind, uint32_t nowMs) {
    return client_.requestNow(kind, nowMs);
  }

  uint8_t socPercent() const {
    return client_.snapshot().socPercent;
  }

  uint16_t packVoltage_mV() const {
    return client_.snapshot().packVoltage_mV;
  }

  bool isFresh(uint32_t nowMs) const {
    return client_.isFresh(nowMs);
  }

  const BmsDdA5::Snapshot& snapshot() const {
    return client_.snapshot();
  }

  const BmsDdA5::Diagnostics& diagnostics() const {
    return client_.diagnostics();
  }

  const BmsDdA5::Config& config() const {
    return client_.config();
  }

private:
  struct WarnState {
    uint32_t lastCountMismatchMs = kNeverMs;
    uint32_t lastProtectionMs = kNeverMs;
    uint32_t lastSumMismatchMs = kNeverMs;
    uint32_t lastCellDeltaMs = kNeverMs;
    uint32_t lastCellLowMs = kNeverMs;
    uint32_t lastTransportMs = kNeverMs;
    uint32_t lastStaleMs = kNeverMs;
    bool staleWarned = false;
  };

  static const uint32_t kNeverMs = 0xFFFFFFFFUL;

  static BmsDdA5::Hooks makeHooks(Adapter* self) {
    BmsDdA5::Hooks hooks;
    hooks.write = &Adapter::writeThunk;
    hooks.readByte = &Adapter::readThunk;
    hooks.setDriverEnable = &Adapter::setDriverEnableThunk;
    hooks.user = self;
    return hooks;
  }

  static size_t writeThunk(const uint8_t* data, size_t len, void* user) {
    Adapter* self = static_cast<Adapter*>(user);
    if (!self || !self->port_) return 0U;
    return self->port_->write(data, len);
  }

  static int readThunk(void* user) {
    Adapter* self = static_cast<Adapter*>(user);
    if (!self || !self->port_) return -1;
    if (self->port_->available() <= 0) return -1;
    return self->port_->read();
  }

  static void setDriverEnableThunk(bool enabled, void* user) {
    Adapter* self = static_cast<Adapter*>(user);
    if (!self) return;

    if (enabled) {
      pinMode(self->dePin_, OUTPUT);
      digitalWrite(self->dePin_, HIGH);
      return;
    }

    digitalWrite(self->dePin_, LOW);
    pinMode(self->dePin_, INPUT_PULLDOWN);
  }

  static bool due(uint32_t nowMs, uint32_t& lastMs, uint32_t intervalMs) {
    if (lastMs == kNeverMs || intervalMs == 0U ||
        static_cast<uint32_t>(nowMs - lastMs) >= intervalMs) {
      lastMs = nowMs;
      return true;
    }
    return false;
  }

  void resetWarnState() {
    warnState_ = WarnState();
  }

  void logWarnEvents(uint32_t nowMs, uint32_t events) {
    if (!logFn_ || events == 0U) return;

    const BmsDdA5::Snapshot& snapshot = client_.snapshot();
    const BmsDdA5::Diagnostics& diagnostics = client_.diagnostics();

    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::BasicUpdated)) {
      if (snapshot.hasCell &&
          snapshot.seriesCellCount > 0U &&
          snapshot.cellCount > 0U &&
          snapshot.seriesCellCount != snapshot.cellCount &&
          due(nowMs, warnState_.lastCountMismatchMs, warnConfig_.countMismatchMs)) {
        logFn_(LOG_WARN, "BAT cnt 03=%u 04=%u",
               snapshot.seriesCellCount, snapshot.cellCount);
      }

      if (snapshot.protectionFlags != 0U &&
          due(nowMs, warnState_.lastProtectionMs, warnConfig_.protectionMs)) {
        logFn_(LOG_WARN, "BAT prot=%04X fet=%02X soc=%u",
               snapshot.protectionFlags, snapshot.fetStatus, snapshot.socPercent);
      }
    }

    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::CellUpdated)) {
      if (snapshot.packVoltage_mV > 0U && snapshot.cellSum_mV > 0U) {
        const int32_t diffMv = static_cast<int32_t>(snapshot.cellSum_mV) -
                               static_cast<int32_t>(snapshot.packVoltage_mV);
        const int32_t absDiffMv = (diffMv < 0) ? -diffMv : diffMv;
        if (absDiffMv > static_cast<int32_t>(warnConfig_.sumMismatchAbs_mV) &&
            due(nowMs, warnState_.lastSumMismatchMs, warnConfig_.sumMismatchMs)) {
          logFn_(LOG_WARN, "BAT sum mismatch:%ld", static_cast<long>(diffMv));
        }
      }

      if (snapshot.cellDelta_mV > warnConfig_.cellDeltaWarn_mV &&
          due(nowMs, warnState_.lastCellDeltaMs, warnConfig_.cellDeltaMs)) {
        logFn_(LOG_WARN, "BAT cell delta=%u", snapshot.cellDelta_mV);
      }

      if (snapshot.cellMin_mV > 0U &&
          snapshot.cellMin_mV < warnConfig_.cellLowWarn_mV &&
          due(nowMs, warnState_.lastCellLowMs, warnConfig_.cellLowMs)) {
        logFn_(LOG_WARN, "BAT cell low=%u", snapshot.cellMin_mV);
      }
    }

    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::Timeout) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT timeout cmd=%02X", static_cast<uint8_t>(diagnostics.lastRequested));
      return;
    }
    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::Overflow) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT overflow");
      return;
    }
    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::BadFrame) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT bad frame");
      return;
    }
    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::BadChecksum) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT crc");
      return;
    }
    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::BadStatus) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT status cmd=%02X", static_cast<uint8_t>(diagnostics.lastRequested));
      return;
    }
    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::CommandMismatch) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT cmd miss");
      return;
    }
    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::LengthError) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT len bad");
      return;
    }
    if (BmsDdA5::hasEvent(events, BmsDdA5::Event::TxShort) &&
        due(nowMs, warnState_.lastTransportMs, warnConfig_.transportMs)) {
      logFn_(LOG_WARN, "BAT tx short");
    }
  }

  void logStaleWarning(uint32_t nowMs) {
    if (!logFn_) return;

    if (client_.isFresh(nowMs)) {
      warnState_.staleWarned = false;
      return;
    }

    const BmsDdA5::Snapshot& snapshot = client_.snapshot();
    const BmsDdA5::Config& cfg = client_.config();

    if (snapshot.lastValidFrameMs == 0U) {
      if (nowMs >= cfg.staleWarnMs &&
          (!warnState_.staleWarned ||
           static_cast<uint32_t>(nowMs - warnState_.lastStaleMs) >= cfg.staleRepeatMs)) {
        logFn_(LOG_WARN, "BAT no valid frame");
        warnState_.staleWarned = true;
        warnState_.lastStaleMs = nowMs;
      }
      return;
    }

    const uint32_t staleAge = static_cast<uint32_t>(nowMs - snapshot.lastValidFrameMs);
    if (staleAge >= cfg.staleWarnMs &&
        (!warnState_.staleWarned ||
         static_cast<uint32_t>(nowMs - warnState_.lastStaleMs) >= cfg.staleRepeatMs)) {
      logFn_(LOG_WARN, "BAT stale: %lus", staleAge / 1000U);
      warnState_.staleWarned = true;
      warnState_.lastStaleMs = nowMs;
    }
  }

  Stream* port_;
  uint8_t dePin_;
  LogFn logFn_;
  BmsDdA5::Hooks hooks_;
  BmsDdA5::Client client_;
  WarnConfig warnConfig_;
  WarnState warnState_;
};

}  // namespace BmsDdA5Firmware
