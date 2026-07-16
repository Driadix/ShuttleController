#include "AS5600.h"
#include "AlertManager.h"
#include "BmsDdA5FirmwareAdapter.hpp"
#include "E22Radio.hpp"
#include "STM32TimerInterrupt.h"
#include "STM32_CAN.h"
#include "ShuttleProtocol.h"
#include "TOF_Sense.h"
#include "TofBusMonitor.h"
#include <IWatchdog.h>
#include <STM32RTC.h>
#include <String.h>
#include <Wire.h>

#pragma region Макросы...
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#endif

#ifndef CONNECTION_LOGS
#define CONNECTION_LOGS 0
#endif

#ifndef I2C_DIAG_LOGS
#define I2C_DIAG_LOGS 0
#endif

constexpr size_t LOG_FORMAT_MAX_PRINTABLE_CHARS = 255;

enum BreadcrumbStage : uint8_t
{
    BC_BOOT = 1,
    BC_LOOP,
    BC_SYSTEM_YIELD,
    BC_BMS_TICK,
    BC_TX_TELEMETRY,
    BC_TX_SENSORS,
    BC_TX_STATS,
    BC_TX_LINK_HEALTH,
    BC_POLL_DISPLAY,
    BC_POLL_RADIO,
    BC_TOF_POLL,
    BC_TOF_PREFLIGHT,
    BC_TOF_PROBE,
    BC_TOF_READ,
    BC_TOF_RECOVERY,
    BC_AS5600_ANGLE,
    BC_SERIAL_COMMAND
};

#pragma pack(push, 1)
struct BreadcrumbRecord
{
    uint32_t magicWord;
    uint32_t sequence;
    uint32_t millisAt;
    uint8_t  stage;
    uint8_t  sensor;
    uint8_t  status;
    uint8_t  detail;
    uint16_t crc16;
    uint16_t reserved;
};
#pragma pack(pop)

#if CONNECTION_LOGS
#define DIAG_ONLY(code)                                                                                                     \
    do                                                                                                                      \
    {                                                                                                                       \
        code;                                                                                                               \
    } while (0)
#else
#define DIAG_ONLY(code)                                                                                                     \
    do                                                                                                                      \
    {                                                                                                                       \
    } while (0)
#endif

#define LOG_RATE_LIMITED(level, interval, format, ...)                                                                      \
    do                                                                                                                      \
    {                                                                                                                       \
        static_assert(                                                                                                      \
            sizeof(format) - 1 <= LOG_FORMAT_MAX_PRINTABLE_CHARS,                                                           \
            "Log format string too long for logStringBuffer.");                                                             \
        static uint32_t lastLog = 0;                                                                                        \
        if (millis() - lastLog > (interval))                                                                                \
        {                                                                                                                   \
            makeLogImpl(level, format, ##__VA_ARGS__);                                                                      \
            lastLog = millis();                                                                                             \
        }                                                                                                                   \
    } while (0)

#define makeLog(level, format, ...)                                                                                         \
    do                                                                                                                      \
    {                                                                                                                       \
        static_assert(                                                                                                      \
            sizeof(format) - 1 <= LOG_FORMAT_MAX_PRINTABLE_CHARS,                                                           \
            "makeLog: format string literal exceeds logStringBuffer capacity.");                                             \
        makeLogImpl(level, format, ##__VA_ARGS__);                                                                          \
    } while (0)

#define makeReliableLog(level, format, ...)                                                                                 \
    do                                                                                                                      \
    {                                                                                                                       \
        static_assert(                                                                                                      \
            sizeof(format) - 1 <= LOG_FORMAT_MAX_PRINTABLE_CHARS,                                                           \
            "makeReliableLog: format string literal exceeds logStringBuffer capacity.");                                     \
        makeReliableLogImpl(level, format, ##__VA_ARGS__);                                                                  \
    } while (0)

void                           makeLogImpl(LogLevel level, const char *format, ...);
void                           makeReliableLogImpl(LogLevel level, const char *format, ...);
void                           makeBootLog(LogLevel level, const char *format, ...);
static const char             *breadcrumbStageName(uint8_t stage);
static bool                    readBreadcrumb(BreadcrumbRecord *out);
static void                    writeBreadcrumb(BreadcrumbStage stage, uint8_t sensor = 0xFF, uint8_t status = 0, uint8_t detail = 0);
static void                    logPreviousBreadcrumbIfWatchdog(uint32_t resetFlags);
static void                    logBreadcrumbSummaryIfDue(uint32_t now);
static inline bool             isValidProvisionedShuttleId(int32_t id);
static inline bool             isProvisionedShuttle();
static inline bool             isSupportedCommand(uint8_t cmd);
static inline bool             isManualMoveCommand(uint8_t cmd);
static inline bool             isManualDistanceCommand(uint8_t cmd);
static inline bool             isContinuousManualCommand(uint8_t cmd);
static inline bool             isLiftCommand(uint8_t cmd);
static inline bool             isUnprovisionedCommandAllowed(uint8_t cmd);
static inline bool             isOutOfChannelExemptCommand(uint8_t cmd);
static inline bool             canAcceptCommandNow(uint8_t cmd, bool fromRadio);
static inline bool             canApplyConfigNow();
static inline bool             isPhysicallyStationary();
static inline void             performSystemReset();
static inline void             clearLatchedAlertsAndReturnToIdle(uint32_t now);
static inline void             resetTofDiagnostics();
static inline void             handlePendingCrash();
static void                    crashF1Irq();
static void                    crashF2Irq();
static void                    crashR1Irq();
static void                    crashR2Irq();
static void                    recordBootResetCause();
static void                    setResetRequestMarker(uint32_t flags);
static uint32_t                consumeResetRequestMarker();
static uint32_t                captureResetReasonFlags();
static const char             *primaryResetCategoryName(uint32_t flags);
static inline void             touchManualSession();
static inline void             beginManualRadioHold(uint32_t now);
static inline void             refreshManualRadioHoldWatchdog(uint32_t now);
static inline void             clearManualRadioHold();
static void                    scheduleBootloaderEntry();
static void                    enableBackupSramAccess(uint32_t readyTimeoutMs);
static bool                    consumeBootloaderResetRequest();
static void                    requestBootloaderResetEntry();
static void                    resetIntoBootloader();
static E22Radio::EnsureOptions makeRadioEnsureOptions();
static E22Radio::LogicalConfig makeRadioDesiredConfig(uint8_t nodeId);
enum class RadioTxAuxPolicy : uint8_t
{
    DropIfBusy,
    SendAfterTimeout
};
static void       reapplyRadioConfigForShuttleId(uint8_t newId, uint8_t previousId);
static inline int transportAvailableForWrite(Stream *port);
static size_t
writeTransportPayload(
    Stream *port,
    const uint8_t *buffer,
    uint16_t length,
    const E22Radio::FixedRoute *fixedRoute = NULL,
    RadioTxAuxPolicy auxPolicy = RadioTxAuxPolicy::DropIfBusy);
static ShuttleState predictTelemetryStateForAcceptedCommand(uint8_t cmd, bool fromRadio);
static void         populateTelemetryPacket(TelemetryPacket *pkt, int16_t shuttleStatusOverride = -1);
static void         sendAck(uint8_t seq, AckResult result, Stream *port, const E22Radio::FixedRoute *fixedRoute = NULL);
static void         sendTelemAck(
            uint8_t                     seq,
            AckResult                   result,
            Stream                     *port,
            int16_t                     shuttleStatusOverride = -1,
            const E22Radio::FixedRoute *fixedRoute            = NULL);
static void sendCommandAck(
    uint8_t   seq,
    AckResult result,
    Stream   *port,
    bool      suppressAck,
    bool      useTelemAck,
    int16_t   shuttleStatusOverride = -1);
static void                  sendTelemetryPacket(Stream *port);
static void                  sendSensorPacket(Stream *port);
static void                  sendStatsPacket(Stream *port);
static void                  sendLinkHealthPacket(Stream *port);
static void                  populateLinkHealthPacket(LinkHealthPacket *pkt);
static bool                  isRadioPacketRssiFresh(uint32_t now);
static BmsDdA5::ActivityHint mapBatteryActivity();
static void                  batterySafetyCheck(uint32_t now);
static void                  batterySafetyReset(uint32_t now);
static void                  initTofI2cBus();
static bool                  performTofSdaClockRecovery(const char *reason);
static bool                  performTofWireReinit(const char *reason);
static bool                  serviceTofBusMonitor(uint32_t now);
static inline bool           tofBusLinesHighNow();
static void                  setTofI2cClock(uint32_t clockHz);
static uint8_t               i2cProbeAddress(uint8_t address);
static uint8_t               scanTofAddressMask();
static void                  logI2cBootScanAtClock(uint32_t clockHz);
static void                  logTofI2cLines(const char *tag);
static void                  logI2cBootScan();
static void                  logTofBootSnapshot();
static void                  logI2cDiagSummaryIfDue(uint32_t now);
static bool                  readAs5600AngleChecked(uint16_t *rawAngle);
static bool                  readAs5600AngleForMotion(uint16_t *rawAngle);
static bool                  readAs5600AngleFresh(int *angleOut);
static uint16_t              readAs5600AngleForTelemetry();
static void                  logBootResetReplay();
static void                  logRadioRawConfig(LogLevel level, const char *tag, const E22Radio::RawConfig &raw);
static void                  logRadioDesiredConfig(LogLevel level, const char *tag, const E22Radio::LogicalConfig &config);
static void                  logRadioEnsureFailure(
                     LogLevel level,
                     const char *tag,
                     const E22Radio::LogicalConfig &desired,
                     const E22Radio::EnsureResult &result);
static void                  formatRadioBaudMask(uint8_t mask, char *buffer, size_t bufferSize);
static void                  retryRadioConfigIfIdle(uint32_t now);
static inline bool           batteryIsHighLoad();
static inline bool           batteryLowFaultLatched();
static bool                  batteryIsMotionLikeStatus(uint8_t st);

static inline void preserveManualStopOnAbort();
static inline bool shouldAbortLoop();
static inline bool isErrorActive();
static inline bool isOverrideCommand(uint8_t cmd);
static inline void countTxFrame(Stream *port);
static inline void countReplyTiming(Stream *port, uint32_t elapsedUs);
static inline bool isShuttleIdle();

void motor_Stop();
void motor_Force_Stop();
void motor_Speed(int spd);
void moove_Before_Pallete_F();
void moove_Before_Pallete_R();
void stop_Before_Pallete_F();
void stop_Before_Pallete_R();
void moove_Distance_F(int dist, int maxSpeed, int minSpeed);
void moove_Distance_F(int dist);
void moove_Distance_R(int dist, int maxSpeed, int minSpeed);
void moove_Distance_R(int dist);
void moove_Forward();
void moove_Reverse();
void moove_Right();
void moove_Left();

// Operations
void lifter_Down();
void lifter_Up();
void lifter_Stop();
void load_Pallete();
void unload_Pallete();
void single_Load();
void long_Load();
void long_Unload();
void long_Unload(uint8_t num);
void pallete_Compacting_F();
void pallete_Compacting_R();
void pallete_Counting_F();
void demo_Mode();

// System
void run_Cmd();
void send_Cmd();
void get_Distance();
void detect_Pallete();
void set_Position();
void blink_Work();
void blink_Warning();
void blink_Error();
void jumpToBootloader();

#define STATS_MAGIC_WORD        0xAA55BEF0
#define BKPSRAM_BASE_ADDR       0x40024000
#define RESET_MARKER_MAGIC_WORD 0x51E7A5E7UL
#define RESET_MARKER_MAGIC_ADDR (BKPSRAM_BASE_ADDR + 0x0FE8)
#define RESET_MARKER_FLAGS_ADDR (BKPSRAM_BASE_ADDR + 0x0FEC)
#define BOOTLOADER_MAGIC_WORD 0xB00710ADUL
#define BOOTLOADER_MAGIC_ADDR (BKPSRAM_BASE_ADDR + 0x0FF0)
#define BOOTLOADER_MAGIC_CONFIRM_WORD 0x4FF8EF52UL
#define BOOTLOADER_MAGIC_CONFIRM_ADDR (BKPSRAM_BASE_ADDR + 0x0FF4)
#define DIAG_BREADCRUMB_MAGIC_WORD 0xBADC0DE5UL
#define DIAG_BREADCRUMB_OFFSET     0x0F80
#define DIAG_BREADCRUMB_ADDR       (BKPSRAM_BASE_ADDR + DIAG_BREADCRUMB_OFFSET)
#define DIAG_BREADCRUMB_MAX_AGE_MS 600000UL

#pragma pack(push, 1)
struct SecureStats
{
    uint32_t    magicWord;
    StatsPacket payload;
    uint16_t    crc16;
    uint16_t    reserved;
};
#pragma pack(pop)

static_assert(sizeof(SecureStats) <= DIAG_BREADCRUMB_OFFSET, "SecureStats overlaps diagnostic breadcrumb.");
static_assert(
    DIAG_BREADCRUMB_OFFSET + sizeof(BreadcrumbRecord) <= 0x0FE8,
    "Diagnostic breadcrumb overlaps reset markers.");

SecureStats *volatile sramStats = (SecureStats *)BKPSRAM_BASE_ADDR;
static volatile uint32_t *const resetMarkerMagic = (volatile uint32_t *)RESET_MARKER_MAGIC_ADDR;
static volatile uint32_t *const resetMarkerFlags = (volatile uint32_t *)RESET_MARKER_FLAGS_ADDR;
static volatile uint32_t *const bootloaderMagicWord = (volatile uint32_t *)BOOTLOADER_MAGIC_ADDR;
static volatile uint32_t *const bootloaderMagicConfirm = (volatile uint32_t *)BOOTLOADER_MAGIC_CONFIRM_ADDR;
static volatile BreadcrumbRecord *const diagBreadcrumb = (volatile BreadcrumbRecord *)DIAG_BREADCRUMB_ADDR;
static uint32_t breadcrumbSequence = 0;
static uint32_t lastBootResetFlags = RESET_REASON_UNKNOWN;
static uint32_t lastBootResetMarkerFlags = RESET_REASON_NONE;

ProtocolParser               parserRadio;
ProtocolParser               parserDisplay;
E22Radio::Radio              e22Radio;
constexpr E22Radio::UartBaud kRadioHostBaud = E22Radio::UartBaud::B57600;

#define STATS_ATOMIC_UPDATE(action)                                                                                         \
    do                                                                                                                      \
    {                                                                                                                       \
        __disable_irq();                                                                                                    \
        action;                                                                                                             \
        sramStats->crc16 = ProtocolUtils::calcCRC16((uint8_t *)&sramStats->payload, sizeof(StatsPacket));                   \
        __enable_irq();                                                                                                     \
    } while (0)

#define CONFIG_SECTOR      FLASH_SECTOR_7
#define CONFIG_SECTOR_BASE 0x08060000
#define CONFIG_SECTOR_SIZE (128 * 1024)
#define CONFIG_PAGE_SIZE   512
#define CONFIG_TOTAL_PAGES (CONFIG_SECTOR_SIZE / CONFIG_PAGE_SIZE)

struct ConfigPageHeader
{
    uint8_t  state;
    uint8_t  reserved[1];
    uint16_t crc16;
};
#define MEDIAN(a, b, c) ((a) + (b) + (c) - min(min(a, b), c) - max(max(a, b), c))

#pragma endregion

#pragma region Пины и дефайны...

#define DL_UP       PC13 // Пин датчика положения лифтера в поднятом состоянии
#define DL_DOWN     PB4  // Пин датчика положения лифтера в опущенном состоянии
#define DATCHIK_F1  PA5  // Пин датчика 1 обнаружения паллета вперед
#define DATCHIK_F2  PC4  // Пин датчика 2 обнаружения паллета вперед
#define DATCHIK_R1  PB6  // Пин датчика 1 обнаружения паллета назад
#define DATCHIK_R2  PD2  // Пин датчика 2 обнаружения паллета назад
#define GREEN_LED   PC1  // Пин зеленого светодиода
#define BOARD_LED   PA1  // Пин светодиода на плате
#define RED_LED     PC3  // Пин красного светодиода ошибки
#define WHITE_LED   PC2  // Пин белого светодиода работы
#define ZOOMER      PA0  // Зумер
#define RADIO_E22_M0  PB15
#define RADIO_E22_M1  PB14
#define RADIO_E22_AUX PC0
#define RS485       PB13 // Пин передачи шины RS485
#define BUMPER_F1   PB7  // Пин бампера вперед
#define BUMPER_F2   PB3  // Пин бампера вперед
#define BUMPER_R1   PC12  // Пин бампера назад
#define BUMPER_R2   PA15  // Пин бампера назад
#define CHANNEL     PB5  // Пин датчика канала
#define TOF_I2C_SDA PB11
#define TOF_I2C_SCL PB10

#define FLASH_SECTOR_SIZE  4 * 1024 // 16 kb
#define EEPROM_PAGE_SIZE   512
#define EEPROM_TOTAL_PAGES FLASH_SECTOR_SIZE / EEPROM_PAGE_SIZE
#define EEPROM_HEADER_SIZE 1
#define EEPROM_DATA_SIZE   EEPROM_PAGE_SIZE - EEPROM_HEADER_SIZE

#define SerialLora    Serial3
#define SerialRS485   Serial2
#define SerialDisplay Serial1

constexpr uint8_t  NO_NEW_CMD                  = 0xFF;
constexpr uint8_t  kFirmwareVersionMajor       = 2;
constexpr uint8_t  kFirmwareVersionMinor       = 0;
constexpr uint8_t  kFirmwareVersionPatch       = 5;
constexpr uint32_t kBootLogSettleMs            = 250;
constexpr uint32_t kBootLogWriteTimeoutMs      = 250;
constexpr uint32_t kManualSessionIdleTimeoutMs = 60000;
constexpr uint32_t kManualRadioHoldWatchdogMs  = 3000;
constexpr uint32_t tofI2cClockHz                = 100000U;
constexpr uint32_t kTofI2cSlowProbeClockHz      = 100000U;
constexpr uint32_t kTofI2cFastProbeClockHz      = 400000U;
constexpr uint32_t kTofI2cRecoveryMinIntervalMs      = 5000U;
constexpr uint32_t kTofLineLowConfirmMs               = 2U;
constexpr uint32_t kTofLineReleaseConfirmMs           = 2U;
constexpr uint32_t kTofStaleTimeoutMs                 = 300U;
constexpr uint32_t kTofFailureLogIntervalMs           = 5000U;
constexpr uint32_t kBmsTofQuietGuardMs                = 5U;
constexpr uint8_t  kAs5600I2cAddress                 = 0x36U;
constexpr uint8_t  kAs5600AngleHighReg               = 0x0EU;
constexpr uint8_t  kAs5600AngleReadLength            = 2U;
constexpr uint8_t  kAs5600ExistingBeginPin           = 4U;
static_assert(kAs5600ExistingBeginPin == 4U, "Keep the controller AS5600 begin pin at 4.");
constexpr uint32_t kRadioConfigFailureWarnIntervalMs = 60000UL;
constexpr uint32_t kRadioPacketRssiFreshMs            = 10000UL;
constexpr uint32_t kLinkHealthPublishMs               = 5000UL;
constexpr uint32_t kRadioRuntimeAuxWaitMs             = 20UL;
constexpr uint32_t kAs5600FailLogIntervalMs           = 30000UL;
constexpr uint32_t kI2cDiagSummaryIntervalMs          = 30000UL;

#pragma endregion

#pragma region Переменные...

char logStringBuffer[256];

enum class CoreOpMode
{
    IDLE,
    AUTO_EXEC,
    MANUAL,
    ERROR
};
CoreOpMode currentMode                    = CoreOpMode::IDLE;
uint32_t   lastManualSessionActivityTime  = 0;
bool       pendingDisplayManualModeBypass = false;
bool       manualRadioHoldActive          = false;
bool       statusSourceRadio              = false;
bool       radioAppendedRssiEnabled       = false;
bool       radioConfigOk                  = false;
uint32_t   manualRadioHoldLastHeartbeatMs = 0;
bool       serialLinksStarted             = false;
E22Radio::EnsureResult radioLastEnsureResult = {};
uint32_t               radioLastRxFrameMs = 0;
uint32_t               radioLastConfigRetryMs = 0;
constexpr uint32_t     kRadioConfigRetryIntervalMs = 60000UL;
constexpr uint32_t     kRadioConfigRetryRxQuietMs  = 10000UL;

HardwareSerial Serial1(PA10, PA9); // Порт для экранцика LilyGo
HardwareSerial Serial2(PA3, PA2);  // Порт под RS485 для BMS батареи
HardwareSerial Serial3(PC7, PC6);  // Порт Lora
static const E22Radio::ControlPins kRadioControlPins = { RADIO_E22_M0, RADIO_E22_M1, RADIO_E22_AUX };

STM32_CAN            Can1(CAN1, ALT, RX_SIZE_256, TX_SIZE_256); // CAN шина на пинах PB8 PB9
static CAN_message_t CAN_TX_msg;                                // Пакет данных CAN на передачу
static CAN_message_t CAN_RX_msg;                                // Пакет данных CAN на прием

STM32Timer ITimer0(TIM1); // Таймер для прерываний

STM32RTC &rtc = STM32RTC::getInstance(); // Часы реального времени

typedef union
{
    int     vint;
    uint8_t bint[4];
} cracked_int_t; // Структура конвертации типа float в массив байтов для отправки по CAN

typedef union
{
    float   v;
    uint8_t b[4];
} cracked_float_t; // Структура конвертации типа float в массив байтов для отправки по CAN

#pragma pack(push, 1)
struct EEPROMData // Структура данных параметров для сохранения а EEProm
{
    uint8_t  shuttleNum;
    uint8_t  calibrateEncoder_F[8];
    uint8_t  calibrateEncoder_R[8];
    uint8_t  calibrateSensor_F[3];
    uint8_t  calibrateSensor_R[3];
    int      blinkTime;
    uint16_t maxSpeed;
    uint16_t minSpeed;
    uint16_t interPalleteDistance;
    uint8_t  inverse;
    uint8_t  fifoLifo;
    int      lifter_Speed;
    uint32_t timingBudget;
    uint8_t  minBattCharge;
    uint16_t shuttleLength;
    int      waitTime;
    int8_t   mprOffset;
    uint8_t  TopLeftXF;
    uint8_t  TopLeftYF;
    uint8_t  BotRightXF;
    uint8_t  BotRightYF;
    uint8_t  TopLeftXR;
    uint8_t  TopLeftYR;
    uint8_t  BotRightXR;
    uint8_t  BotRightYR;
    int8_t   chnlOffset;
};
#pragma pack(pop)

EEPROMData eepromData;

volatile bool      pendingEepromSave       = false;
constexpr uint32_t kBootloaderEntryGraceMs = 15;
constexpr uint32_t kSystemBootloaderAddr   = 0x1FFF0000UL;
volatile bool      pendingBootloaderEntry  = false;
volatile bool      bootloaderStopDone      = false;
volatile uint32_t  bootloaderEntryAtMs     = 0;

AlertManager alertMan;

#if CONNECTION_LOGS
struct LinkDiagCounters
{
    uint32_t radioRxValid;
    uint32_t radioRxCrcFail;
    uint32_t radioRxDropTarget;
    uint32_t radioTxFrames;
    uint32_t radioReplyCount;
    uint32_t radioReplyUsSum;
    uint32_t radioReplyUsMax;
    uint32_t displayRxValid;
    uint32_t displayRxCrcFail;
    uint32_t displayRxDropTarget;
    uint32_t displayTxFrames;
    uint32_t displayReplyCount;
    uint32_t displayReplyUsSum;
    uint32_t displayReplyUsMax;
    uint32_t loopJitterMaxMs;
    uint32_t loopGapOver20Ms;
};

LinkDiagCounters linkDiag            = {};
uint32_t         linkDiagLastYieldMs = 0;
#endif

void setFault(ShuttleFault fault)
{
    alertMan.setFault(fault);
}

void setWarning(ShuttleWarning warn, uint32_t timeoutMs = 5000)
{
    alertMan.setWarning(warn, timeoutMs, millis());
}

void clearWarning(ShuttleWarning warn)
{
    alertMan.clearWarning(warn);
}

void clearAllWarnings()
{
    alertMan.clearAllWarnings();
}

AS5600       as5600;               // Магнитный энкодер на свободном колесе
uint8_t      status           = 0; // Командный статус
ShuttleState currentOperation = STATE_IDLE;
uint8_t      shuttleNum       = 0; // Номер шаттла -сохранять-
uint8_t      channel[1024];        // Массив канала
uint8_t      calibrateEncoder_F[8] = {
    40, 40, 40, 40, 40, 40, 40, 40
}; // Массив данных калибровки магнитного энкодера при движении вперед -сохранять-
uint8_t calibrateEncoder_R[8] = {
    40, 40, 40, 40, 40, 40, 40, 40
}; // Массив данных калибровки магнитного энкодера при движении вперед -сохранять-
uint8_t calibrateSensor_F[3] = { 100, 100, 100 }; // Массив калибровки канального сенсора расстояния вперед -сохранять-
uint8_t calibrateSensor_R[3] = { 100, 100, 100 }; // Массив калибровки канального сенсора расстояния назад -сохранять-

uint8_t  counter              = 0;        // Счетчик для красивых морганий в рабочем режиме
uint8_t  debuger              = 0;        // Счетчик для дебагера
uint8_t  sensorOff            = 0;        // Флаг отключения сенсоров в ручном режиме
uint8_t  lastPallete          = 0;        // Флаг позиции последнего паллета
uint8_t  minBattCharge        = 20;       // Минимальный заряд батареи
uint8_t  endOfChannel         = 0;        // Флаг концов канала
int      blinkTime            = 80;       // Время квантования красивых морганий -сохранять-
int      waitTime             = 15000;    // Время ожидания при выгрузке
int      count                = millis(); // Счетчик времени общего назначения
int      count2               = count;    // Счетчик времени общего назначения
int      countSensor          = count;    // Счетчик времени опроса датчиков
int      countMoove           = count;    // Счетчик времени обновления передачи скорости по Can шине
uint16_t speed                = 0;        // Скорость движения в канале в %
uint16_t maxSpeed             = 96;       // Максимальное значение скорости (от 0 до 100 %) -сохранять-
uint16_t minSpeed             = 3;        // Минимальное значение скорости (лаг для АЦП) -сохранять-
uint16_t oldSpeed             = 0;        // Запомненная cкорость движения в канале для плавного разгона и торможения
uint16_t distance[8]          = { 0 };    // Значения от сенсоров расстояния и датчиков положения
uint16_t palletePosition[16]  = { 0 };    // Массив расстояний до паллет в канале
uint16_t mooveDistance        = 0;    // Значение дистанции для перемещения шаттла по команде движения на заданное расстояние
uint16_t interPalleteDistance = 100;  // Значение дистанции между паллетами -сохранять-
uint16_t shuttleLength        = 1000; // Длинна шаттла (максимальная ширина паллета которые шаттл может перевозить)
uint8_t  detectPalleteF1;             // Флаг датчика обнаружения паллеты вперед 1
uint8_t  detectPalleteF2;             // Флаг датчика обнаружения паллеты вперед 2
uint8_t  detectPalleteR1;             // Флаг датчика обнаружения паллеты назад 1
uint8_t  detectPalleteR2;             // Флаг датчика обнаружения паллеты назад 2
uint8_t  palleteCount  = 0;           // Количество паллет
uint8_t  motorStart    = false;       // Флаг запуска двигателя движения
uint8_t  motorReverse  = 2;           // Флаг направления (0/1), 2 = остановлено
uint8_t  lifterUp      = 0;           // Флаг поднятой платформы
uint8_t  inverse       = 0;           // Флаг инверсии движения -сохранять-
uint8_t  longWork      = 0;           // Флаг продолжительной загрузки/выгрузки
uint8_t  reportCounter = 0;           // Счетчик паузы репортов в секундах
uint8_t  diffPallete   = 0;           // Смещение для паллеты
uint8_t  fifoLifo      = 0;           // Режим FIFO/LIFO -сохранять-
uint8_t  batteryCharge = 0;           // Заряд батареи
uint8_t  UPQuant       = 0;           // Количество паллет на выгрузку
uint8_t  load          = 0;           // Оценка массы нагрузки шаттла 0 - 100
uint8_t  mooveCount    = 0;           // Счетчик пробксовки при малой скорости
int8_t   mprOffset     = 0;           // Смещение значения МПР
int8_t   chnlOffset    = 0;           // Смещение в конце канала

struct BatterySafetyState
{
    bool     lowStopLatched           = false;
    bool     emergencyActionActive    = false;
    uint32_t requireFreshBasicAfterMs = 0;
};

static BmsDdA5::Config makeBatteryPollConfig()
{
    BmsDdA5::Config cfg;
    cfg.startupBasicRetryMs = 1000U;
    cfg.basicIdleMs       = 5000U;
    cfg.basicActiveMs     = 15000U;
    cfg.basicHighLoadMs   = 60000U;
    cfg.lowBatteryBasicMs = 5000U;
    cfg.cellIdleMs        = 60000U;
    cfg.deviceIdleMs      = 300000U;

    cfg.staleWarnMs = 300000U;
    return cfg;
}

static uint8_t  tofSensorErrors[4]          = { 0, 0, 0, 0 };
static uint8_t  tofReadErrors[4]            = { 0, 0, 0, 0 };
static uint8_t  tofTimeoutErrors[4]         = { 0, 0, 0, 0 };
static uint8_t  tofSharedBusErrors[4]       = { 0, 0, 0, 0 };
static uint8_t  tofLastDiagStatus[4]        = { TOF_I2C_OK, TOF_I2C_OK, TOF_I2C_OK, TOF_I2C_OK };
static uint32_t tofLastI2cRecoveryLogMs     = 0;
static bool     tofI2cRecoveryLogged        = false;
static uint32_t tofI2cRecoveryCount         = 0;
static uint32_t tofLastRecoverySummaryLogMs = 0;
static uint32_t tofLastI2cRecoveryAttemptMs = 0;
static bool     tofFailureWarnLogged[4]     = { false, false, false, false };
static bool     tofFaultLogged[4]           = { false, false, false, false };
static uint32_t tofLastTransportSuccessMs[4] = { 0, 0, 0, 0 };
static bool     tofHasTransportSuccess[4]    = { false, false, false, false };
static uint8_t  tofConsecutiveTransportFailures[4] = { 0, 0, 0, 0 };
static uint32_t tofLastFailureLogMs[4]       = { 0, 0, 0, 0 };
static uint32_t tofRuntimeStartMs            = 0;
static bool     tofRuntimeStarted            = false;
static uint8_t  tofBusVoteMask               = 0U;
static bool     tofBusVoteAs5600Failed       = false;

static TofBusState tofBusState               = TofBusState::Ready;
static uint32_t    tofLineLowSinceMs          = 0U;
static uint32_t    tofLinesHighSinceMs        = 0U;
static uint32_t    tofLastBusStateLogMs       = 0U;
static bool        tofSdaRecoveryAttempted    = false;

static uint16_t as5600LastGoodAngleRaw      = 0;
static bool     as5600LastGoodAngleValid    = false;
static bool     as5600Present               = false;
static uint32_t as5600ReadFailCount         = 0;
static uint32_t as5600LastReadFailLogMs     = 0;

constexpr uint8_t CRASH_SRC_F1 = 0x01;
constexpr uint8_t CRASH_SRC_F2 = 0x02;
constexpr uint8_t CRASH_SRC_R1 = 0x04;
constexpr uint8_t CRASH_SRC_R2 = 0x08;
volatile bool     crashPending = false;
volatile uint8_t  crashSourceMask = 0;
volatile uint16_t crashIrqCount = 0;
static uint32_t   crashLastLogMs = 0;

static const char *breadcrumbStageName(uint8_t stage)
{
    switch ((BreadcrumbStage)stage)
    {
    case BC_BOOT:
        return "boot";
    case BC_LOOP:
        return "loop";
    case BC_SYSTEM_YIELD:
        return "yield";
    case BC_BMS_TICK:
        return "bms";
    case BC_TX_TELEMETRY:
        return "tx_tel";
    case BC_TX_SENSORS:
        return "tx_sen";
    case BC_TX_STATS:
        return "tx_stat";
    case BC_TX_LINK_HEALTH:
        return "tx_link";
    case BC_POLL_DISPLAY:
        return "disp_rx";
    case BC_POLL_RADIO:
        return "rad_rx";
    case BC_TOF_POLL:
        return "tof_poll";
    case BC_TOF_PREFLIGHT:
        return "tof_pre";
    case BC_TOF_PROBE:
        return "tof_probe";
    case BC_TOF_READ:
        return "tof_read";
    case BC_TOF_RECOVERY:
        return "tof_rec";
    case BC_AS5600_ANGLE:
        return "as5600";
    case BC_SERIAL_COMMAND:
        return "cmd";
    default:
        return "unknown";
    }
}

static uint16_t breadcrumbCrc(const BreadcrumbRecord &record)
{
    BreadcrumbRecord crcRecord = record;
    crcRecord.magicWord        = 0;
    crcRecord.crc16            = 0;
    return ProtocolUtils::calcCRC16((uint8_t *)&crcRecord, sizeof(BreadcrumbRecord));
}

static bool readBreadcrumb(BreadcrumbRecord *out)
{
    if (out == nullptr)
    {
        return false;
    }

    BreadcrumbRecord local;
    local.magicWord = diagBreadcrumb->magicWord;
    local.sequence  = diagBreadcrumb->sequence;
    local.millisAt  = diagBreadcrumb->millisAt;
    local.stage     = diagBreadcrumb->stage;
    local.sensor    = diagBreadcrumb->sensor;
    local.status    = diagBreadcrumb->status;
    local.detail    = diagBreadcrumb->detail;
    local.crc16     = diagBreadcrumb->crc16;
    local.reserved  = diagBreadcrumb->reserved;

    if (local.magicWord != DIAG_BREADCRUMB_MAGIC_WORD)
    {
        return false;
    }
    if (breadcrumbCrc(local) != local.crc16)
    {
        return false;
    }

    *out = local;
    return true;
}

static void writeBreadcrumb(BreadcrumbStage stage, uint8_t sensor, uint8_t status, uint8_t detail)
{
    BreadcrumbRecord local = {};
    local.magicWord       = DIAG_BREADCRUMB_MAGIC_WORD;
    local.sequence        = ++breadcrumbSequence;
    local.millisAt        = millis();
    local.stage           = (uint8_t)stage;
    local.sensor          = sensor;
    local.status          = status;
    local.detail          = detail;
    local.crc16           = breadcrumbCrc(local);

    __disable_irq();
    diagBreadcrumb->magicWord = 0;
    diagBreadcrumb->sequence  = local.sequence;
    diagBreadcrumb->millisAt  = local.millisAt;
    diagBreadcrumb->stage     = local.stage;
    diagBreadcrumb->sensor    = local.sensor;
    diagBreadcrumb->status    = local.status;
    diagBreadcrumb->detail    = local.detail;
    diagBreadcrumb->crc16     = local.crc16;
    diagBreadcrumb->reserved  = 0;
    diagBreadcrumb->magicWord = local.magicWord;
    __enable_irq();
}

static void logPreviousBreadcrumbIfWatchdog(uint32_t resetFlags)
{
#if I2C_DIAG_LOGS
    if ((resetFlags & RESET_REASON_WATCHDOG) == 0)
    {
        return;
    }

    BreadcrumbRecord previous;
    if (!readBreadcrumb(&previous))
    {
        return;
    }

    if (previous.sequence > breadcrumbSequence)
    {
        breadcrumbSequence = previous.sequence;
    }

    makeReliableLog(
        LOG_WARN,
        "BC prev rst=%s bc=%s q=%lu ms=%lu s=%u st=%u d=%u c=%u",
        primaryResetCategoryName(resetFlags),
        breadcrumbStageName(previous.stage),
        (unsigned long)previous.sequence,
        (unsigned long)previous.millisAt,
        previous.sensor,
        previous.status,
        previous.detail,
        previous.crc16);
#else
    (void)resetFlags;
#endif
}

static void logBreadcrumbSummaryIfDue(uint32_t now)
{
#if I2C_DIAG_LOGS
    static uint32_t lastLogMs = 0;
    if (now - lastLogMs < 5000U)
    {
        return;
    }
    lastLogMs = now;

    BreadcrumbRecord current;
    if (!readBreadcrumb(&current))
    {
        return;
    }

    uint32_t age = (now >= current.millisAt) ? now - current.millisAt : DIAG_BREADCRUMB_MAX_AGE_MS;
    if (age > DIAG_BREADCRUMB_MAX_AGE_MS)
    {
        age = DIAG_BREADCRUMB_MAX_AGE_MS;
    }

    uint16_t timeoutTotal = 0;
    uint16_t busTotal     = 0;
    for (uint8_t i = 0; i < 4; ++i)
    {
        timeoutTotal += tofTimeoutErrors[i];
        busTotal += tofSharedBusErrors[i];
    }

    makeLog(
        LOG_DEBUG,
        "BC %s q=%lu age=%lu s=%u st=%u t=%u b=%u rec=%lu io=%u/%u",
        breadcrumbStageName(current.stage),
        (unsigned long)current.sequence,
        (unsigned long)age,
        current.sensor,
        current.status,
        timeoutTotal,
        busTotal,
        (unsigned long)tofI2cRecoveryCount,
        digitalRead(TOF_I2C_SDA),
        digitalRead(TOF_I2C_SCL));
#else
    (void)now;
#endif
}

static inline bool tofBusLinesHighNow()
{
    return digitalRead(TOF_I2C_SDA) == HIGH && digitalRead(TOF_I2C_SCL) == HIGH;
}

static void setTofI2cClock(uint32_t clockHz)
{
    Wire.setClock(clockHz);
    delayMicroseconds(50);
}

static uint8_t i2cProbeAddress(uint8_t address)
{
    if (!tofBusLinesHighNow())
    {
        return 0xFFU;
    }

    Wire.beginTransmission(address);
    return Wire.endTransmission();
}

static uint8_t scanTofAddressMask()
{
    uint8_t mask = 0;
    for (uint8_t id = 1U; id <= 4U; ++id)
    {
        if (i2cProbeAddress((uint8_t)(TOF_BASE_I2C_ADDR + id)) == 0U)
        {
            mask |= (uint8_t)(1U << (id - 1U));
        }
    }
    return mask;
}

static void logI2cBootScanAtClock(uint32_t clockHz)
{
    setTofI2cClock(clockHz);
    if (!tofBusLinesHighNow())
    {
        makeReliableLog(
            LOG_WARN,
            "I2C scan skip clk=%lu io=%u/%u",
            (unsigned long)clockHz,
            digitalRead(TOF_I2C_SDA),
            digitalRead(TOF_I2C_SCL));
        return;
    }

    const uint8_t err09 = i2cProbeAddress((uint8_t)0x09U);
    const uint8_t err0A = i2cProbeAddress((uint8_t)0x0AU);
    const uint8_t err0B = i2cProbeAddress((uint8_t)0x0BU);
    const uint8_t err0C = i2cProbeAddress((uint8_t)0x0CU);
    const uint8_t err36 = i2cProbeAddress(kAs5600I2cAddress);

    makeReliableLog(
        LOG_INFO,
        "I2C scan clk=%lu 09=%s/%u 0A=%s/%u 0B=%s/%u 0C=%s/%u 36=%s/%u io=%u/%u",
        (unsigned long)clockHz,
        (err09 == 0U) ? "ack" : "miss",
        err09,
        (err0A == 0U) ? "ack" : "miss",
        err0A,
        (err0B == 0U) ? "ack" : "miss",
        err0B,
        (err0C == 0U) ? "ack" : "miss",
        err0C,
        (err36 == 0U) ? "ack" : "miss",
        err36,
        digitalRead(TOF_I2C_SDA),
        digitalRead(TOF_I2C_SCL));
}

static void logI2cBootScan()
{
    logI2cBootScanAtClock(kTofI2cSlowProbeClockHz);
#if I2C_DIAG_LOGS
    if (kTofI2cFastProbeClockHz != kTofI2cSlowProbeClockHz)
    {
        logI2cBootScanAtClock(kTofI2cFastProbeClockHz);
    }
#endif
    setTofI2cClock(tofI2cClockHz);
}

static bool readAs5600AngleChecked(uint16_t *rawAngle)
{
    if (rawAngle == nullptr || !as5600Present)
    {
        return false;
    }

    uint8_t txStatus       = 0xFFU;
    uint8_t requestedBytes = 0U;
    uint8_t availableBytes = 0U;

    if (serviceTofBusMonitor(millis()))
    {
        Wire.beginTransmission(kAs5600I2cAddress);
        Wire.write(kAs5600AngleHighReg);
        txStatus = Wire.endTransmission(true);
        if (txStatus == 0U)
        {
            requestedBytes = Wire.requestFrom(kAs5600I2cAddress, kAs5600AngleReadLength);
            availableBytes = (uint8_t)Wire.available();
            if (requestedBytes == kAs5600AngleReadLength && availableBytes >= kAs5600AngleReadLength)
            {
                const uint16_t high = (uint16_t)Wire.read();
                const uint16_t low  = (uint16_t)Wire.read();
                *rawAngle = (uint16_t)(((high << 8) | low) & 0x0FFFU);
                as5600LastGoodAngleRaw   = *rawAngle;
                as5600LastGoodAngleValid = true;
                return true;
            }

            while (Wire.available() > 0)
            {
                (void)Wire.read();
            }
        }
    }

    as5600ReadFailCount++;
    const uint32_t now = millis();
    if (as5600LastReadFailLogMs == 0U || now - as5600LastReadFailLogMs >= kAs5600FailLogIntervalMs)
    {
        makeReliableLog(
            LOG_WARN,
            "AS5600 fail tx=%u got=%u av=%u cnt=%lu last=%u v=%u io=%u/%u",
            txStatus,
            requestedBytes,
            availableBytes,
            (unsigned long)as5600ReadFailCount,
            as5600LastGoodAngleRaw,
            as5600LastGoodAngleValid ? 1U : 0U,
            digitalRead(TOF_I2C_SDA),
            digitalRead(TOF_I2C_SCL));
        as5600LastReadFailLogMs = (now == 0U) ? 1U : now;
    }
    return false;
}

static bool readAs5600AngleForMotion(uint16_t *rawAngle)
{
    if (rawAngle == nullptr)
    {
        return false;
    }
    if (readAs5600AngleChecked(rawAngle))
    {
        return true;
    }
    if (!as5600LastGoodAngleValid)
    {
        return false;
    }

    *rawAngle = as5600LastGoodAngleRaw;
    return true;
}

static bool readAs5600AngleFresh(int *angleOut)
{
    if (angleOut == nullptr)
    {
        return false;
    }

    uint16_t rawAngle = 0;
    if (!readAs5600AngleChecked(&rawAngle))
    {
        return false;
    }

    *angleOut = rawAngle;
    return true;
}

static uint16_t readAs5600AngleForTelemetry()
{
    uint16_t rawAngle = as5600LastGoodAngleRaw;
    if (readAs5600AngleChecked(&rawAngle))
    {
        return rawAngle;
    }

    return as5600LastGoodAngleValid ? as5600LastGoodAngleRaw : 0U;
}

static void logBootResetReplay()
{
    makeReliableLog(
        LOG_INFO,
        "Reset replay %s flags=%08lX marker=%08lX",
        primaryResetCategoryName(lastBootResetFlags),
        (unsigned long)lastBootResetFlags,
        (unsigned long)lastBootResetMarkerFlags);
    logPreviousBreadcrumbIfWatchdog(lastBootResetFlags);
}

static void logTofI2cLines(const char *tag)
{
    makeReliableLog(
        LOG_INFO,
        "TOF I2C %s clk=%lu io=%u/%u",
        tag != nullptr ? tag : "?",
        (unsigned long)tofI2cClockHz,
        digitalRead(TOF_I2C_SDA),
        digitalRead(TOF_I2C_SCL));
}

static void initTofI2cBus()
{
    Wire.setSDA(TOF_I2C_SDA);
    Wire.setSCL(TOF_I2C_SCL);
    Wire.begin();
    setTofI2cClock(tofI2cClockHz);
}


static const char *tofBusStateName(TofBusState state)
{
    switch (state)
    {
    case TofBusState::Ready:
        return "ready";
    case TofBusState::LowPending:
        return "low_pending";
    case TofBusState::SdaStuck:
        return "sda_stuck";
    case TofBusState::SclBlocked:
        return "scl_blocked";
    case TofBusState::Recovering:
        return "recovering";
    default:
        return "unknown";
    }
}

static void setTofBusState(TofBusState next, const char *reason)
{
    if (tofBusState == next)
    {
        return;
    }

    const TofBusState previous = tofBusState;
    tofBusState                = next;
    if (next == TofBusState::LowPending || (previous == TofBusState::LowPending && next == TofBusState::Ready))
    {
        return;
    }
    const uint32_t now         = millis();
    if (tofLastBusStateLogMs == 0U || now - tofLastBusStateLogMs >= kTofFailureLogIntervalMs)
    {
        makeReliableLog(
            (next == TofBusState::Ready) ? LOG_INFO : LOG_WARN,
            "TOF bus %s>%s %s io=%u/%u",
            tofBusStateName(previous),
            tofBusStateName(next),
            reason != nullptr ? reason : "?",
            digitalRead(TOF_I2C_SDA),
            digitalRead(TOF_I2C_SCL));
        tofLastBusStateLogMs = (now == 0U) ? 1U : now;
    }
}

static bool tofRecoveryCooldownElapsed(uint32_t now)
{
    return tofI2cRecoveryCount == 0U || now - tofLastI2cRecoveryAttemptMs >= kTofI2cRecoveryMinIntervalMs;
}

static void tofDriveLineLow(uint32_t pin)
{
#if defined(OUTPUT_OPEN_DRAIN)
    pinMode(pin, OUTPUT_OPEN_DRAIN);
#else
    pinMode(pin, OUTPUT);
#endif
    digitalWrite(pin, LOW);
}

static void tofReleaseLine(uint32_t pin)
{
#if defined(OUTPUT_OPEN_DRAIN)
    pinMode(pin, OUTPUT_OPEN_DRAIN);
    digitalWrite(pin, HIGH);
#else
    pinMode(pin, INPUT_PULLUP);
#endif
}

static void logTofRecovery(
    const char *reason, uint8_t sdaBefore, uint8_t sclBefore, uint8_t sdaAfter, uint8_t sclAfter)
{
    const uint32_t now = millis();
    if (!tofI2cRecoveryLogged || now - tofLastI2cRecoveryLogMs >= 60000U)
    {
        makeReliableLog(
            LOG_WARN,
            "TOF I2C rec #%lu %s io=%u/%u>%u/%u",
            (unsigned long)tofI2cRecoveryCount,
            reason != nullptr ? reason : "?",
            sdaBefore,
            sclBefore,
            sdaAfter,
            sclAfter);
        tofI2cRecoveryLogged    = true;
        tofLastI2cRecoveryLogMs = (now == 0U) ? 1U : now;
    }
}

static bool performTofWireReinit(const char *reason)
{
    if (!tofBusLinesHighNow())
    {
        return false;
    }

    const uint32_t now       = millis();
    const uint8_t  sdaBefore = digitalRead(TOF_I2C_SDA);
    const uint8_t  sclBefore = digitalRead(TOF_I2C_SCL);
    tofLastI2cRecoveryAttemptMs = now;
    tofI2cRecoveryCount++;
    writeBreadcrumb(BC_TOF_RECOVERY, 0xFF, 0, 0);
    setWarning(WARN_I2C_RECOVERY, 3000);
    setTofBusState(TofBusState::Recovering, reason);

#if defined(TWOWIRE_HAS_END) || defined(WIRE_HAS_END)
    Wire.end();
#endif
    initTofI2cBus();

    const uint8_t sdaAfter = digitalRead(TOF_I2C_SDA);
    const uint8_t sclAfter = digitalRead(TOF_I2C_SCL);
    logTofRecovery(reason, sdaBefore, sclBefore, sdaAfter, sclAfter);
    if (sdaAfter == HIGH && sclAfter == HIGH)
    {
        setTofBusState(TofBusState::Ready, reason);
        tofLineLowSinceMs       = 0U;
        tofLinesHighSinceMs     = 0U;
        tofSdaRecoveryAttempted = false;
        return true;
    }

    tofLineLowSinceMs = now;
    tofSdaRecoveryAttempted = true;
    setTofBusState(sclAfter == LOW ? TofBusState::SclBlocked : TofBusState::SdaStuck, reason);
    return false;
}

static bool performTofSdaClockRecovery(const char *reason)
{
    if (digitalRead(TOF_I2C_SCL) == LOW)
    {
        return false;
    }

    const uint32_t now       = millis();
    const uint8_t  sdaBefore = digitalRead(TOF_I2C_SDA);
    const uint8_t  sclBefore = digitalRead(TOF_I2C_SCL);
    tofLastI2cRecoveryAttemptMs = now;
    tofI2cRecoveryCount++;
    tofSdaRecoveryAttempted = true;
    writeBreadcrumb(BC_TOF_RECOVERY, 0xFF, 0, 1);
    setWarning(WARN_I2C_RECOVERY, 3000);
    setTofBusState(TofBusState::Recovering, reason);

#if defined(TWOWIRE_HAS_END) || defined(WIRE_HAS_END)
    Wire.end();
#endif

    tofReleaseLine(TOF_I2C_SDA);
    tofReleaseLine(TOF_I2C_SCL);
    delayMicroseconds(10);
    for (uint8_t i = 0U; i < 16U && digitalRead(TOF_I2C_SDA) == LOW; ++i)
    {
        if (digitalRead(TOF_I2C_SCL) == LOW)
        {
            break;
        }
        tofDriveLineLow(TOF_I2C_SCL);
        delayMicroseconds(10);
        tofReleaseLine(TOF_I2C_SCL);
        delayMicroseconds(10);
    }

    tofDriveLineLow(TOF_I2C_SDA);
    delayMicroseconds(10);
    tofReleaseLine(TOF_I2C_SCL);
    delayMicroseconds(10);
    tofReleaseLine(TOF_I2C_SDA);
    delayMicroseconds(10);
    initTofI2cBus();

    const uint8_t sdaAfter = digitalRead(TOF_I2C_SDA);
    const uint8_t sclAfter = digitalRead(TOF_I2C_SCL);
    logTofRecovery(reason, sdaBefore, sclBefore, sdaAfter, sclAfter);
    if (sdaAfter == HIGH && sclAfter == HIGH)
    {
        setTofBusState(TofBusState::Ready, reason);
        tofLineLowSinceMs       = 0U;
        tofLinesHighSinceMs     = 0U;
        tofSdaRecoveryAttempted = false;
        return true;
    }

    tofLineLowSinceMs = now;
    setTofBusState(sclAfter == LOW ? TofBusState::SclBlocked : TofBusState::SdaStuck, reason);
    return false;
}

static bool serviceTofBusMonitor(uint32_t now)
{
    const bool sdaHigh = digitalRead(TOF_I2C_SDA) == HIGH;
    const bool sclHigh = digitalRead(TOF_I2C_SCL) == HIGH;

    if (sdaHigh && sclHigh)
    {
        tofLineLowSinceMs = 0U;
        if (tofBusState == TofBusState::Ready)
        {
            return true;
        }
        if (tofBusState == TofBusState::LowPending)
        {
            setTofBusState(TofBusState::Ready, "brief_low");
            return true;
        }
        if (tofBusState == TofBusState::Recovering)
        {
            return false;
        }

        if (tofLinesHighSinceMs == 0U)
        {
            tofLinesHighSinceMs = (now == 0U) ? 1U : now;
            return false;
        }
        if (now - tofLinesHighSinceMs < kTofLineReleaseConfirmMs)
        {
            return false;
        }

        return performTofWireReinit("line_release");
    }

    tofLinesHighSinceMs = 0U;
    if (tofBusState == TofBusState::Ready)
    {
        tofLineLowSinceMs = now;
        setTofBusState(TofBusState::LowPending, "line_low");
        return false;
    }

    if (tofBusState == TofBusState::SclBlocked)
    {
        return false;
    }

    if (tofBusState == TofBusState::LowPending)
    {
        if (now - tofLineLowSinceMs < kTofLineLowConfirmMs)
        {
            return false;
        }
        if (!sclHigh)
        {
            setTofBusState(TofBusState::SclBlocked, "scl_blocked");
            return false;
        }
        setTofBusState(TofBusState::SdaStuck, "sda_stuck");
        tofSdaRecoveryAttempted = false;
    }

    if (!sclHigh)
    {
        setTofBusState(TofBusState::SclBlocked, "scl_blocked");
        return false;
    }

    if (tofBusState == TofBusState::SdaStuck && !tofSdaRecoveryAttempted && tofRecoveryCooldownElapsed(now))
    {
        return performTofSdaClockRecovery("sda_stuck");
    }
    return false;
}

static bool readTofBootPresence(uint8_t id, TofI2cDiagnostics &diag)
{
    diag                  = {};
    diag.address          = TOF_BASE_I2C_ADDR + id;
    diag.expected         = id;
    if (!tofBusLinesHighNow())
    {
        diag.status = TOF_I2C_BUS_STUCK;
        return false;
    }

    const bool ok = TOF_Is_Device_Present(id, &diag);
    IWatchdog.reload();
    return ok;
}
static bool readTofBootPresenceIfAck(uint8_t id, uint8_t ackMask, TofI2cDiagnostics &diag)
{
    diag          = {};
    diag.address  = TOF_BASE_I2C_ADDR + id;
    diag.expected = id;

    if ((ackMask & (uint8_t)(1U << (id - 1U))) == 0U)
    {
        diag.status = TOF_I2C_NO_ACK;
        return false;
    }

    return readTofBootPresence(id, diag);
}

static void logTofBootSnapshot()
{
    if (!tofBusLinesHighNow())
    {
        makeReliableLog(
            LOG_WARN,
            "TOF boot skip clk=%lu io=%u/%u",
            (unsigned long)tofI2cClockHz,
            digitalRead(TOF_I2C_SDA),
            digitalRead(TOF_I2C_SCL));
        return;
    }

    const uint8_t tofAckMask = scanTofAddressMask();
    if (tofAckMask == 0U)
    {
        makeReliableLog(
            LOG_WARN,
            "TOF boot noack clk=%lu io=%u/%u",
            (unsigned long)tofI2cClockHz,
            digitalRead(TOF_I2C_SDA),
            digitalRead(TOF_I2C_SCL));
        return;
    }

    TofI2cDiagnostics channelForward = {};
    TofI2cDiagnostics channelReverse = {};
    TofI2cDiagnostics palletForward  = {};
    TofI2cDiagnostics palletReverse  = {};

    const bool channelForwardOk = readTofBootPresenceIfAck(2, tofAckMask, channelForward);
    const bool channelReverseOk = readTofBootPresenceIfAck(1, tofAckMask, channelReverse);
    const bool palletForwardOk  = readTofBootPresenceIfAck(4, tofAckMask, palletForward);
    const bool palletReverseOk  = readTofBootPresenceIfAck(3, tofAckMask, palletReverse);

    makeReliableLog(
        LOG_INFO,
        "TOF boot m=%02X cf=%u/%s/%u cr=%u/%s/%u pf=%u/%s/%u pr=%u/%s/%u io=%u/%u",
        tofAckMask,
        channelForwardOk,
        TOF_I2C_Status_Name(channelForward.status),
        channelForward.received,
        channelReverseOk,
        TOF_I2C_Status_Name(channelReverse.status),
        channelReverse.received,
        palletForwardOk,
        TOF_I2C_Status_Name(palletForward.status),
        palletForward.received,
        palletReverseOk,
        TOF_I2C_Status_Name(palletReverse.status),
        palletReverse.received,
        digitalRead(TOF_I2C_SDA),
        digitalRead(TOF_I2C_SCL));
}

static void logI2cDiagSummaryIfDue(uint32_t now)
{
#if I2C_DIAG_LOGS
    static uint32_t lastLogMs = 0;
    const uint16_t tofFaultMask = (uint16_t)FAULT_TOF_CH_F | (uint16_t)FAULT_TOF_CH_R |
                                  (uint16_t)FAULT_TOF_PAL_F | (uint16_t)FAULT_TOF_PAL_R;
    const bool hasI2cEvidence = ((alertMan.getWarningCode() & (uint16_t)WARN_I2C_RECOVERY) != 0U) ||
                                ((alertMan.getErrorCode() & tofFaultMask) != 0U) || as5600ReadFailCount != 0U ||
                                tofI2cRecoveryCount != 0U;
    if (!hasI2cEvidence)
    {
        return;
    }
    if (lastLogMs != 0U && now - lastLogMs < kI2cDiagSummaryIntervalMs)
    {
        return;
    }
    lastLogMs = (now == 0U) ? 1U : now;

    makeReliableLog(
        LOG_INFO,
        "I2C diag clk=%lu e=%04X w=%04X tof=%u/%u/%u/%u st=%s/%s/%s/%s b=%u/%u/%u/%u rec=%lu asf=%lu io=%u/%u",
        (unsigned long)tofI2cClockHz,
        alertMan.getErrorCode(),
        alertMan.getWarningCode(),
        tofSensorErrors[0],
        tofSensorErrors[1],
        tofSensorErrors[2],
        tofSensorErrors[3],
        TOF_I2C_Status_Name((TofI2cStatus)tofLastDiagStatus[0]),
        TOF_I2C_Status_Name((TofI2cStatus)tofLastDiagStatus[1]),
        TOF_I2C_Status_Name((TofI2cStatus)tofLastDiagStatus[2]),
        TOF_I2C_Status_Name((TofI2cStatus)tofLastDiagStatus[3]),
        tofSharedBusErrors[0],
        tofSharedBusErrors[1],
        tofSharedBusErrors[2],
        tofSharedBusErrors[3],
        (unsigned long)tofI2cRecoveryCount,
        (unsigned long)as5600ReadFailCount,
        digitalRead(TOF_I2C_SDA),
        digitalRead(TOF_I2C_SCL));
#else
    (void)now;
#endif
}

static BmsDdA5Firmware::Adapter batteryBms(SerialRS485, RS485, makeLogImpl, makeBatteryPollConfig());
static BatterySafetyState       batterySafetyState;

int      lifterCurrent        = 0;    // Ток лифтера для оценки массы поднимаемого груза
int      angle                = 0;    // Значение угла полученное от магнитного экодера
int      oldAngle             = 0;    // Промежуточное значение угла для расчетов
int      startAngle           = 0;    // Промежуточное значение угла для расчетов
int      turnCount            = 0;    // Счетчик оборотов колеса
int      channelLength        = 0;    // Расчетная длинна канала
int      currentPosition      = 0;    // Текущая позиция шаттла
int      oldPosition          = 0;    // Позиция для определения останова шаттла
int      lastPalletePosition  = 0;    // Позиция последнего паллета после загрузки
int      firstPalletePosition = 0;    // Позиция первого паллета при уплотнении вперед
int      lifter_Speed         = 3700; // Скорость двигателя лифтера -сохранять-
int      lifterDelay          = 3800; // Задержка лифтера
int      oldChannelDistanse   = 0;    // Канальная дистанция для фильтрации фантомных срабатываний
int      oldPalleteDistanse   = 0;    // Паллетная дистанция для фильтрации фантомных срабатываний
uint32_t timingBudget         = 40;   // Время измерения датчиками

float temp    = 0;   // Температура чипа
float weelDia = 100; // Диаметр колеса

uint16_t mesRes[2][4];
int      countCrush = 0;
int      startDiff  = 0;

uint8_t  sensorIndex = 0;
uint16_t dist;
uint8_t  tofFilterIndex[4] = { 0, 0, 0, 0 };
uint16_t data[4][5] = {
    { 0, 0, 0, 0, 0 }, // data[0]
    { 0, 0, 0, 0, 0 }, // data[1]
    { 0, 0, 0, 0, 0 }, // data[2]
    { 0, 0, 0, 0, 0 }  // data[3]
};

#pragma endregion

// Инициация устройств
void setup()
{
    if (consumeBootloaderResetRequest())
    {
        jumpToBootloader();
    }

    pinMode(GREEN_LED, OUTPUT);
    pinMode(BOARD_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);
    pinMode(WHITE_LED, OUTPUT);
    pinMode(ZOOMER, OUTPUT);
    pinMode(RADIO_E22_M0, OUTPUT);
    pinMode(RADIO_E22_M1, OUTPUT);
    pinMode(RADIO_E22_AUX, INPUT_PULLUP);
    pinMode(RS485, OUTPUT);
    pinMode(DATCHIK_F1, INPUT_PULLUP);
    pinMode(DATCHIK_R1, INPUT_PULLUP);
    pinMode(DATCHIK_F2, INPUT_PULLUP);
    pinMode(DATCHIK_R2, INPUT_PULLUP);
    pinMode(DL_UP, INPUT_PULLUP);
    pinMode(DL_DOWN, INPUT_PULLUP);
    pinMode(BUMPER_F1, INPUT);
    pinMode(BUMPER_F2, INPUT);
    pinMode(BUMPER_R1, INPUT);
    pinMode(BUMPER_R2, INPUT);
    pinMode(CHANNEL, INPUT);

    attachInterrupt(BUMPER_F1, crashF1Irq, FALLING);
    attachInterrupt(BUMPER_F2, crashF2Irq, FALLING);
    attachInterrupt(BUMPER_R1, crashR1Irq, FALLING);
    attachInterrupt(BUMPER_R2, crashR2Irq, FALLING);

    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    digitalWrite(RED_LED, LOW);
    digitalWrite(WHITE_LED, LOW);
    digitalWrite(ZOOMER, LOW);
    digitalWrite(RS485, LOW);
    digitalWrite(RADIO_E22_M0, LOW);
    digitalWrite(RADIO_E22_M1, LOW);

    SerialDisplay.begin(230400, SERIAL_8E1);
    SerialRS485.begin(9600);
    SerialLora.begin(E22Radio::uartBaudValue(kRadioHostBaud));
    serialLinksStarted = true;

    uint32_t displayWaitStart = millis();
    while (!SerialDisplay && (millis() - displayWaitStart < 1500))
    {
        delay(10);
    }

    delay(kBootLogSettleMs);

    initStatsSRAM();
    recordBootResetCause();
    writeBreadcrumb(BC_BOOT, 0xFF, 0, 0);
    __HAL_RCC_CLEAR_RESET_FLAGS();

    read_EEPROM_Data();
    applyRadioConfigAtBoot();
    analogReadResolution(12);

    makeBootLog(LOG_INFO, "Start init board...");
    makeBootLog(
        LOG_INFO,
        "FW Cntrl_V2 v%u.%u.%u",
        kFirmwareVersionMajor,
        kFirmwareVersionMinor,
        kFirmwareVersionPatch);

    Can1.begin();
    Can1.setBaudRate(500000);
    CAN_TX_msg.flags.extended = 1;
    CAN_RX_msg.flags.extended = 1;

    initTofI2cBus();
    logTofI2cLines("after init");

    // Инициируем магнитный энкодер
    const bool encoderOk = as5600.begin(4);
    as5600Present = encoderOk;
    as5600.setDirection(AS5600_CLOCK_WISE);
    if (encoderOk)
        makeBootLog(LOG_INFO, "Init encoder success...");
    else
        makeBootLog(LOG_ERROR, "Init encoder failed...");

    initTofI2cBus();
    logTofI2cLines("after encoder");
    logI2cBootScan();
    logTofBootSnapshot();

    makeLog(LOG_DEBUG, "Total struct size = %d", sizeof(EEPROMData));
    delay(10);

    makeLog(LOG_INFO, "Initialize RTC date and time.");
    delay(50);

    rtc.setClockSource(STM32RTC::LSE_CLOCK);
    rtc.begin();

    if (!rtc.isTimeSet()) // Инициируем часы RTC
    {
        rtc.setTime(0, 0, 0);
        rtc.setDate(1, 1, 1, 23);
        makeLog(LOG_INFO, "RTC initialized with default date and time.");
    }

    makeLog(LOG_INFO, "Complete initialize RTC date and time.");

    delay(50);
    // char report[256];
    temp = 25 + ((float)(analogRead(ATEMP) * 3200) / 4096 - 760) / 2.5;
    uint8_t hour, minute, second, day, month, year, weekDay;
    rtc.getTime(&hour, &minute, &second, 0, nullptr);
    rtc.getDate(&weekDay, &day, &month, &year);
    makeBootLog(LOG_INFO, "Boot: %02d:%02d:%02d %02d/%02d/%02d T=%.1f", hour, minute, second, day, month, year, temp);
    makeBootLog(
        LOG_INFO,
        "Boot complete FW v%u.%u.%u reset=%s flags=%08lX marker=%08lX",
        kFirmwareVersionMajor,
        kFirmwareVersionMinor,
        kFirmwareVersionPatch,
        primaryResetCategoryName(lastBootResetFlags),
        (unsigned long)lastBootResetFlags,
        (unsigned long)lastBootResetMarkerFlags);
    delay(500);
    IWatchdog.begin(10000000);
    logBootResetReplay();
    batteryBms.begin(millis());
    batteryCharge = batteryBms.socPercent();
    tofRuntimeStartMs = millis();
    tofRuntimeStarted = true;
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        tofLastTransportSuccessMs[i] = tofRuntimeStartMs;
    }
}

// Основной цикл
void loop()
{
    writeBreadcrumb(BC_LOOP, 0xFF, 0, 0);
    SystemYield();
    handlePendingCrash();

    if (pendingEepromSave && motorStart == 0 && motorReverse == 2)
    {
        saveConfigsToFlash();
        pendingEepromSave = false;
    }

    get_Distance();

    if (isErrorActive())
        currentMode = CoreOpMode::ERROR;

    switch (currentMode)
    {
    case CoreOpMode::IDLE:
    {
        if (status != 0 && status != CMD_STOP)
        {

            uint8_t inChannel = digitalRead(CHANNEL);
            delay(5);
            inChannel = digitalRead(CHANNEL) && inChannel;
            delay(5);
            inChannel = digitalRead(CHANNEL) && inChannel;

            if (!inChannel && !isOutOfChannelExemptCommand(status))
            {
                makeLog(LOG_WARN, "Command 0x%02X rejected: Shuttle not in channel", status);
                setWarning(WARN_NOT_IN_CHANNEL, 5000);
                status = 0;
            }
            else
            {
                currentOperation = mapCmdToOperation(status);
                makeLog(LOG_INFO, "Shuttle accepted CMD: 0x%02X -> State %d", status, currentOperation);
                lastPalletePosition = 0;
                send_Cmd();

                if (status == CMD_MANUAL_MODE || pendingDisplayManualModeBypass)
                {
                    const bool bypassedFromDisplay = pendingDisplayManualModeBypass;
                    clearManualRadioHold();
                    touchManualSession();
                    currentOperation = STATE_MANUAL;
                    if (bypassedFromDisplay)
                    {
                        makeLog(LOG_INFO, "Manual mode bypass from display command: 0x%02X", status);
                    }
                    makeLog(
                        LOG_INFO,
                        "Manual enter cmd=%02X src=%s",
                        status,
                        bypassedFromDisplay ? "display" : (statusSourceRadio ? "radio" : "display"));
                    pendingDisplayManualModeBypass = false;
                    currentMode                    = CoreOpMode::MANUAL;
                    send_Cmd();
                }
                else
                {
                    clearManualRadioHold();
                    currentMode = CoreOpMode::AUTO_EXEC;
                }
            }
        }

        if (digitalRead(WHITE_LED))
            digitalWrite(WHITE_LED, LOW);
        Can1.read(CAN_RX_msg);

        if (millis() - count > 1000 && !motorStart)
        {
            counter = 0;
            digitalToggle(GREEN_LED);
            digitalToggle(BOARD_LED);
            reportCounter++;
            count = millis();
            SystemYield();
            if (reportCounter == 10)
            {
                reportCounter = 0;
            }
            send_Cmd();
        }

        detect_Pallete();
        break;
    }

    case CoreOpMode::AUTO_EXEC:
    {
        run_Cmd();

        if (status != CMD_STOP)
        {
            status = 0;
        }
        currentOperation = STATE_IDLE;
        currentMode      = CoreOpMode::IDLE;
        break;
    }

    case CoreOpMode::MANUAL:
    {
        if (status == CMD_MOVE_RIGHT_MAN)
        {
            moove_Right();
        }
        else if (status == CMD_MOVE_LEFT_MAN)
        {
            moove_Left();
        }
        else if (status == CMD_MOVE_DIST_R || status == CMD_MOVE_DIST_F)
        {
            clearManualRadioHold();
            currentOperation = STATE_MANUAL;
            makeLog(
                LOG_INFO,
                "Manual step start dir=%s dist=%ld",
                status == CMD_MOVE_DIST_F ? "forward" : "reverse",
                (long)mooveDistance);
            send_Cmd();
            touchManualSession();
            if (status == CMD_MOVE_DIST_F)
            {
                moove_Distance_F(mooveDistance);
            }
            else
            {
                moove_Distance_R(mooveDistance);
            }
            if (!shouldAbortLoop())
            {
                touchManualSession();
                status           = CMD_MANUAL_MODE;
                currentOperation = STATE_MANUAL;
                makeLog(LOG_INFO, "Manual step done -> ready");
                send_Cmd();
            }
            else
            {
                makeLog(LOG_WARN, "Manual step abort status=%02X", status);
            }
        }
        else if (status == CMD_STOP_MANUAL)
        {
            clearManualRadioHold();
            touchManualSession();
            pendingDisplayManualModeBypass = false;
            status                         = CMD_MANUAL_MODE;
            currentOperation               = STATE_MANUAL;
            makeLog(LOG_INFO, "Manual stop -> ready");
            send_Cmd();
        }
        else if (status == CMD_LIFT_UP || status == CMD_LIFT_DOWN)
        {
            clearManualRadioHold();
            currentOperation = STATE_MANUAL;
            makeLog(LOG_INFO, "Manual lift start dir=%s", status == CMD_LIFT_UP ? "up" : "down");
            send_Cmd();
            touchManualSession();
            if (status == CMD_LIFT_UP)
            {
                lifter_Up();
            }
            else
            {
                lifter_Down();
            }
            if (!shouldAbortLoop())
            {
                touchManualSession();
                status           = CMD_MANUAL_MODE;
                currentOperation = STATE_MANUAL;
                makeLog(LOG_INFO, "Manual lift done -> ready");
                send_Cmd();
            }
            else
            {
                makeLog(LOG_WARN, "Manual lift abort status=%02X", status);
            }
        }
        else if (status == CMD_LOAD)
        {
            currentOperation = STATE_LOAD;
            run_Cmd();
        }
        else if (status == CMD_UNLOAD)
        {
            currentOperation = STATE_UNLOAD;
            run_Cmd();
        }
        else if (status == CMD_LONG_LOAD)
        {
            currentOperation = STATE_LONG_LOAD;
            run_Cmd();
        }
        else if (status == CMD_LONG_UNLOAD)
        {
            currentOperation = STATE_LONG_UNLOAD;
            run_Cmd();
        }

        if (millis() - lastManualSessionActivityTime > kManualSessionIdleTimeoutMs)
        {
            clearManualRadioHold();
            pendingDisplayManualModeBypass = false;
            makeLog(LOG_WARN, "Manual session timeout -> idle");
            setWarning(WARN_MANUAL_TIMEOUT, 5000);
            currentOperation = STATE_IDLE;
            currentMode      = CoreOpMode::IDLE;
            status           = CMD_STOP;
        }

        if (millis() - count > 330)
        {
            digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
            digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));
            count = millis();
            SystemYield();
            while (Can1.read(CAN_RX_msg))
                ;
            reportCounter++;
            if (reportCounter == 3)
            {
                delay(5);
                makeLog(LOG_DEBUG, "manual mode...");
                reportCounter = 0;
            }
        }

        if (status == CMD_STOP)
        {
            clearManualRadioHold();
            pendingDisplayManualModeBypass = false;
            makeLog(LOG_INFO, "Manual exit -> idle");
            currentOperation = STATE_IDLE;
            currentMode      = CoreOpMode::IDLE;
        }
        break;
    }
    case CoreOpMode::ERROR:
    {
        currentOperation = STATE_ERROR;
        clearManualRadioHold();
        if (!isPhysicallyStationary())
        {
            motor_Force_Stop();
        }
        if (digitalRead(WHITE_LED))
            digitalWrite(WHITE_LED, LOW);
        blink_Error();

        if (status == CMD_SYSTEM_RESET)
        {
            performSystemReset();
        }
        else if (status == CMD_RESET_ERROR)
        {
            clearLatchedAlertsAndReturnToIdle(millis());
        }
        else if (status == CMD_SAVE_EEPROM)
        {
            saveEEPROMData(eepromData);
            status = 0;
        }

        detect_Pallete();
        break;
    }
    }
}

// Функции
#pragma region Функции...

#pragma region Функции управления двигателем движения...

// Установка скорости движения
void motor_Speed(int spd)
{
    if (motorReverse == 2 || millis() - countMoove < 50)
        return;
    countMoove = millis();
    while (Can1.read(CAN_RX_msg))
        ;
    cracked_int_t hexSpeed;
    CAN_TX_msg.id  = (100);
    CAN_TX_msg.len = 4;
    uint8_t accel  = 18;
    if (spd >= 10 && spd - oldSpeed >= 10)
        spd = oldSpeed + 10;
    if (spd > 100)
        spd = 100;
    if (spd <= 100 && spd >= 0)
    {
        if (spd > oldSpeed)
        {
            uint8_t steps = (spd - oldSpeed) / 2;
            for (uint8_t i = 0; i < steps; i++)
            {
                blink_Work();
                get_Distance();
                hexSpeed.vint = minSpeed + oldSpeed * maxSpeed / 100 + (spd - oldSpeed) * i * maxSpeed / (steps * 100);
                if (motorReverse ^ inverse)
                    hexSpeed.vint = -hexSpeed.vint;
                hexSpeed.vint *= 1000;
                for (uint8_t j = 0; j < 4; j++)
                {
                    CAN_TX_msg.buf[j] = (unsigned char)hexSpeed.bint[3 - j];
                }
                Can1.write(CAN_TX_msg);
                set_Position();
                count = millis();
                if (i > 2 && i <= 40 && lifterUp)
                    accel = 80 - i * 15 / 10;
                else
                    accel = 35;
                while (millis() - count < accel)
                {
                    SystemYield();
                    blink_Work();
                    get_Distance();
                    if (shouldAbortLoop())
                    {
                        preserveManualStopOnAbort();
                        motor_Stop();
                        return;
                    }
                }
            }
            if (spd)
                hexSpeed.vint = minSpeed + spd * maxSpeed / 100;
            else
                hexSpeed.vint = 0;
            if (motorReverse ^ inverse)
                hexSpeed.vint = -hexSpeed.vint;
            hexSpeed.vint *= 1000;
            for (uint8_t i = 0; i < 4; i++)
            {
                CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
            }
            Can1.write(CAN_TX_msg);
            while (Can1.read(CAN_RX_msg))
                ;
            oldSpeed = spd;
        }
        else if (spd < oldSpeed)
        {
            uint8_t steps = (oldSpeed - spd) / 2;
            if (distance[0] <= 400 || distance[1] <= 400)
                steps /= 2;
            for (uint8_t i = 0; i < steps; i++)
            {
                blink_Work();
                get_Distance();
                hexSpeed.vint = minSpeed + oldSpeed * maxSpeed / 100 - (oldSpeed - spd) * i * maxSpeed / (steps * 100);
                if (motorReverse ^ inverse)
                    hexSpeed.vint = -hexSpeed.vint;
                hexSpeed.vint *= 1000;
                for (uint8_t j = 0; j < 4; j++)
                {
                    CAN_TX_msg.buf[j] = (unsigned char)hexSpeed.bint[3 - j];
                }
                Can1.write(CAN_TX_msg);
                set_Position();
                count = millis();
                if (lifterUp)
                    accel = 10 + i * 20 / steps;
                while (millis() - count < accel)
                {
                    SystemYield();
                    get_Distance();
                    if (shouldAbortLoop())
                    {
                        preserveManualStopOnAbort();
                        motor_Stop();
                        return;
                    }
                    if (motorReverse == 2)
                        return;
                }
            }
            if (spd)
                hexSpeed.vint = minSpeed + spd * maxSpeed / 100;
            else
                hexSpeed.vint = 0;
            if (motorReverse ^ inverse)
                hexSpeed.vint = -hexSpeed.vint;
            hexSpeed.vint *= 1000;
            for (uint8_t i = 0; i < 4; i++)
            {
                CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
            }
            Can1.write(CAN_TX_msg);
            while (Can1.read(CAN_RX_msg))
                ;
            oldSpeed = spd;
        }
        else if (spd == oldSpeed)
        {
            hexSpeed.vint = minSpeed + spd * maxSpeed / 100;
            if (motorReverse ^ inverse)
                hexSpeed.vint = -hexSpeed.vint;
            hexSpeed.vint *= 1000;
            for (uint8_t i = 0; i < 4; i++)
            {
                CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
            }
            Can1.write(CAN_TX_msg);
            while (Can1.read(CAN_RX_msg))
                ;
        }
    }
    else
    {
        hexSpeed.vint = 0;
        for (uint8_t i = 0; i < 4; i++)
        {
            CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
        }
        Can1.write(CAN_TX_msg);
        while (Can1.read(CAN_RX_msg))
            ;
        oldSpeed = 0;
    }
    set_Position();
}

// Установка движения вперед
void motor_Start_Forward()
{
    motorStart   = 1;
    motorReverse = 0;
}

// Установка движения назад
void motor_Start_Reverse()
{
    motorStart   = 1;
    motorReverse = 1;
}

// Остановка движения
void motor_Stop()
{
    if (motorReverse == 2)
        return;
    Can1.read(CAN_RX_msg);
    makeLog(LOG_INFO, "Motor stop, speed = %d", oldSpeed);
    CAN_TX_msg.id  = (100);
    CAN_TX_msg.len = 4;
    cracked_int_t hexSpeed;
    if (oldSpeed > 1)
    {
        uint8_t maxi = (uint8_t)oldSpeed / 2;
        for (uint8_t i = maxi; i > 0; i--)
        {
            blink_Work();
            get_Distance();
            hexSpeed.vint = minSpeed + oldSpeed * maxSpeed * i / (maxi * 100);
            hexSpeed.vint *= 1000;
            if (motorReverse ^ inverse)
                hexSpeed.vint = -hexSpeed.vint;
            for (uint8_t j = 0; j < 4; j++)
            {
                CAN_TX_msg.buf[j] = (unsigned char)hexSpeed.bint[3 - j];
            }
            Can1.write(CAN_TX_msg);
            while (Can1.read(CAN_RX_msg))
                ;
            if ((status == CMD_LONG_UNLOAD || status == CMD_UNLOAD) && lifterUp && distance[3] + 100 < distance[1])
                delay(10);
            else if (maxi > 10)
                delay(10 + i * 20 / maxi);
            else
                delay(50);
        }
    }
    set_Position();
    hexSpeed.vint = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
    }
    Can1.write(CAN_TX_msg);
    while (Can1.read(CAN_RX_msg))
        ;
    if ((status == CMD_LONG_UNLOAD || status == CMD_UNLOAD) && lifterUp && distance[3] + 100 < distance[1])
    {
        for (uint8_t i = 0; i < 10; i++)
        {
            count = millis();
            while (millis() - count < 100)
                blink_Work();
            Can1.write(CAN_TX_msg);
            while (Can1.read(CAN_RX_msg))
                ;
        }
    }
    else
    {
        for (uint8_t i = 0; i < 1; i++)
        {
            count = millis();
            while (millis() - count < 100)
                blink_Work();
            Can1.write(CAN_TX_msg);
            while (Can1.read(CAN_RX_msg))
                ;
        }
    }
    oldSpeed     = 0;
    motorStart   = 0;
    motorReverse = 2;
    mooveCount   = 0;
    oldPosition  = currentPosition;
}

// Форсмажорная остановка движения
void motor_Force_Stop()
{
    cracked_int_t hexSpeed;
    oldSpeed       = 0;
    hexSpeed.vint  = 0;
    CAN_TX_msg.id  = (100);
    CAN_TX_msg.len = 4;
    for (uint8_t i = 0; i < 4; i++)
    {
        CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
    }
    Can1.write(CAN_TX_msg);
    while (Can1.read(CAN_RX_msg))
        ;
    motorStart   = 0;
    motorReverse = 2;
    mooveCount   = 0;
    oldPosition  = currentPosition;
}
#pragma endregion

#pragma region Функции управления лифтером

// Подъем платформы
void lifter_Up()
{
    if (!digitalRead(DL_UP) && digitalRead(DL_DOWN))
    {
        makeLog(LOG_DEBUG, "Lifter is up... status = %d", status);
        return;
    }
    makeLog(LOG_INFO, "Moove lifter up...");
    int           k           = 0;
    int           summCurrent = 0;
    int           current     = 0;
    cracked_int_t hexSpeed;
    uint32_t      cnt = millis();
    CAN_TX_msg.id     = (101);
    CAN_TX_msg.len    = 4;
    for (uint8_t j = 5; j < 50; j += 5)
    {
        hexSpeed.vint = -j;
        hexSpeed.vint *= 1000;
        for (uint8_t i = 0; i < 4; i++)
        {
            CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
        }
        Can1.write(CAN_TX_msg);
        while (millis() - cnt < 30)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                preserveManualStopOnAbort();
                return;
            }
        }
        cnt = millis();
    }
    cnt           = millis();
    hexSpeed.vint = -50000;
    for (uint8_t i = 0; i < 4; i++)
    {
        CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
    }
    if (digitalRead(DL_UP))
        Can1.write(CAN_TX_msg);
    while (digitalRead(DL_UP))
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            preserveManualStopOnAbort();
            return;
        }
        if (millis() - cnt > (uint32_t)lifterDelay)
        {
            lifter_Stop();
            setFault(FAULT_LIFTER_TIMEOUT);
            makeLog(LOG_ERROR, "Lifter timeout!");
            status = CMD_STOP;
            break;
        }
        delay(10);
        Can1.write(CAN_TX_msg);
        blink_Work();
        get_Distance();

        while (Can1.read(CAN_RX_msg))
        {
            if (!CAN_RX_msg.flags.remote && CAN_RX_msg.id == 2405)
            {
                current = CAN_RX_msg.buf[4] * 256 + CAN_RX_msg.buf[5];
                if (lifterCurrent < current && k > 3)
                    lifterCurrent = current;
                k++;
                if (k > 3)
                    summCurrent += current;
            }
        }
    }
    Can1.write(CAN_TX_msg);
    if (digitalRead(DL_DOWN))
        lifterUp = 1;
    lifter_Stop();
    summCurrent /= k;
    if (lifterCurrent > 500)
    {
        STATS_ATOMIC_UPDATE(sramStats->payload.lifterOverloadCount++);
        lifterCurrent = 250;
    }
    makeLog(LOG_DEBUG, "Summ = %d", summCurrent);
    STATS_ATOMIC_UPDATE(sramStats->payload.liftUpCounter++);
}

// Опускание платформы
void lifter_Down()
{
    if (!digitalRead(DL_DOWN) && digitalRead(DL_UP))
    {
        makeLog(LOG_DEBUG, "Lifter is down... status = %d", status);
        return;
    }
    makeLog(LOG_INFO, "Moove lifter down... status = %d", status);
    uint32_t      cnt = millis();
    cracked_int_t hexSpeed;
    CAN_TX_msg.id  = (101);
    CAN_TX_msg.len = 4;
    for (uint8_t j = 5; j < 50; j += 5)
    {
        hexSpeed.vint = j;
        hexSpeed.vint *= 1000;
        for (uint8_t i = 0; i < 4; i++)
        {
            CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
        }
        Can1.write(CAN_TX_msg);
        while (millis() - cnt < 30)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                preserveManualStopOnAbort();
                return;
            }
        }
        cnt = millis();
    }
    cnt           = millis();
    hexSpeed.vint = 50000;
    for (uint8_t i = 0; i < 4; i++)
    {
        CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
    }
    if (digitalRead(DL_DOWN))
        Can1.write(CAN_TX_msg);
    while (digitalRead(DL_DOWN))
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            preserveManualStopOnAbort();
            return;
        }
        if (millis() - cnt > (uint32_t)lifterDelay)
        {
            lifter_Stop();
            setFault(FAULT_LIFTER_TIMEOUT);
            makeLog(LOG_ERROR, "Lifter timeout!");
            status = CMD_STOP;
            break;
        }
        delay(10);
        Can1.write(CAN_TX_msg);
        blink_Work();
        get_Distance();
    }

    if (!digitalRead(DL_DOWN))
        lifterUp = 0;
    lifter_Stop();
    load          = 0;
    lifterCurrent = 0;
    STATS_ATOMIC_UPDATE(sramStats->payload.liftDownCounter++);
}

// Остановка платформы
void lifter_Stop()
{
    makeLog(LOG_INFO, "Stop lifter...");
    cracked_int_t hexSpeed;
    hexSpeed.vint  = 0;
    CAN_TX_msg.id  = (101);
    CAN_TX_msg.len = 4;
    for (uint8_t i = 0; i < 4; i++)
    {
        CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];
    }
    Can1.write(CAN_TX_msg);
}
#pragma endregion

#pragma region Служебные функции

enum ParseState
{
    STATE_WAIT_SYNC1,
    STATE_WAIT_SYNC2,
    STATE_READ_HEADER,
    STATE_READ_PAYLOAD,
    STATE_READ_CRC
};

static inline bool buildRemoteRouteForShuttleId(uint8_t shuttleId, E22Radio::FixedRoute *outRoute)
{
    if (!outRoute || !E22Radio::isValidNodeId(shuttleId))
    {
        return false;
    }

    *outRoute = E22Radio::fixedRouteToRemote(shuttleId, E22Radio::kDefaultChannel440);
    return true;
}

static inline int transportAvailableForWrite(Stream *port)
{
    if (!port)
    {
        return 0;
    }

    const int available = port->availableForWrite();
    if (port != &SerialLora)
    {
        return available;
    }

    return (available > 3) ? (available - 3) : 0;
}

static size_t
writeTransportPayload(
    Stream *port,
    const uint8_t *buffer,
    uint16_t length,
    const E22Radio::FixedRoute *fixedRoute,
    RadioTxAuxPolicy auxPolicy)
{
    if (!port || !buffer || length == 0)
    {
        return 0;
    }
    if (transportAvailableForWrite(port) < length)
    {
        return 0;
    }

    if (port != &SerialLora)
    {
        return port->write(buffer, length);
    }

    if (e22Radio.hasAuxPin() && !e22Radio.waitAuxHigh(kRadioRuntimeAuxWaitMs))
    {
        LOG_RATE_LIMITED(
            LOG_WARN,
            1000,
            "Radio TX AUX busy len=%u policy=%u",
            length,
            auxPolicy == RadioTxAuxPolicy::SendAfterTimeout ? 1 : 0);
        if (auxPolicy == RadioTxAuxPolicy::DropIfBusy)
        {
            return 0;
        }
    }

    E22Radio::FixedRoute        route      = { 0, 0, 0 };
    const E22Radio::FixedRoute *routeToUse = fixedRoute;
    if (routeToUse == NULL)
    {
        if (!buildRemoteRouteForShuttleId(shuttleNum, &route))
        {
            return 0;
        }
        routeToUse = &route;
    }

    const uint8_t routeHeader[3] = { routeToUse->addh, routeToUse->addl, routeToUse->channel };
    const size_t  routeWritten   = port->write(routeHeader, sizeof(routeHeader));
    const size_t  payloadWritten = port->write(buffer, length);
    if (routeWritten != sizeof(routeHeader) || payloadWritten != length)
    {
        return 0;
    }
    return length;
}

void sendAck(uint8_t seq, AckResult result, Stream *port, const E22Radio::FixedRoute *fixedRoute)
{
    if (!port)
        return;
    uint8_t   localTxBuffer[sizeof(FrameHeader) + sizeof(AckPacket) + 2];
    AckPacket pkt;
    pkt.refSeq = seq;
    pkt.result = result;

    FrameHeader *header       = (FrameHeader *)localTxBuffer;
    header->sync1             = PROTOCOL_SYNC_1_V2;
    header->sync2             = PROTOCOL_SYNC_2_V2;
    header->length            = sizeof(AckPacket);
    header->targetID          = TARGET_ID_NONE;
    static uint8_t seqCounter = 0;
    header->seq               = seqCounter++;
    header->msgID             = MSG_ACK;

    memcpy(localTxBuffer + sizeof(FrameHeader), &pkt, sizeof(AckPacket));
    uint16_t totalLen = sizeof(FrameHeader) + header->length;
    ProtocolUtils::appendCRC(localTxBuffer, totalLen);

    if (writeTransportPayload(port, localTxBuffer, totalLen + 2, fixedRoute, RadioTxAuxPolicy::SendAfterTimeout) == totalLen + 2)
    {
        countTxFrame(port);
    }
}

void sendTelemAck(
    uint8_t                     seq,
    AckResult                   result,
    Stream                     *port,
    int16_t                     shuttleStatusOverride,
    const E22Radio::FixedRoute *fixedRoute)
{
    if (!port || pendingBootloaderEntry)
        return;

    uint8_t        localTxBuffer[sizeof(FrameHeader) + sizeof(AckTelemPacket) + 2];
    AckTelemPacket pkt;
    pkt.ack.refSeq = seq;
    pkt.ack.result = result;
    populateTelemetryPacket(&pkt.telemetry, shuttleStatusOverride);

    FrameHeader *header       = (FrameHeader *)localTxBuffer;
    header->sync1             = PROTOCOL_SYNC_1_V2;
    header->sync2             = PROTOCOL_SYNC_2_V2;
    header->length            = sizeof(AckTelemPacket);
    header->targetID          = TARGET_ID_NONE;
    static uint8_t seqCounter = 0;
    header->seq               = seqCounter++;
    header->msgID             = MSG_ACK_TELEM;

    memcpy(localTxBuffer + sizeof(FrameHeader), &pkt, sizeof(AckTelemPacket));
    uint16_t totalLen = sizeof(FrameHeader) + header->length;
    ProtocolUtils::appendCRC(localTxBuffer, totalLen);

    if (writeTransportPayload(port, localTxBuffer, totalLen + 2, fixedRoute, RadioTxAuxPolicy::SendAfterTimeout) == totalLen + 2)
    {
        countTxFrame(port);
    }
}

void sendCommandAck(
    uint8_t   seq,
    AckResult result,
    Stream   *port,
    bool      suppressAck,
    bool      useTelemAck,
    int16_t   shuttleStatusOverride)
{
    if (suppressAck)
        return;
    if (useTelemAck)
        sendTelemAck(seq, result, port, shuttleStatusOverride);
    else
        sendAck(seq, result, port);
}

uint8_t processPacket(FrameHeader *header, uint8_t *payload, Stream *replyPort)
{
    if (header->targetID != TARGET_ID_NONE && header->targetID != TARGET_ID_BROADCAST && header->targetID != shuttleNum)
    {
        DIAG_ONLY(
            if (replyPort == &SerialLora) linkDiag.radioRxDropTarget++;
            else if (replyPort == &SerialDisplay) linkDiag.displayRxDropTarget++;);

        LOG_RATE_LIMITED(
            LOG_WARN,
            2000,
            "Drop tgt src=%s exp=%u got=%u msg=%02X",
            (replyPort == &SerialLora) ? "radio" : "display",
            shuttleNum,
            header->targetID,
            header->msgID);
        return NO_NEW_CMD;
    }

    bool    requiresNoAck    = (header->msgID & MSG_FLAG_NO_ACK) != 0;
    bool    requiresTelemAck = (header->msgID & MSG_FLAG_REQ_TELEM) != 0;
    bool    useTelemAck      = requiresTelemAck && replyPort == &SerialLora;
    uint8_t realMsgID        = header->msgID & MSG_ID_MASK;

    if (realMsgID == MSG_REQ_HEARTBEAT)
    {
        if (replyPort == &SerialLora && manualRadioHoldActive)
        {
            refreshManualRadioHoldWatchdog(millis());
            touchManualSession();
        }
        uint32_t startUs = micros();
        sendTelemetryPacket(replyPort);
        countReplyTiming(replyPort, micros() - startUs);
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_REQ_SENSORS)
    {
        uint32_t startUs = micros();
        sendSensorPacket(replyPort);
        countReplyTiming(replyPort, micros() - startUs);
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_REQ_STATS)
    {
        uint32_t startUs = micros();
        sendStatsPacket(replyPort);
        countReplyTiming(replyPort, micros() - startUs);
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_REQ_LINK_HEALTH)
    {
        uint32_t startUs = micros();
        sendLinkHealthPacket(replyPort);
        countReplyTiming(replyPort, micros() - startUs);
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_CMD_SIMPLE || realMsgID == MSG_CMD_WITH_ARG)
    {
        uint8_t reqCmd =
            (realMsgID == MSG_CMD_SIMPLE) ? ((SimpleCmdPacket *)payload)->cmdType : ((ParamCmdPacket *)payload)->cmdType;

        if (!isSupportedCommand(reqCmd))
        {
            sendCommandAck(header->seq, ACK_REJECTED, replyPort, requiresNoAck, useTelemAck);
            return NO_NEW_CMD;
        }

        if (reqCmd == CMD_SAVE_EEPROM)
        {
            pendingEepromSave = true;
            sendCommandAck(header->seq, ACK_OK, replyPort, requiresNoAck, useTelemAck);
            return NO_NEW_CMD;
        }
        if (reqCmd == CMD_GET_CONFIG)
        {
            sendCommandAck(header->seq, ACK_OK, replyPort, requiresNoAck, useTelemAck);
            return NO_NEW_CMD;
        }

        if (reqCmd == CMD_FIRMWARE_UPDATE)
        {
            if (replyPort != &SerialDisplay)
            {
                sendAck(header->seq, ACK_REJECTED, replyPort);
                return NO_NEW_CMD;
            }

            if (!isPhysicallyStationary())
            {
                sendAck(header->seq, ACK_BUSY, replyPort);
                return NO_NEW_CMD;
            }

            sendAck(header->seq, ACK_OK, replyPort);
            scheduleBootloaderEntry();
            return NO_NEW_CMD;
        }

        if (!isProvisionedShuttle() && !isUnprovisionedCommandAllowed(reqCmd))
        {
            if (reqCmd == CMD_MANUAL_MODE || isContinuousManualCommand(reqCmd) || isManualDistanceCommand(reqCmd) ||
                isLiftCommand(reqCmd))
            {
                makeLog(LOG_WARN, "Manual reject cmd=%02X reason=not_prov", reqCmd);
            }
            sendCommandAck(header->seq, ACK_BAD_ENVIRONMENT, replyPort, requiresNoAck, useTelemAck);
            return NO_NEW_CMD;
        }

        bool inChannel = digitalRead(CHANNEL);

        if (isErrorActive() && !isOverrideCommand(reqCmd))
        {
            if (reqCmd == CMD_MANUAL_MODE || isContinuousManualCommand(reqCmd) || isManualDistanceCommand(reqCmd) ||
                isLiftCommand(reqCmd))
            {
                makeLog(LOG_WARN, "Manual reject cmd=%02X reason=error", reqCmd);
            }
            sendCommandAck(header->seq, ACK_ERROR_STATE, replyPort, requiresNoAck, useTelemAck);
            return NO_NEW_CMD;
        }

        if (!inChannel && !isOutOfChannelExemptCommand(reqCmd))
        {
            if (reqCmd == CMD_MANUAL_MODE || isContinuousManualCommand(reqCmd) || isManualDistanceCommand(reqCmd) ||
                isLiftCommand(reqCmd))
            {
                makeLog(LOG_WARN, "Manual reject cmd=%02X reason=channel", reqCmd);
            }
            setWarning(WARN_NOT_IN_CHANNEL, 5000);
            sendCommandAck(header->seq, ACK_BAD_ENVIRONMENT, replyPort, requiresNoAck, useTelemAck);
            return NO_NEW_CMD;
        }

        if (canAcceptCommandNow(reqCmd, replyPort == &SerialLora))
        {
            if (realMsgID == MSG_CMD_WITH_ARG)
            {
                ParamCmdPacket *cmdArgs = (ParamCmdPacket *)payload;
                if (isManualDistanceCommand(reqCmd))
                    mooveDistance = cmdArgs->arg;
                else if (reqCmd == CMD_LONG_UNLOAD_QTY)
                    UPQuant = (uint8_t)cmdArgs->arg;
            }

            int16_t telemStateOverride = -1;
            if (useTelemAck && replyPort == &SerialLora)
            {
                telemStateOverride = (int16_t)predictTelemetryStateForAcceptedCommand(reqCmd, true);
            }
            sendCommandAck(header->seq, ACK_OK, replyPort, requiresNoAck, useTelemAck, telemStateOverride);
            return reqCmd;
        }
        else
        {
            if (reqCmd == CMD_MANUAL_MODE || isContinuousManualCommand(reqCmd) || isManualDistanceCommand(reqCmd) ||
                isLiftCommand(reqCmd))
            {
                makeLog(LOG_WARN, "Manual reject cmd=%02X reason=busy", reqCmd);
            }
            sendCommandAck(header->seq, ACK_BUSY, replyPort, requiresNoAck, useTelemAck);
            return NO_NEW_CMD;
        }
    }
    else if (realMsgID == MSG_SET_DATETIME)
    {
        DateTimePacket *dt = (DateTimePacket *)payload;
        if (!requiresNoAck)
            sendAck(header->seq, ACK_OK, replyPort);

        rtc.setTime(dt->hour, dt->minute, dt->second);
        rtc.setDate(getWeekDay(dt->day, dt->month, dt->year + 2000), dt->day, dt->month, dt->year);
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_CONFIG_SET)
    {
        ConfigPacket *cfg = (ConfigPacket *)payload;
        if (!canApplyConfigNow())
        {
            if (!requiresNoAck)
                sendAck(header->seq, ACK_BUSY, replyPort);
            return NO_NEW_CMD;
        }

        switch (cfg->paramID)
        {
        case CFG_SHUTTLE_NUM:
        {
            if (!isValidProvisionedShuttleId(cfg->value))
            {
                if (!requiresNoAck)
                    sendAck(header->seq, ACK_REJECTED, replyPort);
                return NO_NEW_CMD;
            }
            uint8_t newId = (uint8_t)cfg->value;
            if (eepromData.shuttleNum == newId)
            {
                if (!requiresNoAck)
                    sendAck(header->seq, ACK_OK, replyPort);
                return NO_NEW_CMD;
            }
            const uint8_t              oldId       = shuttleNum;
            E22Radio::FixedRoute        ackRoute    = { 0, 0, 0 };
            const E22Radio::FixedRoute *ackRoutePtr = NULL;
            if (replyPort == &SerialLora)
            {
                const uint8_t oldReplyId = E22Radio::isValidNodeId(header->targetID) ? header->targetID : oldId;
                if (buildRemoteRouteForShuttleId(oldReplyId, &ackRoute))
                {
                    ackRoutePtr = &ackRoute;
                }
            }
            eepromData.shuttleNum = newId;
            shuttleNum            = newId;
            pendingEepromSave     = true;
            if (!requiresNoAck)
                sendAck(header->seq, ACK_OK, replyPort, ackRoutePtr);
            reapplyRadioConfigForShuttleId(newId, oldId);
            return NO_NEW_CMD;
        }
        case CFG_INTER_PALLET:
        {
            uint16_t value                  = (uint16_t)cfg->value;
            eepromData.interPalleteDistance = value;
            interPalleteDistance            = value;
            break;
        }
        case CFG_SHUTTLE_LEN:
        {
            uint16_t value           = (uint16_t)cfg->value;
            eepromData.shuttleLength = value;
            shuttleLength            = value;
            break;
        }
        case CFG_MAX_SPEED:
        {
            uint16_t value      = (uint16_t)cfg->value;
            eepromData.maxSpeed = value;
            maxSpeed            = value;
            break;
        }
        case CFG_MIN_BATT:
        {
            uint8_t value            = (uint8_t)cfg->value;
            eepromData.minBattCharge = value;
            minBattCharge            = value;
            break;
        }
        case CFG_WAIT_TIME:
        {
            int value           = (int)cfg->value;
            eepromData.waitTime = value;
            waitTime            = value;
            break;
        }
        case CFG_MPR_OFFSET:
        {
            int8_t value         = (int8_t)cfg->value;
            eepromData.mprOffset = value;
            mprOffset            = value;
            break;
        }
        case CFG_CHNL_OFFSET:
        {
            int8_t value          = (int8_t)cfg->value;
            eepromData.chnlOffset = value;
            chnlOffset            = value;
            break;
        }
        case CFG_FIFO_LIFO:
        {
            uint8_t value       = (uint8_t)cfg->value;
            eepromData.fifoLifo = value;
            fifoLifo            = value;
            break;
        }
        case CFG_REVERSE_MODE:
            if (eepromData.inverse != (uint8_t)cfg->value)
            {
                currentPosition = channelLength - currentPosition - 800;
            }
            eepromData.inverse = (uint8_t)cfg->value;
            inverse            = (uint8_t)cfg->value;
            break;
        default:
            if (!requiresNoAck)
                sendAck(header->seq, ACK_REJECTED, replyPort);
            return NO_NEW_CMD;
        }
        if (!requiresNoAck)
            sendAck(header->seq, ACK_OK, replyPort);
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_CONFIG_SYNC_PUSH)
    {
        FullConfigPacket *fullCfg = (FullConfigPacket *)payload;
        if (!canApplyConfigNow())
        {
            if (!requiresNoAck)
                sendAck(header->seq, ACK_BUSY, replyPort);
            return NO_NEW_CMD;
        }

        if (!isValidProvisionedShuttleId(fullCfg->shuttleNumber))
        {
            if (!requiresNoAck)
                sendAck(header->seq, ACK_REJECTED, replyPort);
            return NO_NEW_CMD;
        }

        bool                        shuttleIdChanged = (eepromData.shuttleNum != fullCfg->shuttleNumber);
        const uint8_t               oldId            = shuttleNum;
        E22Radio::FixedRoute        ackRoute         = { 0, 0, 0 };
        const E22Radio::FixedRoute *ackRoutePtr      = NULL;
        if (shuttleIdChanged)
        {
            if (replyPort == &SerialLora)
            {
                const uint8_t oldReplyId = E22Radio::isValidNodeId(header->targetID) ? header->targetID : oldId;
                if (buildRemoteRouteForShuttleId(oldReplyId, &ackRoute))
                {
                    ackRoutePtr = &ackRoute;
                }
            }
        }

        eepromData.interPalleteDistance = fullCfg->interPallet;
        interPalleteDistance            = fullCfg->interPallet;
        eepromData.shuttleLength        = fullCfg->shuttleLen;
        shuttleLength                   = fullCfg->shuttleLen;
        eepromData.maxSpeed             = fullCfg->maxSpeed;
        maxSpeed                        = fullCfg->maxSpeed;
        eepromData.waitTime             = fullCfg->waitTime;
        waitTime                        = fullCfg->waitTime;
        eepromData.mprOffset            = (int8_t)fullCfg->mprOffset;
        mprOffset                       = (int8_t)fullCfg->mprOffset;
        eepromData.chnlOffset           = (int8_t)fullCfg->chnlOffset;
        chnlOffset                      = (int8_t)fullCfg->chnlOffset;
        eepromData.shuttleNum           = fullCfg->shuttleNumber;
        shuttleNum                      = fullCfg->shuttleNumber;
        eepromData.minBattCharge        = fullCfg->minBatt;
        minBattCharge                   = fullCfg->minBatt;
        eepromData.fifoLifo             = fullCfg->fifoLifo;
        fifoLifo                        = fullCfg->fifoLifo;
        eepromData.inverse              = fullCfg->reverseMode;
        inverse                         = fullCfg->reverseMode;

        if (shuttleIdChanged)
            pendingEepromSave = true;

        if (!requiresNoAck)
            sendAck(header->seq, ACK_OK, replyPort, ackRoutePtr);
        if (shuttleIdChanged)
            reapplyRadioConfigForShuttleId(fullCfg->shuttleNumber, oldId);
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_CONFIG_SYNC_REQ)
    {
        uint32_t         startUs = micros();
        FullConfigPacket rep;
        rep.interPallet   = interPalleteDistance;
        rep.shuttleLen    = shuttleLength;
        rep.maxSpeed      = maxSpeed;
        rep.waitTime      = waitTime;
        rep.mprOffset     = mprOffset;
        rep.chnlOffset    = chnlOffset;
        rep.shuttleNumber = shuttleNum;
        rep.minBatt       = minBattCharge;
        rep.fifoLifo      = fifoLifo;
        rep.reverseMode   = inverse;

        uint8_t      localTxBuffer[sizeof(FrameHeader) + sizeof(FullConfigPacket) + 2];
        FrameHeader *repHeader = (FrameHeader *)localTxBuffer;
        repHeader->sync1       = PROTOCOL_SYNC_1_V2;
        repHeader->sync2       = PROTOCOL_SYNC_2_V2;
        repHeader->length      = sizeof(FullConfigPacket);
        repHeader->targetID    = TARGET_ID_NONE;
        static uint8_t repSeq  = 0;
        repHeader->seq         = repSeq++;
        repHeader->msgID       = MSG_CONFIG_SYNC_REP;

        memcpy(localTxBuffer + sizeof(FrameHeader), &rep, sizeof(FullConfigPacket));
        uint16_t totalLen = sizeof(FrameHeader) + repHeader->length;
        ProtocolUtils::appendCRC(localTxBuffer, totalLen);

        if (transportAvailableForWrite(replyPort) >= totalLen + 2 &&
            writeTransportPayload(replyPort, localTxBuffer, totalLen + 2, NULL) == totalLen + 2)
        {
            countTxFrame(replyPort);
            countReplyTiming(replyPort, micros() - startUs);
        }
        return NO_NEW_CMD;
    }
    else if (realMsgID == MSG_CONFIG_GET)
    {
        uint32_t      startUs = micros();
        ConfigPacket *req     = (ConfigPacket *)payload;
        ConfigPacket  rep;
        rep.paramID = req->paramID;
        rep.value   = 0;

        switch (req->paramID)
        {
        case CFG_SHUTTLE_NUM:
            rep.value = shuttleNum;
            break;
        case CFG_INTER_PALLET:
            rep.value = interPalleteDistance;
            break;
        case CFG_SHUTTLE_LEN:
            rep.value = shuttleLength;
            break;
        case CFG_MAX_SPEED:
            rep.value = maxSpeed;
            break;
        case CFG_MIN_BATT:
            rep.value = minBattCharge;
            break;
        case CFG_WAIT_TIME:
            rep.value = waitTime;
            break;
        case CFG_MPR_OFFSET:
            rep.value = mprOffset;
            break;
        case CFG_CHNL_OFFSET:
            rep.value = chnlOffset;
            break;
        case CFG_FIFO_LIFO:
            rep.value = fifoLifo;
            break;
        case CFG_REVERSE_MODE:
            rep.value = inverse;
            break;
        }

        uint8_t      localTxBuffer[sizeof(FrameHeader) + sizeof(ConfigPacket) + 2];
        FrameHeader *repHeader = (FrameHeader *)localTxBuffer;
        repHeader->sync1       = PROTOCOL_SYNC_1_V2;
        repHeader->sync2       = PROTOCOL_SYNC_2_V2;
        repHeader->length      = sizeof(ConfigPacket);
        repHeader->targetID    = TARGET_ID_NONE;
        static uint8_t repSeq  = 0;
        repHeader->seq         = repSeq++;
        repHeader->msgID       = MSG_CONFIG_REP;

        memcpy(localTxBuffer + sizeof(FrameHeader), &rep, sizeof(ConfigPacket));

        uint16_t totalLen = sizeof(FrameHeader) + repHeader->length;
        ProtocolUtils::appendCRC(localTxBuffer, totalLen);

        if (transportAvailableForWrite(replyPort) >= totalLen + 2 &&
            writeTransportPayload(replyPort, localTxBuffer, totalLen + 2, NULL) == totalLen + 2)
        {
            countTxFrame(replyPort);
            countReplyTiming(replyPort, micros() - startUs);
        }
        else
        {
            LOG_RATE_LIMITED(LOG_WARN, 1000, "Dropped Config Reply: TX Buffer Full");
        }

        return NO_NEW_CMD;
    }
    return NO_NEW_CMD;
}

uint8_t pollSerial(Stream &port, ProtocolParser &parser, bool isRadioPort)
{
    uint32_t now     = millis();
    uint8_t  lastCmd = NO_NEW_CMD;
    while (!pendingBootloaderEntry && port.available() > 0)
    {
        FrameHeader *header = parser.feed(port.read(), now);
        if (parser.crcError)
        {
            DIAG_ONLY(if (isRadioPort) linkDiag.radioRxCrcFail++; else linkDiag.displayRxCrcFail++;);
            parser.crcError = false;
            // LOG_RATE_LIMITED(LOG_WARN, 2000, "fail side=shuttle dir=rx reason=crc src=%s count=%lu",
            //                  isRadioPort ? "radio" : "display",
            //                  (unsigned long)(isRadioPort ? linkDiag.radioRxCrcFail : linkDiag.displayRxCrcFail));
        }
        if (header != nullptr)
        {
            DIAG_ONLY(if (isRadioPort) linkDiag.radioRxValid++; else linkDiag.displayRxValid++;);
            if (isRadioPort)
            {
                radioLastRxFrameMs = now;
            }
            if (isRadioPort && radioAppendedRssiEnabled)
            {
                uint8_t rssiRaw = 0;
                e22Radio.readLastPacketRssiRaw(rssiRaw, E22Radio::kDefaultPacketRssiTimeoutMs);
            }
            uint8_t res = processPacket(header, (uint8_t *)header + sizeof(FrameHeader), &port);
            if (res != NO_NEW_CMD)
            {
                if (res == CMD_STOP || res == CMD_STOP_MANUAL)
                    return res;
                lastCmd = res;
            }
        }
    }
    return lastCmd;
}

// Выполнение команд с пульта ДУ
void run_Cmd()
{
    if (status == CMD_MOVE_LEFT_MAN)
    {
        send_Cmd();
        moove_Reverse();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_MOVE_RIGHT_MAN)
    {
        send_Cmd();
        moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_LIFT_UP)
    {
        send_Cmd();
        lifter_Up();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_LIFT_DOWN)
    {
        send_Cmd();
        lifter_Down();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_STOP || status == CMD_STOP_MANUAL)
    {
        if (motorStart)
            motor_Stop();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_LOAD)
    {
        send_Cmd();
        lifter_Down();
        single_Load();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_UNLOAD)
    {
        send_Cmd();
        lifter_Down();
        moove_Forward();
        unload_Pallete();
        status = CMD_MOVE_RIGHT_MAN;
        moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_MOVE_DIST_R)
    {
        send_Cmd();
        moove_Distance_R(mooveDistance);
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_MOVE_DIST_F)
    {
        send_Cmd();
        moove_Distance_F(mooveDistance);
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_CALIBRATE)
    {
        send_Cmd();
        calibrate_Encoder_F();
        calibrate_Encoder_R();
        if (status != CMD_STOP)
            moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_DEMO)
    {
        demo_Mode();
        firstPalletePosition = 0;
    }
    else if (status == CMD_COUNT_PALLETS)
    {
        pallete_Counting_F();
        status = CMD_STOP;
        send_Cmd();
        status = CMD_COUNT_PALLETS;
        makeLog(LOG_INFO, "Pallete count = %d", palleteCount);
        status = CMD_MOVE_RIGHT_MAN;
        moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_SAVE_EEPROM)
    {
        saveEEPROMData(eepromData);
        status = CMD_STOP;
    }
    else if (status == CMD_COMPACT_F)
    {
        send_Cmd();
        pallete_Compacting_F();
        status = CMD_STOP;
        if (status != CMD_STOP)
            moove_Forward();
        send_Cmd();
    }
    else if (status == CMD_COMPACT_R)
    {
        send_Cmd();
        pallete_Compacting_R();
        firstPalletePosition = 0;
        status               = CMD_STOP;
        if (status != CMD_STOP)
            moove_Forward();
        send_Cmd();
    }
    else if (status == CMD_LONG_LOAD)
    {
        send_Cmd();
        lifter_Down();
        long_Load();
        if (status != CMD_STOP)
            moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_LONG_UNLOAD)
    {
        send_Cmd();
        longWork = 1;
        lifter_Down();
        moove_Forward();
        long_Unload();
        longWork = 0;
        if ((status == CMD_STOP && distance[1] > 100) || isErrorActive())
            return;
        status = CMD_MOVE_RIGHT_MAN;
        moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_LONG_UNLOAD_QTY)
    {
        send_Cmd();
        longWork = 1;
        lifter_Down();
        moove_Forward();
        status = CMD_LONG_UNLOAD_QTY;
        long_Unload(UPQuant);
        UPQuant  = 0;
        longWork = 0;
        if ((status == CMD_STOP && distance[1] > 100) || isErrorActive())
            return;
        status = CMD_MOVE_RIGHT_MAN;
        moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
    else if (status == CMD_HOME)
    {
        moove_Forward();
        status = CMD_STOP;
        send_Cmd();
    }
}

// Ответы на запросы от пульта или сетевого клиента
void send_Cmd()
{
    sendTelemetryPacket(&SerialDisplay);
}

// Получение данных с датчиков обраружения паллет
void detect_Pallete()
{
    if (inverse)
    {
        detectPalleteF1 = digitalRead(DATCHIK_R1);
        detectPalleteF2 = digitalRead(DATCHIK_R2);
        detectPalleteR1 = digitalRead(DATCHIK_F1);
        detectPalleteR2 = digitalRead(DATCHIK_F2);
        delay(5);
        if (digitalRead(DATCHIK_R1) != detectPalleteF1)
        {
            delay(5);
            detectPalleteF1 = digitalRead(DATCHIK_R1);
        }
        if (digitalRead(DATCHIK_R2) != detectPalleteF2)
        {
            delay(5);
            detectPalleteF2 = digitalRead(DATCHIK_R2);
        }
        if (digitalRead(DATCHIK_F1) != detectPalleteR1)
        {
            delay(5);
            detectPalleteR1 = digitalRead(DATCHIK_F1);
        }
        if (digitalRead(DATCHIK_F2) != detectPalleteR2)
        {
            delay(5);
            detectPalleteR2 = digitalRead(DATCHIK_F2);
        }
    }
    else
    {
        detectPalleteF1 = digitalRead(DATCHIK_F1);
        detectPalleteF2 = digitalRead(DATCHIK_F2);
        detectPalleteR1 = digitalRead(DATCHIK_R1);
        detectPalleteR2 = digitalRead(DATCHIK_R2);
        delay(5);
        if (digitalRead(DATCHIK_F1) != detectPalleteF1)
        {
            delay(5);
            detectPalleteF1 = digitalRead(DATCHIK_F1);
        }
        if (digitalRead(DATCHIK_F2) != detectPalleteF2)
        {
            delay(5);
            detectPalleteF2 = digitalRead(DATCHIK_F2);
        }
        if (digitalRead(DATCHIK_R1) != detectPalleteR1)
        {
            delay(5);
            detectPalleteR1 = digitalRead(DATCHIK_R1);
        }
        if (digitalRead(DATCHIK_R2) != detectPalleteR2)
        {
            delay(5);
            detectPalleteR2 = digitalRead(DATCHIK_R2);
        }
    }
}

static ShuttleFault tofFaultForSensor(uint8_t sensorIndex)
{
    switch (sensorIndex)
    {
    case 0:
        return FAULT_TOF_CH_R;
    case 1:
        return FAULT_TOF_CH_F;
    case 2:
        return FAULT_TOF_PAL_R;
    case 3:
        return FAULT_TOF_PAL_F;
    default:
        return FAULT_NONE;
    }
}

static const char *tofNameForSensor(uint8_t sensorIndex)
{
    switch (sensorIndex)
    {
    case 0:
        return "ID1 Channel R";
    case 1:
        return "ID2 Channel F";
    case 2:
        return "ID3 Pallet R";
    case 3:
        return "ID4 Pallet F";
    default:
        return "unknown";
    }
}

static void incrementTofFailureCounter(uint8_t &counter)
{
    if (counter < 250)
    {
        counter++;
    }
}

static void resetTofBusVote()
{
    tofBusVoteMask         = 0U;
    tofBusVoteAs5600Failed = false;
}

static bool probeAs5600ForBusVote()
{
    if (!tofBusLinesHighNow())
    {
        return false;
    }

    Wire.beginTransmission(kAs5600I2cAddress);
    if (Wire.write(kAs5600AngleHighReg) != 1U || Wire.endTransmission(true) != 0U)
    {
        return false;
    }

    const uint8_t requested = Wire.requestFrom(kAs5600I2cAddress, kAs5600AngleReadLength);
    uint8_t received = 0U;
    uint8_t bytes[kAs5600AngleReadLength] = { 0U, 0U };
    while (Wire.available() > 0 && received < kAs5600AngleReadLength)
    {
        bytes[received++] = (uint8_t)Wire.read();
    }
    while (Wire.available() > 0)
    {
        (void)Wire.read();
    }

    if (requested != kAs5600AngleReadLength || received != kAs5600AngleReadLength)
    {
        return false;
    }

    as5600LastGoodAngleRaw = (uint16_t)((((uint16_t)bytes[0] << 8) | bytes[1]) & 0x0FFFU);
    as5600LastGoodAngleValid = true;
    return true;
}

static void recordTofBusVoteFailure(uint8_t sensor)
{
    if (sensor >= 4U || tofBusState != TofBusState::Ready || !tofBusLinesHighNow())
    {
        return;
    }

    tofBusVoteMask |= (uint8_t)(1U << sensor);
    if (tofBusVoteMask != 0x0FU)
    {
        return;
    }

    bool as5600FailedThisCall = false;
    if (!tofBusVoteAs5600Failed)
    {
        if (probeAs5600ForBusVote())
        {
            LOG_RATE_LIMITED(LOG_INFO, 30000U, "TOF bus_vote AS5600 ok; no recovery");
            resetTofBusVote();
            return;
        }
        tofBusVoteAs5600Failed = true;
        as5600FailedThisCall    = true;
        LOG_RATE_LIMITED(LOG_WARN, 5000U, "TOF bus_vote 4+AS5600 failed");
    }

    const uint32_t now = millis();
    if (tofRecoveryCooldownElapsed(now) && tofBusLinesHighNow())
    {
        if (!as5600FailedThisCall && probeAs5600ForBusVote())
        {
            resetTofBusVote();
            return;
        }
        (void)performTofWireReinit("bus_vote");
        resetTofBusVote();
    }
}

static void recordTofFailure(uint8_t sensor, const TofI2cDiagnostics *diag)
{
    const uint32_t now = millis();
    incrementTofFailureCounter(tofSensorErrors[sensor]);
    incrementTofFailureCounter(tofReadErrors[sensor]);
    incrementTofFailureCounter(tofConsecutiveTransportFailures[sensor]);

    if (diag != nullptr)
    {
        tofLastDiagStatus[sensor] = (uint8_t)diag->status;
        if (diag->status == TOF_I2C_WIRE_TIMEOUT)
        {
            incrementTofFailureCounter(tofTimeoutErrors[sensor]);
        }
        if (diag->status == TOF_I2C_WIRE_ERROR || diag->status == TOF_I2C_WIRE_TIMEOUT ||
            diag->status == TOF_I2C_READ_FAILED_UNKNOWN)
        {
            incrementTofFailureCounter(tofSharedBusErrors[sensor]);
        }
    }

    if (!tofFailureWarnLogged[sensor] || now - tofLastFailureLogMs[sensor] >= kTofFailureLogIntervalMs)
    {
        makeReliableLog(
            LOG_WARN,
            "TOF %s transport %s n=%u a=%02X reg=%02X ex=%u got=%u wr=%u io=%u/%u",
            tofNameForSensor(sensor),
            TOF_I2C_Status_Name((TofI2cStatus)tofLastDiagStatus[sensor]),
            tofConsecutiveTransportFailures[sensor],
            diag != nullptr ? diag->address : 0,
            diag != nullptr ? diag->reg : 0,
            diag != nullptr ? diag->expected : 0,
            diag != nullptr ? diag->received : 0,
            diag != nullptr ? diag->wireStatus : 0U,
            digitalRead(TOF_I2C_SDA),
            digitalRead(TOF_I2C_SCL));
        tofFailureWarnLogged[sensor] = true;
        tofLastFailureLogMs[sensor]  = (now == 0U) ? 1U : now;
    }

    recordTofBusVoteFailure(sensor);
}

static void checkTofStale(uint32_t now)
{
    if (!tofRuntimeStarted)
    {
        return;
    }

    for (uint8_t sensor = 0U; sensor < 4U; ++sensor)
    {
        const uint32_t referenceMs =
            tofHasTransportSuccess[sensor] ? tofLastTransportSuccessMs[sensor] : tofRuntimeStartMs;
        const uint32_t staleAge = now - referenceMs;
        if (staleAge >= kTofStaleTimeoutMs)
        {
            setFault(tofFaultForSensor(sensor));
            if (!tofFaultLogged[sensor])
            {
                makeReliableLog(
                    LOG_ERROR,
                    "TOF %s stale age=%lu last=%s n=%u st=%s io=%u/%u",
                    tofNameForSensor(sensor),
                    (unsigned long)staleAge,
                    tofHasTransportSuccess[sensor] ? "valid" : "none",
                    tofConsecutiveTransportFailures[sensor],
                    TOF_I2C_Status_Name((TofI2cStatus)tofLastDiagStatus[sensor]),
                    digitalRead(TOF_I2C_SDA),
                    digitalRead(TOF_I2C_SCL));
                tofFaultLogged[sensor] = true;
            }
        }
    }
}

static void recordTofSuccess(uint8_t sensor)
{
    const uint32_t now = millis();
    tofLastTransportSuccessMs[sensor]       = now;
    tofHasTransportSuccess[sensor]          = true;
    tofConsecutiveTransportFailures[sensor] = 0U;
    resetTofBusVote();

    const bool recovered = tofSensorErrors[sensor] != 0U || tofReadErrors[sensor] != 0U ||
                           tofTimeoutErrors[sensor] != 0U || tofSharedBusErrors[sensor] != 0U;
    if (recovered && (tofLastRecoverySummaryLogMs == 0U || now - tofLastRecoverySummaryLogMs >= 60000U))
    {
        makeReliableLog(
            LOG_INFO,
            "TOF %s rec total=%u r=%u t=%u b=%u io=%u/%u",
            tofNameForSensor(sensor),
            tofSensorErrors[sensor],
            tofReadErrors[sensor],
            tofTimeoutErrors[sensor],
            tofSharedBusErrors[sensor],
            digitalRead(TOF_I2C_SDA),
            digitalRead(TOF_I2C_SCL));
        tofLastRecoverySummaryLogMs = now;
    }

    tofSensorErrors[sensor] = 0;
    tofReadErrors[sensor]   = 0;
    tofTimeoutErrors[sensor] = 0;
    tofSharedBusErrors[sensor] = 0;
    tofLastDiagStatus[sensor] = TOF_I2C_OK;
    tofFailureWarnLogged[sensor] = false;
}

// Опрос всех сенсоров расстояния
void get_Distance()
{
    const uint32_t now = millis();
    checkTofStale(now);
    if (now - countSensor < 8U)
        return;

    const uint32_t lastBmsTxFinishedMs = batteryBms.lastTxFinishedMs();
    if (batteryBms.isTxActive() ||
        (lastBmsTxFinishedMs != 0U && now - lastBmsTxFinishedMs < kBmsTofQuietGuardMs))
    {
        return;
    }

    if (!serviceTofBusMonitor(now))
    {
        return;
    }

    TOF_Parameter sensor = {};
    countSensor = now;

    uint8_t currentSensor = sensorIndex;
    sensorIndex           = (sensorIndex + 1) % 4;
    const uint8_t filterIndex = tofFilterIndex[currentSensor];

    const uint8_t sensorId = currentSensor + 1;
    writeBreadcrumb(BC_TOF_POLL, currentSensor, 0, filterIndex);
    TofI2cDiagnostics diag = {};
    writeBreadcrumb(BC_TOF_READ, currentSensor, 0, filterIndex);
    if (!TOF_Inquire_I2C_Decoding_ByID(sensorId, &sensor, &diag))
    {
        writeBreadcrumb(BC_TOF_READ, currentSensor, diag.status, diag.wireStatus);
        recordTofFailure(currentSensor, &diag);
        return;
    }

    recordTofSuccess(currentSensor);
    writeBreadcrumb(BC_TOF_POLL, currentSensor, TOF_I2C_OK, filterIndex);
    uint16_t dist = 1500;
    if (sensor.dis_status == 1U && sensor.dis > 0U && sensor.dis <= 1500U)
    {
        dist = (uint16_t)sensor.dis;
    }

    data[currentSensor][filterIndex] = dist;
    tofFilterIndex[currentSensor]    = (uint8_t)((filterIndex + 1U) % 5U);
    const uint16_t filteredDistance  = filter_Distance(data[currentSensor]);
    if (inverse && currentSensor == 0)
        distance[1] = filteredDistance;
    else if (inverse && currentSensor == 1)
        distance[0] = filteredDistance;
    else if (inverse && currentSensor == 2)
        distance[3] = filteredDistance;
    else if (inverse && currentSensor == 3)
        distance[2] = filteredDistance;
    else
        distance[currentSensor] = filteredDistance;
}

uint16_t filter_Distance(uint16_t arr[5])
{
    uint16_t minVal = arr[0];
    uint16_t maxVal = arr[0];
    uint32_t sum    = 0;

    // Находим min и max, а также сумму всех элементов
    for (int i = 0; i < 5; i++)
    {
        if (arr[i] < minVal)
            minVal = arr[i];
        if (arr[i] > maxVal)
            maxVal = arr[i];
        sum += arr[i];
    }

    // Вычитаем min и max из суммы
    sum = sum - minVal - maxVal;

    // Возвращаем среднее арифметическое оставшихся 3 значений
    return (uint16_t)(sum / 3);
}

// Определяет текущую позицию шаттла в канале по переднему фронту (передней панели)
void set_Position()
{
    uint16_t rawAngle = 0;
    if (!readAs5600AngleForMotion(&rawAngle))
    {
        return;
    }
    angle = rawAngle;
    int diff = 0;
    if ((motorReverse == 0) ^ inverse)
    {
        if (oldAngle - angle > 0 && oldAngle - angle <= 2000)
        {
            uint8_t s = oldAngle / 512;
            uint8_t f = angle / 512;
            if (s == f)
            {
                diff = (oldAngle - angle) * (int)calibrateEncoder_F[s] / 512;
            }
            else
            {
                diff = ((512 * s - angle) * (int)calibrateEncoder_F[f] + (oldAngle - 512 * s) * (int)calibrateEncoder_F[s]) /
                       512;
            }
        }
        else if (angle - oldAngle > 2000)
        {
            diff = (oldAngle * (int)calibrateEncoder_F[0] + (4096 - angle) * (int)calibrateEncoder_F[7]) / 512;
        }
        else if (angle - oldAngle > 0 && angle - oldAngle <= 2000)
        {
            uint8_t s = oldAngle / 512;
            uint8_t f = angle / 512;
            if (s == f)
                diff = (angle - oldAngle) * (int)calibrateEncoder_F[s] / 512;
            else
                diff = ((angle - 512 * f) * (int)calibrateEncoder_F[f] + (512 * f - oldAngle) * (int)calibrateEncoder_F[s]) /
                       512;
        }
        else if (oldAngle - angle > 2000)
        {
            diff = (angle * calibrateEncoder_F[0] + (4096 - oldAngle) * calibrateEncoder_F[7]) / 512;
        }
        if (diff > 0 && !inverse)
        {
            currentPosition -= diff;
            oldAngle = angle;
            STATS_ATOMIC_UPDATE(sramStats->payload.totalDist += diff);
        }
        else if (diff > 0)
        {
            currentPosition += diff;
            oldAngle = angle;
            STATS_ATOMIC_UPDATE(sramStats->payload.totalDist += diff);
        }
    }
    else if ((motorReverse == 1) ^ inverse)
    {
        if (oldAngle - angle > 0 && oldAngle - angle <= 2000)
        {
            uint8_t s = oldAngle / 512;
            uint8_t f = angle / 512;
            if (s == f)
            {
                diff = (oldAngle - angle) * (int)calibrateEncoder_R[s] / 512;
            }
            else
            {
                diff = ((512 * s - angle) * (int)calibrateEncoder_R[f] + (oldAngle - 512 * s) * (int)calibrateEncoder_R[s]) /
                       512;
            }
        }
        else if (angle - oldAngle > 2000)
        {
            diff = (oldAngle * (int)calibrateEncoder_R[0] + (4096 - angle) * (int)calibrateEncoder_R[7]) / 512;
        }
        else if (angle - oldAngle > 0 && angle - oldAngle <= 2000)
        {
            uint8_t s = oldAngle / 512;
            uint8_t f = angle / 512;
            if (s == f)
                diff = (angle - oldAngle) * (int)calibrateEncoder_R[s] / 512;
            else
                diff = ((angle - 512 * f) * (int)calibrateEncoder_R[f] + (512 * f - oldAngle) * (int)calibrateEncoder_R[s]) /
                       512;
        }
        else if (oldAngle - angle > 2000)
        {
            diff = (angle * calibrateEncoder_R[0] + (4096 - oldAngle) * calibrateEncoder_R[7]) / 512;
        }
        if (diff > 0 && !inverse)
        {
            currentPosition += diff;
            oldAngle = angle;
            STATS_ATOMIC_UPDATE(sramStats->payload.totalDist += diff);
        }
        else if (diff != 0)
        {
            currentPosition -= diff;
            oldAngle = angle;
            STATS_ATOMIC_UPDATE(sramStats->payload.totalDist += abs(diff));
        }
    }
    if (currentPosition < 0)
    {
        if (lastPalletePosition)
            lastPalletePosition -= currentPosition;
        channelLength -= currentPosition;
        currentPosition = 0;
    }
    if (channelLength - shuttleLength < currentPosition)
        channelLength = currentPosition + shuttleLength;
}

// Переключение между FiFo и LiFo
void fifoLifo_Inverse()
{
    if (inverse)
    {
        inverse         = 0;
        currentPosition = channelLength - currentPosition - 800;
    }
    else
    {
        inverse = 1;
    }
}

// Моргание светодиодом в режиме работы
void blink_Work()
{
    static uint16_t lastLoggedSpeed = 0xFFFF;
    if (millis() - count2 > (uint32_t)blinkTime)
    { // Счетчик блинков, всего 20 по blinkTime
        counter++;
        Can1.read(CAN_RX_msg); // Считывание Can шины для очистки буфера
        if (counter == 10 && status != CMD_MANUAL_MODE)
        { // Тики для сообщения позиции в канале
            set_Position();
            makeLog(LOG_DEBUG, "Position = %d", currentPosition);
            int diff = abs(currentPosition - oldPosition);
            if (currentPosition == 0)
                diff = abs(channelLength - shuttleLength - oldPosition);
            if (motorStart && oldSpeed && diff < 10)
            {
                if (!mooveCount)
                    countCrush = millis();
                mooveCount = 1;
            }
            else
            {
                oldPosition = currentPosition;
                mooveCount  = 0;
                if (currentPosition == 0)
                    oldPosition = channelLength - shuttleLength;
            }
            if (mooveCount && millis() - countCrush > 1500)
            {
                motor_Stop();
                status = CMD_STOP;
                setFault(FAULT_MOTOR_STALL);
                STATS_ATOMIC_UPDATE(sramStats->payload.motorStallCount++);
                return;
            }
        }
        else if (counter == 1 || counter == 5)
        {
            digitalWrite(GREEN_LED, HIGH);
            digitalWrite(WHITE_LED, HIGH);
        }
        else if ((counter == 2 || counter == 12) && status != CMD_MANUAL_MODE)
        {
            if (motorStart)
            {
                if (oldSpeed != lastLoggedSpeed)
                {
                    makeLog(LOG_DEBUG, "Speed = %d", oldSpeed);
                    lastLoggedSpeed = oldSpeed;
                }
            }
            else
            {
                makeLog(LOG_DEBUG, "standing...");
            }
        }
        else if (counter == 3 || counter == 7)
        {
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(WHITE_LED, LOW);
            SystemYield();
        }
        else if (counter == 4 && status != CMD_MANUAL_MODE)
        {
            if (lifterCurrent)
            {
                makeLog(LOG_DEBUG, "LCrnt = %d", lifterCurrent);
            }
        }
        else if ((counter == 11 || counter == 19) && status != CMD_MANUAL_MODE)
        {
            send_Cmd();
        }
        else if (counter == 20)
        {
            counter = 0;
        }
        count2 = millis();
    }
    return;
}

// Моргание светодиодом на предупреждение
void blink_Warning()
{
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(ZOOMER, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    count = millis();
    while (millis() - count < 100)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(ZOOMER, LOW);
            digitalWrite(BOARD_LED, LOW);
            return;
        }
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(ZOOMER, LOW);
    digitalWrite(BOARD_LED, LOW);
    count = millis();
    while (millis() - count < 100)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(ZOOMER, LOW);
            digitalWrite(BOARD_LED, LOW);
            return;
        }
    }
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(ZOOMER, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    count = millis();
    while (millis() - count < 100)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(ZOOMER, LOW);
            digitalWrite(BOARD_LED, LOW);
            return;
        }
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(ZOOMER, LOW);
    digitalWrite(BOARD_LED, LOW);
    count = millis();
    while (millis() - count < 100)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(ZOOMER, LOW);
            digitalWrite(BOARD_LED, LOW);
            return;
        }
    }
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(ZOOMER, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    count = millis();
    while (millis() - count < 100)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(ZOOMER, LOW);
            digitalWrite(BOARD_LED, LOW);
            return;
        }
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(ZOOMER, LOW);
    digitalWrite(BOARD_LED, LOW);
    return;
}

// Моргание светодиодом в режиме ошибки
void blink_Error()
{
    static int     countError = millis();
    static uint8_t errCounter = 0;

    if (millis() - countError < 200)
        return;

    if (errCounter == 0 || errCounter == 2 || errCounter == 4 || errCounter == 6 || errCounter == 8)
    {
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        digitalWrite(BOARD_LED, HIGH);
        errCounter++;
    }
    else if (errCounter == 1 || errCounter == 3 || errCounter == 5 || errCounter == 7)
    {
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        digitalWrite(BOARD_LED, LOW);
        errCounter++;
    }
    else if (errCounter == 9)
    {
        digitalWrite(GREEN_LED, LOW);
        errCounter++;
        SystemYield();
        if (!isPhysicallyStationary())
        {
            motor_Stop();
        }
        Can1.read(CAN_RX_msg);
    }
    else if (errCounter == 14)
    {
        debuger++;
        if (debuger == 1)
        {
            makeLog(LOG_ERROR, "Shuttle ERROR! Code: %04X", alertMan.getErrorCode());

            sendTelemetryPacket(&SerialDisplay);
            sendTelemetryPacket(&SerialLora);
        }
        else if (debuger == 10)
        {
            debuger = 0;
        }

        send_Cmd();
        errCounter = 0;
    }
    else
    {
        errCounter++;
    }

    countError = millis();
}

#pragma endregion

#pragma region Рабочие функции шаттла

// Остановка перед паллетом к началу канала (к выгрузке)
void stop_Before_Pallete_F()
{
    makeLog(LOG_DEBUG, "Start stopping before pallete F... at %d", distance[3]);
    moove_Before_Pallete_F();
    if (shouldAbortLoop())
        return;
    if (oldSpeed == 0 && distance[1] > 250)
        motor_Speed(20);
    uint16_t otstup = 70;
    if (lifterUp)
        otstup = 100;
    otstup += chnlOffset;
    if (distance[3] <= 1000)
    {
        count = millis();
        while (distance[3] >= 900 && distance[1] > otstup)
        {
            int spd = (distance[3] - 200) / 20;
            if (spd > distance[1] / 23)
                spd = distance[1] / 23;
            if (spd - oldSpeed > 3)
                spd = oldSpeed + 3;
            if (spd < 5)
                spd = 5;
            motor_Speed(spd);
            SystemYield();
            if (shouldAbortLoop())
            {
                preserveManualStopOnAbort();
                motor_Stop();
                return;
            }
            count = millis();
            set_Position();
            get_Distance();
        }
        int dist  = distance[3];
        int diffP = currentPosition + startDiff;
        makeLog(LOG_DEBUG, "Speed = %d | Position = %d", oldSpeed, currentPosition);
        int     diff = distance[3];
        uint8_t i    = 1;
        uint8_t j    = 1;
        get_Distance();
        while (i < 4)
        { // Фиксируем несколько измерений расстояния  до паллета
            while (millis() - count < 100)
            {
                SystemYield();
                get_Distance();
                if (shouldAbortLoop())
                {
                    preserveManualStopOnAbort();
                    motor_Stop();
                    return;
                }
            }
            count = millis();

            if (distance[3] > 600 && distance[3] < 900)
            {
                makeLog(LOG_DEBUG, "Distance F = %d", distance[3]);
                dist += distance[3];
                i++;
            }
            if (distance[3] > 900)
            {
                j++;
                if (j == 3)
                {
                    dist  = distance[3];
                    diffP = currentPosition + startDiff;
                    diff  = distance[3];
                    i     = 1;
                    j--;
                }
            }
            int spd = (distance[3] - 200) / 20;
            if (oldSpeed < spd && distance[3] < 1000 && distance[1] > 1000)
                motor_Speed(oldSpeed);
            else if (distance[3] >= 1000 && distance[1] > 1000)
                moove_Before_Pallete_F();
            else if (spd > 7 && distance[1] > 1000)
                motor_Speed(spd);
            else if (distance[1] <= 1000 && distance[1] > 100 + chnlOffset)
            {
                spd = distance[1] / 23;
                if (spd < 5)
                    spd = 5;
                else if (spd > oldSpeed)
                    spd = oldSpeed;
                motor_Speed(spd);
            }
            else if (distance[1] <= 100 + chnlOffset)
            {
                motor_Stop();
                makeLog(LOG_INFO, "Stopped at the end of channel...");
                return;
            }
            else
                motor_Speed(7);
            set_Position();
        }
        makeLog(LOG_DEBUG, "Distance F = %d", distance[3]);
        diff = abs(diff - distance[3]) * 4 / 5;
        dist = dist / 4 - interPalleteDistance - 100 - diff - mprOffset;
        if (shuttleLength == 1000 || shuttleLength == 1200)
            dist -= 25;
        dist *= 0.96;
        moove_Distance_F(dist, oldSpeed, 7);
    }
    motor_Stop();
    if (distance[1] > 250)
        makeLog(LOG_DEBUG, "Stopped before pallete F...");
    else
        makeLog(LOG_DEBUG, "Stopped at the end of channel...");
}

// Остановка перед паллетом к началу канала (к выгрузке)
void moove_Before_Pallete_F()
{
    makeLog(LOG_INFO, "Going before pallete F...");
    uint8_t findPallete = 1;
    get_Distance();
    if (motorStart)
        motor_Speed(oldSpeed);
    motor_Start_Forward();
    oldChannelDistanse = distance[1];
    oldPalleteDistanse = distance[3];
    count              = millis();
    while (findPallete)
    { // Двигаемся до обнаружения поддона или конца канала
        SystemYield();
        if (shouldAbortLoop())
        { // Проверка на стоп и ошибки
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        blink_Work();
        get_Distance();
        if (millis() - count > 50)
        {
            count = millis();
            set_Position();
            if (distance[1] >= 1500 && distance[3] >= 1500)
            {
                if (lifterUp && firstPalletePosition)
                {
                    if (currentPosition - firstPalletePosition > 3000)
                        motor_Speed(80);
                    else if ((currentPosition - firstPalletePosition - 600) / 30 > 40)
                        motor_Speed((currentPosition - firstPalletePosition - 600) / 30);
                    else
                        motor_Speed(40);
                }
                else if (lifterUp)
                {
                    motor_Speed(80);
                }
                else
                    motor_Speed(90);
            }
            else if (distance[1] >= distance[3] && distance[3] >= 1000)
            {
                int spd = distance[3] / 25;
                if (spd > oldSpeed)
                    if (oldSpeed > 20)
                        motor_Speed(oldSpeed);
                    else
                        motor_Speed(20);
                else
                    motor_Speed(spd);
            }
            else if (distance[1] >= distance[3] && distance[3] < 1000)
            {
                findPallete = 0;
            }
            else if (distance[1] > 150 + chnlOffset && distance[1] <= 1500)
            {
                int spd = distance[1] / 25;
                if (lifterUp && spd > 40)
                    if (spd > oldSpeed && oldSpeed)
                        spd = oldSpeed;
                    else
                        spd = 40;
                if (spd < 6)
                    spd = 6;
                if (spd > oldSpeed)
                    if (oldSpeed > 35)
                        motor_Speed(oldSpeed);
                    else
                        motor_Speed(oldSpeed + 5);
                else
                    motor_Speed(spd);
                if (distance[3] < 1000)
                    findPallete = 0;
            }
            else if (distance[1] > 90 + chnlOffset && distance[1] <= 150 + chnlOffset)
            {
                motor_Speed(5);
            }
            else if (distance[1] <= 90 + chnlOffset)
            {
                findPallete     = 0;
                currentPosition = 60;
            }
            else if (oldSpeed)
            {
                motor_Speed(oldSpeed);
            }
            else
                motor_Speed(5);
            if (shouldAbortLoop())
                return;
        }
    }
    if (distance[1] > 150)
        makeLog(LOG_DEBUG, "Before at %d Pos = %d", distance[3], currentPosition);
    else
    {
        makeLog(LOG_DEBUG, "End of channel F... at %d", distance[1]);
        currentPosition = distance[1] - 30;
    }
}

// Остановка перед паллетом к концу канала (загрузка)
void stop_Before_Pallete_R()
{
    makeLog(LOG_DEBUG, "Start stopping before pallete R... at %d", distance[2]);
    moove_Before_Pallete_R();
    if (shouldAbortLoop())
        return;
    if (oldSpeed == 0 && distance[0] > 250)
        motor_Speed(20);
    uint16_t otstup = 70;
    if (lifterUp)
        otstup = 100;
    otstup += chnlOffset;
    if (distance[2] <= 1000)
    {
        count = millis();
        while (distance[2] >= 900 && distance[0] > otstup)
        {
            int spd = (distance[2] - 200) / 20;
            if (spd > distance[0] / 23)
                spd = distance[0] / 23;
            if (spd - oldSpeed > 3)
                spd = oldSpeed + 3;
            if (spd < 5)
                spd = 5;
            motor_Speed(spd);
            while (millis() - count < 50)
            {
                SystemYield();
                blink_Work();
                get_Distance();
                if (shouldAbortLoop())
                {
                    preserveManualStopOnAbort();
                    motor_Stop();
                    return;
                }
            }
            count = millis();
            set_Position();
        }
        int dist = distance[2];
        makeLog(LOG_DEBUG, "Speed = %d position = %d", oldSpeed, currentPosition);
        int     diff  = distance[2];
        int     diffP = currentPosition + startDiff;
        uint8_t i     = 1;
        uint8_t j     = 1;
        while (i < 4)
        {
            SystemYield();
            while (millis() - count < 100)
            {
                SystemYield();
                blink_Work();
                get_Distance();
                if (shouldAbortLoop())
                {
                    preserveManualStopOnAbort();
                    motor_Stop();
                    return;
                }
            }
            count = millis();
            if (distance[2] > 600 && distance[2] < 900)
            {
                makeLog(LOG_DEBUG, "Distance R = %d", distance[2]);
                dist += distance[2];
                i++;
            }
            if (distance[2] > 900)
            {
                j++;
                if (j == 3)
                {
                    dist  = distance[2];
                    diffP = currentPosition + +startDiff;
                    diff  = distance[2];
                    i     = 1;
                    j--;
                }
            }
            int spd = (distance[2] - 200) / 20;
            if (oldSpeed < spd && distance[2] < 1000 && distance[0] > 1000)
                motor_Speed(oldSpeed);
            else if (distance[2] >= 1000 && distance[0] > 1000)
                moove_Before_Pallete_R();
            else if (spd > 7 && distance[0] > 1000)
                motor_Speed(spd);
            else if (distance[0] <= 1000 && distance[0] > 100 + chnlOffset)
            {
                spd = distance[0] / 23;
                if (spd < 5)
                    spd = 5;
                else if (spd > oldSpeed && oldSpeed > 5)
                    spd = oldSpeed;
                motor_Speed(spd);
            }
            else if (distance[0] <= 100 + chnlOffset)
            {
                motor_Stop();
                makeLog(LOG_INFO, "Stopped at the end of channel...");
                return;
            }
            else
                motor_Speed(7);
            set_Position();
        }
        makeLog(LOG_DEBUG, "Distance R = %d", distance[2]);
        diff = abs(diff - distance[2]) * 4 / 5;
        dist = dist / 4 + diffPallete - interPalleteDistance - 100 - diff - mprOffset;
        if (shuttleLength == 1000 || shuttleLength == 1200)
            dist -= 25;
        dist *= 0.96;
        moove_Distance_R(dist, oldSpeed, 7);
    }
    motor_Stop();
    if (distance[0] > 250)
        makeLog(LOG_DEBUG, "Stopped before pallete R...");
    else
        makeLog(LOG_DEBUG, "Stopped at the end of channel...");
}

// Остановка перед паллетом к концу канала (загрузка)
void moove_Before_Pallete_R()
{
    makeLog(LOG_INFO, "Going before pallete R...");
    uint8_t findPallete = 1;
    if (motorStart)
        motor_Speed(oldSpeed);
    get_Distance();
    oldChannelDistanse = distance[0];
    oldPalleteDistanse = distance[2];
    if (motorStart)
        motor_Speed(oldSpeed);
    motor_Start_Reverse();
    count = millis();
    while (findPallete)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        blink_Work();
        get_Distance();
        if (millis() - count > 50)
        {
            count = millis();
            set_Position();
            if (distance[0] >= 1500 && distance[2] >= 1500)
            {
                if (lifterUp && lastPallete)
                {
                    int diff = lastPalletePosition - currentPosition;
                    if (diff > 3000)
                        motor_Speed(80);
                    else if ((diff - 600) / 30 > oldSpeed)
                    {
                        if (oldSpeed < 50)
                            motor_Speed(50);
                        else
                            motor_Speed(oldSpeed);
                    }
                    else if ((diff - 600) / 30 > 40)
                        motor_Speed((diff - 600) / 30);
                    else
                        motor_Speed(40);
                }
                else if (lifterUp && channelLength > 3000)
                {
                    int diff = channelLength - shuttleLength - currentPosition;
                    if (diff > 2000)
                        motor_Speed(70);
                    else if (diff < 100)
                    {
                        if (maxSpeed > 86)
                            motor_Speed(60);
                        else
                            motor_Speed(66);
                    }
                    else if (diff / 25 > oldSpeed)
                    {
                        if (oldSpeed < 20)
                            motor_Speed(20);
                        else
                            motor_Speed(oldSpeed);
                    }
                    else if (diff / 25 > 50)
                        motor_Speed(diff / 25);
                    else
                        motor_Speed(50);
                }
                else if (lifterUp)
                {
                    if (maxSpeed > 86)
                        motor_Speed(60);
                    else
                        motor_Speed(66);
                }
                else if (
                    channelLength - shuttleLength - currentPosition > 1800 ||
                    channelLength - shuttleLength - currentPosition < 100)
                {
                    motor_Speed(90);
                }
                else if (
                    channelLength - shuttleLength - currentPosition >= 100 &&
                    channelLength - shuttleLength - currentPosition <= 1800)
                {
                    int spd = (channelLength - shuttleLength - currentPosition) / 20;
                    if (!endOfChannel && spd < 50)
                        spd = 50;
                    if (endOfChannel && spd > oldSpeed && oldSpeed)
                    {
                        motor_Speed(oldSpeed);
                    }
                    else
                        motor_Speed(spd);
                }
                else
                    motor_Speed(5);
            }
            else if (distance[0] >= distance[2] && distance[2] >= 1000)
            {
                int spd = distance[2] / 25;
                if (spd > oldSpeed)
                    if (oldSpeed > 20)
                        motor_Speed(oldSpeed);
                    else
                        motor_Speed(20);
                else
                    motor_Speed(spd);
            }
            else if (distance[0] >= distance[2] && distance[2] < 1000)
            {
                findPallete = 0;
            }
            else if (distance[0] > 150 + chnlOffset && distance[0] <= 1500)
            {
                int spd = distance[0] / 25;
                if (lifterUp && spd > 40)
                    spd = 40;
                if (spd < 6)
                    spd = 6;
                if (spd > oldSpeed)
                    if (oldSpeed > 35)
                        motor_Speed(oldSpeed);
                    else
                        motor_Speed(oldSpeed + 5);
                else
                    motor_Speed(spd);
                if (distance[2] < 1000)
                    findPallete = 0;
            }
            else if (distance[0] > 90 + chnlOffset && distance[0] <= 150 + chnlOffset)
                motor_Speed(5);
            else if (distance[0] <= 90 + chnlOffset)
            {
                findPallete = 0;
            }
            else
                motor_Speed(oldSpeed);
            if (shouldAbortLoop())
                return;
        }
    }
    if (distance[0] > 150)
        makeLog(LOG_DEBUG, "Before at %d Pos = %d", distance[2], currentPosition);
    else
    {
        makeLog(LOG_DEBUG, "End of channel R... at %d", distance[0]);
        channelLength = currentPosition + shuttleLength + distance[0] - 30;
        endOfChannel  = 1;
    }
}

// Движение вперед на заданное расстояние
void moove_Distance_F(int dist)
{
    moove_Distance_F(dist, 100, 10);
    motor_Stop();
}

// Движение вперед на заданное расстояние
void moove_Distance_F(int dist, int maxSpeed, int minSpeed)
{
    get_Distance();
    if (distance[1] < 70)
    {
        makeLog(LOG_WARN, "End of channel F, can't moove... ");
        return;
    }
    makeLog(LOG_INFO, "Moove F distance = %d Pos = %d", dist, currentPosition);
    uint16_t startAngle = 0;
    if (!readAs5600AngleForMotion(&startAngle))
    {
        setFault(FAULT_MOVE_TIMEOUT);
        return;
    }
    uint16_t currentAngle = startAngle;
    uint8_t  moove        = 1;
    count                 = millis();
    if (maxSpeed < 2)
        maxSpeed = 2;
    if (minSpeed > maxSpeed)
        minSpeed = maxSpeed - 1;
    if (dist > 500 && dist <= 1000 && maxSpeed > 50)
        maxSpeed = 50;
    if (dist > 300 && dist <= 500 && maxSpeed > 30)
        maxSpeed = 30;
    if (dist <= 300 && maxSpeed > 20)
        maxSpeed = 20;
    if (dist > 500)
        dist -= 50;
    else if (dist > 50)
        dist -= 10;
    motor_Start_Forward();
    get_Distance();
    int cnt = millis();
    count   = millis();
    while (moove)
    { // Двигаемся до конца заданного расстояния
        SystemYield();
        if (shouldAbortLoop())
        { // Проверка на стоп и ошибки
            preserveManualStopOnAbort();
            motor_Stop();
            return;
        }
        blink_Work();
        get_Distance();
        if (millis() - count > 50)
        {
            set_Position();
            if (!readAs5600AngleForMotion(&currentAngle))
            {
                setFault(FAULT_MOVE_TIMEOUT);
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            int diff = 0;
            if (!inverse)
            {
                if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                    {
                        diff = (startAngle - currentAngle) * calibrateEncoder_F[s] / 512;
                    }
                    else
                    {
                        diff = ((512 * s - currentAngle) * calibrateEncoder_F[f] +
                                (startAngle - 512 * s) * calibrateEncoder_F[s]) /
                               512;
                    }
                }
                else if (currentAngle - startAngle > 2000)
                {
                    diff = (startAngle * calibrateEncoder_F[0] + (4096 - currentAngle) * calibrateEncoder_F[7]) / 512;
                }
                else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                        diff = (currentAngle - startAngle) * calibrateEncoder_F[s] / 512;
                    else
                        diff = ((currentAngle - 512 * f) * calibrateEncoder_F[f] +
                                (512 * f - startAngle) * calibrateEncoder_F[s]) /
                               512;
                }
                else if (startAngle - currentAngle > 2000)
                {
                    diff = (currentAngle * calibrateEncoder_F[0] + (4096 - startAngle) * calibrateEncoder_F[7]) / 512;
                }
            }
            else
            {
                if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                    {
                        diff = (startAngle - currentAngle) * (int)calibrateEncoder_R[s] / 512;
                    }
                    else
                    {
                        diff = ((512 * s - currentAngle) * (int)calibrateEncoder_R[f] +
                                (startAngle - 512 * s) * (int)calibrateEncoder_R[s]) /
                               512;
                    }
                }
                else if (currentAngle - startAngle > 2000)
                {
                    diff = (startAngle * calibrateEncoder_R[0] + (4096 - currentAngle) * calibrateEncoder_R[7]) / 512;
                }
                else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                        diff = (currentAngle - startAngle) * (int)calibrateEncoder_R[s] / 512;
                    else
                        diff = ((currentAngle - 512 * f) * (int)calibrateEncoder_R[f] +
                                (512 * f - startAngle) * (int)calibrateEncoder_R[s]) /
                               512;
                }
                else if (startAngle - currentAngle > 2000)
                {
                    diff = (currentAngle * calibrateEncoder_R[0] + (4096 - startAngle) * calibrateEncoder_R[7]) / 512;
                }
            }
            if (diff < 0)
                diff = 0;
            if (diff != 0)
            {
                dist -= diff;
                startAngle = currentAngle;
                cnt        = millis();
            }

            if (distance[1] >= maxSpeed * 15 && dist >= maxSpeed * 15)
            {
                motor_Speed(maxSpeed);
            }
            else if (distance[1] <= 90 + chnlOffset)
            {
                motor_Stop();
                moove           = 0;
                currentPosition = 0;
            }
            else if (dist >= distance[1] && distance[1] < maxSpeed * 15)
            {
                motor_Speed((int)(distance[1] / 15));
            }
            else if (dist < maxSpeed * 15 && dist > minSpeed * 15)
            {
                motor_Speed((int)(dist / 15));
            }
            else if (dist > 0 && dist <= minSpeed * 15)
            {
                if (oldSpeed > minSpeed)
                {
                    motor_Speed(minSpeed);
                }
                else if (oldSpeed)
                {
                    motor_Speed(oldSpeed);
                }
                else
                    motor_Speed(minSpeed);
            }
            else if (dist <= 0)
            {
                moove = 0;
            }
            if (lifterUp && millis() - cnt > 3000 && dist < 30)
            {
                motor_Stop();
                return;
            }
            else if (millis() - cnt > 5000)
            {
                setFault(FAULT_MOVE_TIMEOUT);
                motor_Stop();
                lifter_Down();
                status = CMD_STOP;
                return;
            }
            count = millis();
        }
    }
    makeLog(LOG_DEBUG, "End mooving, position = %d", currentPosition);
}

// Движение назад на заданное расстояние
void moove_Distance_R(int dist)
{
    moove_Distance_R(dist, 100, 10);
    motor_Stop();
}

// Движение назад на заданное расстояние
void moove_Distance_R(int dist, int maxSpeed, int minSpeed)
{
    get_Distance();
    if (distance[0] < 70)
    {
        makeLog(LOG_WARN, "End of channel R, can't moove... ");
        return;
    }
    makeLog(LOG_INFO, "Moove R distance = %d Pos = %d", dist, currentPosition);
    uint16_t startAngle = 0;
    if (!readAs5600AngleForMotion(&startAngle))
    {
        setFault(FAULT_MOVE_TIMEOUT);
        return;
    }
    uint16_t currentAngle = startAngle;
    uint8_t  moove        = 1;
    count                 = millis();
    if (dist > 500 && dist <= 1000 && maxSpeed > 50)
        maxSpeed = 50;
    if (dist > 300 && dist <= 500 && maxSpeed > 30)
        maxSpeed = 30;
    if (dist <= 300 && maxSpeed > 20)
        maxSpeed = 20;
    if (dist > 500)
        dist -= 50;
    else if (dist > 50)
        dist -= 10;
    if (maxSpeed < 5)
        maxSpeed = 5;
    if (minSpeed > maxSpeed)
        minSpeed = maxSpeed - 1;
    motor_Start_Reverse();
    get_Distance();
    int cnt = millis();
    count   = millis();
    while (moove)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            preserveManualStopOnAbort();
            motor_Stop();
            return;
        }
        blink_Work();
        get_Distance();
        if (millis() - count > 50)
        {
            set_Position();
            if (!readAs5600AngleForMotion(&currentAngle))
            {
                setFault(FAULT_MOVE_TIMEOUT);
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            int diff = 0;
            if (inverse)
            {
                if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                    {
                        diff = (startAngle - currentAngle) * (int)calibrateEncoder_F[s] / 512;
                    }
                    else
                    {
                        diff = ((512 * s - currentAngle) * (int)calibrateEncoder_F[f] +
                                (startAngle - 512 * s) * (int)calibrateEncoder_F[s]) /
                               512;
                    }
                }
                else if (currentAngle - startAngle > 2000)
                {
                    diff =
                        (startAngle * (int)calibrateEncoder_F[0] + (4096 - currentAngle) * (int)calibrateEncoder_F[7]) / 512;
                }
                else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                        diff = (currentAngle - startAngle) * (int)calibrateEncoder_F[s] / 512;
                    else
                        diff = ((currentAngle - 512 * f) * (int)calibrateEncoder_F[f] +
                                (512 * f - startAngle) * (int)calibrateEncoder_F[s]) /
                               512;
                }
                else if (startAngle - currentAngle > 2000)
                {
                    diff = (currentAngle * calibrateEncoder_F[0] + (4096 - startAngle) * calibrateEncoder_F[7]) / 512;
                }
            }
            else
            {
                if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                    {
                        diff = (startAngle - currentAngle) * (int)calibrateEncoder_R[s] / 512;
                    }
                    else
                    {
                        diff = ((512 * s - currentAngle) * (int)calibrateEncoder_R[f] +
                                (startAngle - 512 * s) * (int)calibrateEncoder_R[s]) /
                               512;
                    }
                }
                else if (currentAngle - startAngle > 2000)
                {
                    diff =
                        (startAngle * (int)calibrateEncoder_R[0] + (4096 - currentAngle) * (int)calibrateEncoder_R[7]) / 512;
                }
                else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
                {
                    uint8_t s = startAngle / 512;
                    uint8_t f = currentAngle / 512;
                    if (s == f)
                        diff = (currentAngle - startAngle) * (int)calibrateEncoder_R[s] / 512;
                    else
                        diff = ((currentAngle - 512 * f) * (int)calibrateEncoder_R[f] +
                                (512 * f - startAngle) * (int)calibrateEncoder_R[s]) /
                               512;
                }
                else if (startAngle - currentAngle > 2000)
                {
                    diff = (currentAngle * calibrateEncoder_R[0] + (4096 - startAngle) * calibrateEncoder_R[7]) / 512;
                }
            }

            if (diff > 0)
            {
                dist -= diff;
                startAngle = currentAngle;
                cnt        = millis();
            }

            if (distance[0] >= maxSpeed * 15 && dist >= maxSpeed * 15)
            {
                motor_Speed(maxSpeed);
            }
            else if (distance[0] <= 90 + chnlOffset)
            {
                motor_Stop();
                moove         = 0;
                channelLength = currentPosition + shuttleLength;
            }
            else if (dist >= distance[0] && distance[0] < maxSpeed * 15)
            {
                motor_Speed((int)(distance[0] / 15));
            }
            else if (dist < maxSpeed * 15 && dist > minSpeed * 15)
            {
                motor_Speed((int)(dist / 15));
            }
            else if (dist > 0 && dist <= minSpeed * 15)
            {
                if (oldSpeed > minSpeed)
                {
                    motor_Speed(minSpeed);
                }
                else if (oldSpeed)
                {
                    motor_Speed(oldSpeed);
                }
                else
                    motor_Speed(minSpeed);
            }
            else if (dist <= 0)
                moove = 0;
            if (lifterUp && millis() - cnt > 3000 && dist < 30)
            {
                motor_Stop();
                return;
            }
            else if (millis() - cnt > 5000)
            {
                setFault(FAULT_MOVE_TIMEOUT);
                motor_Stop();
                lifter_Down();
                status = CMD_STOP;
                return;
            }
            count = millis();
        }
    }
    makeLog(LOG_DEBUG, "End mooving, position = %d", currentPosition);
}

// Движение вперед до конца канала
void moove_Forward()
{
    makeLog(LOG_INFO, "Start moove forward... Status = %d", status);
    motor_Start_Forward();
    detect_Pallete();
    if (lifterUp)
    {
        stop_Before_Pallete_F();
        if (status != CMD_STOP)
            lifter_Down();
        return;
    }
    get_Distance();
    oldChannelDistanse = distance[1];
    uint8_t moove      = 1;
    count              = millis();
    while (moove)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            preserveManualStopOnAbort();
            motor_Stop();
            return;
        }
        blink_Work();
        get_Distance();
        if (millis() - count > 50)
        {
            set_Position();
            detect_Pallete();
            if (currentPosition < 0)
                currentPosition = 0;
            if (distance[1] >= 1500)
            {
                speed = 100;
            }
            else if (distance[1] > 90 + chnlOffset && distance[1] < 1500)
            {
                speed = distance[1] / 20;
                if (oldSpeed > 5 && speed > oldSpeed)
                    speed = oldSpeed;
                if (speed < 5)
                    speed = 5;
                if (speed > 80)
                    speed = 80;
                if (oldSpeed > 50 && speed > oldSpeed)
                    speed = oldSpeed;
            }
            else if (distance[1] <= 90 + chnlOffset)
            {
                if ((status == CMD_LOAD || status == CMD_LONG_LOAD) && distance[1] > 80)
                    speed = 5;
                else
                {
                    speed           = 0;
                    moove           = 0;
                    currentPosition = 60;
                    makeLog(LOG_INFO, "End of channel, stop moove forward...");
                }
            }
            motor_Speed(speed);
            if (shouldAbortLoop())
                return;
            count = millis();
        }
    }
    motor_Stop();
}

// Движение назад до конца канала
void moove_Reverse()
{
    makeLog(LOG_INFO, "Start moove reverse... Status = %d", status);
    motor_Start_Reverse();
    detect_Pallete();
    if (lifterUp)
    {
        stop_Before_Pallete_R();
        if (status != CMD_STOP)
            lifter_Down();
        return;
    }
    get_Distance();
    oldChannelDistanse = distance[0];
    uint8_t moove      = 1;
    count              = millis();
    while (moove)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            preserveManualStopOnAbort();
            motor_Stop();
            return;
        }
        blink_Work();
        get_Distance();
        if (millis() - count > timingBudget + 5)
        {
            set_Position();
            detect_Pallete();
            if (distance[0] >= 1500)
                speed = 100;
            else if (distance[0] > 90 + chnlOffset && distance[0] < 1500)
            {
                speed = distance[0] / 20;
                if (oldSpeed > 5 && speed > oldSpeed)
                    speed = oldSpeed;
                if (speed < 5)
                    speed = 5;
            }
            else if (distance[0] <= 90 + chnlOffset)
            {
                speed         = 0;
                startAngle    = angle;
                turnCount     = 0;
                moove         = 0;
                channelLength = currentPosition + shuttleLength + distance[0] - 30;
                endOfChannel  = 1;
                makeLog(LOG_INFO, "End of channel, stop moove reverse...");
            }
            motor_Speed(speed);
            if (shouldAbortLoop())
                return;
            count = millis();
        }
    }
    motor_Stop();
}

// Выгрузка паллеты
void unload_Pallete()
{
    makeLog(LOG_INFO, "Start unloading pallete...");
    if (fifoLifo)
        fifoLifo_Inverse();
    uint8_t moove      = 0;
    uint8_t frontBoard = 0;
    int     currentPalletePosition;
    int     palleteLenght = 0;
    startDiff             = 0;
    lifter_Down();
    get_Distance();
    if (distance[0] < 90 + chnlOffset)
        startDiff = 20;

    // Подъезжаем к паллету
    if (distance[2] > 750)
    { // Если свободен канал вперед, едем к паллету
        moove_Before_Pallete_R();
        frontBoard = 0;
    }
    if (shouldAbortLoop())
    { // Проверка на ошибки и стоп
        oldSpeed = 0;
        if (fifoLifo)
            fifoLifo_Inverse();
        return;
    }
    moove = 1;
    motor_Start_Reverse();
    if (oldSpeed > 20 || distance[0] < 250 + chnlOffset)
        motor_Speed(oldSpeed);
    else
        motor_Speed(28);
    int cnt = millis();
    while (moove)
    { // Едем до определения поддона
        SystemYield();
        if (shouldAbortLoop())
        { // Проверка на ошибки и стоп
            motor_Stop();
            status = CMD_STOP;
            if (fifoLifo)
                fifoLifo_Inverse();
            return;
        }
        blink_Work();
        get_Distance();
        detect_Pallete();
        if (detectPalleteR1 && detectPalleteR2 && !frontBoard)
        { // Увидели переднюю доску и едем фиксированное расстояние
            set_Position();
            currentPalletePosition = currentPosition;
            int dst                = 600;
            if (shuttleLength == 1200)
                dst = 670;
            if (channelLength - currentPosition - shuttleLength < 1500 && shuttleLength == 800)
                dst = 500;
            moove_Distance_R(dst, oldSpeed, 10);
            frontBoard = 1;
        }
        else if (detectPalleteR1 && detectPalleteR2 && frontBoard)
        { // Определяем что доехали до последней доски поддона
            uint8_t maxbb = 2 + (150 - maxSpeed) / 10;
            if (distance[0] < 300)
                maxbb += 3;
            if (maxbb >= 3 && shuttleLength == 1200)
            {
                maxbb -= 3;
            }
            else if (maxbb < 3 && shuttleLength == 1200)
            {
                maxbb = 0;
            }
            for (uint8_t i = 0; i < maxbb; i++)
            { // Задержка для доезда под доску
                count = millis();
                while (millis() - count < 100)
                {
                    SystemYield();
                    blink_Work();
                    if (shouldAbortLoop())
                    {
                        preserveManualStopOnAbort();
                        motor_Stop();
                        return;
                    }
                }
                set_Position();
                palleteLenght = abs(currentPalletePosition - currentPosition);
                detect_Pallete();
                if (frontBoard && detectPalleteF1 && detectPalleteF2)
                {
                    frontBoard = 0;
                    i          = maxbb - 1;
                }
                motor_Speed(oldSpeed);
            }
            moove             = 0;
            uint16_t pltMaxLn = shuttleLength - 20;
            // if (shuttleLength == 800) pltMaxLn -= 20;
            // else if (shuttleLength == 1000) pltMaxLn -= 100;
            // else pltMaxLn -= 150;
            if (frontBoard && palleteLenght >= pltMaxLn)
            { // Если шаттл видит видит последнюю доску не увидев заднюю проехав больше своей длины то ой
                motor_Stop();
                makeLog(LOG_ERROR, "Pallete error in BB...");
                blink_Warning();
                setWarning(WARN_PALLET_SIZE_ERROR, 5000);
                moove_Forward();
                status = CMD_STOP;
                return;
            }
            motor_Stop();
        }
        if (moove && millis() - count > 50)
        { // Пока не нашли заднюю доску едем
            set_Position();
            int spd = distance[0] / 23;
            if (spd > oldSpeed)
                spd = oldSpeed;
            if (spd < 5)
                spd = 5;
            motor_Speed(spd);
            if ((millis() - cnt > 2000000 / maxSpeed || distance[0] < 80))
            {
                makeLog(LOG_ERROR, "Pallete error...");
                motor_Stop();
                blink_Warning();
                setWarning(WARN_OBSTACLE_AHEAD, 5000);
                if (fifoLifo)
                    fifoLifo_Inverse();
                return;
            }
            count = millis();
        }
    }

    set_Position();
    makeLog(LOG_DEBUG, "Pallete lenght = %d", palleteLenght);
    if (shouldAbortLoop())
    {
        if (fifoLifo)
            fifoLifo_Inverse();
        return;
    }
    // Поднимаем паллет
    get_Distance();
    if (distance[3] < 900)
    { // Проверка что поддон есть куда везти
        lifter_Down();
        blink_Warning();
        setWarning(WARN_PALLET_NOT_FOUND, 5000);
        moove_Forward();
        status = CMD_STOP;
        return;
    }
    lifter_Up();
    int pstn = 0;
    if (distance[2] < 600)
        pstn = currentPosition;
    detect_Pallete();
    if (!detectPalleteF1 || !detectPalleteF2)
    { // Перехват если требуется
        int dist = 100;
        if (shuttleLength == 1000)
            dist = 250;
        else if (shuttleLength == 1200)
            dist = 450;
        moove_Distance_F(dist, 12, 10);
        motor_Stop();
        lifter_Down();
        moove = 1;
        motor_Start_Reverse();
        while (moove)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status = CMD_STOP;
                if (fifoLifo)
                    fifoLifo_Inverse();
                return;
            }
            blink_Work();
            get_Distance();
            if (millis() - count > 50)
            {
                if (distance[0] < 90 + chnlOffset)
                {
                    motor_Stop();
                    channelLength = currentPosition + shuttleLength + distance[0] - 30;
                    status        = CMD_STOP;
                    moove         = 0;
                    endOfChannel  = 1;
                }
                else
                {
                    uint8_t detect = 1;
                    detect_Pallete();
                    if (detectPalleteF1 && detectPalleteF2)
                    {
                        moove = 0;
                        motor_Stop();
                        detect = 0;
                    }
                    delay(5);
                    if (detectPalleteF1 && detectPalleteF2)
                    {
                        moove = 0;
                        motor_Stop();
                        detect = 0;
                    }
                    else if (detect)
                    {
                        set_Position();
                        motor_Speed(10);
                    }
                }
                count = millis();
            }
        }
        motor_Stop();
        lifter_Up();
    }
    if (status == CMD_UNLOAD || status == CMD_LONG_UNLOAD || status == CMD_LONG_UNLOAD_QTY)
    { // Везем в начало канала
        moove_Before_Pallete_F();
        motor_Stop();
        if (distance[1] > 150)
        {
            while (distance[3] < 800)
            {
                SystemYield();
                if (shouldAbortLoop())
                {
                    motor_Stop();
                    status = CMD_STOP;
                    if (fifoLifo)
                        fifoLifo_Inverse();
                    return;
                }
                blink_Work();
                get_Distance();
            }
            while (millis() - count < (uint32_t)waitTime)
            {
                SystemYield();
                blink_Work();
                get_Distance();
                if (shouldAbortLoop())
                {
                    motor_Stop();
                    status = CMD_STOP;
                    if (fifoLifo)
                        fifoLifo_Inverse();
                    return;
                }
            }
            motor_Start_Forward();
            motor_Speed(20);
            while (distance[1] > 90 + chnlOffset)
            {
                SystemYield();
                if (shouldAbortLoop())
                {
                    motor_Stop();
                    status = CMD_STOP;
                    if (fifoLifo)
                        fifoLifo_Inverse();
                    return;
                }
                blink_Work();
                get_Distance();
                if (millis() - count > 50)
                {
                    int spd = 0;
                    if (distance[1] > 400 + chnlOffset)
                        spd = 20;
                    else if (distance[1] <= 400 + chnlOffset && distance[1] > 120 + chnlOffset)
                        spd = distance[1] / 20;
                    else
                        spd = 6;
                    if (distance[3] < 600 && distance[1] > distance[3])
                    {
                        spd = 0;
                        motor_Stop();
                    }
                    else
                    {
                        motor_Start_Forward();
                        motor_Speed(spd);
                    }
                    set_Position();
                    count = millis();
                }
            }
            currentPosition = distance[1] - 30;
            motor_Stop();
        }
    }
    else
    {
        stop_Before_Pallete_F();
    }
    if (shouldAbortLoop())
    { // Проверка на стоп и ошибки
        if (fifoLifo)
            fifoLifo_Inverse();
        return;
    }
    // Опускаем паллет
    if (!longWork && lifterUp)
        lifter_Down();
    if (palleteLenght < 850 && pstn)
        lastPalletePosition = pstn + 800 + interPalleteDistance;
    else if (pstn)
        lastPalletePosition = pstn + 1000 + interPalleteDistance;
    if (pstn)
        lastPallete = 1;
    else
        lastPallete = 0;
    if (lastPallete)
    {
        makeLog(LOG_DEBUG, "Last pallete position after unload = %d", lastPalletePosition);
    }
    STATS_ATOMIC_UPDATE(sramStats->payload.unloadCounter++);
    if (fifoLifo)
        fifoLifo_Inverse();
}

// Загрузка паллеты
void load_Pallete()
{
    makeLog(LOG_INFO, "Start loading pallete...");
    uint8_t moove      = 1;
    uint8_t frontBoard = 1;
    int     currentPalletePosition;
    int     palleteLenght = 0;
    startDiff             = 0;
    lifter_Down();
    if (lastPalletePosition && lastPalletePosition < shuttleLength * 2)
    { // Проверка что канал не забит
        blink_Warning();
        setWarning(WARN_CHANNEL_FULL, 5000);
        status = CMD_STOP;
        return;
    }
    get_Distance();
    if (distance[1] < 90 + chnlOffset)
        startDiff = 20;
    detect_Pallete();
    if (!((detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2) && distance[1] < 450 + chnlOffset &&
          distance[3] > 400))
    { // Проверка шаттла на нахождение в начале канала, сработка паллетного датчика (паллет сверху) и свободное место впереди
        frontBoard = 0;
        // Подъезжаем к паллету
        if (distance[3] > 750)
        { // Если есть куда ехать назад (канал свободен назад), едем к паллету
            moove_Before_Pallete_F();
        }
        if (shouldAbortLoop())
        {
            oldSpeed = 0;
            return;
        }
        // Двигаемся под паллет
        if (distance[1] > 150)
        { // Стартуем если не в конце канала
            moove = 1;
            motor_Start_Forward();
            if (oldSpeed > 20)
                motor_Speed(oldSpeed);
            else
                motor_Speed(20);
        }
        count   = millis();
        int cnt = millis();
        while (moove)
        { // Едем до определения поддона
            SystemYield();
            if (shouldAbortLoop())
            { // Проверка на стоп и ошибки
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            blink_Work();
            get_Distance();
            detect_Pallete();
            if (detectPalleteF1 && detectPalleteF2 && !frontBoard)
            { // Увидели переднюю доску и едем фиксированное расстояние
                set_Position();
                currentPalletePosition = currentPosition;
                int dst                = 600;
                if (shuttleLength == 1200)
                    dst = 670;

                // if (channelLength - currentPosition - shuttleLength < 1500 && shuttleLength == 800) dst = 500;
                moove_Distance_F(dst, oldSpeed, 10);
                frontBoard = 1;
            }
            else if (detectPalleteF1 && detectPalleteF2 && frontBoard)
            { // Определяем что доехали до последней доски короткого поддона
                uint8_t maxbb = 3 + (150 - maxSpeed) / 10;
                if (distance[1] < 300)
                    maxbb += 3;
                if (maxbb >= 3 && shuttleLength == 1200)
                {
                    maxbb -= 3;
                }
                else if (maxbb < 3 && shuttleLength == 1200)
                {
                    maxbb = 0;
                }
                for (uint8_t i = 0; i < maxbb; i++)
                { // Задержка для доезда под доску
                    count = millis();
                    while (millis() - count < 100)
                    {
                        SystemYield();
                        blink_Work();
                        if (shouldAbortLoop())
                        {
                            preserveManualStopOnAbort();
                            motor_Stop();
                            return;
                        }
                    }
                    detect_Pallete();
                    set_Position();
                    palleteLenght = abs(currentPalletePosition - currentPosition);
                    if (frontBoard && detectPalleteR1 && detectPalleteR2)
                    {
                        frontBoard = 0;
                        i          = maxbb - 1;
                    }
                    motor_Speed(oldSpeed);
                }
                moove = 0;
                motor_Stop();
                uint16_t pltMaxLn = shuttleLength - 20;
                // if (shuttleLength == 800) pltMaxLn -= 20;
                // else if (shuttleLength == 1000) pltMaxLn -= 100;
                // else pltMaxLn -= 150;
                if (frontBoard && palleteLenght >= pltMaxLn)
                { // Если шаттл видит последнюю доску проехав свою длину и не увидел заднюю то ой
                    motor_Stop();
                    makeLog(LOG_ERROR, "Pallete error in BB... PLenght = %d", palleteLenght);
                    blink_Warning();
                    setWarning(WARN_PALLET_SIZE_ERROR, 5000);
                    moove_Forward();
                    status = CMD_STOP;
                    return;
                }
            }
            if (moove && millis() - count > 50)
            {
                set_Position();
                int spd = distance[1] / 23;
                if (spd > oldSpeed)
                    spd = oldSpeed;
                if (spd < 5)
                    spd = 5;
                motor_Speed(spd);
                detect_Pallete();
                if (distance[1] < 80 && (detectPalleteR1 || detectPalleteR2) && (detectPalleteF1 || detectPalleteF2))
                {
                    startDiff = 20;
                    motor_Stop();
                    moove = 0;
                }
                if ((millis() - cnt > 2000000 / maxSpeed || distance[1] < 80))
                {
                    makeLog(LOG_ERROR, "Pallete error...");
                    motor_Stop();
                    blink_Warning();
                    setWarning(WARN_OBSTACLE_AHEAD, 5000);
                    return;
                }
                count = millis();
            }
        }
    }
    else if (
        (detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2) && distance[1] < 300 + chnlOffset &&
        distance[2] <= interPalleteDistance + 600)
    {
        motor_Stop();
        blink_Warning();
        setWarning(WARN_OBSTACLE_AHEAD, 5000);
        status = CMD_STOP;
        return;
    }
    else if ((detectPalleteR1 || detectPalleteR2) && !(detectPalleteF1 || detectPalleteF2))
    {
        uint8_t mv = 1;
        while (mv)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                preserveManualStopOnAbort();
                motor_Stop();
                return;
            }
            moove_Distance_R(10, 10, 10);
            get_Distance();
            detect_Pallete();
            if (distance[1] < 70 || detectPalleteR1 || detectPalleteR2)
                mv = 0;
        }
        get_Distance();
        if (distance[3] < 500 && distance[3] < distance[1])
        {
            motor_Stop();
            blink_Warning();
            setWarning(WARN_OBSTACLE_AHEAD, 5000);
            status = CMD_STOP;
            return;
        }
        frontBoard = 0;
    }
    else if (detectPalleteF1 && detectPalleteF2)
        frontBoard = 0;
    set_Position();
    if (shouldAbortLoop())
        return;
    // Поднимаем паллет
    lifter_Up();
    // Перехват если требуется
    detect_Pallete();
    if (!detectPalleteR1 || !detectPalleteR2)
    {
        int dist = 100;
        if (shuttleLength == 1000)
            dist = 250;
        else if (shuttleLength == 1200)
            dist = 450;
        moove_Distance_R(dist, 15, 10);
        motor_Stop();
        lifter_Down();
        motor_Start_Forward();
        moove = 1;
        while (moove)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            blink_Work();
            get_Distance();
            if (millis() - count > 50)
            {
                if (distance[1] < 90 + chnlOffset)
                {
                    motor_Stop();
                    status = CMD_STOP;
                    moove  = 0;
                }
                else
                {
                    uint8_t detect = 1;
                    detect_Pallete();
                    if (detectPalleteR1 && detectPalleteR2)
                    {
                        moove = 0;
                        motor_Stop();
                        detect = 0;
                    }
                    delay(5);
                    if (detectPalleteR1 && detectPalleteR2)
                    {
                        moove = 0;
                        motor_Stop();
                        detect = 0;
                    }
                    else if (detect)
                    {
                        set_Position();
                        motor_Speed(10);
                    }
                }
                count = millis();
            }
        }
        motor_Stop();
        lifter_Up();
        diffPallete = 0;
    }
    // Везем на выгрузку
    stop_Before_Pallete_R();
    if (shouldAbortLoop())
        return;
    // Опускаем паллет
    if (!longWork && lifterUp)
    {
        lifter_Down();
        lastPallete         = 1;
        lastPalletePosition = currentPosition;
    }
    if (lastPallete)
    {
        makeLog(LOG_DEBUG, "Last pallete position after load = %d", lastPalletePosition);
    }
    STATS_ATOMIC_UPDATE(sramStats->payload.loadCounter++);
}

// Единичная загрузка
void single_Load()
{
    moove_Forward();
    if (shouldAbortLoop())
        return;
    get_Distance();
    detect_Pallete();
    if ((detectPalleteF1 && detectPalleteF2 && shuttleLength != 800) ||
        (detectPalleteF1 && detectPalleteF2 && detectPalleteR1 && detectPalleteR2))
        load_Pallete();
    else if (detectPalleteF1 && detectPalleteF2 && shuttleLength == 800)
    {
        blink_Warning();
        setWarning(WARN_PALLET_SIZE_ERROR, 5000);
    }
    else
    {
        motor_Start_Reverse();
        motor_Speed(10);
        while (!(detectPalleteF1 && detectPalleteF2) && distance[1] < 400 + chnlOffset)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            blink_Work();
            detect_Pallete();
            get_Distance();
            if (millis() - count > 50)
            {
                set_Position();
                motor_Speed(10);
                count = millis();
            }
        }
        motor_Stop();
        if ((detectPalleteF1 && detectPalleteF2 && shuttleLength != 800) ||
            (detectPalleteF1 && detectPalleteF2 && detectPalleteR1 && detectPalleteR2))
        {
            diffPallete = 10;
            load_Pallete();
        }
        else if (detectPalleteF1 && detectPalleteF2 && shuttleLength == 800)
        {
            blink_Warning();
            setWarning(WARN_PALLET_SIZE_ERROR, 5000);
        }
        else
        {
            blink_Warning();
            makeLog(LOG_ERROR, "Single load fail...");
            setWarning(WARN_PALLET_NOT_FOUND, 5000);
        }
    }
    moove_Forward();
}

// Пересчет паллет
void pallete_Counting_F()
{
    makeLog(LOG_INFO, "Start counting pallete forward...");
    lifter_Down();
    palleteCount = 0;
    // Двигаемся в начало канала
    moove_Forward();
    detect_Pallete();
    uint8_t palleteOnStart = 0;
    if ((detectPalleteF1 && detectPalleteF2) || (detectPalleteR1 && detectPalleteR2))
        palleteOnStart++;
    if (shouldAbortLoop())
        return;
    // Теперь к первому паллету в загрузке
    get_Distance();
    detect_Pallete();
    if (distance[2] > 1000 && !(detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2))
    {
        moove_Before_Pallete_R();
        if (shouldAbortLoop())
            return;
        if (distance[0] < 150 + chnlOffset)
        {
            motor_Stop();
            return;
        }
    }
    // Запускаем цикл пересчета
    get_Distance();
    motor_Start_Reverse();
    motor_Speed(28);
    uint8_t boardCount    = 0;
    int     count         = millis();
    int     countBoard    = count;
    int     boardPosition = 0;
    uint8_t moove         = 1;
    while (moove)
    {
        SystemYield();
        // Двигаемся и считаем паллеты
        if (shouldAbortLoop())
        {
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        blink_Work();
        get_Distance();
        detect_Pallete();
        if (detectPalleteR1 && detectPalleteR2)
        {
            boardCount++;
            makeLog(
                LOG_DEBUG,
                "Find board, count = %d Time between = %d %d",
                boardCount,
                millis() - countBoard,
                currentPosition);
            if (boardCount - 3 * (int)(boardCount / 3) == 1)
            {
                set_Position();
                boardPosition                 = currentPosition;
                palletePosition[palleteCount] = currentPosition;
                palleteCount++;
                STATS_ATOMIC_UPDATE(sramStats->payload.lifetimePalletsDetected++);
            }
            if (boardCount && boardCount - 3 * (int)(boardCount / 3) == 0)
            {
                set_Position();
                makeLog(LOG_DEBUG, "Pallete width = %d %d", currentPosition - boardPosition, currentPosition);
            }

            while (moove && (detectPalleteR1 || detectPalleteR2))
            {
                SystemYield();
                blink_Work();
                if (shouldAbortLoop())
                {
                    preserveManualStopOnAbort();
                    motor_Stop();
                    return;
                }
                get_Distance();
                detect_Pallete();
                if (millis() - count > 50)
                {
                    int spd = oldSpeed;
                    if (distance[0] <= 560 + chnlOffset && distance[0] > 120 + chnlOffset)
                        spd = (distance[0] / 20);
                    else if (distance[0] <= 120 + chnlOffset && distance[0] > 80 + chnlOffset)
                        spd = 6;
                    else if (distance[0] <= 100 + chnlOffset)
                    {
                        motor_Stop();
                        moove = 0;
                        makeLog(LOG_DEBUG, "End channel on counting with pallete ...");
                    }
                    else
                        spd = 28;
                    if (moove)
                        motor_Speed(spd);
                    set_Position();
                    count = millis();
                }
            }
        }

        if (millis() - count > 50 && moove)
        {
            int spd = oldSpeed;
            if (distance[0] <= 560 + chnlOffset && distance[0] > 120 + chnlOffset)
                spd = (distance[0] / 20);
            else if (distance[0] <= 120 + chnlOffset && distance[0] > 80 + chnlOffset)
                spd = 6;
            else if (distance[0] <= 100 + chnlOffset)
            {
                motor_Stop();
                moove = 0;
                makeLog(LOG_DEBUG, "End channel on counting with pallete ...");
            }
            else
                spd = 28;
            if (moove)
                motor_Speed(spd);
            set_Position();
            count = millis();
        }
    }
    palleteCount = lrint((float)boardCount / 3) + palleteOnStart;
    set_Position();
    channelLength = currentPosition + shuttleLength;
    return;
}

// Уплотнение вперед
void pallete_Compacting_F()
{
    makeLog(LOG_INFO, "Start compacting pallete forward...");
    if (!digitalRead(DL_DOWN))
        lifter_Down();
    moove_Reverse();
    if (shouldAbortLoop())
        return;
    get_Distance();
    status = CMD_COMPACT_F;
    detect_Pallete();
    while (distance[3] < 700 && (detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2) &&
           distance[1] > 100 + chnlOffset)
    {
        SystemYield();
        moove_Distance_F(100, 25, 25);
        if (shouldAbortLoop())
        {
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        status = CMD_COMPACT_F;
        get_Distance();
        if (distance[1] / 20 < oldSpeed)
            motor_Speed(distance[1] / 20);
        else
            motor_Speed(oldSpeed);
    }
    while (status != CMD_STOP)
    {
        SystemYield();
        blink_Work();
        load_Pallete();
        STATS_ATOMIC_UPDATE(sramStats->payload.compactCounter++);
        if (distance[1] < 150 && !lifterUp)
            return;
        if (shouldAbortLoop())
        {
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        status = CMD_COMPACT_F;
    }
}

// Уплотнение назад
void pallete_Compacting_R()
{
    makeLog(LOG_INFO, "Start compacting pallete reverse...");
    moove_Forward();
    if (shouldAbortLoop())
        return;
    get_Distance();
    status = CMD_COMPACT_R;
    detect_Pallete();
    while (distance[2] < 700 && (detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2) &&
           distance[0] > 100 + chnlOffset)
    {
        SystemYield();
        moove_Distance_R(100, 25, 25);
        if (shouldAbortLoop())
        {
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        status = CMD_COMPACT_R;
        get_Distance();
        if (distance[0] / 20 < oldSpeed)
            motor_Speed(distance[0] / 20);
        else
            motor_Speed(oldSpeed);
    }
    status = CMD_COMPACT_R;
    while (status != CMD_STOP)
    {
        SystemYield();
        blink_Work();
        unload_Pallete();
        STATS_ATOMIC_UPDATE(sramStats->payload.compactCounter++);
        if (distance[0] < 150 && !lifterUp)
            return;
        if (shouldAbortLoop())
        {
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        status               = CMD_COMPACT_R;
        firstPalletePosition = currentPosition;
    }
    firstPalletePosition = 0;
}

// Продолжительная загрузка
void long_Load()
{
    makeLog(LOG_INFO, "Starting continuos load...");
    status = CMD_LONG_LOAD;
    moove_Forward();
    if (shouldAbortLoop())
        return;
    status = CMD_LONG_LOAD;
    lifter_Down();
    get_Distance();
    detect_Pallete();
    if (!(detectPalleteF1 && detectPalleteF2))
    {
        motor_Start_Reverse();
        motor_Speed(10);
        while (!(detectPalleteF1 && detectPalleteF2) && distance[1] < 400 + chnlOffset)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            blink_Work();
            detect_Pallete();
            get_Distance();
            if (millis() - count > 50)
            {
                set_Position();
                motor_Speed(10);
                count = millis();
            }
        }
        motor_Stop();
        if (!(detectPalleteF1 && detectPalleteF2))
        {
            blink_Warning();
            setWarning(WARN_PALLET_NOT_FOUND, 5000);
            uint8_t wait = 1;
            moove_Distance_R(shuttleLength + 100, 60, 30);
            motor_Stop();
            count = millis();
            while (wait)
            {
                SystemYield();
                if (shouldAbortLoop())
                {
                    preserveManualStopOnAbort();
                    return;
                }
                blink_Work();
                get_Distance();
                detect_Pallete();
                if ((detectPalleteF1 && detectPalleteF2) || distance[3] < 1000)
                {
                    count = millis();
                    while (millis() - count < 10000)
                    {
                        SystemYield();
                        if (shouldAbortLoop())
                        {
                            preserveManualStopOnAbort();
                            get_Distance();
                            return;
                        }
                        blink_Work();
                    }
                    wait = 0;
                }
            }
        }
        if (detectPalleteR1 && detectPalleteR2)
            diffPallete = 10;
    }
    while (1)
    {
        status = CMD_LONG_LOAD;
        load_Pallete();
        if (lastPalletePosition && lastPalletePosition < shuttleLength * 2)
        {
            blink_Warning();
            setWarning(WARN_CHANNEL_FULL, 5000);
            status = 0;
            return;
        }
        uint8_t wait = 0;
        get_Distance();
        if (distance[1] < 90 + chnlOffset && !isErrorActive())
        {
            status = CMD_LONG_LOAD;
            moove_Distance_R(shuttleLength + 300, 60, 30);
            motor_Stop();
            wait = 1;
        }
        else if (shouldAbortLoop())
            return;
        else
            status = CMD_LONG_LOAD;
        count = millis();
        while (wait)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                preserveManualStopOnAbort();
                return;
            }
            blink_Work();
            get_Distance();
            detect_Pallete();
            if ((detectPalleteF1 && detectPalleteF2) || distance[3] < 1000)
            {
                count = millis();
                while (millis() - count < 10000)
                {
                    SystemYield();
                    if (shouldAbortLoop())
                    {
                        preserveManualStopOnAbort();
                        return;
                    }
                    blink_Work();
                }
                wait = 0;
            }
        }
    }
}

// Продолжительная выгрузка
void long_Unload()
{
    uint16_t oldInterPalleteDistance = interPalleteDistance;
    interPalleteDistance             = 700;
    makeLog(LOG_INFO, "Starting continuos unload...");
    if (fifoLifo)
        fifoLifo_Inverse();
    moove_Forward();
    if (shouldAbortLoop())
    {
        if (fifoLifo)
            fifoLifo_Inverse();
        interPalleteDistance = oldInterPalleteDistance;
        return;
    }
    uint8_t detect = 1;
    while (detect)
    {
        if (fifoLifo)
            fifoLifo_Inverse();
        unload_Pallete();
        if (fifoLifo)
            fifoLifo_Inverse();
        if (distance[0] < 200 + chnlOffset && !isErrorActive())
        {
            detect = 0;
            status = CMD_LONG_UNLOAD;
            break;
        }
        else if (shouldAbortLoop())
        {
            if (fifoLifo)
                fifoLifo_Inverse();
            interPalleteDistance = oldInterPalleteDistance;
            return;
        }
        count = millis();
        while (distance[3] < 900 && distance[1] > 700)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status               = CMD_STOP;
                interPalleteDistance = oldInterPalleteDistance;
                if (fifoLifo)
                    fifoLifo_Inverse();
                return;
            }
            blink_Work();
            get_Distance();
        }
        if (distance[1] > 700)
        {
            count = millis();
            while (distance[1] > 90)
            {
                SystemYield();
                if (shouldAbortLoop())
                {
                    motor_Stop();
                    status               = CMD_STOP;
                    interPalleteDistance = oldInterPalleteDistance;
                    if (fifoLifo)
                        fifoLifo_Inverse();
                    return;
                }
                blink_Work();
                get_Distance();
                if (millis() - count > 50)
                {
                    if (distance[1] > 400 + chnlOffset)
                        motor_Speed(20);
                    else if (distance[1] <= 400 + chnlOffset && distance[1] > 120 + chnlOffset)
                        motor_Speed(distance[1] / 20);
                    else
                        motor_Speed(6);
                    count = millis();
                }
            }
        }
        motor_Stop();
        lifter_Down();
        moove_Distance_R(shuttleLength + 500, 80, 80);
    }
    interPalleteDistance = oldInterPalleteDistance;
    if (fifoLifo)
        fifoLifo_Inverse();
}

// Продолжительная выгрузка заданного количества паллет
void long_Unload(uint8_t num)
{
    uint16_t oldInterPalleteDistance = interPalleteDistance;
    interPalleteDistance             = 700;
    makeLog(LOG_INFO, "Starting continuos unload...");
    if (fifoLifo)
        fifoLifo_Inverse();
    moove_Forward();
    if (shouldAbortLoop())
    {
        if (fifoLifo)
            fifoLifo_Inverse();
        interPalleteDistance = oldInterPalleteDistance;
        return;
    }
    uint8_t detect = 1;
    while (detect && num)
    {
        status = CMD_LONG_UNLOAD_QTY;
        send_Cmd();
        if (fifoLifo)
            fifoLifo_Inverse();
        unload_Pallete();
        if (fifoLifo)
            fifoLifo_Inverse();
        get_Distance();
        blink_Work();
        if (distance[0] < 200 + chnlOffset && !isErrorActive())
        {
            detect = 0;
            status = CMD_LONG_UNLOAD_QTY;
            break;
        }
        else if (shouldAbortLoop())
        {
            if (fifoLifo)
                fifoLifo_Inverse();
            interPalleteDistance = oldInterPalleteDistance;
            return;
        }
        status = CMD_LONG_UNLOAD_QTY;
        send_Cmd();
        count = millis();
        while (distance[3] < 900 && distance[1] > 700)
        {
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status               = CMD_STOP;
                interPalleteDistance = oldInterPalleteDistance;
                if (fifoLifo)
                    fifoLifo_Inverse();
                return;
            }
            blink_Work();
            get_Distance();
        }
        if (distance[1] > 700)
        {
            while (distance[1] > 90)
            {
                SystemYield();
                if (shouldAbortLoop())
                {
                    motor_Stop();
                    status               = CMD_STOP;
                    interPalleteDistance = oldInterPalleteDistance;
                    if (fifoLifo)
                        fifoLifo_Inverse();
                    return;
                }
                blink_Work();
                get_Distance();
                if (millis() - count > 50)
                {
                    if (distance[1] > 400 + chnlOffset)
                        motor_Speed(20);
                    else if (distance[1] <= 400 + chnlOffset && distance[1] > 120 + chnlOffset)
                        motor_Speed(distance[1] / 20);
                    else
                        motor_Speed(6);
                    count = millis();
                }
            }
        }

        motor_Stop();
        lifter_Down();
        num--;
        UPQuant--;
        status = CMD_LONG_UNLOAD_QTY;
        send_Cmd();
        if (num)
            moove_Distance_R(shuttleLength + 500, 80, 80);
    }
    interPalleteDistance = oldInterPalleteDistance;
    if (fifoLifo)
        fifoLifo_Inverse();
}

// Движение назад в ручном режиме
void moove_Right()
{
    makeLog(LOG_INFO, "Manual reverse hold...");
    uint8_t manualCount = 6;
    uint8_t moove       = 1;
    motor_Start_Reverse();
    int cnt = millis();
    get_Distance();

    motor_Speed(manualCount);

    while (moove)
    { // Едем пока держат кнопку
        SystemYield();
        if (shouldAbortLoop())
        {
            motor_Stop();
            makeLog(LOG_INFO, "Manual stop requested.");
            oldSpeed = 0;
            return;
        }
        if (status != CMD_MOVE_RIGHT_MAN)
        {
            motor_Stop();
            oldSpeed = 0;
            return;
        }

        blink_Work();
        get_Distance();
        if (millis() - cnt > 50)
        {
            if (manualCount < 60)
                manualCount += 3;
            cnt = millis();
            if (sensorOff)
                motor_Speed(manualCount);
            else
            {
                if (distance[0] >= 1500)
                    motor_Speed(manualCount);
                else if (distance[0] >= 90 + chnlOffset && distance[0] < 1500)
                    if (distance[0] / 25 < manualCount)
                        motor_Speed(distance[0] / 25);
                    else
                        motor_Speed(manualCount);
                else if (distance[0] < 90 + chnlOffset)
                {
                    motor_Stop();
                    clearManualRadioHold();
                    touchManualSession();
                    status           = CMD_MANUAL_MODE;
                    currentOperation = STATE_MANUAL;
                    makeLog(LOG_INFO, "Manual reverse stopped by sensor");
                    setWarning(WARN_END_OF_CHANNEL, 3000);
                    oldSpeed = 0;
                    return;
                }
            }
            set_Position();
        }
    }
    motor_Stop();
    oldSpeed = 0;
    return;
}

// Движение вперед в ручном режиме
void moove_Left()
{
    makeLog(LOG_INFO, "Manual forward hold...");
    uint8_t manualCount = 6;
    uint8_t moove       = 1;
    motor_Start_Forward();
    int cnt = millis();
    get_Distance();
    motor_Speed(manualCount);

    while (moove)
    { // Едем пока держат кнопку
        SystemYield();
        if (shouldAbortLoop())
        {
            motor_Stop();
            makeLog(LOG_INFO, "Manual stop requested.");
            oldSpeed = 0;
            return;
        }
        if (status != CMD_MOVE_LEFT_MAN)
        {
            motor_Stop();
            oldSpeed = 0;
            return;
        }
        blink_Work();
        get_Distance();
        if (millis() - cnt > 50)
        {
            if (manualCount < 60)
                manualCount += 3;
            cnt = millis();
            if (sensorOff)
                motor_Speed(manualCount);
            else
            {
                if (distance[1] >= 1500)
                    motor_Speed(manualCount);
                else if (distance[1] >= 90 + chnlOffset && distance[1] < 1500)
                    if (distance[1] / 25 < manualCount)
                        motor_Speed(distance[1] / 25);
                    else
                        motor_Speed(manualCount);
                else if (distance[1] < 90 + chnlOffset)
                {
                    motor_Stop();
                    clearManualRadioHold();
                    touchManualSession();
                    status           = CMD_MANUAL_MODE;
                    currentOperation = STATE_MANUAL;
                    makeLog(LOG_INFO, "Manual forward stopped by sensor");
                    setWarning(WARN_END_OF_CHANNEL, 3000);
                    oldSpeed = 0;
                    return;
                }
            }
            set_Position();
        }
    }
    motor_Stop();
    oldSpeed = 0;
    return;
}

// Демо режим
void demo_Mode()
{
    makeLog(LOG_INFO, "Start DEMO mode...");
    lifter_Down();
    moove_Reverse();
    lastPalletePosition = 0;
    if (shouldAbortLoop())
        return;
    get_Distance();
    detect_Pallete();
    uint8_t moove = 0;
    while (distance[3] < 700 && distance[1] > 100 + chnlOffset)
    {
        SystemYield();
        moove_Distance_F(100, 25, 25);
        if (shouldAbortLoop())
        {
            motor_Stop();
            return;
        }
        get_Distance();
        if (distance[1] / 20 < oldSpeed)
            motor_Speed(distance[1] / 20);
        else
            motor_Speed(oldSpeed);
        moove = 1;
    }
    while (1)
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            motor_Stop();
            status = CMD_STOP;
            return;
        }
        while (status != CMD_STOP)
        {
            SystemYield();
            count = millis();
            while (millis() - count < 1000 && !moove)
            {
                SystemYield();
                if (shouldAbortLoop())
                {
                    motor_Stop();
                    status = CMD_STOP;
                    return;
                }
                blink_Work();
                get_Distance();
            }
            moove = 0;
            load_Pallete();
            if (isErrorActive())
            {
                motor_Stop();
                return;
            }
            if (distance[1] < 200)
                status = CMD_STOP;
        }
        if ((status == CMD_STOP && distance[1] > 200) || isErrorActive())
        {
            motor_Stop();
            return;
        }
        else
            status = CMD_DEMO;
        motor_Stop();
        count = millis();
        while (millis() - count < 2000)
        {
            SystemYield();
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            blink_Work();
            get_Distance();
        }
        if (shouldAbortLoop())
            return;
        while (status != CMD_STOP)
        {
            SystemYield();
            count = millis();
            while (millis() - count < 1000)
            {
                SystemYield();
                if (shouldAbortLoop())
                {
                    motor_Stop();
                    status = CMD_STOP;
                    return;
                }
                blink_Work();
                get_Distance();
            }
            unload_Pallete();
            firstPalletePosition = currentPosition;
            if (isErrorActive())
            {
                motor_Stop();
                return;
            }
            if (distance[0] < 200)
                status = CMD_STOP;
        }
        firstPalletePosition = 0;
        if (!isErrorActive() && status == CMD_STOP && distance[1] < 200)
            status = CMD_DEMO;
        else if ((status == CMD_STOP && distance[0] > 200) || isErrorActive())
        {
            motor_Stop();
            return;
        }
        else
            status = CMD_DEMO;
        motor_Stop();
        count = millis();
        while (millis() - count < 2000)
        {
            SystemYield();
            SystemYield();
            if (shouldAbortLoop())
            {
                motor_Stop();
                status = CMD_STOP;
                return;
            }
            blink_Work();
            get_Distance();
        }
        if (shouldAbortLoop())
            return;
    }
}

#pragma endregion

#pragma region Технические функции

void SystemYield()
{
    uint32_t currentMillis = millis();
    writeBreadcrumb(BC_SYSTEM_YIELD, 0xFF, 0, 0);
    IWatchdog.reload();
    alertMan.processTimeouts(currentMillis);
    handlePendingCrash();

    if (pendingBootloaderEntry)
    {
        if (!bootloaderStopDone)
        {
            motor_Stop();
            SerialDisplay.flush();
            SerialLora.flush();
            bootloaderEntryAtMs = currentMillis;
            bootloaderStopDone  = true;
            return;
        }

        if (currentMillis - bootloaderEntryAtMs < kBootloaderEntryGraceMs)
        {
            return;
        }

        resetIntoBootloader();
        return;
    }

    static uint32_t timerTelemetry = 0;
    static uint32_t timerSensors   = 0;
    static uint32_t timerStats     = 0;
    static uint32_t timerUptime    = 0;
    static uint32_t timerRadioDiag = 0;
    static uint32_t timerLinkHealth = 0;
    static bool     lastLinkHealthValid = false;
    static bool     lastLinkHealthValidInitialized = false;

    if (!radioConfigOk && currentMillis >= kRadioConfigFailureWarnIntervalMs &&
        currentMillis - timerRadioDiag >= kRadioConfigFailureWarnIntervalMs)
    {
        timerRadioDiag = currentMillis;
        const uint8_t radioAddress =
            (isProvisionedShuttle() && E22Radio::isValidNodeId(shuttleNum)) ? shuttleNum : E22Radio::kAddressLowUnassigned;
        logRadioEnsureFailure(LOG_ERROR, "E22 60s", makeRadioDesiredConfig(radioAddress), radioLastEnsureResult);
    }
    retryRadioConfigIfIdle(currentMillis);

    writeBreadcrumb(BC_BMS_TICK, 0xFF, 0, 0);
    batteryBms.tick(currentMillis, mapBatteryActivity());
    batteryCharge = batteryBms.socPercent();
    batterySafetyCheck(currentMillis);
    logI2cDiagSummaryIfDue(currentMillis);

    if (currentMillis - timerTelemetry >= 300)
    {
        if (SerialDisplay.availableForWrite() >= (int)(sizeof(TelemetryPacket) + sizeof(FrameHeader) + 2U))
        {
            timerTelemetry = currentMillis;
            writeBreadcrumb(BC_TX_TELEMETRY, 0xFF, 0, 0);
            sendTelemetryPacket(&SerialDisplay);
        }
    }
    if (currentMillis - timerSensors >= 500)
    {
        if (SerialDisplay.availableForWrite() >= (int)(sizeof(SensorPacket) + sizeof(FrameHeader) + 2U))
        {
            timerSensors = currentMillis;
            writeBreadcrumb(BC_TX_SENSORS, 0xFF, 0, 0);
            sendSensorPacket(&SerialDisplay);
        }
    }
    if (currentMillis - timerStats >= 5000)
    {
        if (SerialDisplay.availableForWrite() >= (int)(sizeof(StatsPacket) + sizeof(FrameHeader) + 2U))
        {
            timerStats = currentMillis;
            writeBreadcrumb(BC_TX_STATS, 0xFF, 0, 0);
            sendStatsPacket(&SerialDisplay);
        }
    }
    const bool linkHealthValid = isRadioPacketRssiFresh(currentMillis);
    const bool linkHealthValidityChanged =
        !lastLinkHealthValidInitialized || linkHealthValid != lastLinkHealthValid;
    if (linkHealthValidityChanged || currentMillis - timerLinkHealth >= kLinkHealthPublishMs)
    {
        if (SerialDisplay.availableForWrite() >= (int)(sizeof(LinkHealthPacket) + sizeof(FrameHeader) + 2U))
        {
            timerLinkHealth                  = currentMillis;
            lastLinkHealthValid              = linkHealthValid;
            lastLinkHealthValidInitialized   = true;
            writeBreadcrumb(BC_TX_LINK_HEALTH, 0xFF, linkHealthValid ? 1 : 0, 0);
            sendLinkHealthPacket(&SerialDisplay);
        }
    }
    if (currentMillis - timerUptime >= 60000)
    {
        timerUptime = currentMillis;
        STATS_ATOMIC_UPDATE(sramStats->payload.totalUptimeMinutes++);
    }

    writeBreadcrumb(BC_POLL_DISPLAY, 0xFF, 0, 0);
    uint8_t cmdDisp = pollSerial(SerialDisplay, parserDisplay, false);
    if (pendingBootloaderEntry)
        return;
    writeBreadcrumb(BC_POLL_RADIO, 0xFF, 0, 0);
    uint8_t cmdRad = pollSerial(SerialLora, parserRadio, true);
    if (pendingBootloaderEntry)
        return;
    if (cmdDisp != NO_NEW_CMD || cmdRad != NO_NEW_CMD)
    {
        writeBreadcrumb(BC_SERIAL_COMMAND, 0xFF, cmdRad != NO_NEW_CMD ? cmdRad : cmdDisp, cmdDisp != NO_NEW_CMD ? 1 : 2);
    }

    if (cmdDisp == CMD_STOP || cmdDisp == CMD_STOP_MANUAL || cmdRad == CMD_STOP || cmdRad == CMD_STOP_MANUAL)
    {
        status            = (cmdRad == CMD_STOP_MANUAL || cmdDisp == CMD_STOP_MANUAL) ? CMD_STOP_MANUAL : CMD_STOP;
        statusSourceRadio = (cmdRad == CMD_STOP || cmdRad == CMD_STOP_MANUAL);
        clearManualRadioHold();
        pendingDisplayManualModeBypass = false;
        if (status == CMD_STOP_MANUAL && currentMode == CoreOpMode::MANUAL)
        {
            touchManualSession();
            makeLog(LOG_INFO, "Manual stop cmd src=%s", cmdRad == CMD_STOP_MANUAL ? "radio" : "display");
        }
        motor_Stop();
    }
    else if (cmdRad != NO_NEW_CMD)
    {
        if (canAcceptCommandNow(cmdRad, true))
        {
            if (currentMode == CoreOpMode::MANUAL && cmdRad == CMD_MANUAL_MODE)
            {
                clearManualRadioHold();
                pendingDisplayManualModeBypass = false;
                status                         = CMD_STOP;
            }
            else
            {
                status = cmdRad;
            }
            statusSourceRadio = true;
            if (currentMode == CoreOpMode::MANUAL && (isManualMoveCommand(cmdRad) || isManualDistanceCommand(cmdRad) ||
                                                      isLiftCommand(cmdRad) || cmdRad == CMD_STOP_MANUAL))
            {
                touchManualSession();
            }
            if (isManualMoveCommand(cmdRad))
            {
                beginManualRadioHold(currentMillis);
                pendingDisplayManualModeBypass = false;
            }
            else
            {
                clearManualRadioHold();
            }
        }
    }
    else if (cmdDisp != NO_NEW_CMD)
    {
        if (canAcceptCommandNow(cmdDisp, false))
        {
            if (currentMode == CoreOpMode::MANUAL && cmdDisp == CMD_MANUAL_MODE)
            {
                clearManualRadioHold();
                pendingDisplayManualModeBypass = false;
                status                         = CMD_STOP;
            }
            else
            {
                status = cmdDisp;
            }
            statusSourceRadio = false;
            if (currentMode == CoreOpMode::MANUAL && (isManualMoveCommand(cmdDisp) || isManualDistanceCommand(cmdDisp) ||
                                                      isLiftCommand(cmdDisp) || cmdDisp == CMD_STOP_MANUAL))
            {
                touchManualSession();
            }
            if (isManualMoveCommand(cmdDisp))
            {
                clearManualRadioHold();
                if (currentMode != CoreOpMode::MANUAL)
                {
                    pendingDisplayManualModeBypass = true;
                }
            }
            else
            {
                clearManualRadioHold();
            }
        }
    }

    if (manualRadioHoldActive && currentMode == CoreOpMode::MANUAL &&
        (status == CMD_MOVE_RIGHT_MAN || status == CMD_MOVE_LEFT_MAN) && manualRadioHoldLastHeartbeatMs != 0 &&
        currentMillis - manualRadioHoldLastHeartbeatMs > kManualRadioHoldWatchdogMs)
    {
        clearManualRadioHold();
        status                         = CMD_STOP_MANUAL;
        statusSourceRadio              = true;
        pendingDisplayManualModeBypass = false;
        makeLog(LOG_WARN, "Manual radio watchdog timeout");
        motor_Stop();
    }

    if (status == CMD_SYSTEM_RESET)
    {
        performSystemReset();
    }

    logBreadcrumbSummaryIfDue(currentMillis);
}

static void saveStatsPayload()
{
    sramStats->magicWord = STATS_MAGIC_WORD;
    sramStats->reserved  = 0;
    sramStats->crc16     = ProtocolUtils::calcCRC16((uint8_t *)&sramStats->payload, sizeof(StatsPacket));
}

void initStatsSRAM()
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    HAL_PWREx_EnableBkUpReg();

    uint32_t start = millis();
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_BRR) == RESET)
    {
        if (millis() - start > 100)
        {
            makeLog(LOG_ERROR, "Backup Regulator timeout!");
            break;
        }
    }

    if (sramStats->magicWord == STATS_MAGIC_WORD)
    {
        uint16_t calcCRC = ProtocolUtils::calcCRC16((uint8_t *)&sramStats->payload, sizeof(StatsPacket));
        if (calcCRC == sramStats->crc16)
        {
            makeLog(LOG_INFO, "SRAM Stats loaded successfully.");
            return;
        }
    }

    makeLog(LOG_WARN, "SRAM Stats Corrupt/Empty. Initializing to zero.");
    memset((void *)&sramStats->payload, 0, sizeof(StatsPacket));
    saveStatsPayload();
}

static void setResetRequestMarker(uint32_t flags)
{
    enableBackupSramAccess(100);
    *resetMarkerFlags = flags;
    *resetMarkerMagic = RESET_MARKER_MAGIC_WORD;
}

static uint32_t consumeResetRequestMarker()
{
    if (*resetMarkerMagic != RESET_MARKER_MAGIC_WORD)
    {
        return RESET_REASON_NONE;
    }

    const uint32_t flags = *resetMarkerFlags;
    *resetMarkerFlags   = RESET_REASON_NONE;
    *resetMarkerMagic   = 0;
    return flags;
}

static uint32_t captureResetReasonFlags()
{
    uint32_t flags = RESET_REASON_NONE;

#if defined(RCC_FLAG_IWDGRST)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
        flags |= RESET_REASON_WATCHDOG;
#endif
#if defined(RCC_FLAG_WWDGRST)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
        flags |= RESET_REASON_WATCHDOG;
#endif
#if defined(RCC_FLAG_SFTRST)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
        flags |= RESET_REASON_SOFTWARE;
#endif
#if defined(RCC_FLAG_PINRST)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
        flags |= RESET_REASON_PIN;
#endif
#if defined(RCC_FLAG_PORRST)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
        flags |= RESET_REASON_POWER;
#endif
#if defined(RCC_FLAG_BORRST)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))
        flags |= RESET_REASON_POWER;
#endif
#if defined(RCC_FLAG_LPWRRST)
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
        flags |= RESET_REASON_LOW_POWER;
#endif

    if (flags == RESET_REASON_NONE)
    {
        flags = RESET_REASON_UNKNOWN;
    }
    return flags;
}

static const char *primaryResetCategoryName(uint32_t flags)
{
    if ((flags & RESET_REASON_WATCHDOG) != 0)
        return "watchdog";
    if ((flags & RESET_REASON_SOFTWARE) != 0)
        return ((flags & RESET_REASON_BOOTLOADER_REQUEST) != 0) ? "bootloader" : "software";
    if ((flags & RESET_REASON_POWER) != 0)
        return "power";
    if ((flags & RESET_REASON_PIN) != 0)
        return "pin";
    return "other";
}

static void recordBootResetCause()
{
    const uint32_t rccFlags    = captureResetReasonFlags();
    const uint32_t markerFlags = consumeResetRequestMarker();
    uint32_t       flags       = rccFlags | markerFlags;
    if (markerFlags != RESET_REASON_NONE)
    {
        flags |= RESET_REASON_SOFTWARE;
    }
    if (flags == RESET_REASON_NONE)
    {
        flags = RESET_REASON_UNKNOWN;
    }

    lastBootResetFlags       = flags;
    lastBootResetMarkerFlags = markerFlags;

    STATS_ATOMIC_UPDATE(
        sramStats->payload.lastResetFlags = flags;
        if ((flags & RESET_REASON_WATCHDOG) != 0)
        {
            sramStats->payload.resetWatchdogCount++;
        }
        else if ((flags & RESET_REASON_SOFTWARE) != 0)
        {
            sramStats->payload.resetSoftwareCount++;
        }
        else if ((flags & RESET_REASON_POWER) != 0)
        {
            sramStats->payload.resetPowerCount++;
        }
        else if ((flags & RESET_REASON_PIN) != 0)
        {
            sramStats->payload.resetPinCount++;
        }
        else
        {
            sramStats->payload.resetOtherCount++;
        });

    makeBootLog(
        LOG_INFO,
        "Reset %s flags=%08lX marker=%08lX cnt=%u/%u/%u/%u/%u",
        primaryResetCategoryName(flags),
        (unsigned long)flags,
        (unsigned long)markerFlags,
        sramStats->payload.resetWatchdogCount,
        sramStats->payload.resetSoftwareCount,
        sramStats->payload.resetPinCount,
        sramStats->payload.resetPowerCount,
        sramStats->payload.resetOtherCount);
    logPreviousBreadcrumbIfWatchdog(flags);
}

// Процедура калибровки энкодера вперед
void calibrate_Encoder_R()
{
    makeLog(LOG_INFO, "Start calibrating encoder to Reverse");
    if (!readAs5600AngleFresh(&angle))
    {
        setFault(FAULT_MOVE_TIMEOUT);
        motor_Stop();
        return;
    }

    if (inverse)
    {
        inverse = 0;
    }
    int summ = 0;

    // Двигаемся к концу канала
    moove_Forward();
    if (shouldAbortLoop())
    {
        if (inverse)
        {
        }
        return;
    }
    // Выставляем 0 на энкодере
    if (!readAs5600AngleFresh(&angle))
    {
        setFault(FAULT_MOVE_TIMEOUT);
        motor_Stop();
        return;
    }
    motor_Start_Reverse();
    motor_Speed(5);
    int cnt = millis();
    count   = millis();
    const uint32_t zeroStartMs = millis();
    while (!(angle > 4086 || angle < 10))
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            motor_Stop();
            return;
        }
        if (millis() - zeroStartMs > 30000UL)
        {
            setFault(FAULT_MOVE_TIMEOUT);
            motor_Stop();
            return;
        }
        if (millis() - count > 100)
        {
            motor_Speed(5);
            count = millis();
        }
        if (millis() - cnt > 5)
        {
            if (!readAs5600AngleFresh(&angle))
            {
                setFault(FAULT_MOVE_TIMEOUT);
                motor_Stop();
                return;
            }
            cnt   = millis();
        }
    }

    // Запускаем цикл измерений
    uint8_t i = 0;
    cnt       = millis();
    const uint32_t calibrationStartMs = millis();
    while (i < 8)
    {
        if (millis() - calibrationStartMs > 120000UL)
        {
            setFault(FAULT_MOVE_TIMEOUT);
            motor_Stop();
            return;
        }
        delay(3);
        if (millis() - count > 100)
        {
            motor_Speed(5);
            count = millis();
        }
        if (!readAs5600AngleFresh(&angle))
        {
            setFault(FAULT_MOVE_TIMEOUT);
            motor_Stop();
            return;
        }
        if (angle > (7 - i) * 512 - 10 && angle < (7 - i) * 512 + 10)
        {
            calibrateEncoder_R[7 - i] = millis() - cnt;
            summ += calibrateEncoder_R[7 - i];
            cnt = millis();
            i++;
        }
        SystemYield();
        blink_Work();
    }
    motor_Stop();

    // Выводим данные
    String calData = "Calibrate_R data: ";
    for (i = 0; i < 8; i++)
    {
        calData += String(calibrateEncoder_R[i]) + "/";
        calibrateEncoder_R[i] = lrint(weelDia * 3.2 / 8 + calibrateEncoder_R[i] * weelDia * 3.2 / summ) / 2;
        calData += String(calibrateEncoder_R[i]) + " ";
        eepromData.calibrateEncoder_R[i] = calibrateEncoder_R[i];
    }
    makeLog(LOG_DEBUG, calData.c_str());
}

// Процедура калибровки энкодера вперед
void calibrate_Encoder_F()
{
    makeLog(LOG_INFO, "Start calibrating encoder to Forward");

    String calF = "Current calibrate F data: ";
    for (uint8_t i = 0; i < 8; i++)
        calF += String(calibrateEncoder_F[i]) + " ";
    makeLog(LOG_DEBUG, calF.c_str());

    String calR = "Current calibrate R data: ";
    for (uint8_t i = 0; i < 8; i++)
        calR += String(calibrateEncoder_R[i]) + " ";
    makeLog(LOG_DEBUG, calR.c_str());
    delay(50);
    if (inverse)
    {
        inverse = 0;
    }
    int summ = 0;

    // Двигаемся к концу канала
    moove_Distance_R(2000);
    if (shouldAbortLoop())
    {
        return;
    }
    // Выставляем 0 на энкодере
    if (!readAs5600AngleFresh(&angle))
    {
        setFault(FAULT_MOVE_TIMEOUT);
        motor_Stop();
        return;
    }
    motor_Start_Forward();
    motor_Speed(5);
    int cnt = millis();
    count   = millis();
    const uint32_t zeroStartMs = millis();
    while (!(angle > 4086 || angle < 10))
    {
        SystemYield();
        if (shouldAbortLoop())
        {
            motor_Stop();
            return;
        }
        if (millis() - zeroStartMs > 30000UL)
        {
            setFault(FAULT_MOVE_TIMEOUT);
            motor_Stop();
            return;
        }
        if (millis() - count > 100)
        {
            motor_Speed(5);
            count = millis();
        }
        if (millis() - cnt > 5)
        {
            if (!readAs5600AngleFresh(&angle))
            {
                setFault(FAULT_MOVE_TIMEOUT);
                motor_Stop();
                return;
            }
            cnt   = millis();
        }
    }

    // Запускаем цикл измерений
    uint8_t i = 0;
    cnt       = millis();
    const uint32_t calibrationStartMs = millis();
    while (i < 8)
    {
        if (millis() - calibrationStartMs > 120000UL)
        {
            setFault(FAULT_MOVE_TIMEOUT);
            motor_Stop();
            return;
        }
        delay(3);
        if (millis() - count > 100)
        {
            motor_Speed(5);
            count = millis();
        }
        if (!readAs5600AngleFresh(&angle))
        {
            setFault(FAULT_MOVE_TIMEOUT);
            motor_Stop();
            return;
        }
        if (angle > ((i + 1) * 512) - 10 && angle < (i + 1) * 512 + 10)
        {
            calibrateEncoder_F[i] = millis() - cnt;
            summ += calibrateEncoder_F[i];
            cnt = millis();
            i++;
        }
        SystemYield();
        blink_Work();
    }
    motor_Stop();

    // Выводим данные

    String calData = "Calibrate_F data: ";
    for (i = 0; i < 8; i++)
    {
        calData += String(calibrateEncoder_F[i]) + "/";
        calibrateEncoder_F[i] = lrint(weelDia * 3.2 / 8 + calibrateEncoder_F[i] * weelDia * 3.2 / summ) / 2;
        calData += String(calibrateEncoder_F[i]) + " ";
        eepromData.calibrateEncoder_F[i] = calibrateEncoder_F[i];
    }
    makeLog(LOG_DEBUG, calData.c_str());
}

bool loadConfigsFromFlash()
{
    bool found = false;
    for (uint16_t i = 0; i < CONFIG_TOTAL_PAGES; i++)
    {
        uint32_t          pageAddr = CONFIG_SECTOR_BASE + (i * CONFIG_PAGE_SIZE);
        ConfigPageHeader *header   = (ConfigPageHeader *)pageAddr;

        if (header->state == 0xAA)
        {
            uint32_t payloadAddr = pageAddr + sizeof(ConfigPageHeader);
            uint16_t crc         = ProtocolUtils::calcCRC16((uint8_t *)payloadAddr, sizeof(EEPROMData));

            if (crc == header->crc16)
            {
                memcpy(&eepromData, (void *)payloadAddr, sizeof(EEPROMData));
                found = true;
                break;
            }
            else
            {
                makeLog(LOG_WARN, "Config page %d CRC mismatch", i);
            }
        }
    }
    return found;
}

void saveConfigsToFlash()
{
    if (motorStart != 0 || motorReverse != 2)
    {
        makeLog(LOG_ERROR, "FATAL: Blocked Flash Erase while moving!");
        return;
    }

    uint8_t pageBuffer[CONFIG_PAGE_SIZE];
    memset(pageBuffer, 0xFF, CONFIG_PAGE_SIZE);

    ConfigPageHeader *header = (ConfigPageHeader *)pageBuffer;
    header->state            = 0xAA;
    memcpy(pageBuffer + sizeof(ConfigPageHeader), &eepromData, sizeof(EEPROMData));
    header->crc16 = ProtocolUtils::calcCRC16((uint8_t *)&eepromData, sizeof(EEPROMData));

    HAL_FLASH_Unlock();

    int currentActivePageIdx = -1;
    for (int i = 0; i < CONFIG_TOTAL_PAGES; i++)
    {
        if (((ConfigPageHeader *)(CONFIG_SECTOR_BASE + i * CONFIG_PAGE_SIZE))->state == 0xAA)
        {
            currentActivePageIdx = i;
            break;
        }
    }

    uint16_t nextPageIdx = (currentActivePageIdx == -1) ? 0 : (currentActivePageIdx + 1);
    if (nextPageIdx >= CONFIG_TOTAL_PAGES)
        nextPageIdx = 0;

    bool needErase = false;
    if (nextPageIdx == 0 && currentActivePageIdx != -1)
        needErase = true;
    else
    {
        ConfigPageHeader *nextHeader = (ConfigPageHeader *)(CONFIG_SECTOR_BASE + nextPageIdx * CONFIG_PAGE_SIZE);
        if (nextHeader->state != 0xFF)
        {
            needErase   = true;
            nextPageIdx = 0;
        }
    }

    if (needErase)
    {
        FLASH_EraseInitTypeDef eraseStruct;
        uint32_t               sectorError = 0;
        eraseStruct.TypeErase              = FLASH_TYPEERASE_SECTORS;
        eraseStruct.VoltageRange           = FLASH_VOLTAGE_RANGE_3;
        eraseStruct.Sector                 = CONFIG_SECTOR;
        eraseStruct.NbSectors              = 1;

        HAL_FLASHEx_Erase(&eraseStruct, &sectorError);
        nextPageIdx          = 0;
        currentActivePageIdx = -1;
    }

    uint32_t newPageAddr = CONFIG_SECTOR_BASE + (nextPageIdx * CONFIG_PAGE_SIZE);

    uint32_t *dataPtr = (uint32_t *)pageBuffer;
    for (size_t i = 0; i < CONFIG_PAGE_SIZE; i += 4)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, newPageAddr + i, *dataPtr++);
    }

    if (currentActivePageIdx != -1 && currentActivePageIdx != nextPageIdx)
    {
        uint32_t oldPageAddr = CONFIG_SECTOR_BASE + (currentActivePageIdx * CONFIG_PAGE_SIZE);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, oldPageAddr, 0x00);
    }

    HAL_FLASH_Lock();
    makeLog(LOG_INFO, "Configs saved to Flash Page %d", nextPageIdx);

    digitalWrite(ZOOMER, HIGH);
    delay(1000);
    digitalWrite(ZOOMER, LOW);
}

void saveEEPROMData(const EEPROMData &data)
{
    (void)data;
    saveConfigsToFlash();
}

// Чтение параметров с флэш памяти контроллера
void read_EEPROM_Data()
{
    // Чтение параметров из EEProm
    uint8_t calibrData = lrint(weelDia * 3.4 / 8);
    for (uint8_t i = 0; i < 8; i++)
    {
        eepromData.calibrateEncoder_F[i] = calibrData;
        eepromData.calibrateEncoder_R[i] = calibrData;
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        eepromData.calibrateSensor_F[i] = calibrateSensor_F[i];
        eepromData.calibrateSensor_R[i] = calibrateSensor_R[i];
    }
    eepromData.shuttleNum           = shuttleNum;
    eepromData.blinkTime            = blinkTime;
    eepromData.maxSpeed             = maxSpeed;
    eepromData.minSpeed             = minSpeed;
    eepromData.interPalleteDistance = interPalleteDistance;
    eepromData.inverse              = inverse;
    eepromData.fifoLifo             = fifoLifo;
    eepromData.lifter_Speed         = lifter_Speed;
    eepromData.timingBudget         = timingBudget;
    eepromData.minBattCharge        = minBattCharge;
    eepromData.shuttleLength        = shuttleLength;
    eepromData.waitTime             = waitTime;
    eepromData.mprOffset            = mprOffset;
    eepromData.TopLeftXF            = 4;
    eepromData.TopLeftYF            = 12;
    eepromData.BotRightXF           = 12;
    eepromData.BotRightYF           = 4;
    eepromData.TopLeftXR            = 4;
    eepromData.TopLeftYR            = 12;
    eepromData.BotRightXR           = 12;
    eepromData.BotRightYR           = 4;
    eepromData.chnlOffset           = chnlOffset;

    if (loadConfigsFromFlash())
    {
        /*for (uint8_t i = 0; i < 8; i++) {
          calibrateEncoder_F[i] = eepromData.calibrateEncoder_F[i];
          calibrateEncoder_R[i] = eepromData.calibrateEncoder_R[i];
        }*/
        for (uint8_t i = 0; i < 3; i++)
        {
            calibrateSensor_F[i] = eepromData.calibrateSensor_F[i];
            calibrateSensor_R[i] = eepromData.calibrateSensor_R[i];
        }
        shuttleNum           = eepromData.shuttleNum;
        blinkTime            = eepromData.blinkTime;
        maxSpeed             = eepromData.maxSpeed;
        minSpeed             = eepromData.minSpeed;
        interPalleteDistance = eepromData.interPalleteDistance;
        inverse              = eepromData.inverse;
        fifoLifo             = eepromData.fifoLifo;
        lifter_Speed         = eepromData.lifter_Speed;
        timingBudget         = eepromData.timingBudget;
        minBattCharge        = eepromData.minBattCharge;
        shuttleLength        = eepromData.shuttleLength;
        waitTime             = eepromData.waitTime;
        mprOffset            = eepromData.mprOffset;
        chnlOffset           = eepromData.chnlOffset;
    }
    else
    {
        shuttleNum            = 0;
        eepromData.shuttleNum = 0;
        saveConfigsToFlash();
        makeLog(LOG_WARN, "EEPROM reset. Provisioning State (ID=0).");
    }

    if (!isValidProvisionedShuttleId(shuttleNum))
    {
        shuttleNum            = 0;
        eepromData.shuttleNum = 0;
        makeLog(LOG_WARN, "Invalid shuttle ID. Provisioning State (ID=0)");
    }

    if (minBattCharge > 50)
        minBattCharge = 20;
    if (waitTime < 5000)
        waitTime = 5000;
    else if (waitTime > 30000)
        waitTime = 30000;
}

static bool batteryIsMotionLikeStatus(uint8_t st)
{
    switch (st)
    {
    case CMD_MOVE_RIGHT_MAN:
    case CMD_MOVE_LEFT_MAN:
    case CMD_MOVE_DIST_R:
    case CMD_MOVE_DIST_F:
    case CMD_LIFT_UP:
    case CMD_LIFT_DOWN:
        return true;
    default:
        return false;
    }
}

static inline bool batteryIsHighLoad()
{
    return (motorStart != 0) || batteryIsMotionLikeStatus(status);
}

static inline bool batteryLowFaultLatched()
{
    return batterySafetyState.lowStopLatched || ((alertMan.getErrorCode() & FAULT_LOW_BATTERY) != 0);
}

static BmsDdA5::ActivityHint mapBatteryActivity()
{
    if (batteryLowFaultLatched())
        return BmsDdA5::ActivityHint::LowBatteryFault;
    if (batteryIsHighLoad())
        return BmsDdA5::ActivityHint::HighLoad;

    if (currentMode == CoreOpMode::IDLE || currentMode == CoreOpMode::ERROR || isShuttleIdle())
    {
        return BmsDdA5::ActivityHint::Idle;
    }

    return BmsDdA5::ActivityHint::Active;
}

static void batterySafetyCheck(uint32_t now)
{
    if (batterySafetyState.lowStopLatched || batterySafetyState.emergencyActionActive)
        return;
    if (!batteryBms.hasFreshBasic(now))
        return;

    const BmsDdA5::Snapshot &snapshot = batteryBms.snapshot();
    if (batterySafetyState.requireFreshBasicAfterMs != 0U &&
        static_cast<int32_t>(snapshot.lastBasicUpdateMs - batterySafetyState.requireFreshBasicAfterMs) < 0)
    {
        return;
    }
    batterySafetyState.requireFreshBasicAfterMs = 0U;

    if (snapshot.socPercent > minBattCharge)
        return;

    batterySafetyState.lowStopLatched        = true;
    batterySafetyState.emergencyActionActive = true;
    makeLog(
        LOG_ERROR,
        "Low batt stop soc=%u V=%u prot=%04X",
        snapshot.socPercent,
        snapshot.packVoltage_mV,
        snapshot.protectionFlags);
    lifter_Down();
    moove_Forward();
    setFault(FAULT_LOW_BATTERY);
    STATS_ATOMIC_UPDATE(sramStats->payload.lowBatteryEvents++);
    status                                   = CMD_STOP;
    batterySafetyState.emergencyActionActive = false;
}

static void batterySafetyReset(uint32_t now)
{
    batterySafetyState.lowStopLatched           = false;
    batterySafetyState.emergencyActionActive    = false;
    batterySafetyState.requireFreshBasicAfterMs = now;
    (void)batteryBms.requestNow(BmsDdA5::RequestKind::BasicInfo, now);
}

static inline void markCrashFromIsr(uint8_t sourceBit)
{
    crashSourceMask |= sourceBit;
    crashPending = true;
    if (crashIrqCount < 0xFFFF)
    {
        crashIrqCount++;
    }
}

static void crashF1Irq()
{
    markCrashFromIsr(CRASH_SRC_F1);
}

static void crashF2Irq()
{
    markCrashFromIsr(CRASH_SRC_F2);
}

static void crashR1Irq()
{
    markCrashFromIsr(CRASH_SRC_R1);
}

static void crashR2Irq()
{
    markCrashFromIsr(CRASH_SRC_R2);
}
static inline uint16_t crashFaultMaskFromSources(uint8_t sources)
{
    uint16_t faultMask = 0;
    if (sources & CRASH_SRC_F1)
        faultMask |= static_cast<uint16_t>(FAULT_BUMPER_F1);
    if (sources & CRASH_SRC_F2)
        faultMask |= static_cast<uint16_t>(FAULT_BUMPER_F2);
    if (sources & CRASH_SRC_R1)
        faultMask |= static_cast<uint16_t>(FAULT_BUMPER_R1);
    if (sources & CRASH_SRC_R2)
        faultMask |= static_cast<uint16_t>(FAULT_BUMPER_R2);
    return faultMask;
}

static inline void setBumperFaultsFromMask(uint16_t faultMask)
{
    if (faultMask & static_cast<uint16_t>(FAULT_BUMPER_F1))
        setFault(FAULT_BUMPER_F1);
    if (faultMask & static_cast<uint16_t>(FAULT_BUMPER_F2))
        setFault(FAULT_BUMPER_F2);
    if (faultMask & static_cast<uint16_t>(FAULT_BUMPER_R1))
        setFault(FAULT_BUMPER_R1);
    if (faultMask & static_cast<uint16_t>(FAULT_BUMPER_R2))
        setFault(FAULT_BUMPER_R2);
}

// Обработка срабатывания бампера из обычного потока, не из ISR.
static inline void handlePendingCrash()
{
    __disable_irq();
    const bool     pending = crashPending;
    const uint8_t  sources = crashSourceMask;
    const uint16_t irqSeen = crashIrqCount;
    crashPending           = false;
    crashSourceMask        = 0;
    __enable_irq();

    if (!pending)
    {
        return;
    }

    const uint16_t bumperFaultMask = crashFaultMaskFromSources(sources);
    const uint16_t newFaultMask    = bumperFaultMask & ~alertMan.getErrorCode();
    setBumperFaultsFromMask(bumperFaultMask);
    status = CMD_STOP;

    if (!isPhysicallyStationary())
    {
        motor_Force_Stop();
        oldSpeed = 0;
    }

    if (newFaultMask != 0)
    {
        STATS_ATOMIC_UPDATE(sramStats->payload.crashCount++);
    }

    const uint32_t now = millis();
    if (crashLastLogMs == 0 || now - crashLastLogMs >= 1000U)
    {
        crashLastLogMs = now;
        makeLog(
            LOG_WARN,
            "Bumper crash src=%02X irq=%u pins F=%u%u R=%u%u",
            sources,
            irqSeen,
            digitalRead(BUMPER_F1),
            digitalRead(BUMPER_F2),
            digitalRead(BUMPER_R1),
            digitalRead(BUMPER_R2));
    }
}

// Обработка ошибок HardFault
void HardFault_Handler(void)
{
    struct
    {
        uint32_t r0;
        uint32_t r1;
        uint32_t r2;
        uint32_t r3;
        uint32_t r12;
        uint32_t lr;
        uint32_t pc;
        uint32_t psr;
    } *stack_ptr; // Указатель на текущее значение стека(SP)

    asm("TST lr, #4 \n"         // Тестируем 3ий бит указателя стека(побитовое И)
        "ITE EQ \n"             // Значение указателя стека имеет бит 3?
        "MRSEQ %[ptr], MSP  \n" // Да, сохраняем основной указатель стека
        "MRSNE %[ptr], PSP  \n" // Нет, сохраняем указатель стека процесса
        : [ptr] "=r"(stack_ptr));

    volatile uint32_t BFAR = 0xFFFFFFFF;

    asm("MRS %[bfar], PSP  \n" : [bfar] "=r"(BFAR));

    volatile uint32_t CFSR = 0xFFFFFFFF;

    asm("MRS %[cfsr], PSP  \n" : [cfsr] "=r"(CFSR));

    volatile uint32_t HFSR = 0xFFFFFFFF;

    asm("MRS %[hfsr], PSP  \n" : [hfsr] "=r"(HFSR));

    volatile uint32_t HFSR_R = SCB->HFSR;
    (void)HFSR_R;

    count     = millis();
    uint8_t k = 0;
    while (1)
    {
        if (millis() - count > 150)
        {
            digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
            digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));
            digitalWrite(RED_LED, !digitalRead(RED_LED));
            count = millis();
            k++;
        }
        if (k == 20)
        {
            makeLog(
                LOG_ERROR,
                "HardFault! R0=%lx R1=%lx R2=%lx R3=%lx",
                stack_ptr->r0,
                stack_ptr->r1,
                stack_ptr->r2,
                stack_ptr->r3);
            makeLog(
                LOG_ERROR, "R12=%lx LR=%lx PC=%lx PSR=%lx", stack_ptr->r12, stack_ptr->lr, stack_ptr->pc, stack_ptr->psr);
            makeLog(LOG_ERROR, "BFAR=%lx CFSR=%lx HFSR=%lx", BFAR, CFSR, HFSR);
            k = 0;
        }
    }
}

// Служебные функции установки даты и времени
bool isValidDateTime(int hour, int minute, int second, int day, int month, int year)
{
    if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59)
    {
        return false;
    }
    if (month < 1 || month > 12 || day < 1 || day > 31)
    {
        return false;
    }
    if (month == 2 && day > (isLeapYear(year) ? 29 : 28))
    {
        return false;
    }
    if ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30)
    {
        return false;
    }
    return true;
}

uint8_t getWeekDay(int day, int month, int year)
{
    if (month < 3)
    {
        month += 12;
        year -= 1;
    }
    int k       = year % 100;
    int j       = year / 100;
    int weekDay = (day + (13 * (month + 1)) / 5 + k + (k / 4) + (j / 4) - (2 * j)) % 7;
    return (weekDay + 5) % 7 + 1;
}

bool isLeapYear(int year)
{
    return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}

// Программный переход в загрузчик DFU
void jumpToBootloader()
{
    const uint32_t bootStack       = *(__IO uint32_t *)kSystemBootloaderAddr;
    const uint32_t bootJumpAddress = *(__IO uint32_t *)(kSystemBootloaderAddr + 4U);
    void (*bootJump)(void)        = (void (*)(void))bootJumpAddress;

    if (serialLinksStarted)
    {
        SerialDisplay.flush();
        SerialLora.flush();
        SerialRS485.flush();
        SerialDisplay.end();
        SerialLora.end();
        SerialRS485.end();
        serialLinksStarted = false;
    }

    HAL_RCC_DeInit();
    HAL_DeInit();

    __disable_irq();

    for (uint8_t i = 0; i < 8; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

#if defined(__HAL_RCC_SYSCFG_CLK_ENABLE)
    __HAL_RCC_SYSCFG_CLK_ENABLE();
#endif
#if defined(__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH)
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();
#endif

    SCB->VTOR = kSystemBootloaderAddr;
    __DSB();
    __ISB();

    // Устанавливаем MSP из загрузчика и передаем управление ROM bootloader.
    __set_MSP(bootStack);
    __enable_irq();
    bootJump();

    // Эта точка не должна быть достигнута
    while (1)
        ;
}

void blink()
{
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    delay(100);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    delay(100);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    delay(100);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    delay(100);
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    delay(100);
    digitalWrite(RED_LED, LOW);
    // digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
}

static void applyRadioConfigAtBoot()
{
    const bool    shuttleAddressValid = isProvisionedShuttle() && E22Radio::isValidNodeId(shuttleNum);
    const uint8_t radioAddress        = shuttleAddressValid ? shuttleNum : E22Radio::kAddressLowUnassigned;
    if (!shuttleAddressValid)
    {
        makeLog(LOG_WARN, "Radio ID %u invalid, using addr=0", shuttleNum);
    }

    const E22Radio::LogicalConfig desiredConfig = makeRadioDesiredConfig(radioAddress);
    const E22Radio::EnsureOptions ensureOptions = makeRadioEnsureOptions();
    const uint32_t                freqKhz       = E22Radio::channelFrequencyKhz(desiredConfig.channel);

    e22Radio.init(&SerialLora, kRadioControlPins, E22Radio::uartBaudValue(kRadioHostBaud), NULL);
    logRadioDesiredConfig(LOG_INFO, "E22 want", desiredConfig);

    const E22Radio::EnsureResult ensureResult = e22Radio.ensureConfig(desiredConfig, ensureOptions);
    radioLastEnsureResult                     = ensureResult;
    radioConfigOk                             = ensureResult.ok();
    radioAppendedRssiEnabled                  = radioConfigOk && desiredConfig.appendRssiEnabled;
    if (ensureResult.ok())
    {
        makeLog(
            LOG_INFO,
            "E22 ok a=%u c=%u f=%lu.%03lu b=%lu",
            radioAddress,
            desiredConfig.channel,
            (unsigned long)(freqKhz / 1000UL),
            (unsigned long)(freqKhz % 1000UL),
            (unsigned long)E22Radio::uartBaudValue(desiredConfig.uartBaud));
        logRadioRawConfig(LOG_INFO, "E22 got", ensureResult.finalConfig);
    }
    else
    {
        logRadioEnsureFailure(LOG_WARN, "E22 fail", desiredConfig, ensureResult);
    }
}

static E22Radio::EnsureOptions makeRadioEnsureOptions()
{
    E22Radio::EnsureOptions ensureOptions = {};
    ensureOptions.maxAttempts             = 3;
    ensureOptions.modeSettleMs            = 100;
    ensureOptions.auxTimeoutMs            = 300;
    ensureOptions.ioTimeoutMs             = 250;
    ensureOptions.readCmdSettleMs         = 30;
    ensureOptions.writeSettleMs           = 100;
    ensureOptions.baudSwitchSettleMs      = 30;
    ensureOptions.postModeGuardMs         = 20;
    ensureOptions.configCommandBaud       = 9600;
    ensureOptions.packetRssiReadTimeoutMs = E22Radio::kDefaultPacketRssiTimeoutMs;
    return ensureOptions;
}

static E22Radio::LogicalConfig makeRadioDesiredConfig(uint8_t nodeId)
{
    const uint8_t radioAddress = E22Radio::isValidNodeId(nodeId) ? nodeId : E22Radio::kAddressLowUnassigned;
    E22Radio::LogicalConfig cfg = {};
    cfg.address                 = E22Radio::shuttleAddressFromNodeId(radioAddress);
    cfg.channel                 = E22Radio::kDefaultChannel440;
    cfg.uartBaud                = kRadioHostBaud;
    cfg.airDataRate             = E22Radio::AirDataRate::B4800;
    cfg.txPower                 = E22Radio::TxPower::Dbm27;
    cfg.parity                  = E22Radio::UartParity::U8N1;
    cfg.netId                   = 0x00;
    cfg.ambientRssiEnabled      = true;
    cfg.appendRssiEnabled       = true;
    cfg.cryptKey                = E22Radio::kDefaultCryptKey;
    cfg.transmissionMode        = E22Radio::TransmissionMode::Fixed;
    cfg.lbtEnabled              = false;
    cfg.subPacketSize           = E22Radio::SubPacketSize::Bytes240;
    return cfg;
}

static void reapplyRadioConfigForShuttleId(uint8_t newId, uint8_t previousId)
{
    const E22Radio::LogicalConfig desiredConfig = makeRadioDesiredConfig(newId);
    const E22Radio::EnsureResult  ensureResult  = e22Radio.ensureConfig(desiredConfig, makeRadioEnsureOptions());
    const uint32_t                freqKhz       = E22Radio::channelFrequencyKhz(desiredConfig.channel);
    logRadioDesiredConfig(LOG_INFO, "E22ID want", desiredConfig);
    radioLastEnsureResult  = ensureResult;
    radioConfigOk             = ensureResult.ok();
    radioAppendedRssiEnabled  = radioConfigOk && desiredConfig.appendRssiEnabled;
    if (ensureResult.ok())
    {
        makeLog(
            LOG_INFO,
            "E22 ID ok a=%u c=%u f=%lu.%03lu",
            desiredConfig.address.addl,
            desiredConfig.channel,
            (unsigned long)(freqKhz / 1000UL),
            (unsigned long)(freqKhz % 1000UL));
        logRadioRawConfig(LOG_INFO, "E22ID got", ensureResult.finalConfig);
        return;
    }

    makeLog(LOG_WARN, "E22 ID change failed %u>%u", previousId, newId);
    logRadioEnsureFailure(LOG_WARN, "E22ID fail", desiredConfig, ensureResult);
}

static void logRadioRawConfig(LogLevel level, const char *tag, const E22Radio::RawConfig &raw)
{
    makeLog(level, "%s ah=%02X al=%02X net=%02X", tag, raw.addh, raw.addl, raw.netid);
    makeLog(level, "%s r0=%02X r1=%02X ch=%02X r3=%02X", tag, raw.reg0, raw.reg1, raw.reg2, raw.reg3);
}

static void logRadioDesiredConfig(LogLevel level, const char *tag, const E22Radio::LogicalConfig &config)
{
    logRadioRawConfig(level, tag, E22Radio::encode(config));
}

static void formatRadioBaudMask(uint8_t mask, char *buffer, size_t bufferSize)
{
    if (buffer == NULL || bufferSize == 0)
        return;

    buffer[0] = '\0';
    static const uint32_t kBauds[8] = { 1200UL, 2400UL, 4800UL, 9600UL, 19200UL, 38400UL, 57600UL, 115200UL };
    size_t used = 0;
    for (uint8_t i = 0; i < 8 && used < bufferSize; ++i)
    {
        if ((mask & (1u << i)) == 0)
            continue;
        const int written = snprintf(
            buffer + used,
            bufferSize - used,
            "%s%lu",
            used == 0 ? "" : ",",
            (unsigned long)kBauds[i]);
        if (written < 0)
            break;
        used += static_cast<size_t>(written);
        if (used >= bufferSize)
        {
            buffer[bufferSize - 1] = '\0';
            break;
        }
    }
    if (buffer[0] == '\0')
    {
        snprintf(buffer, bufferSize, "none");
    }
}

static void logRadioEnsureFailure(
    LogLevel level,
    const char *tag,
    const E22Radio::LogicalConfig &desired,
    const E22Radio::EnsureResult &result)
{
    char baudList[56];
    formatRadioBaudMask(result.diag.triedBaudMask, baudList, sizeof(baudList));

    makeLog(
        level,
        "%s %s a=%u c=%u attempts=%u bauds=%s last=%lu pins M0/M1/AUX=%u/%u/%u cnt aux/read/write/verify=%u/%u/%u/%u finalAux=%u",
        tag,
        E22Radio::statusToString(result.status),
        desired.address.addl,
        desired.channel,
        result.attemptsUsed,
        baudList,
        (unsigned long)result.diag.lastBaud,
        digitalRead(RADIO_E22_M0),
        digitalRead(RADIO_E22_M1),
        digitalRead(RADIO_E22_AUX),
        result.diag.auxTimeoutCount,
        result.diag.readTimeoutCount,
        result.diag.writeFailureCount,
        result.diag.verifyMismatchCount,
        result.diag.finalAuxHigh ? 1 : 0);
    logRadioDesiredConfig(level, "E22 desired", desired);
    if (result.hasFinalConfig)
    {
        logRadioRawConfig(level, "E22 final", result.finalConfig);
    }
    else
    {
        makeLog(level, "E22 final config unreadable");
    }
}

static void retryRadioConfigIfIdle(uint32_t now)
{
    if (radioConfigOk || pendingBootloaderEntry || !isPhysicallyStationary())
    {
        return;
    }
    if (now - radioLastConfigRetryMs < kRadioConfigRetryIntervalMs)
    {
        return;
    }
    if (radioLastRxFrameMs != 0 && now - radioLastRxFrameMs < kRadioConfigRetryRxQuietMs)
    {
        return;
    }

    radioLastConfigRetryMs = now;
    makeLog(LOG_WARN, "E22 idle config retry");
    applyRadioConfigAtBoot();
}

static ShuttleState mapCmdToOperation(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_LOAD:
        return STATE_LOAD;
    case CMD_UNLOAD:
        return STATE_UNLOAD;
    case CMD_LONG_LOAD:
        return STATE_LONG_LOAD;
    case CMD_LONG_UNLOAD:
        return STATE_LONG_UNLOAD;
    case CMD_LONG_UNLOAD_QTY:
        return STATE_LONG_UNLOAD_QTY;
    case CMD_COMPACT_F:
    case CMD_COMPACT_R:
        return STATE_COMPACT;
    case CMD_DEMO:
        return STATE_DEMO;
    case CMD_COUNT_PALLETS:
        return STATE_COUNT_PALLETS;
    case CMD_MOVE_DIST_F:
        return STATE_MOVE_FWD;
    case CMD_MOVE_DIST_R:
        return STATE_MOVE_REV;
    case CMD_LIFT_UP:
        return STATE_LIFT_UP;
    case CMD_LIFT_DOWN:
        return STATE_LIFT_DOWN;
    case CMD_HOME:
        return STATE_HOME;
    case CMD_CALIBRATE:
        return STATE_CALIBRATE;
    case CMD_MANUAL_MODE:
    case CMD_MOVE_RIGHT_MAN:
    case CMD_MOVE_LEFT_MAN:
        return STATE_MANUAL;
    default:
        return STATE_IDLE;
    }
}

static ShuttleState predictTelemetryStateForAcceptedCommand(uint8_t cmd, bool fromRadio)
{
    if (cmd == CMD_MANUAL_MODE)
    {
        if (currentMode == CoreOpMode::MANUAL && fromRadio)
        {
            return STATE_IDLE;
        }
        return STATE_MANUAL;
    }

    if (currentMode == CoreOpMode::MANUAL)
    {
        if (isContinuousManualCommand(cmd) || isManualDistanceCommand(cmd) || isLiftCommand(cmd))
        {
            return STATE_MANUAL;
        }
    }

    return mapCmdToOperation(cmd);
}

static inline bool isValidProvisionedShuttleId(int32_t id)
{
    return (id >= E22Radio::kAddressNodeMin && id <= E22Radio::kAddressNodeMax);
}

static inline bool isProvisionedShuttle()
{
    return isValidProvisionedShuttleId(shuttleNum);
}

static void scheduleBootloaderEntry()
{
    pendingBootloaderEntry = true;
    bootloaderStopDone     = false;
    bootloaderEntryAtMs    = millis();
}

static void enableBackupSramAccess(uint32_t readyTimeoutMs)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    HAL_PWREx_EnableBkUpReg();

    const uint32_t start = millis();
    while (__HAL_PWR_GET_FLAG(PWR_FLAG_BRR) == RESET && (millis() - start) < readyTimeoutMs)
    {
    }
}

static bool consumeBootloaderResetRequest()
{
    enableBackupSramAccess(100);
    if (*bootloaderMagicWord != BOOTLOADER_MAGIC_WORD || *bootloaderMagicConfirm != BOOTLOADER_MAGIC_CONFIRM_WORD)
        return false;

    *bootloaderMagicWord    = 0;
    *bootloaderMagicConfirm = 0;
    return true;
}

static void requestBootloaderResetEntry()
{
    enableBackupSramAccess(100);
    setResetRequestMarker(RESET_REASON_SOFTWARE | RESET_REASON_BOOTLOADER_REQUEST);
    *bootloaderMagicWord    = BOOTLOADER_MAGIC_WORD;
    *bootloaderMagicConfirm = BOOTLOADER_MAGIC_CONFIRM_WORD;
}

static void resetIntoBootloader()
{
    requestBootloaderResetEntry();
    HAL_NVIC_SystemReset();
    while (1)
    {
    }
}

static inline bool isSupportedCommand(uint8_t cmd)
{
    switch (cmd)
    {
    case CMD_STOP:
    case CMD_STOP_MANUAL:
    case CMD_SYSTEM_RESET:
    case CMD_RESET_ERROR:
    case CMD_MANUAL_MODE:
    case CMD_DEMO:
    case CMD_HOME:
    case CMD_MOVE_RIGHT_MAN:
    case CMD_MOVE_LEFT_MAN:
    case CMD_MOVE_DIST_R:
    case CMD_MOVE_DIST_F:
    case CMD_LIFT_UP:
    case CMD_LIFT_DOWN:
    case CMD_CALIBRATE:
    case CMD_LOAD:
    case CMD_UNLOAD:
    case CMD_LONG_LOAD:
    case CMD_LONG_UNLOAD:
    case CMD_LONG_UNLOAD_QTY:
    case CMD_COMPACT_F:
    case CMD_COMPACT_R:
    case CMD_COUNT_PALLETS:
    case CMD_SAVE_EEPROM:
    case CMD_GET_CONFIG:
    case CMD_FIRMWARE_UPDATE:
        return true;
    default:
        return false;
    }
}

static inline bool isManualMoveCommand(uint8_t cmd)
{
    return (cmd == CMD_MOVE_RIGHT_MAN || cmd == CMD_MOVE_LEFT_MAN);
}

static inline bool isManualDistanceCommand(uint8_t cmd)
{
    return (cmd == CMD_MOVE_DIST_R || cmd == CMD_MOVE_DIST_F);
}

static inline bool isContinuousManualCommand(uint8_t cmd)
{
    return (isManualMoveCommand(cmd) || cmd == CMD_STOP_MANUAL);
}

static inline bool isLiftCommand(uint8_t cmd)
{
    return (cmd == CMD_LIFT_UP || cmd == CMD_LIFT_DOWN);
}

static inline bool isUnprovisionedCommandAllowed(uint8_t cmd)
{
    return (cmd == CMD_STOP || cmd == CMD_STOP_MANUAL || cmd == CMD_SYSTEM_RESET || cmd == CMD_RESET_ERROR);
}

static inline bool isOutOfChannelExemptCommand(uint8_t cmd)
{
    return (
        cmd == CMD_STOP || cmd == CMD_STOP_MANUAL || cmd == CMD_SYSTEM_RESET || cmd == CMD_SAVE_EEPROM ||
        cmd == CMD_GET_CONFIG || cmd == CMD_RESET_ERROR || isLiftCommand(cmd));
}

static inline bool isOverrideCommand(uint8_t cmd)
{
    return (
        cmd == CMD_STOP || cmd == CMD_STOP_MANUAL || cmd == CMD_SYSTEM_RESET || cmd == CMD_RESET_ERROR ||
        cmd == CMD_SAVE_EEPROM || cmd == CMD_GET_CONFIG);
}

static inline bool isShuttleIdle()
{
    return (status == 0 || status == CMD_STOP);
}

static inline bool canAcceptCommandNow(uint8_t cmd, bool fromRadio)
{
    if (isOverrideCommand(cmd))
    {
        return true;
    }

    if (isManualMoveCommand(cmd))
    {
        if (currentMode == CoreOpMode::MANUAL)
        {
            return true;
        }

        return (!fromRadio && isShuttleIdle());
    }

    if (isManualDistanceCommand(cmd))
    {
        return (currentMode == CoreOpMode::MANUAL || isShuttleIdle());
    }

    if (isLiftCommand(cmd))
    {
        return (currentMode == CoreOpMode::MANUAL || (currentMode == CoreOpMode::IDLE && isShuttleIdle()));
    }

    if (cmd == CMD_MANUAL_MODE)
    {
        return (isShuttleIdle() || currentMode == CoreOpMode::MANUAL);
    }

    return isShuttleIdle();
}

static inline bool canApplyConfigNow()
{
    if (!isPhysicallyStationary() || pendingBootloaderEntry || batterySafetyState.emergencyActionActive)
    {
        return false;
    }

    return (currentMode == CoreOpMode::IDLE || currentMode == CoreOpMode::ERROR) && isShuttleIdle();
}

static inline bool isPhysicallyStationary()
{
    return (motorStart == 0 && motorReverse == 2);
}

static inline void performSystemReset()
{
    makeLog(LOG_INFO, "Reboot system by external command...");
    setResetRequestMarker(RESET_REASON_SOFTWARE);
    delay(20);
    HAL_NVIC_SystemReset();
}

static inline void clearLatchedAlertsAndReturnToIdle(uint32_t now)
{
    alertMan.clearAllFaults();
    alertMan.clearAllWarnings();
    batterySafetyReset(now);
    resetTofDiagnostics();
    clearManualRadioHold();
    pendingDisplayManualModeBypass = false;
    statusSourceRadio              = false;
    status                         = 0;
    currentOperation               = STATE_IDLE;
    digitalWrite(RED_LED, LOW);
    currentMode = CoreOpMode::IDLE;
}

static inline void resetTofDiagnostics()
{
    const uint32_t now = millis();
    for (uint8_t i = 0; i < 4; ++i)
    {
        tofSensorErrors[i] = 0;
        tofReadErrors[i]   = 0;
        tofTimeoutErrors[i] = 0;
        tofSharedBusErrors[i] = 0;
        tofLastDiagStatus[i] = TOF_I2C_OK;
        tofFailureWarnLogged[i] = false;
        tofFaultLogged[i]       = false;
        tofLastTransportSuccessMs[i]       = now;
        tofHasTransportSuccess[i]          = false;
        tofConsecutiveTransportFailures[i] = 0U;
        tofLastFailureLogMs[i]             = 0U;
    }
    tofRuntimeStartMs          = now;
    tofRuntimeStarted          = true;
    tofI2cRecoveryLogged    = false;
    tofLastI2cRecoveryLogMs = 0;
    tofLastRecoverySummaryLogMs = 0;
    resetTofBusVote();
}

static inline void touchManualSession()
{
    lastManualSessionActivityTime = millis();
}

static inline void beginManualRadioHold(uint32_t now)
{
    manualRadioHoldActive          = true;
    manualRadioHoldLastHeartbeatMs = now;
}

static inline void refreshManualRadioHoldWatchdog(uint32_t now)
{
    if (manualRadioHoldActive)
    {
        manualRadioHoldLastHeartbeatMs = now;
    }
}

static inline void clearManualRadioHold()
{
    manualRadioHoldActive          = false;
    manualRadioHoldLastHeartbeatMs = 0;
}

static inline void preserveManualStopOnAbort()
{
    if (status != CMD_STOP_MANUAL)
    {
        status = CMD_STOP;
    }
}

static inline bool isErrorActive()
{
    return (alertMan.getErrorCode() != 0);
}

static inline bool shouldAbortLoop()
{
    return (status == CMD_STOP || status == CMD_STOP_MANUAL || isErrorActive());
}

static bool writeTransportPayloadWait(Stream *port, const uint8_t *buffer, uint16_t length, uint32_t timeoutMs)
{
    if (!port || !buffer || length == 0)
    {
        return false;
    }

    const uint32_t startedAt = millis();
    if (port == &SerialLora)
    {
        while (transportAvailableForWrite(port) < length)
        {
            if (millis() - startedAt >= timeoutMs)
            {
                return false;
            }
            if (IWatchdog.isEnabled())
            {
                IWatchdog.reload();
            }
            delay(1);
        }
        return writeTransportPayload(port, buffer, length, NULL) == length;
    }

    uint16_t written = 0;
    while (written < length)
    {
        if (millis() - startedAt >= timeoutMs)
        {
            return false;
        }

        int available = transportAvailableForWrite(port);
        if (available <= 0)
        {
            if (IWatchdog.isEnabled())
            {
                IWatchdog.reload();
            }
            delay(1);
            continue;
        }

        uint16_t chunk = length - written;
        if ((uint16_t)available < chunk)
        {
            chunk = (uint16_t)available;
        }

        const size_t sent = port->write(buffer + written, chunk);
        if (sent == 0)
        {
            if (IWatchdog.isEnabled())
            {
                IWatchdog.reload();
            }
            delay(1);
            continue;
        }
        written += (uint16_t)sent;
    }

    return true;
}

static bool sendLogFrame(LogLevel level, const char *msg, Stream *port, bool waitForSpace)
{
    if (pendingBootloaderEntry || msg == NULL)
        return false;

    bool sentAll = true;
    const char *cursor    = msg;
    size_t      remaining = strlen(msg);

    do
    {
        const uint8_t msgLen =
            (remaining > LOG_MAX_PRINTABLE_CHARS) ? LOG_MAX_PRINTABLE_CHARS : static_cast<uint8_t>(remaining);
        uint8_t logBuffer[sizeof(FrameHeader) + 1 + MAX_LOG_STRING_LEN + 2];

        FrameHeader *header = (FrameHeader *)logBuffer;
        header->sync1       = PROTOCOL_SYNC_1_V2;
        header->sync2       = PROTOCOL_SYNC_2_V2;
        header->length      = 1 + msgLen + 1;
        header->targetID    = TARGET_ID_NONE;

        static uint8_t seqCounter = 0;
        header->seq               = seqCounter++;
        header->msgID             = MSG_LOG;

        logBuffer[sizeof(FrameHeader)] = (uint8_t)level;

        memcpy(logBuffer + sizeof(FrameHeader) + 1, cursor, msgLen);
        logBuffer[sizeof(FrameHeader) + 1 + msgLen] = '\0';

        uint16_t totalLen = sizeof(FrameHeader) + header->length;
        ProtocolUtils::appendCRC(logBuffer, totalLen);

        const size_t writeLen = totalLen + 2;
        const bool sent = waitForSpace
            ? writeTransportPayloadWait(port, logBuffer, writeLen, kBootLogWriteTimeoutMs)
            : (writeTransportPayload(port, logBuffer, writeLen, NULL) == writeLen);
        if (!sent)
        {
            sentAll = false;
            break;
        }

        cursor += msgLen;
        remaining -= msgLen;
    } while (remaining > 0);

    if (sentAll && waitForSpace && port != nullptr)
    {
        port->flush();
    }
    return sentAll;
}

void sendLog(LogLevel level, const char *msg, Stream *port = &SerialDisplay)
{
    (void)sendLogFrame(level, msg, port, false);
}

void makeLogImpl(LogLevel level, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(logStringBuffer, sizeof(logStringBuffer), format, args);
    va_end(args);

    sendLog(level, logStringBuffer, &SerialDisplay);
}

void makeReliableLogImpl(LogLevel level, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(logStringBuffer, sizeof(logStringBuffer), format, args);
    va_end(args);

    (void)sendLogFrame(level, logStringBuffer, &SerialDisplay, true);
}

void makeBootLog(LogLevel level, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(logStringBuffer, sizeof(logStringBuffer), format, args);
    va_end(args);

    (void)sendLogFrame(level, logStringBuffer, &SerialDisplay, true);
}

static inline void countTxFrame(Stream *port)
{
#if CONNECTION_LOGS
    if (port == &SerialLora)
        linkDiag.radioTxFrames++;
    else if (port == &SerialDisplay)
        linkDiag.displayTxFrames++;
#else
    (void)port;
#endif
}

static inline void countReplyTiming(Stream *port, uint32_t elapsedUs)
{
#if CONNECTION_LOGS
    if (port == &SerialLora)
    {
        linkDiag.radioReplyCount++;
        linkDiag.radioReplyUsSum += elapsedUs;
        if (elapsedUs > linkDiag.radioReplyUsMax)
            linkDiag.radioReplyUsMax = elapsedUs;
    }
    else if (port == &SerialDisplay)
    {
        linkDiag.displayReplyCount++;
        linkDiag.displayReplyUsSum += elapsedUs;
        if (elapsedUs > linkDiag.displayReplyUsMax)
            linkDiag.displayReplyUsMax = elapsedUs;
    }
#else
    (void)port;
    (void)elapsedUs;
#endif
}

void sendTelemetryPacket(Stream *port)
{
    if (!port || pendingBootloaderEntry)
        return;
    uint8_t         localTxBuffer[sizeof(FrameHeader) + sizeof(TelemetryPacket) + 2];
    TelemetryPacket pkt;
    populateTelemetryPacket(&pkt);

    FrameHeader *header       = (FrameHeader *)localTxBuffer;
    header->sync1             = PROTOCOL_SYNC_1_V2;
    header->sync2             = PROTOCOL_SYNC_2_V2;
    header->length            = sizeof(TelemetryPacket);
    header->targetID          = TARGET_ID_NONE;
    static uint8_t seqCounter = 0;
    header->seq               = seqCounter++;
    header->msgID             = MSG_HEARTBEAT;

    memcpy(localTxBuffer + sizeof(FrameHeader), &pkt, sizeof(TelemetryPacket));

    uint16_t totalLen = sizeof(FrameHeader) + header->length;
    ProtocolUtils::appendCRC(localTxBuffer, totalLen);

    if (writeTransportPayload(port, localTxBuffer, totalLen + 2, NULL) == totalLen + 2)
    {
        countTxFrame(port);
    }
}

void populateTelemetryPacket(TelemetryPacket *pkt, int16_t shuttleStatusOverride)
{
    if (!pkt)
        return;

    pkt->errorCode         = alertMan.getErrorCode();
    pkt->warningCode       = alertMan.getWarningCode();
    pkt->shuttleStatus     = (shuttleStatusOverride >= 0) ? (ShuttleState)shuttleStatusOverride : currentOperation;
    pkt->currentPosition   = currentPosition;
    pkt->speed             = speed;
    pkt->batteryCharge     = batteryCharge;
    pkt->batteryVoltage_mV = batteryBms.packVoltage_mV();
    pkt->shuttleNumber     = shuttleNum;
    pkt->palleteCount      = palleteCount;

    pkt->stateFlags = 0;
    if (lifterUp)
        pkt->stateFlags |= (1 << 0);
    if (motorStart)
        pkt->stateFlags |= (1 << 1);
    if (motorReverse)
        pkt->stateFlags |= (1 << 2);
    if (inverse)
        pkt->stateFlags |= (1 << 3);
    if (digitalRead(CHANNEL))
        pkt->stateFlags |= (1 << 4);
    if (fifoLifo)
        pkt->stateFlags |= (1 << 5);
}

void sendSensorPacket(Stream *port)
{
    if (!port || pendingBootloaderEntry)
        return;
    uint8_t localTxBuffer[sizeof(FrameHeader) + sizeof(SensorPacket) + 2];
    temp = 25 + ((float)(analogRead(ATEMP) * 3200) / 4096 - 760) / 2.5;

    SensorPacket pkt;
    pkt.distanceF      = distance[1];
    pkt.distanceR      = distance[0];
    pkt.distancePltF   = distance[3];
    pkt.distancePltR   = distance[2];
    writeBreadcrumb(BC_AS5600_ANGLE, 0xFF, 0, 0);
    pkt.angle          = readAs5600AngleForTelemetry();
    writeBreadcrumb(BC_AS5600_ANGLE, 0xFF, 0, 1);
    pkt.lifterCurrent  = lifterCurrent;
    pkt.temperature_dC = (int16_t)(temp * 10);

    pkt.hardwareFlags = 0;
    if (detectPalleteF1)
        pkt.hardwareFlags |= HW_FLAG_PALLET_F1;
    if (detectPalleteF2)
        pkt.hardwareFlags |= HW_FLAG_PALLET_F2;
    if (detectPalleteR1)
        pkt.hardwareFlags |= HW_FLAG_PALLET_R1;
    if (detectPalleteR2)
        pkt.hardwareFlags |= HW_FLAG_PALLET_R2;
    if (!digitalRead(BUMPER_F1))
        pkt.hardwareFlags |= HW_FLAG_BUMPER_F1;
    if (!digitalRead(BUMPER_F2))
        pkt.hardwareFlags |= HW_FLAG_BUMPER_F2;
    if (!digitalRead(BUMPER_R1))
        pkt.hardwareFlags |= HW_FLAG_BUMPER_R1;
    if (!digitalRead(BUMPER_R2))
        pkt.hardwareFlags |= HW_FLAG_BUMPER_R2;
    if (!digitalRead(DL_UP))
        pkt.hardwareFlags |= HW_FLAG_LIFTER_UP;
    if (!digitalRead(DL_DOWN))
        pkt.hardwareFlags |= HW_FLAG_LIFTER_DOWN;

    FrameHeader *header       = (FrameHeader *)localTxBuffer;
    header->sync1             = PROTOCOL_SYNC_1_V2;
    header->sync2             = PROTOCOL_SYNC_2_V2;
    header->length            = sizeof(SensorPacket);
    header->targetID          = TARGET_ID_NONE;
    static uint8_t seqCounter = 0;
    header->seq               = seqCounter++;
    header->msgID             = MSG_SENSORS;

    memcpy(localTxBuffer + sizeof(FrameHeader), &pkt, sizeof(SensorPacket));

    uint16_t totalLen = sizeof(FrameHeader) + header->length;
    ProtocolUtils::appendCRC(localTxBuffer, totalLen);

    if (writeTransportPayload(port, localTxBuffer, totalLen + 2, NULL) == totalLen + 2)
    {
        countTxFrame(port);
    }
}

void sendStatsPacket(Stream *port)
{
    if (!port || pendingBootloaderEntry)
        return;
    uint8_t     localTxBuffer[sizeof(FrameHeader) + sizeof(StatsPacket) + 2];
    StatsPacket pkt;
    pkt.totalDist               = sramStats->payload.totalDist;
    pkt.loadCounter             = sramStats->payload.loadCounter;
    pkt.unloadCounter           = sramStats->payload.unloadCounter;
    pkt.compactCounter          = sramStats->payload.compactCounter;
    pkt.liftUpCounter           = sramStats->payload.liftUpCounter;
    pkt.liftDownCounter         = sramStats->payload.liftDownCounter;
    pkt.lifetimePalletsDetected = sramStats->payload.lifetimePalletsDetected;
    pkt.totalUptimeMinutes      = sramStats->payload.totalUptimeMinutes;
    pkt.motorStallCount         = sramStats->payload.motorStallCount;
    pkt.lifterOverloadCount     = sramStats->payload.lifterOverloadCount;
    pkt.crashCount              = sramStats->payload.crashCount;
    pkt.resetWatchdogCount      = sramStats->payload.resetWatchdogCount;
    pkt.resetSoftwareCount      = sramStats->payload.resetSoftwareCount;
    pkt.resetPinCount           = sramStats->payload.resetPinCount;
    pkt.resetPowerCount         = sramStats->payload.resetPowerCount;
    pkt.resetOtherCount         = sramStats->payload.resetOtherCount;
    pkt.lowBatteryEvents        = sramStats->payload.lowBatteryEvents;
    pkt.lastResetFlags          = sramStats->payload.lastResetFlags;

    FrameHeader *header       = (FrameHeader *)localTxBuffer;
    header->sync1             = PROTOCOL_SYNC_1_V2;
    header->sync2             = PROTOCOL_SYNC_2_V2;
    header->length            = sizeof(StatsPacket);
    header->targetID          = TARGET_ID_NONE;
    static uint8_t seqCounter = 0;
    header->seq               = seqCounter++;
    header->msgID             = MSG_STATS;

    memcpy(localTxBuffer + sizeof(FrameHeader), &pkt, sizeof(StatsPacket));

    uint16_t totalLen = sizeof(FrameHeader) + header->length;
    ProtocolUtils::appendCRC(localTxBuffer, totalLen);

    if (writeTransportPayload(port, localTxBuffer, totalLen + 2, NULL) == totalLen + 2)
    {
        countTxFrame(port);
    }
}

static bool isRadioPacketRssiFresh(uint32_t now)
{
    if (!radioConfigOk || !e22Radio.hasLastPacketRssi())
    {
        return false;
    }
    const uint32_t sampleAt = e22Radio.lastPacketRssiAtMs();
    return sampleAt != 0 && (now - sampleAt <= kRadioPacketRssiFreshMs);
}

static void populateLinkHealthPacket(LinkHealthPacket *pkt)
{
    if (!pkt)
        return;

    const uint32_t now          = millis();
    const bool     hasFreshRssi = isRadioPacketRssiFresh(now);

    pkt->packetRssiRaw   = hasFreshRssi ? e22Radio.lastPacketRssiRaw() : 0;
    pkt->packetRssiDbm   = hasFreshRssi ? e22Radio.lastPacketRssiDbm() : 0;
    pkt->packetRssiAgeMs = hasFreshRssi ? (now - e22Radio.lastPacketRssiAtMs()) : 0xFFFFFFFFUL;
    pkt->flags           = 0;

    if (hasFreshRssi)
        pkt->flags |= LINK_HEALTH_RSSI_VALID;
    if (radioConfigOk)
        pkt->flags |= LINK_HEALTH_RADIO_CONFIG_OK;
    if (e22Radio.hasAuxPin())
        pkt->flags |= LINK_HEALTH_AUX_PRESENT;
    if (e22Radio.hasAuxPin() && e22Radio.isAuxHigh())
        pkt->flags |= LINK_HEALTH_AUX_HIGH;
}

static void sendLinkHealthPacket(Stream *port)
{
    if (!port || pendingBootloaderEntry)
        return;

    uint8_t          localTxBuffer[sizeof(FrameHeader) + sizeof(LinkHealthPacket) + 2];
    LinkHealthPacket pkt;
    populateLinkHealthPacket(&pkt);

    FrameHeader *header       = (FrameHeader *)localTxBuffer;
    header->sync1             = PROTOCOL_SYNC_1_V2;
    header->sync2             = PROTOCOL_SYNC_2_V2;
    header->length            = sizeof(LinkHealthPacket);
    header->targetID          = TARGET_ID_NONE;
    static uint8_t seqCounter = 0;
    header->seq               = seqCounter++;
    header->msgID             = MSG_LINK_HEALTH;

    memcpy(localTxBuffer + sizeof(FrameHeader), &pkt, sizeof(LinkHealthPacket));

    uint16_t totalLen = sizeof(FrameHeader) + header->length;
    ProtocolUtils::appendCRC(localTxBuffer, totalLen);

    if (writeTransportPayload(port, localTxBuffer, totalLen + 2, NULL) == totalLen + 2)
    {
        countTxFrame(port);
    }
}

#pragma endregion

#pragma endregion
