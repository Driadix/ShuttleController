#include <Wire.h> 
#include "AS5600.h"
#include <vl53l0x_class.h>
//#include <vl53l1_class.h>
//#include <vl53l4cd_class.h>
#include <String.h>
#include "STM32_CAN.h"
#include <EEPROM.h>
#include "STM32TimerInterrupt.h"
#include <IWatchdog.h>
#include <STM32RTC.h>


#pragma region Пины и дефайны...
#define XSHUT_F PE13        // Пин включения канального сенсора расстояния вперед PE13
#define XSHUT_R PE9         // Пин включения канального сенсора расстояния назад PE9
#define XSHUT_PF PE11       // Пин включения паллетного сенсора расстояния вперед PE11
#define XSHUT_PR PE7        // Пин включения паллетного сенсора расстояния назад PE7
#define DL_UP PD7           // Пин датчика положения лифтера в поднятом состоянии PD7
#define DL_DOWN PD15         // Пин датчика положения лифтера в опущенном состоянии PC7
#define DATCHIK_F1 PC7      // Пин датчика 1 обнаружения паллета вперед
#define DATCHIK_F2 PB3      // Пин датчика 2 обнаружения паллета вперед
#define DATCHIK_R1 PD5      // Пин датчика 1 обнаружения паллета назад
#define DATCHIK_R2 PD3      // Пин датчика 2 обнаружения паллета назад
#define GREEN_LED PD12      // Пин зеленого светодиода 
#define BUMPER_LED PD14       // Пин светодиода на плате
#define BOARD_LED PA1       // Пин светодиода на плате
#define RED_LED PD11        // Пин красного светодиода ошибки
#define WHITE_LED PD10      // Пин белого светодиода работы
#define ZOOMER PA15         // Зумер
#define LORA PA5            // Пин включения радиомодуля
#define RS485 PB15          // Пин передачи шины RS485
#define BUMPER_F PC6        // Пин бампера вперед
#define BUMPER_R PA8        // Пин бампера назад
#define CHANNEL PD1         // Пин датчика канала

#define RST PE2
#define BOOT PE4

#define SD_CLK_DIV 64
#define SDX_D0 PC8
#define SDX_D1 PC9
#define SDX_D2 PC10
#define SDX_D3 PC11
#define SDX_D0 PC8
#define SDX_CK PC12
#define SDX_CMD PD2

#define FLASH_SECTOR_SIZE 4 * 1024 // 16 kb
#define EEPROM_PAGE_SIZE 512
#define EEPROM_TOTAL_PAGES FLASH_SECTOR_SIZE / EEPROM_PAGE_SIZE
#define EEPROM_HEADER_SIZE 1
#define EEPROM_DATA_SIZE EEPROM_PAGE_SIZE - EEPROM_HEADER_SIZE
#pragma endregion

#pragma region Переменные...

VL53L0X sens_chnl_f(&Wire, XSHUT_F);             // Канальный сенсор расстояния вперед
VL53L0X sens_chnl_r(&Wire, XSHUT_R);             // Канальный сенсор расстояния назад
VL53L0X sens_plt_F(&Wire, XSHUT_PF);            // Паллетный сенсор расстояния вперед
VL53L0X sens_plt_R(&Wire, XSHUT_PR);            // Паллетный сенсор расстояния назад

VL53L0X *sensor_channel_f = &sens_chnl_f;        // Канальный сенсор расстояния вперед
VL53L0X *sensor_channel_r = &sens_chnl_r;        // Канальный сенсор расстояния назад
VL53L0X *sensor_pallete_F = &sens_plt_F;        // Паллетный сенсор расстояния вперед
VL53L0X *sensor_pallete_R = &sens_plt_R;        // Паллетный сенсор расстояния назад


HardwareSerial Serial2(PA3, PA2);               // Второй изолированный канал UART
HardwareSerial Serial3(PD9, PD8);               // Канал под RS485 для BMS батареи
STM32_CAN Can1 (CAN1, ALT, RX_SIZE_256, TX_SIZE_256); // CAN шина на пинах PB8 PB9
static CAN_message_t CAN_TX_msg;                // Пакет данных CAN на передачу
static CAN_message_t CAN_RX_msg;                // Пакет данных CAN на прием

STM32Timer ITimer0(TIM1);                       // Таймер для прерываний

STM32RTC& rtc = STM32RTC::getInstance();        // Часы реального времени

typedef union
{
  int vint;
  uint8_t bint[4];
}
cracked_int_t;                                // Структура конвертации типа float в массив байтов для отправки по CAN

typedef union
{
  float v;
  uint8_t b[4];
}
cracked_float_t;                                // Структура конвертации типа float в массив байтов для отправки по CAN

struct EEPROMData                             // Структура данных параметров для сохранения а EEProm
{
    uint8_t shuttleNum;
    uint8_t calibrateEncoder_F[8];
    uint8_t calibrateEncoder_R[8];
    uint8_t calibrateSensor_F[3];
    uint8_t calibrateSensor_R[3];
    int blinkTime;
    uint16_t maxSpeed;
    uint16_t minSpeed;
    uint16_t interPalleteDistance;
    uint8_t inverse;
    uint8_t fifoLifo;
    int lifter_Speed;
    uint32_t timingBudget;
    uint8_t minBattCharge;
    uint16_t shuttleLength;
    int waitTime;
    int8_t mprOffset;
    uint8_t TopLeftXF;
    uint8_t TopLeftYF;
    uint8_t BotRightXF;
    uint8_t BotRightYF;
    uint8_t TopLeftXR;
    uint8_t TopLeftYR;
    uint8_t BotRightXR;
    uint8_t BotRightYR;
    int8_t chnlOffset;
} eepromData;

struct EEPROMStat                             // Структура данных статистики для сохранения а EEProm
{
    uint32_t load;
    uint32_t unload;
    uint32_t compact;
    uint32_t liftUp;
    uint32_t liftDown;
    uint32_t totalDist;
} eepromStat;

AS5600 as5600;                                  // Магнитный энкодер на свободном колесе
const String shuttleStatus[28] = {"Stand_By", "Moove_Forward", "Moove_Back", "Lift_Up", "Lift_Down", "Moove_Stop", "Load_Pallete", "Unload_Pallete", "Moove_DistanceR", "Moove_DistanceF", "Calibrate_Sensors", 
                                  "Demo_Mode", "Pallete_Counting", "Save_EEProm", "Pallete_Compacting_F", "Pallete_Compacting_R", "Get_Parametrs", "Pallete_Sensor_Test", "TOF_Sensor_Test", "Error_Request",
                                  "Shuttle_Evacuate", "Long_Load", "Long_Unload", "Long_Quantity_Unload", "Error_Reset", "Manual_Mode", "Log_Mode", "Back_To_Start"}; // Расшифровка командных статусов
const String shuttleNums[26] = {"A1", "B2", "C3", "D4", "E5", "F6", "G7", "H8", "I9", "J10", "11", "12", "13", "14", "15", "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26" };  // Номера шаттлов 
const String shuttleErrors[16] = {"No_Error", "Channel_F_Error", "Channel_R_Error", "Channel_DF_Error", "Channel_DR_Error", "Pallete_F_Error", "Pallete_R_Error", "Pallete_DF_Error", "Pallete_DR_Error", "Lifter_Error", "Moove_Fault", "Low_Battery", "Crash"};  // Статусы ошибок
const String shuttleWarnings[16] = {"No_Warning", "Out_Of_Channel", "Battery charge < 20%", "Pallete_Not_Found", "Pallete_Damaged"};  // Статусы ошибок
uint8_t status = 0;                             // Командный статус
uint8_t errorStatus[16] = {0, };   // Номера статусов ошибки
uint8_t warningStatus = 0;                      // Номер статуса предупреждений
uint8_t shuttleNum = 0;                         // Номер шаттла -сохранять-
uint8_t channel[1024];                          // Массив канала
uint8_t calibrateEncoder_F[8] = {40, 40, 40, 40, 40, 40, 40, 40};            // Массив данных калибровки магнитного энкодера при движении вперед -сохранять-
uint8_t calibrateEncoder_R[8] = {40, 40, 40, 40, 40, 40, 40, 40};            // Массив данных калибровки магнитного энкодера при движении вперед -сохранять-
uint8_t calibrateSensor_F[3] = {100, 100, 100}; // Массив калибровки канального сенсора расстояния вперед -сохранять-
uint8_t calibrateSensor_R[3] = {100, 100, 100}; // Массив калибровки канального сенсора расстояния назад -сохранять-
String dataStr = "";                            // Строка для записи в логфайл

uint8_t counter = 0;        // Счетчик для красивых морганий в рабочем режиме
uint8_t debuger = 0;        // Счетчик для дебагера
uint8_t sensorNum = 0;      // Номер датчика для повторной инициализации
uint8_t pultConnect = 0;    // Флаг подключения пульта ДУ
uint8_t sensorFault = 1;    // Флаг сбоя сенсора TOF
uint8_t sensorOff = 0;      // Флаг отключения сенсоров в ручном режиме
uint8_t lastPallete = 0;    // Флаг позиции последнего паллета
uint8_t minBattCharge = 20; // Минимальный заряд батареи
uint8_t battCount = 0;      // Счетчик опроса батареи
uint8_t oldBattCharge = 0;  // Невалидированные значения заряда батареи
uint8_t endOfChannel = 0;   // Флаг концов канала
int blinkTime = 80;         // Время квантования красивых морганий -сохранять-
int waitTime = 15000;        // Время ожидания при выгрузке
int pingCount;              // Счетчик времени связи с пультом ДУ
int count = millis();       // Счетчик времени общего назначения
int count2 = count;      // Счетчик времени общего назначения
int countLora = count;   // Счетчик времени радиопередатчика
int countPult = count;   // Счетчик времени пульта
int countSensor = count; // Счетчик времени опроса датчиков
int countBatt = count;   // Счетчик времени батареи
int pageAddressData = 0;    // Адрес флэш памяти с параметрами
int pageAddressStat = 0;    // Адрес флэш памяти со статистикой
uint16_t speed = 0;         // Скорость движения в канале в %
uint16_t maxSpeed = 96;     // Максимальное значение скорости (от 0 до 100 %) -сохранять-
uint16_t minSpeed = 3;      // Минимальное значение скорости (лаг для АЦП) -сохранять-
uint16_t oldSpeed = 0;      // Запомненная cкорость движения в канале для плавного разгона и торможения
uint16_t errorCode = 0;     // Битмап ошибок
uint16_t SDsize = 0;        // Объем SD карты
uint16_t distance[8] = {0}; // Значения от сенсоров расстояния и датчиков положения
uint8_t sensitivity[8] = {0}; // Чувствительность сенсовров KCPS
uint16_t palletePosition[16] = {0}; // Массив расстояний до паллет в канале
uint16_t mooveDistance = 0; // Значение дистанции для перемещения шаттла по команде движения на заданное расстояние
uint16_t interPalleteDistance = 100; // Значение дистанции между паллетами -сохранять-
uint16_t shuttleLength = 1000;   // Длинна шаттла (максимальная ширина паллета которые шаттл может перевозить)
uint8_t detectPalleteF1;    // Флаг датчика обнаружения паллеты вперед 1
uint8_t detectPalleteF2;    // Флаг датчика обнаружения паллеты вперед 2
uint8_t detectPalleteR1;    // Флаг датчика обнаружения паллеты назад 1
uint8_t detectPalleteR2;    // Флаг датчика обнаружения паллеты назад 2
uint8_t palleteCount = 0;   // Счетчик паллет
uint8_t motorStart = false; // Флаг запуска двигателя движения
uint8_t motorReverse = 0;   // Флаг реверсивного (1) или прямого (0) движения
uint8_t turnFlag = 0;       // Флаг совершения оборота колесом шаттла
uint8_t lifterUp = 0;       // Флаг поднятой платформы
uint8_t inverse = 0;        // Флаг инверсии движения -сохранять-
uint8_t longWork = 0;       // Флаг продолжительной загрузки/выгрузки
uint8_t reportCounter = 0;  // Счетчик паузы репортов в секундах
uint8_t diffPallete = 0;    // Смещение для паллеты
uint8_t fifoLifo = 0;       // Режим FIFO/LIFO -сохранять-
uint8_t evacuate = 0;       // Режим эвакуации
uint8_t batteryCharge = 0;  // Заряд батареи
uint8_t palletQuant = 0;    // Количество подсчитанных паллет
uint8_t UPQuant = 0;        // Количество паллет на выгрузку
uint8_t load = 0;           // Оценка массы нагрузки шаттла 0 - 100
uint8_t mooveCount = 0;     // Счетчик пробксовки при малой скорости
int8_t mprOffset = 0;       // Смещение значения МПР
int8_t chnlOffset = 0;      // Смещение в конце канала
float batteryVoltage = 0;   // Напряжение на батарее
int lifterCurrent = 0;      // Ток лифтера для оценки массы поднимаемого груза
int angle = 0;              // Значение угла полученное от магнитного экодера
int oldAngle = 0;           // Промежуточное значение угла для расчетов
int oldAngle2 = 0;          // Промежуточное значение угла для расчетов
int startAngle = 0;         // Промежуточное значение угла для расчетов
int finishAngle = 0;        // Промежуточное значение угла для расчетов
int speedAngle = 0;         // Угловая скорость (для рассчетов)
int oldDistance = 0;        // Промежуточное значение дистации для расчетов
int turnCount = 0;          // Счетчик оборотов колеса 
int turnTime = 0;           // Время оборота, для расчетов
int maxTurnCount = 0;       // Счетчик оборотов колеса на весь канал
int channelLength = 0;      // Расчетная длинна канала
int currentPosition = 0;    // Текущая позиция шаттла
int oldPosition = 0;        // Позиция для определения останова шаттла 
int startPosition = 0;      // Стартовая позиция шаттла
int finishPosition = 0;     // Финишная позиция шаттла
int lastPalletePosition = 0;// Позиция последнего паллета после загрузки
int firstPalletePosition = 0; // Позиция первого паллета при уплотнении вперед
int lifter_Speed = 3700;    // Скорость двигателя лифтера -сохранять-
int oldChannelDistanse = 0; // Канальная дистанция для фильтрации фантомных срабатываний
int oldPalleteDistanse = 0; // Паллетная дистанция для фильтрации фантомных срабатываний
uint32_t timingBudget = 70; // Время измерения датчиками TOF -сохранять-

uint16_t mesRes[2][4];
int countManual = millis();
int countCrush = 0;
int startDiff = 0;
#pragma endregion

// the setup function runs once when you press reset or power the board
void setup() 
{
  IWatchdog.begin(30000000);
  pinMode(XSHUT_F, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);  
  pinMode(XSHUT_PF, OUTPUT);
  pinMode(XSHUT_PR, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BOARD_LED, OUTPUT);
  pinMode(BUMPER_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(WHITE_LED, OUTPUT);
  pinMode(ZOOMER, OUTPUT);
  pinMode(LORA, OUTPUT);
  pinMode(RS485, OUTPUT);
  pinMode(DATCHIK_F1, INPUT_PULLUP);
  pinMode(DATCHIK_R1, INPUT_PULLUP);
  pinMode(DATCHIK_F2, INPUT_PULLUP);
  pinMode(DATCHIK_R2, INPUT_PULLUP);
  pinMode(DL_UP, INPUT_PULLUP);
  pinMode(DL_DOWN, INPUT_PULLUP);
  pinMode(BUMPER_F, INPUT);
  pinMode(BUMPER_R, INPUT);
  pinMode(CHANNEL, INPUT);

  pinMode(RST, INPUT);
  pinMode(BOOT, INPUT);

  attachInterrupt(BUMPER_F, crash, FALLING);
  attachInterrupt(BUMPER_R, crash, FALLING);

  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BOARD_LED, HIGH);
  digitalWrite(BUMPER_LED, HIGH);
  digitalWrite(RED_LED, LOW);
  digitalWrite(WHITE_LED, LOW);
  digitalWrite(ZOOMER, LOW);
  digitalWrite(RS485, LOW);
  pinMode(RS485, INPUT_PULLDOWN);

  digitalWrite(XSHUT_F, LOW);
  digitalWrite(XSHUT_R, LOW);
  digitalWrite(XSHUT_PF, LOW);
  digitalWrite(XSHUT_PR, LOW);
  
  Serial.begin(115200);               // USBшный UART порт
  Serial1.begin(230400, SERIAL_8E1);  // UART порт на пинах РА2 РА3, на экранчик (LILYGO-S3)   
  Serial2.begin(9600);                // UART порт на пинах РА9 РА10, используется для радиомодуля
  Serial3.begin(9600);                // UART порт на пинах РD9 РD8, RS485 батареи

  rtc.setClockSource(STM32RTC::LSE_CLOCK);
  rtc.begin();

  if (!rtc.isTimeSet())               // Инициируем часы RTC
  {
    rtc.setTime(0, 0, 0);
    rtc.setDate(1, 1, 1, 23);
    Serial.println("RTC initialized with default date and time.");
  }

   // Чтение параметров из EEProm
  for (uint8_t i = 0; i < 8; i++)
  {
    eepromData.calibrateEncoder_F[i] = calibrateEncoder_F[i];
    eepromData.calibrateEncoder_R[i] = calibrateEncoder_R[i];
  }
  for (uint8_t i = 0; i < 3; i++)
  {
    eepromData.calibrateSensor_F[i] = calibrateSensor_F[i];
    eepromData.calibrateSensor_R[i] = calibrateSensor_R[i];
  }
  eepromData.shuttleNum = shuttleNum;
  eepromData.blinkTime = blinkTime;
  eepromData.maxSpeed = maxSpeed;
  eepromData.minSpeed = minSpeed;
  eepromData.interPalleteDistance = interPalleteDistance;
  eepromData.inverse = inverse;
  eepromData.fifoLifo = fifoLifo;
  eepromData.lifter_Speed = lifter_Speed;
  eepromData.timingBudget = timingBudget;
  eepromData.minBattCharge = minBattCharge;
  eepromData.shuttleLength = shuttleLength;
  eepromData.waitTime = waitTime;
  eepromData.mprOffset = mprOffset;
  eepromData.TopLeftXF = 4;
  eepromData.TopLeftYF = 12;
  eepromData.BotRightXF = 12;
  eepromData.BotRightYF = 4;
  eepromData.TopLeftXR = 4;
  eepromData.TopLeftYR = 12;
  eepromData.BotRightXR = 12;
  eepromData.BotRightYR = 4;
  eepromData.chnlOffset = chnlOffset;

  if (readEEPROMData(eepromData))
  {
    for (uint8_t i = 0; i < 8; i++)
    {
      calibrateEncoder_F[i] = eepromData.calibrateEncoder_F[i];
      calibrateEncoder_R[i] = eepromData.calibrateEncoder_R[i];
    }
    for (uint8_t i = 0; i < 3; i++)
    {
      calibrateSensor_F[i] = eepromData.calibrateSensor_F[i];
      calibrateSensor_R[i] = eepromData.calibrateSensor_R[i];
    }
    shuttleNum = eepromData.shuttleNum;
    blinkTime = eepromData.blinkTime;
    maxSpeed = eepromData.maxSpeed;
    minSpeed = eepromData.minSpeed;
    interPalleteDistance = eepromData.interPalleteDistance;
    inverse = eepromData.inverse;
    fifoLifo = eepromData.fifoLifo;
    lifter_Speed = eepromData.lifter_Speed;
    timingBudget = eepromData.timingBudget;
    minBattCharge = eepromData.minBattCharge;
    shuttleLength = eepromData.shuttleLength;
    waitTime = eepromData.waitTime;
    mprOffset = eepromData.mprOffset;
    chnlOffset = eepromData.chnlOffset;
  }
  else saveEEPROMData(eepromData);
  if (minBattCharge > 50) minBattCharge = 20;
  if (waitTime < 5000) waitTime = 5000;
  else if (waitTime > 30000) waitTime = 30000;

  eepromStat.load = 0;
  eepromStat.unload = 0;
  eepromStat.compact = 0;
  eepromStat.liftUp = 0;
  eepromStat.liftDown = 0;
  eepromStat.totalDist = 0;

  analogReadResolution(12);
  delay (2000);
    
  dataStr = "Start init board...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  
  Can1.begin();
  Can1.setBaudRate(500000);
  CAN_TX_msg.flags.extended = 1;
  CAN_RX_msg.flags.extended = 1;
     
  Wire.setSDA(PB11);
  Wire.setSCL(PB10);
  Wire.setClock(100000);
  Wire.begin();

  // Инициируем магнитный энкодер
  as5600.begin(4);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.  
  dataStr = "Init encoder success...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  
  // Прописываем параметры связи радиомодуля
  digitalWrite(LORA, HIGH);
  delay(100);
  dataStr = "Write LoRa settings...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
    
  Serial2.write(0xC0); // C0 - сохранить настройки, C2 - сбросить после отключения от питания
  Serial2.write(256);  // Верхний байт адреса. Если оба байта 0xFF - передача и прием по всем адресам на канале
  Serial2.write(256);  // Нижний байт адреса. Если оба байта 0xFF - передача и прием по всем адресам на канале
  Serial2.write(0x1C); // Параметры скорости
  Serial2.write(0x10); // Канал (частота), 0x00 - 410 МГц, шаг частоты - 2 МГц 
  Serial2.write(0x46); // Служебные опции
  delay(100);
  digitalWrite(LORA, LOW);
 
  Serial.println("Total struct size = " + String(sizeof(EEPROMData) + sizeof(EEPROMStat)) + "  " + String(sizeof(EEPROMData)));
  delay (100);
  
  // Инициируем сенсоры расстояния
  ITimer0.enableTimer();
  ITimer0.attachInterruptInterval(timingBudget * 2000, NULL);
  sensor_channel_f->dev = 0x60;
  initialise_Sensor(sensor_channel_f);
  sensor_channel_r->dev = 0x62;
  initialise_Sensor(sensor_channel_r);
  sensor_pallete_F->dev = 0x64;
  initialise_Sensor(sensor_pallete_F);
  sensor_pallete_R->dev = 0x66;
  initialise_Sensor(sensor_pallete_R);
  if (inverse) {sensor_channel_f = &sens_chnl_r;  sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F;}
  ITimer0.detachInterrupt();
  ITimer0.disableTimer();

  read_BatteryCharge();
  if (!batteryCharge) {delay(200); read_BatteryCharge();}
  if (!batteryCharge) {delay(200); read_BatteryCharge();}
  uint8_t newBC = batteryCharge;
  delay(100);
  read_BatteryCharge();
  if (batteryCharge != newBC || batteryCharge == 11) {if (batteryCharge != 0 || batteryCharge != 11) newBC = batteryCharge; delay(200); read_BatteryCharge();}
  if (batteryCharge != newBC || batteryCharge == 11) {delay(250); read_BatteryCharge();}
  if (!batteryCharge) batteryCharge = newBC;
  Serial1.println("CB " + String(batteryCharge));
  Serial1.println("CV " + String(batteryVoltage));
  //sensor_Report();
  if (digitalRead(DL_DOWN) && digitalRead(CHANNEL)) lifter_Down();

  delay(50);
  char report[256];
  float temp = 25 + ((float)(analogRead(ATEMP) * 3200) / 4096 - 760) / 2.5;
  uint8_t hour, minute, second, day, month, year, weekDay;
  rtc.getTime(&hour, &minute, &second, 0, nullptr);
  rtc.getDate(&weekDay, &day, &month, &year);
  snprintf(report, sizeof(report), "Time %02d:%02d:%02d, Date:%02d/%02d/%02d",hour, minute, second, day, month, year);
  dataStr = String(report);
  dataStr += "  Temperature = " + String(temp);
  Serial.println(dataStr);
  Serial1.println(dataStr);
}
  
// the loop function runs over and over again forever

void loop()
{
  if (!errorStatus[0] && status == 25)
  {
    uint8_t statusTmp = 0;
    if (digitalRead(CHANNEL)) statusTmp = get_Cmd_Manual();
    if (statusTmp == 1) {moove_Right(); countManual = millis();}
    else if (statusTmp == 2) {moove_Left(); countManual = millis();}
    else if (statusTmp == 3) {lifter_Up(); countManual = millis();}
    else if (statusTmp == 4) {lifter_Down(); countManual = millis();}
    else if (statusTmp == 6) {status = 6; run_Cmd();}
    else if (statusTmp == 7) {status = 7; run_Cmd();}
    else if (statusTmp == 21) {status = 21; run_Cmd();}
    else if (statusTmp == 22) {status = 22; run_Cmd();}

    if(millis() - countManual > 120000) status = 0;
    if (millis() - count > 330) 
    {
      digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
      digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));
      count = millis();
      IWatchdog.reload();
      while (Can1.read(CAN_RX_msg));
      while (Serial1.available()) Serial1.read();
      get_Distance();
      reportCounter++;
      if (reportCounter == 3)
      {
        Serial.println(" ");
        Serial1.println(" ");
        delay(5);
        dataStr = "manual mode...";
        Serial.println(dataStr);
        Serial1.println(dataStr);
        reportCounter = 0;
      }
    }
  }
  else if (!errorStatus[0])
  {
    uint8_t statusTmp = 0;
    uint8_t inChannel = digitalRead(CHANNEL);
    delay(25);
    inChannel = digitalRead(CHANNEL) && inChannel;
    delay(25);
    inChannel = digitalRead(CHANNEL) && inChannel;
    delay(25);
    inChannel = digitalRead(CHANNEL) && inChannel;
    delay(25);
    inChannel = digitalRead(CHANNEL) && inChannel;
    
    statusTmp = get_Cmd();
    
    if (statusTmp != status)
    {
      dataStr = "Shuttle status CMD changed = " + String(shuttleStatus[statusTmp]) + "  (" + String(statusTmp) + ")";
      Serial.println(dataStr);
      Serial1.println(dataStr);
      if (!inChannel && (statusTmp == 13 || statusTmp == 16 || statusTmp == 17 || statusTmp == 19 || statusTmp == 24)) status = statusTmp;
      else if (!inChannel) status = 0;
      else if (inChannel) status = statusTmp;
      send_Cmd();
      run_Cmd();
    }
    if (digitalRead(WHITE_LED)) digitalWrite(WHITE_LED, LOW);

    if (status == 16 && millis() - count2 > timingBudget + 10) {send_Cmd(); count2 = millis();}

    Can1.read(CAN_RX_msg);
      
    if (millis() - count > 1000 && !motorStart) 
    {
      counter = 0;
      digitalWrite(GREEN_LED, !digitalRead(GREEN_LED));
      digitalWrite(BOARD_LED, !digitalRead(BOARD_LED));
      reportCounter++;
      count = millis();
      IWatchdog.reload();
      //sensor_Report();
      if (reportCounter == 10)
      {
        sensor_Report();
        reportCounter = 0;
      }
      send_Cmd();
      if ((reportCounter == 3 || reportCounter == 8)) read_BatteryCharge();
      else {Serial1.println("online..."); delay(20); Serial1.println("CB " + String(batteryCharge)); delay(20); Serial1.println("CV " + String(batteryVoltage));}
      if (status == 16 || status == 17 || status == 19 || status == 13) status = 0;
      uint8_t oldStatus = status; status = 16; if (status != 25) send_Cmd(); status = oldStatus;
      if (!inChannel) Serial2.print(shuttleNums[shuttleNum] + "wc001!");
      else if (batteryCharge < 20) Serial2.print(shuttleNums[shuttleNum] + "wc002!");
      else Serial2.print(shuttleNums[shuttleNum] + "wc000!");
    }
    if (millis() - countPult > 100000 && pultConnect) {pultConnect = 0; status = 0;}
    //else if (millis() - countPult > 180000 && distance[1] > 150 && inChannel) {Serial1.println("Disconnect time off, dist = " + String(distance[1]) + " in channel = " + String(digitalRead(CHANNEL))); moove_Forward();}
  }
  else 
  {
    dataStr = "Shuttle ERROR !!! Errors:";
    Serial.println(dataStr);
    Serial1.println(dataStr);
    IWatchdog.reload();
    Can1.read(CAN_RX_msg);
    if( digitalRead(WHITE_LED)) digitalWrite(WHITE_LED, LOW);
    uint8_t i = 0;
    while (errorStatus[i]) 
    {
      dataStr = shuttleErrors[errorStatus[i]];
      Serial.println(dataStr);
      Serial1.println(dataStr);
      i++;
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    count = millis();
    while (millis() - count < 200)
    {
      if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    count = millis();
    while (millis() - count < 200)
    {
     if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    count = millis();
    while (millis() - count < 200)
    {
     if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    count = millis();
    while (millis() - count < 200)
    {
     if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    IWatchdog.reload();
    count = millis();
    while (millis() - count < 200)
    {
      if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    count = millis();
    while (millis() - count < 200)
    {
     if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BOARD_LED, LOW);
    count = millis();
    while (millis() - count < 200)
    {
      if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BOARD_LED, HIGH);
    count = millis();
    while (millis() - count < 200)
    {
      if (status = get_Cmd() == 24)
      {
        uint8_t i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if(status == 100) status = 19;
    }
    status = 19;
    send_Cmd();
    sensor_Report();
    digitalWrite(GREEN_LED, LOW);
    IWatchdog.reload();
    count = millis();
    while (millis() - count < 1000)
    {
      if (status = get_Cmd() == 24)
      {
        i = 0;
        while (errorStatus[i]) {errorStatus[i] = 0; i++;}
        errorCode = 0;
      }
      else if (status == 100) status = 19;
      
      
      i = 0;
      while (errorStatus[i])
      {
        if (batteryCharge > minBattCharge && errorStatus[i] == 11) errorStatus[i] = 0;
        //if (digitalRead(BUMPER_F) && digitalRead(BUMPER_R) && errorStatus[i] == 12) errorStatus[i] = 0;
        i++;
      }
    }
    if (!errorStatus[0]) digitalWrite(RED_LED, LOW);
    status = 16;
    send_Cmd();
    status = 19;
    read_BatteryCharge();
  }
  if (millis() - countSensor > timingBudget + 5) 
  {
    countSensor = millis();
    get_Distance();
    //Serial.println("Time of get distances  = " + String(millis() - countSensor));
    

    /*if (mesRes[0][0] == distance[0] && distance[0] != 1500 && distance[0] != 0) mesRes[1][0]++;
    else mesRes[1][0] = 0;
    mesRes[0][0] = distance[0];
    if (mesRes[1][0] >= 10) {mesRes[1][0] = 0; if (inverse) sensorNum == 0; else sensorNum == 1; Serial.println("Reinit sensor 0...");}

    if (mesRes[0][1] == distance[1] && distance[1] != 1500 && distance[1] != 0) mesRes[1][1]++;
    else mesRes[1][1] = 0;
    mesRes[0][1] = distance[1];
    if (mesRes[1][1] >= 10) {mesRes[1][1] = 0; if (inverse) sensorNum == 1; else sensorNum == 0; Serial.println("Reinit sensor 1...");}

    if (mesRes[0][2] == distance[2] && distance[2] != 1500 && distance[2] != 0) mesRes[1][2]++;
    else mesRes[1][2] = 0;
    mesRes[0][2] = distance[2];
    if (mesRes[1][2] >= 10) {mesRes[1][2] = 0; if (inverse) sensorNum == 2; else sensorNum == 3; Serial.println("Reinit sensor 2...");}

    if (mesRes[0][3] == distance[3] && distance[3] != 1500 && distance[3] != 0) mesRes[1][3]++;
    else mesRes[1][3] = 0;
    mesRes[0][3] = distance[3];
    if (mesRes[1][3] >= 10) {mesRes[1][3] = 0; if (inverse) sensorNum == 3; else sensorNum == 2; Serial.println("Reinit sensor 3...");}*/
  }
}

// the other functions
#pragma region Функции...

#pragma region Функции управления двигателем движения...
void motor_Speed(int spd)
{ 
  if (motorReverse == 2) return;
  while (Can1.read(CAN_RX_msg));
  cracked_int_t hexSpeed;
  CAN_TX_msg.id = (100);
  CAN_TX_msg.len = 4;
  uint8_t accel = 30;
  int position;
  if (spd >= 10 && spd - oldSpeed >= 10) spd = oldSpeed + 10;
  if (spd > 100) spd = 100;
  if (spd <= 100 && spd >= 0) 
  {    
    if (spd > oldSpeed)
    {
      uint8_t steps = (spd - oldSpeed) / 2;
      for (uint8_t i = 0; i < steps; i++) 
      {
        blink_Work();
        hexSpeed.vint = minSpeed + oldSpeed * maxSpeed / 100  + (spd - oldSpeed) * i * maxSpeed / (steps * 100);
        if (motorReverse ^ inverse) hexSpeed.vint = -hexSpeed.vint;
        hexSpeed.vint *= 1000;
        for (uint8_t j = 0; j < 4; j++) {CAN_TX_msg.buf[j] = (unsigned char)hexSpeed.bint[3 - j];}
        Can1.write(CAN_TX_msg);
        set_Position();
        count = millis();
        if (i > 2 && i <= 40 && lifterUp) accel = 80 - i * 15 / 10;
        else accel = 35;
        while (millis() - count < accel)
        {
          blink_Work();
          if (status != 25 && get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
        }
      }
      if (spd) hexSpeed.vint = minSpeed + spd * maxSpeed / 100;
      else hexSpeed.vint = 0;
      if (motorReverse ^ inverse) hexSpeed.vint = -hexSpeed.vint;
      hexSpeed.vint *= 1000;
      for (uint8_t i = 0; i < 5; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
      Can1.write(CAN_TX_msg);
      while (Can1.read(CAN_RX_msg));
      oldSpeed = spd;
    }
    else if (spd < oldSpeed)
    {
      uint8_t steps = (oldSpeed - spd) / 2;
      float currentSpeed;
      for (uint8_t i = 0; i < steps; i++) 
      {
        blink_Work();
        hexSpeed.vint = minSpeed + oldSpeed * maxSpeed / 100 - (oldSpeed - spd) * i * maxSpeed / (steps * 100);
        if (motorReverse ^ inverse) hexSpeed.vint = -hexSpeed.vint;
        hexSpeed.vint *= 1000;
        for (uint8_t j = 0; j < 4; j++) {CAN_TX_msg.buf[j] = (unsigned char)hexSpeed.bint[3 - j];}
        Can1.write(CAN_TX_msg);
        set_Position();  
        count = millis();
        if (lifterUp) accel = 20 + i * 30 / steps;
        while (millis() - count < accel)
        {
          if (status != 25 && get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
          if (status == 25) get_Cmd_Manual();
          if (motorReverse == 2) return;
        }
      }
      if (spd) hexSpeed.vint = minSpeed + spd * maxSpeed / 100;
      else hexSpeed.vint = 0;
      if (motorReverse ^ inverse) hexSpeed.vint = -hexSpeed.vint;
      hexSpeed.vint *= 1000;
      for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
      Can1.write(CAN_TX_msg);
      while (Can1.read(CAN_RX_msg));
      oldSpeed = spd;
    }
    else if (spd == oldSpeed)
    {
      hexSpeed.vint = minSpeed + spd * maxSpeed / 100;
      if (motorReverse ^ inverse) hexSpeed.vint = -hexSpeed.vint;
      hexSpeed.vint *= 1000;
      for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
      Can1.write(CAN_TX_msg);
      while (Can1.read(CAN_RX_msg));
    }
  }
  else
  {
    hexSpeed.vint = 0;
    for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
    Can1.write(CAN_TX_msg);
    while (Can1.read(CAN_RX_msg));
    oldSpeed = 0;
  }

  set_Position();
}

void motor_Start_Forward()
{
  motorStart = 1;
  motorReverse = 0;
}

void motor_Start_Reverse()
{
  motorStart = 1;
  motorReverse = 1;
}

void motor_Stop()
{
  if (motorReverse == 2) return;
  Can1.read(CAN_RX_msg);
  Serial.println("Motor stop, speed = " + String(oldSpeed));
  Serial1.println("Motor stop, speed = " + String(oldSpeed));
  CAN_TX_msg.id = (100);
  CAN_TX_msg.len = 4;
  cracked_int_t hexSpeed;  
  if (oldSpeed > 1) 
  {
    uint8_t maxi = (uint8_t)oldSpeed / 2;
    for (uint8_t i = maxi; i > 0; i--) 
    {
      blink_Work();
      hexSpeed.vint = minSpeed + oldSpeed * maxSpeed * i / (maxi * 100);
      hexSpeed.vint *= 1000;
      if (motorReverse ^ inverse) hexSpeed.vint = -hexSpeed.vint;
      for (uint8_t j = 0; j < 4; j++) {CAN_TX_msg.buf[j] = (unsigned char)hexSpeed.bint[3 - j];}
      Can1.write(CAN_TX_msg);
      while (Can1.read(CAN_RX_msg));
      if ((status == 22 || status == 7) && lifterUp && distance[3] + 100 < distance[1]) delay(10);
      else if (maxi > 10) delay(10 + i * 20 / maxi);
      else delay(50);
    }
  }
  set_Position();
  hexSpeed.vint = 0;
  for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
  Can1.write(CAN_TX_msg);
  while (Can1.read(CAN_RX_msg));
  if ((status == 22 || status == 7) && lifterUp && distance[3] + 100 < distance[1])
  {
    for (uint8_t i = 0; i < 10; i++)
    {
      count = millis();
      while (millis() - count < 100) blink_Work();
      Can1.write(CAN_TX_msg);
      while (Can1.read(CAN_RX_msg));
    }
  }
  else 
  {
     for (uint8_t i = 0; i < 1; i++)
    {
      count = millis();
      while (millis() - count < 100) blink_Work();
      Can1.write(CAN_TX_msg);
      while (Can1.read(CAN_RX_msg));
    }
  }
  oldSpeed = 0;
  motorStart = 0; 
  motorReverse = 2;
  mooveCount = 0;
  oldPosition = currentPosition;
}

void motor_Force_Stop()
{
  cracked_int_t hexSpeed;
  oldSpeed = 0;
  hexSpeed.vint = 0;
  CAN_TX_msg.id = (100);
  CAN_TX_msg.len = 4;
  for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
  Can1.write(CAN_TX_msg);
  while (Can1.read(CAN_RX_msg));
  motorStart = 0; 
  motorReverse = 2;
  mooveCount = 0;
  oldPosition = currentPosition;
}
#pragma endregion

#pragma region Функции управления лифтером
void lifter_Up()
{
  Serial.println(" ");
  Serial.println("Moove lifter up...");
  Serial1.println(" ");
  Serial1.println("Moove lifter up...");
  int k = 0;
  int summCurrent = 0;
  int current = 0;
  cracked_int_t hexSpeed;
  int cnt = millis();
  int cnt2 = millis();
  hexSpeed.vint = -50;
  hexSpeed.vint *= 1000;
  CAN_TX_msg.id = (101);
  CAN_TX_msg.len = 4;
  for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
  if (digitalRead(DL_UP)) Can1.write(CAN_TX_msg);
  while (digitalRead(DL_UP)) 
  {
    if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}
    if (millis() - cnt > 5000) break;
    delay(10);
    Can1.write(CAN_TX_msg);
    blink_Work();
    
    while (Can1.read(CAN_RX_msg))
    {
      if (!CAN_RX_msg.flags.remote && CAN_RX_msg.id == 2405) 
      {
        current = CAN_RX_msg.buf[4] * 256 + CAN_RX_msg.buf[5];
        //Serial1.println("Current UP = " + String(current) + " : " + String(CAN_RX_msg.buf[0]) + " " +  String(CAN_RX_msg.buf[1]) + " " +  String(CAN_RX_msg.buf[2]) + " " +  String(CAN_RX_msg.buf[3]));
        if (lifterCurrent < current && k > 3) lifterCurrent = current; 
        k++;
        if (k > 3) summCurrent += current;
        cnt2 = millis();
      }
    }
  }
  Can1.write(CAN_TX_msg);
  cnt = millis();
  if(digitalRead(DL_UP)) 
  {
    while (digitalRead(DL_UP)) 
    {
      if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}
      if (millis() - cnt > 5000) {lifter_Stop(); add_Error(9); status = 5; break;}
      delay(10);
      Can1.write(CAN_TX_msg);
      while (Can1.read(CAN_RX_msg)); {}
      blink_Work();
    }
  }
  if (digitalRead(DL_DOWN)) lifterUp = 1;
  lifter_Stop();
  summCurrent /= k;
  if (lifterCurrent > 500) lifterCurrent = 250;
  Serial1.println("Summ = " + String(summCurrent));
}

void lifter_Down()
{ 
  Serial.println(" ");
  Serial.println("Moove lifter down... status = " + String(status));
  Serial1.println(" ");
  Serial1.println("Moove lifter down...");
  int cnt = millis();
  cracked_int_t hexSpeed;
  hexSpeed.vint = 50000;
  CAN_TX_msg.id = (101);
  CAN_TX_msg.len = 4;
  for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
  if (digitalRead(DL_DOWN)) Can1.write(CAN_TX_msg);
  while (digitalRead(DL_DOWN)) 
  {
    if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}
    if (millis() - cnt > 5000) break;
    delay(10);
    Can1.write(CAN_TX_msg);
    blink_Work();
  }  
  cnt = millis();
  if(digitalRead(DL_DOWN)) 
  {
    while (digitalRead(DL_DOWN))
    {
      if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}
      if (millis() - cnt > 5000) {lifter_Stop(); add_Error(9); status = 5; break;}
      delay(10);
      Can1.write(CAN_TX_msg);
      blink_Work();
    }
  }
  if (!digitalRead(DL_DOWN)) lifterUp = 0;
  lifter_Stop();
  load = 0;
  lifterCurrent = 0;
}

void lifter_Stop()
{
  Serial.println("Stop lifter...");
  Serial1.println("Stop lifter...");
  cracked_int_t hexSpeed;
  hexSpeed.vint = 0;
  CAN_TX_msg.id = (101);
  CAN_TX_msg.len = 4;
  for (uint8_t i = 0; i < 4; i++) {CAN_TX_msg.buf[i] = (unsigned char)hexSpeed.bint[3 - i];}
  Can1.write(CAN_TX_msg);
}
#pragma endregion

#pragma region Служебные функции 

uint8_t get_Cmd() //Запрос команд с пульта ДУ
{
  if (millis() - countLora < 20) { return status;}
  int8_t inByte = 0;
  int8_t data = 0;
  uint8_t distf = 1;
  uint8_t statusTmp = 100;
  countLora = millis();
  String inStr = "";
  if (Serial2.available()) // Получаем команду
  {
    inByte = Serial2.read();
    char inChar = (char) inByte;
    inStr += inChar;
    delayMicroseconds(1750);
    inByte = Serial2.read();
    inChar = (char) inByte;
    inStr += inChar;
    while(inStr != shuttleNums[shuttleNum] && Serial2.available())
    {
      inStr = inStr.substring(1,2);
      delayMicroseconds(1750);
      inByte = Serial2.read();
      inChar = (char) inByte;
      inStr += inChar;
    }
    if (inStr == shuttleNums[shuttleNum]) 
    {
      data = 1;
      int cnt = millis();
      while(inStr.length() <= 7 && millis() - cnt < 50)
      {
        delayMicroseconds(100);
        if (Serial2.available())
        {
          inByte = Serial2.read();
          inChar = (char) inByte;
          inStr += inChar;
        }
      }
      //if (inStr.length() == 8) while (Serial2.available()) Serial2.read();
    }
  }
  if (data)
  {
    //Serial1.println(inStr);
    //Serial.println(inStr);
    String tempStr = inStr.substring(0,2);
    if (tempStr == shuttleNums[shuttleNum])
    {
      if (!pultConnect) {digitalWrite(ZOOMER, HIGH); delay(500); digitalWrite(ZOOMER, LOW); pultConnect = 1;}
      //if (!pultConnect) {zoomer(500); pultConnect = 1;}
      countPult = millis();
      tempStr = inStr.substring(2,8);
      if (tempStr == "dCharg") send_Cmd();                         // Коннект с пультом
      //else if (tempStr == "dRight") {Serial2.print(inStr + "!"); if (status == 25 && millis() - pingCount > 2000) {moove_Right();} else if (status != 25) statusTmp = 1;}              // Движение вперед
      //else if (tempStr == "dLeft_") {Serial2.print(inStr + "!"); if (status == 25 && millis() - pingCount > 2000) {moove_Left();} else if (status != 25) statusTmp = 2;}               // Движение назад
      else if (tempStr == "dUp___") {Serial2.print(inStr + "!"); if (status == 25) lifter_Up(); else statusTmp = 3;}                 // Поднять платформу вверх
      else if (tempStr == "dDown_") {Serial2.print(inStr + "!"); if (status == 25) lifter_Down(); else statusTmp = 4;}               // Опустить платфоорму
      else if (tempStr == "dStop_") {statusTmp = 5; Serial2.print(inStr + "!"); diffPallete = 0;}               // Остановка
      else if (tempStr == "dLoad_") {Serial2.print(inStr + "!"); statusTmp = 6;}               // Выгрузка паллеты 
      else if (tempStr == "dUnld_") {Serial2.print(inStr + "!"); statusTmp = 7;}               // Загрузка паллеты
      else if (tempStr == "dClbr_") {Serial2.print(inStr + "!"); statusTmp = 10;}              // Калибровка
      else if (tempStr == "dDemo_") {Serial2.print(inStr + "!"); statusTmp = 11;}              // Демо режим
      else if (tempStr == "dGetQu") {Serial2.print(inStr + "!"); statusTmp = 12;}              // Подсчет паллет
      else if (tempStr == "dSaveC") {Serial2.print(inStr + "!"); statusTmp = 13;}              // Сохранение параметров на флэш память
      else if (tempStr == "dComFo") {Serial2.print(inStr + "!"); statusTmp = 14;}              // Уплотнение паллет вперед
      else if (tempStr == "dComBa") {Serial2.print(inStr + "!"); statusTmp = 15;}              // Уплотнение паллет назад
      else if (tempStr == "dSGet_") {Serial2.print(inStr + "!"); statusTmp = 16; send_Cmd();}  // Запрос параметров из настроек
      else if (tempStr == "dDataP") {Serial2.print(inStr + "!"); statusTmp = 17; send_Cmd();}  // Запрос данных из отладки
      else if (tempStr == "tError") {Serial2.print(inStr + "!"); statusTmp = 19;}              // Запрос ошибок
      else if (tempStr == "dEvOn_") {Serial2.print(inStr + "!"); statusTmp = 20; evacuate = 1;}// Включение режима эвакуации
      else if (tempStr == "dLLoad") {Serial2.print(inStr + "!"); statusTmp = 21;}              // Продолжительная загрузка
      else if (tempStr == "dLUnld") {Serial2.print(inStr + "!"); statusTmp = 22;}              // Продолжительная выгрузка
      else if (tempStr == "dReset") {Serial2.print(inStr + "!"); statusTmp = 24;}              // Сброс ошибок
      else if (tempStr == "dManua") {Serial2.print(inStr + "!"); statusTmp = 25; countManual = millis(); send_Cmd();}  // Ручной режим
      else if (tempStr == "dGetLg") {Serial2.print(inStr + "!"); statusTmp = 26; send_Cmd();}  // Журналирование
      else if (tempStr == "dHome_") {Serial2.print(inStr + "!"); statusTmp = 27;}              // В начало канала
      else if (tempStr == "dWaitT") {Serial2.print(inStr + "!"); statusTmp = 29;}              // Время ожидания при загрузке
      else if (tempStr == "dMprOf") {Serial2.print(inStr + "!"); statusTmp = 30;}              // Запрос смещения МПР
      else if (tempStr == "dEvOff") {Serial2.print(inStr); evacuate = 0;}                // Выключение режима эвакуации
      else if (tempStr == "ngPing") {if (millis() - pingCount < 800) pingCount = millis();}        // Удержание движения
      else if (tempStr == "dFIFO_") {Serial2.print(inStr + "!"); fifoLifo = 0; eepromData.fifoLifo = fifoLifo;}                // Установка режима FIFO
      else if (tempStr == "dLIFO_") {Serial2.print(inStr + "!"); fifoLifo = 1; eepromData.fifoLifo = fifoLifo;}                // Установка режима LIFO
      else if (tempStr == "dRevOn") {Serial2.print(inStr + "!"); inverse = 0; eepromData.inverse = inverse; sensor_channel_f = &sens_chnl_f;  sensor_channel_r = &sens_chnl_r; sensor_pallete_F = &sens_plt_F; sensor_pallete_R = &sens_plt_R; currentPosition = channelLength - currentPosition - 800;}
      else if (tempStr == "dReOff") {Serial2.print(inStr + "!"); inverse = 1; eepromData.inverse = inverse; sensor_channel_f = &sens_chnl_r;  sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F; currentPosition = channelLength - currentPosition - 800;}

      tempStr = inStr.substring(2,5);
      if (tempStr == "dNN") {shuttleNum = inStr.substring(5,8).toInt() - 1;  eepromData.shuttleNum = shuttleNum;}  // Установка номера шаттла
      else if (tempStr == "dQt") {UPQuant = inStr.substring(5,8).toInt(); statusTmp = 23;}      // Выгрузка заданного количества паллет
      else if (tempStr == "dDm") {interPalleteDistance = inStr.substring(5,8).toInt(); eepromData.interPalleteDistance = interPalleteDistance;}    // Установка межпаллетного расстояния
      else if (tempStr == "dSl") {shuttleLength = inStr.substring(5,8).toInt() * 10; eepromData.shuttleLength = shuttleLength;}                    // Установка длинны шаттла
      else if (tempStr == "dSp")                                                                // Установка максимальной скорости
      {
        maxSpeed = inStr.substring(5,8).toInt();
        if (maxSpeed > 96) maxSpeed = 96;
        if (maxSpeed < minSpeed) maxSpeed = minSpeed + 5;
        eepromData.maxSpeed = maxSpeed;
      }
      else if (tempStr == "dBc")                                                                // Установка минимального заряда батареи
      {
        minBattCharge = inStr.substring(5,8).toInt();
        if (minBattCharge > 50) minBattCharge = 50;
        if (minBattCharge < 0) minBattCharge = 0;
        eepromData.minBattCharge = minBattCharge;
      }
      else if (tempStr == "dMr")                                                                // Движение назад на заданное расстояние
      {
        statusTmp = 8;
        mooveDistance = inStr.substring(5,8).toInt() * 10;
      }
      else if (tempStr == "dMf")                                                                // Движение вперед на заданное расстояние
      {
        statusTmp = 9;
        mooveDistance = inStr.substring(5,8).toInt() * 10;
      }
      else if (tempStr == "dWt")                                                                // Время ожидания при выгрузке
      {
        statusTmp = 0;
        waitTime = inStr.substring(5,7).toInt() * 1000;
        eepromData.waitTime = waitTime;
      }
      else if (tempStr == "dMo")                                                                // Смещение МПР
      {
        statusTmp = 0;
        mprOffset = (int8_t)inStr.substring(5,8).toInt() - 100;
        eepromData.mprOffset = mprOffset;
      }
      else if (tempStr == "dMc")                                                                // Смещение конца канала
      {
        statusTmp = 0;
        chnlOffset = (int8_t)inStr.substring(5,8).toInt() - 100;
        eepromData.chnlOffset = chnlOffset;
      }
    }
  }
  
  inByte = 0;
  int8_t inBytes[30];
  uint8_t i = 0;
  inStr = "";
  data = 0;
  while(Serial1.available()) // Получаем команду
  {
    inByte = Serial1.read();
    inBytes[i] = inByte;
    i++;
    char inChar = (char) inByte;
    inStr += inChar;
    data = 1;
    delayMicroseconds(1750);
    if (inStr.length() > 30) break;
  }
  if (data)
  {
    //Serial.println("String from LiLyGo:  " + inStr);
    //Serial1.print("GS " + String(shuttleNum));
    String tempStr = inStr.substring(0,2);
    if (tempStr == shuttleNums[shuttleNum])
    {
      countPult = millis();
      tempStr = inStr.substring(2,8);
      if (tempStr == "dCharg") send_Cmd();                         // Коннект с пультом
      else if (tempStr == "dRight") {statusTmp = 1;}               // Движение вперед
      else if (tempStr == "dLeft_") {statusTmp = 2;}               // Движение назад
      else if (tempStr == "dUp___") {statusTmp = 3;}               // Поднять платформу вверх
      else if (tempStr == "dDown_") {statusTmp = 4;}               // Опустить платфоорму
      else if (tempStr == "dStop_") {statusTmp = 5; diffPallete = 0;}               // Остановка
      else if (tempStr == "dLoad_") {statusTmp = 6;}               // Выгрузка паллеты 
      else if (tempStr == "dUnld_") {statusTmp = 7;}               // Загрузка паллеты
      else if (tempStr == "dClbr_") {statusTmp = 10;}              // Калибровка
      else if (tempStr == "dDemo_") {statusTmp = 11;}              // Демо режим
      else if (tempStr == "dGetQu") {statusTmp = 12;}              // Подсчет паллет
      else if (tempStr == "dSaveC") {statusTmp = 13;}              // Сохранение параметров на флэш память
      else if (tempStr == "dComFo") {statusTmp = 14;}              // Уплотнение паллет вперед
      else if (tempStr == "dComBa") {statusTmp = 15;}              // Уплотнение паллет назад
      else if (tempStr == "dSpGet") {statusTmp = 16; send_Cmd();}  // Запрос параметров из настроек
      else if (tempStr == "dDataP") {statusTmp = 17; send_Cmd();}  // Запрос данных из отладки
      else if (tempStr == "tError") {statusTmp = 19; send_Cmd();}  // Запрос ошибок
      else if (tempStr == "dEvOn_") {statusTmp = 20; evacuate = 1;}// Включение режима эвакуации
      else if (tempStr == "dLLoad") {statusTmp = 21;}              // Продолжительная загрузка
      else if (tempStr == "dLUnld") {statusTmp = 22;}              // Продолжительная выгрузка
      else if (tempStr == "dReset") {statusTmp = 24;}              // Сброс ошибок
      else if (tempStr == "dManua") {statusTmp = 25; countManual = millis(); send_Cmd();}  // Ручной режим
      else if (tempStr == "dHome_") {statusTmp = 27;}              // В начало канала
      else if (tempStr == "dWaitT") {statusTmp = 29;}              // Время ожидания при загрузке
      else if (tempStr == "dEvOff") {evacuate = 0;}                // Выключение режима эвакуации
      else if (tempStr == "ngPing") pingCount = millis();          // Удержание движения
      else if (tempStr == "dLIFO_") {fifoLifo = 0;}                // Установка режима LIFO
      else if (tempStr == "dFIFO_") {fifoLifo = 1;}                // Установка режима FIFO
      else if (tempStr == "dRevOn") {inverse = 0; eepromData.inverse = inverse; sensor_channel_f = &sens_chnl_f;  sensor_channel_r = &sens_chnl_r; sensor_pallete_F = &sens_plt_F; sensor_pallete_R = &sens_plt_R; currentPosition = channelLength - currentPosition - 800;}
      else if (tempStr == "dReOff") {inverse = 1; eepromData.inverse = inverse; sensor_channel_f = &sens_chnl_r;  sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F; currentPosition = channelLength - currentPosition - 800;}

      tempStr = inStr.substring(2,5);
      if (tempStr == "dNN") {shuttleNum = inStr.substring(5,8).toInt() - 1;  eepromData.shuttleNum = shuttleNum;}  // Установка номера шаттла
      else if (tempStr == "dQt") {UPQuant = inStr.substring(5,8).toInt(); statusTmp = 23;}  // Выгрузка заданного количества паллет
      else if (tempStr == "dDm") {interPalleteDistance = inStr.substring(5,8).toInt(); eepromData.interPalleteDistance = interPalleteDistance;}    // Установка межпаллетного расстояния
      else if (tempStr == "dSl") {shuttleLength = inStr.substring(5,8).toInt() * 10; eepromData.shuttleLength = shuttleLength;}                    // Установка длинны шаттла
      else if (tempStr == "dSp")                                                                // Установка максимальной скорости
      {
        maxSpeed = inStr.substring(5,8).toInt();
        if (maxSpeed > 96) maxSpeed = 96;
        if (maxSpeed < minSpeed) maxSpeed = minSpeed + 5;
        eepromData.maxSpeed = maxSpeed;
      }
      else if (tempStr == "dBc")                                                                // Установка минимального заряда батареи
      {
        minBattCharge = inStr.substring(5,8).toInt();
        if (minBattCharge > 50) minBattCharge = 50;
        if (minBattCharge < 0) minBattCharge = 0;
        eepromData.minBattCharge = minBattCharge;
      }
       else if (tempStr == "dMr")                                                                // Движение назад на заданное расстояние
      {
        statusTmp = 8;
        mooveDistance = inStr.substring(5,8).toInt() * 10;
      }
      else if (tempStr == "dMf")                                                                // Движение вперед на заданное расстояние
      {
        statusTmp = 9;
        mooveDistance = inStr.substring(5,8).toInt() * 10;
      }
      else if (tempStr == "dWt")                                                                // Время ожидания при выгрузке
      {
        statusTmp = 0;
        waitTime = inStr.substring(5,7).toInt() * 1000;
        eepromData.waitTime = waitTime;
      }
      else if (tempStr == "dMo")                                                                // Смещение МПР
      {
        statusTmp = 0;
        mprOffset = (int8_t)inStr.substring(5,8).toInt() - 100;
        eepromData.mprOffset = mprOffset;
      }
      else if (tempStr == "dMc")                                                                // Смещение конца канала
      {
        statusTmp = 0;
        chnlOffset = (int8_t)inStr.substring(5,8).toInt() - 100;
        eepromData.chnlOffset = chnlOffset;
      }
    }
    else if (tempStr == "GS")
    {
      Serial1.print("GS " + String(shuttleNum));
      delay(5);
      Serial1.print("GS " + String(shuttleNum));
    }
    else if (tempStr == "DT")
    {
      String input = inStr.substring(3,23);
      int hour, minute, second, day, month, year;
      if (sscanf(input.c_str(), "%d:%d:%d %d.%d.%d", &hour, &minute, &second, &day, &month, &year) == 6) {
      if (isValidDateTime(hour, minute, second, day, month, year)) {
        rtc.setTime(hour, minute, second);
        rtc.setDate(getWeekDay(day, month, year), day, month, year - 2000);
        Serial.println("RTC updated successfully.");
      } else {
        Serial.println("Invalid date/time format or value.");
      }
    } else {
      Serial.println("Invalid format. Use: HH:MM:SS DD/MM/YYYY");
    }
    }
  }

 if (statusTmp == 100) return status;
 else return statusTmp;
}

uint8_t get_Cmd_Manual() //Запрос команд с пульта ДУ
{
  int8_t inByte = 0;
  int8_t data = 0;
  char inChar;
  String inStr = "";
  countLora = millis();
  if (Serial2.available()) // Получаем команду
  {
    inByte = Serial2.read();
    inChar = (char) inByte;
    inStr += inChar;
    delayMicroseconds(1750);
    inByte = Serial2.read();
    inChar = (char) inByte;
    inStr += inChar;
    while(inStr != shuttleNums[shuttleNum] && Serial2.available())
    {
      inStr = inStr.substring(1,2);
      delayMicroseconds(1750);
      inByte = Serial2.read();
      inChar = (char) inByte;
      inStr += inChar;
    }
    if (inStr.substring(0,2) == shuttleNums[shuttleNum]) 
    {
      int cnt = millis();
      while(inStr.length() <= 7 && millis() - cnt < 50)
      {
        delayMicroseconds(1750);
        if (Serial2.available())
        {
          inByte = Serial2.read();
          inChar = (char) inByte;
          inStr += inChar;
        }
      }
      if (inStr.length() == 8) data = 1;
    }
  }
  if (data)
  {
    String tempStr = inStr.substring(2,8);
    //if (!pultConnect) {digitalWrite(ZOOMER, HIGH); delay(500); digitalWrite(ZOOMER, LOW); pultConnect = 1;}
    if (!pultConnect) {digitalWrite(ZOOMER, HIGH); delay(500); digitalWrite(ZOOMER, LOW); pultConnect = 1;}
    countPult = millis();
    if (tempStr == "dCharg") send_Cmd();                                // Коннект с пультом
    else if (tempStr == "dRight") {send_Cmd(); return 1;}               // Движение вперед
    else if (tempStr == "dLeft_") {send_Cmd(); return 2;}               // Движение назад
    else if (tempStr == "dUp___") {send_Cmd(); return 3;}               // Поднять платформу вверх
    else if (tempStr == "dDown_") {send_Cmd(); return 4;}               // Опустить платфоорму
    else if (tempStr == "dLoad_") {send_Cmd(); return 6;}               // Выгрузка паллеты 
    else if (tempStr == "dUnld_") {send_Cmd(); return 7;}               // Загрузка паллеты
    else if (tempStr == "dLLoad") {send_Cmd(); return 21;}              // Продолжительная загрузка
    else if (tempStr == "dLUnld") {send_Cmd(); return 22;}              // Продолжительная выгрузка
    else if (tempStr == "dStop_") {status = 5; send_Cmd(); return status;}                                              // Остановка
    else if (tempStr == "dStopM") {send_Cmd(); return 55;}               // Остановка в ручном режиме
    else if (tempStr == "ngPing") {send_Cmd(); pingCount = millis(); return 100;}        // Удержание движения
  }
  return 0;
}

void run_Cmd() // Выполнение команд с пульта ДУ
{
  if (status == 1) {send_Cmd(); moove_Reverse(); status = 0; send_Cmd();}
  else if (status == 2) {send_Cmd(); moove_Forward(); status = 0; send_Cmd();}
  else if (status == 3) {send_Cmd(); lifter_Up(); status = 0; send_Cmd();}  
  else if (status == 4) {send_Cmd(); lifter_Down(); status = 0; send_Cmd();}
  else if (status == 5) {if (motorStart) motor_Stop(); status = 0; send_Cmd();}
  else if (status == 6) {send_Cmd(); if (lifterUp) lifter_Down(); single_Load(); status = 0; send_Cmd();}
  else if (status == 7) {send_Cmd(); if (lifterUp) lifter_Down(); moove_Forward(); unload_Pallete(); status = 2; moove_Forward(); status = 0; send_Cmd();}
  else if (status == 8) {send_Cmd(); moove_Distance_R(mooveDistance); status = 0; send_Cmd();}
  else if (status == 9) {send_Cmd(); moove_Distance_F(mooveDistance); status = 0; send_Cmd();}
  else if (status == 10) {send_Cmd(); calibrate_Encoder_F(); calibrate_Encoder_R(); if (status != 5) moove_Forward(); status = 0; send_Cmd();}
  else if (status == 11) {demo_Mode(); firstPalletePosition = 0;}
  else if (status == 12) {pallete_Counting_F(); status = 0; send_Cmd(); status = 12; dataStr = "Pallete count = " + String(palleteCount) + " " + String(currentPosition); Serial.println(dataStr);
                         Serial1.println(dataStr); status = 2; moove_Forward(); status = 0; send_Cmd();}
  else if (status == 13) saveEEPROMData(eepromData);
  else if (status == 14) {send_Cmd(); pallete_Compacting_F(); status = 0; if (status != 5) moove_Forward(); send_Cmd();}
  else if (status == 15) {send_Cmd(); pallete_Compacting_R(); firstPalletePosition = 0; status = 0; if (status != 5) moove_Forward(); send_Cmd();}
  else if (status == 21) {send_Cmd(); if (lifterUp) lifter_Down(); moove_Forward(); long_Load(); if (status != 5) moove_Forward(); status = 0; send_Cmd();}
  else if (status == 22) {send_Cmd(); longWork = 1; if (lifterUp) lifter_Down(); moove_Forward(); long_Unload(); longWork = 0; if (status == 5 && distance[1] > 100 || errorStatus[0]) return; status = 2; moove_Forward(); status = 0; send_Cmd();}
  else if (status == 23) {send_Cmd(); longWork = 1; Serial.println("Pallet quant = " + String(palletQuant)); if (lifterUp) lifter_Down(); moove_Forward(); status = 23; long_Unload(UPQuant); UPQuant = 0; longWork = 0; if (status == 5 && distance[1] > 100 || errorStatus[0]) return; status = 2; moove_Forward(); status = 0; send_Cmd();}
  else if (status == 27) {moove_Reverse(); status = 0; send_Cmd();}
}

void send_Cmd()
{
  char report[64];
  String tempStr = shuttleNums[shuttleNum] + "t1" + fifoLifo;
  uint8_t mpro = 100 + mprOffset;
  uint8_t chnlo = 100 + chnlOffset;   
  if (!errorStatus[0])
  {
    switch (status)
    {
      case 0: // Ожидание
        Serial2.print(tempStr + "10:" + batteryCharge + ":" + palleteCount + "!");
        Serial2.print(shuttleNums[shuttleNum] + "t3" + String(palleteCount) + "!");
        break;
      case 1: // Ручной движение вперед
        Serial2.print(tempStr + "14:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 2: // Ручной движение назад
        Serial2.print(tempStr + "15:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 3: // Ручной поднятие платформы
        Serial2.print(tempStr + "16:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 4: // Ручной опускание платформы
        Serial2.print(tempStr + "17:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 5: // Ожидание (Стоп режим)
        Serial2.print(tempStr + "10:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 6:  // Загрузка
        Serial2.print(tempStr + "2:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 7: // Выгрузка
        Serial2.print(tempStr + "3:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 11: // Демо режим
        Serial2.print(tempStr + "6:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 12: // Подсчет паллет
        Serial2.print(tempStr + "7:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 13: // Подсчет паллет
        Serial2.print(shuttleNums[shuttleNum] + "t3" + String(palleteCount) + "!");
        break;
      case 14: // Уплотнение вперед
        Serial2.print(tempStr + "4:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 15: // плотнение вперед
        Serial2.print(tempStr + "4:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 16: // Передача данных из настроек
        Serial2.print(shuttleNums[shuttleNum] + "t4" + inverse + String(maxSpeed) + ":" + interPalleteDistance + ":" + minBattCharge + ":" + String(shuttleLength) + "!");
        delay(20);
        tempStr = shuttleNums[shuttleNum] + "wo";
        if (mpro < 10) tempStr += "00" + String(mpro);
        else if (mpro < 100) tempStr += "0" + String(mpro);
        else tempStr += String(mpro);
        tempStr += ":";
        if (chnlo < 9) tempStr += "00" + String(chnlo);
        else if (chnlo < 99) tempStr += "0" + String(chnlo);
        else tempStr += String(chnlo);
        Serial2.print(tempStr + "!");
        delay(20);
        tempStr = shuttleNums[shuttleNum] + "wt";
        if (waitTime < 10000) tempStr += "0" + String(waitTime / 1000);
        else tempStr += String(waitTime / 1000);
        Serial2.print(tempStr + "!");
        break;
      case 17: // Передача данных отладки
        detect_Pallete();
        get_Distance();
        tempStr = shuttleNums[shuttleNum] + "yt%4d:%4d:%4d:%4d:%4d:%1d:%1d:%1d:%1d!";
        snprintf(report, sizeof(report), tempStr.c_str(), distance[0], distance[1], 
                 distance[2], distance[3], as5600.readAngle(), detectPalleteF1, detectPalleteF2, 
                 detectPalleteR1, detectPalleteR2);
        Serial2.print(report);
        //Serial.println("DataP string: " + String(report));
        break;
      case 19: // Выгрузка ошибок
        tempStr = shuttleNums[shuttleNum] + "rc";
        for (uint8_t i = 0; i < sizeof(errorStatus); i++) bitWrite(errorCode, errorStatus[i], 1);
        Serial2.print(tempStr + "0" + errorCode + ":0!");
        break;
      case 21: // Продолжительная загрузка
        Serial2.print(tempStr + "11:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 22: // Продолжительная выгрузка
        Serial2.print(tempStr + "12:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 23: // Продолжительная выгрузка
        //Serial2.print(tempStr + "13:" + batteryCharge + ":" + palleteCount + "!");
        //delay(20);
        tempStr = shuttleNums[shuttleNum] + "pq";
        Serial2.print(tempStr + "0" + String(UPQuant) + "!");
        break;
      case 25: // Ручной режим
        Serial2.print(tempStr + "1:" + batteryCharge + ":" + palleteCount + "!");
        break;
      case 29: // Время ожидания при загрузке
        tempStr = shuttleNums[shuttleNum] + "wt";
        if (waitTime < 10000) tempStr += "0" + String(waitTime / 1000);
        else tempStr += String(waitTime / 1000);
        Serial2.print(tempStr + "!");
        break;
      case 30: // Смещение МПР
        tempStr = shuttleNums[shuttleNum] + "wo";
        if (mpro < 9) tempStr += "00" + String(mpro);
        else if (mpro < 99) tempStr += "0" + String(mpro);
        else tempStr += String(mpro);
        tempStr += ":";
        if (chnlo < 9) tempStr += "00" + String(chnlo);
        else if (chnlo < 99) tempStr += "0" + String(chnlo);
        else tempStr += String(chnlo);
        Serial2.print(tempStr + "!");
        break;
    }
  }
  else
  {
    Serial2.print(tempStr + "9:" + batteryCharge + ":" + palleteCount + "!");
    delay(50);
    tempStr = shuttleNums[shuttleNum] + "rc";
    for (uint8_t i = 0; i < sizeof(errorStatus); i++) bitWrite(errorCode, errorStatus[i], 1);
    bitWrite(errorCode, 0, 1);
    Serial2.print(tempStr + "0" + errorCode + ":0!");
  }  
}

void sensor_Report() // Опрос сенсоров
{
  //Serial.println("Start reporting...");
  char report[64];

  uint8_t NewDataReady = 0;
  int statusS;
  angle = as5600.readAngle();
  dataStr = "-----------------------------------------------";
  Serial.println(dataStr);
  Serial1.println(dataStr);
    
  float temp = 25 + ((float)(analogRead(ATEMP) * 3200) / 4096 - 760) / 2.5;
  uint8_t hour, minute, second, day, month, year, weekDay;
  rtc.getTime(&hour, &minute, &second, 0, nullptr);
  rtc.getDate(&weekDay, &day, &month, &year);
  snprintf(report, sizeof(report), "Time:%02d:%02d:%02d, Date:%02d/%02d/%02d",hour, minute, second, day, month, year);
  dataStr = String(report);
  dataStr += "  Temperature = " + String(temp);
  Serial.println(dataStr);
  Serial1.println(dataStr);
    
  snprintf(report, sizeof(report), "Angle = %d | Lenght = %d | position = %d ", angle, channelLength, currentPosition);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "Batt voltage = " + String(batteryVoltage) + "V Charge = " + String(batteryCharge) + "% limit = " + String(minBattCharge) + "%";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = "Inverse = ";
  if (inverse) dataStr += "YES   FIFO_LIFO = ";
  else dataStr += "NO   FIFO_LIFO = ";
  if (fifoLifo) dataStr += "LIFO";
  else dataStr += "FIFO";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  snprintf(report, sizeof(report), "Forwrd dist = %d | Revrs dist = %d ", distance[1], distance[0]);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  snprintf(report, sizeof(report), "Forwrd plt dist = %d | Revrs plt dist = %d ", distance[3], distance[2]);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  detect_Pallete();
  snprintf(report, sizeof(report), "Plt dtchk F1 = %d | Plt dtchk F2 = %d ", detectPalleteF1, detectPalleteF2);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  snprintf(report, sizeof(report), "Plt dtchk R1 = %d | Plt dtchk R2 = %d ", detectPalleteR1, detectPalleteR2);
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = "In channel: ";
  if (digitalRead(CHANNEL)) dataStr += "YES";
  else dataStr += "NO";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = "Lifter UP: ";
  if (digitalRead(DL_UP)) dataStr += "NO | Lifter DOWN: ";
  else dataStr += "YES | Lifter DOWN: ";
  if (!digitalRead(DL_DOWN)) dataStr += "YES";
  else dataStr += "NO";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  delay(20);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  snprintf(report, sizeof(report), "Bumper forward = %d | Bumper reverse = %d ", digitalRead(BUMPER_F), digitalRead(BUMPER_R));
  dataStr = String(report);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "MPR = " + String(interPalleteDistance) + " max speed = " + String(maxSpeed);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "Wait time on unload = " + String(waitTime / 1000) + " Sec";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = "Zero point MPR = " + String(mprOffset) + " channel offset = " + String(chnlOffset);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  dataStr = " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (sizeof(shuttleStatus) > status)
  {
    dataStr = "Status = " + String(shuttleStatus[status]) + "  (" + String(status) + ")";
    Serial.println(dataStr);
    Serial1.println(dataStr);
      }
  else
  {
    dataStr = "Shuttle status num = " + String(status);
    Serial.println(dataStr);
    Serial1.println(dataStr);
      }
  delay(20);
  Serial.println("Shuttle number = " + String(shuttleNum + 1) + " Shuttle length = " + String(shuttleLength));
  Serial1.println("Shuttle number = " + String(shuttleNum + 1) + " Shuttle length = " + String(shuttleLength));
 
  dataStr = "-----------------------------------------------";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  Serial1.println("CB " + String(batteryCharge));
  if (!errorStatus[0] && (batteryCharge > 0 && batteryCharge <= minBattCharge)) {if (lifterUp) lifter_Down(); moove_Forward(); add_Error(11); status = 5; return;}
  Serial1.println("CV " + String(batteryVoltage));
}

void add_Error(uint8_t error) // Добавление ошибки
{
  uint8_t i = 0;
  while (errorStatus[i]) {if (errorStatus[i] == error) return; i++;}
  errorStatus[i] = error;
}

void detect_Pallete() // Получение данных с датчиков обраружения паллет
{
  if (inverse)
  {
    detectPalleteF1 = digitalRead(DATCHIK_R1);
    detectPalleteF2 = digitalRead(DATCHIK_R2);
    detectPalleteR1 = digitalRead(DATCHIK_F1);
    detectPalleteR2 = digitalRead(DATCHIK_F2);
    delay(5);
    if (digitalRead(DATCHIK_R1) != detectPalleteF1) {delay(5); detectPalleteF1 = digitalRead(DATCHIK_R1);}
    if (digitalRead(DATCHIK_R2) != detectPalleteF2) {delay(5); detectPalleteF2 = digitalRead(DATCHIK_R2);}
    if (digitalRead(DATCHIK_F1) != detectPalleteR1) {delay(5); detectPalleteR1 = digitalRead(DATCHIK_F1);}
    if (digitalRead(DATCHIK_F2) != detectPalleteR2) {delay(5); detectPalleteR2 = digitalRead(DATCHIK_F2);}
  }
  else
  {
    detectPalleteF1 = digitalRead(DATCHIK_F1);
    detectPalleteF2 = digitalRead(DATCHIK_F2);
    detectPalleteR1 = digitalRead(DATCHIK_R1);
    detectPalleteR2 = digitalRead(DATCHIK_R2);
    delay(5);
    if (digitalRead(DATCHIK_F1) != detectPalleteF1) {delay(5); detectPalleteF1 = digitalRead(DATCHIK_F1);}
    if (digitalRead(DATCHIK_F2) != detectPalleteF2) {delay(5); detectPalleteF2 = digitalRead(DATCHIK_F2);}
    if (digitalRead(DATCHIK_R1) != detectPalleteR1) {delay(5); detectPalleteR1 = digitalRead(DATCHIK_R1);}
    if (digitalRead(DATCHIK_R2) != detectPalleteR2) {delay(5); detectPalleteR2 = digitalRead(DATCHIK_R2);}
  }
}

uint16_t get_Distance(VL53L0X *sensor) //Получение данных о расстоянии от сенсоров VL53L0X
{
  uint16_t dst = 0;
  VL53L0X_RangingMeasurementData_t pRangingMeasurementData;

  uint16_t dist;
  if (sensor->dev == 0x60) {sensorNum = 0; if (inverse) dist = distance[0]; else dist = distance[1];}
  else if (sensor->dev == 0x62) {sensorNum = 1; if (!inverse) dist = distance[0]; else dist = distance[1];}
  else if (sensor->dev == 0x64) {sensorNum = 2; if (inverse) dist = distance[2]; else dist = distance[3];}
  else {sensorNum = 3; if (!inverse) dist = distance[2]; else dist = distance[3];}

  ITimer0.enableTimer();
  if (!ITimer0.attachInterruptInterval(timingBudget * 2000, reinitialise_Sensor)) 
  {
    dataStr = "Can't set ITimer0...";
    Serial.println(dataStr);
    Serial1.println(dataStr);
  }
  
  uint8_t st = sensor->GetRangingMeasurementData(&pRangingMeasurementData);
  dst = pRangingMeasurementData.RangeMilliMeter;
  uint16_t cps = pRangingMeasurementData.SignalRateRtnMegaCps / 64;  
  sensor->ClearInterruptMask(0);

  if (pRangingMeasurementData.RangeStatus == 3) {Serial.println("none interrupt"); reinitialise_Sensor();}

  ITimer0.detachInterrupt();
  ITimer0.disableTimer();

  if (pRangingMeasurementData.RangeStatus == 0 && cps > 60 && dst > 800 && dst <= 1500) {}
  else if (pRangingMeasurementData.RangeStatus == 1 && dst > 1200 && dst <= 1500) {}
  else if (pRangingMeasurementData.RangeStatus == 0 && cps > 150 && dst > 400 && dst <= 800) {}
  else if (pRangingMeasurementData.RangeStatus == 0 && cps > 700 && dst > 40 && dst <= 400) {}
  else if (pRangingMeasurementData.RangeStatus == 0 && (cps > 2000 && dst <= 40)) dst = 40;
  else if (pRangingMeasurementData.RangeStatus != 0 && dst <= 1200) dst = dist + 10;
  else dst = 1500;
  if (motorStart && oldSpeed > 30 && ((dist > 1200 && dst < 400))) dst = dist;
  if (((sensorNum == 0 && inverse) || (sensorNum == 1 && !inverse) && dst < distance[2] + 100 &&  dst > distance[2] - 40)) dst = dist + 10;
  else if (((sensorNum == 1 && inverse) || (sensorNum == 0 && !inverse) && dst < distance[3] + 100 &&  dst > distance[3] - 40)) dst = dist +10;
  if (dst > 1500) dst = 1500;
  return dst;
}

void get_Distance()
{
  distance[0] = get_Distance(sensor_channel_r);
  distance[1] = get_Distance(sensor_channel_f);
  distance[2] = get_Distance(sensor_pallete_R);
  distance[3] = get_Distance(sensor_pallete_F);
}

void demo_Mode() //Демо режим
{
  dataStr = "Start DEMO mode...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (!digitalRead(DL_DOWN)) lifter_Down();
  moove_Reverse();
  if (status == 5 || errorStatus[0]) return;  
  get_Distance();
  count = millis(); 
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
  get_Distance();
  detect_Pallete();
  uint8_t moove = 0;
  while (distance[3] < 1000)
  {
    moove_Distance_F(10, 25, 25);
    motor_Speed(oldSpeed);
    moove = 1;
  }
  //if (moove) moove_Distance_F(shuttleLength + 500, 40, 60);
  
  while (1)
  {
    if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    while (status !=5) 
    {
      count = millis();
      while(millis() - count < 1000 && !moove) 
      {
        if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
        blink_Work();
      }
      moove = 0;
      load_Pallete();
      sensor_Report();
      if (errorStatus[0]) {motor_Stop(); return;}
      if (distance[1] < 200) status = 5;
    }
    if (status == 5 && distance[1] > 200 || errorStatus[0]) {motor_Stop(); return;}
    else status = 11;
    motor_Stop();
    count = millis();
    read_BatteryCharge();
    if (batteryCharge > 0 && batteryCharge <= minBattCharge) {add_Error(11); if (lifterUp) lifter_Down(); moove_Forward(); status = 5; return;}
    while(millis() - count < 3000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    sensor_Report();
    if (status == 5 || errorStatus[0]) return; 
    moove_Reverse();
    if (errorStatus[0]) {motor_Stop(); return;}
    if (status == 5 || errorStatus[0]) return;
    sensor_Report();
    moove_Forward();
    if (errorStatus[0]) {motor_Stop(); return;}
    if (status == 5 || errorStatus[0]) return;    
    count = millis();
    read_BatteryCharge();
    if (batteryCharge > 0 && batteryCharge <= minBattCharge) {add_Error(11); if (lifterUp) lifter_Down(); moove_Forward(); status = 5; return;}
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    count = millis();
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    count = millis();
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    sensor_Report();
    if (status == 5 || errorStatus[0]) return; 
    while (status !=5) 
    {      
      count = millis();
      while(millis() - count < 1000) 
      {
        if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
        blink_Work();
      }
      unload_Pallete();
      sensor_Report();
      firstPalletePosition = currentPosition;
      if (errorStatus[0]) {motor_Stop(); return;}
      if (distance[0] < 200) status = 5;
    }
    firstPalletePosition = 0;
    if (status == 5 && distance[0] > 200 || errorStatus[0]) {motor_Stop(); return;}
    else status = 11;
    motor_Stop();
    count = millis();
    read_BatteryCharge();
    if (batteryCharge > 0 && batteryCharge <= minBattCharge) {add_Error(11); if (lifterUp) lifter_Down(); moove_Forward(); status = 5; return;}
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    count = millis();
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    count = millis();
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    sensor_Report();
    if (status == 5 || errorStatus[0]) return; 
    moove_Forward();
    if (errorStatus[0]) {motor_Stop(); return;}
    if (status == 5 || errorStatus[0]) return;
    sensor_Report();
    moove_Reverse();
    if (errorStatus[0]) {motor_Stop(); return;}
    if (status == 5 || errorStatus[0]) return;    
    count = millis();
    read_BatteryCharge();
    if (batteryCharge > 0 && batteryCharge <= minBattCharge) {add_Error(11); if (lifterUp) lifter_Down(); moove_Forward(); status = 5; return;}
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    count = millis();
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    count = millis();
    while(millis() - count < 1000) 
    {
      if(get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
    }
    sensor_Report();
    if (status == 5 || errorStatus[0]) return; 
  }
}

void set_Position()  // Определяет текущую позицию шаттла в канале по переднему фронту (передней панели)
{
  angle = as5600.readAngle();
  int oldAng = oldAngle;
  while (angle > 4096 || angle < 0) angle = as5600.readAngle();
  int diff = 0;
  if (motorReverse == 0 ^ inverse)
  {
    if (oldAngle - angle > 0 && oldAngle - angle <= 2000)
    {
      uint8_t s = oldAngle / 512;
      uint8_t f = angle / 512;
      if (s == f) {diff = (oldAngle - angle) * (int)calibrateEncoder_F[s] / 512;}
      else {diff = ((512 * s - angle) * (int)calibrateEncoder_F[f] + (oldAngle - 512 * s) * (int)calibrateEncoder_F[s]) / 512;}
      //Serial1.print("[0] 0<dAng<2000 ");
    }
    else if (angle - oldAngle > 2000) {diff = (oldAngle * (int)calibrateEncoder_F[0] + (4096 - angle) * (int)calibrateEncoder_F[7]) / 512;}
    else if (angle - oldAngle > 0 && angle - oldAngle <= 2000)
    {
      uint8_t s = oldAngle / 512;
      uint8_t f = angle / 512;
      if (s == f) diff = (angle - oldAngle) * (int)calibrateEncoder_F[s] / 512;
      else diff = ((angle - 512 * f) * (int)calibrateEncoder_F[f] + (512 * f - oldAngle) * (int)calibrateEncoder_F[s]) / 512;
      //Serial1.print("[2] 0<dAng<2000 ");
    }
    else if (oldAngle - angle > 2000) {diff = (angle * calibrateEncoder_F[0] + (4096 - oldAngle) * calibrateEncoder_F[7]) / 512; /*Serial1.print("[3] dAng>2000 ");*/}
    if (diff > 0 && !inverse) {currentPosition -= diff; oldAngle = angle;}
    else if (diff > 0) {currentPosition += diff; oldAngle = angle;}    
  }
  else if (motorReverse == 1 ^ inverse)
  {
    if (oldAngle - angle > 0 && oldAngle - angle <= 2000)
    {
      uint8_t s = oldAngle / 512;
      uint8_t f = angle / 512;
      if (s == f) {diff = (oldAngle - angle) * (int)calibrateEncoder_R[s] / 512;}
      else {diff = ((512 * s - angle) * (int)calibrateEncoder_R[f] + (oldAngle - 512 * s) * (int)calibrateEncoder_R[s]) / 512;}
      //Serial1.print("[4] 0<dAng<2000 ");
    }
    else if (angle - oldAngle > 2000) {diff = (oldAngle * (int)calibrateEncoder_R[0] + (4096 - angle) * (int)calibrateEncoder_R[7]) / 512;}
    else if (angle - oldAngle > 0 && angle - oldAngle <= 2000)
    {
      uint8_t s = oldAngle / 512;
      uint8_t f = angle / 512;
      if (s == f) diff = (angle - oldAngle) * (int)calibrateEncoder_R[s] / 512;
      else diff = ((angle - 512 * f) * (int)calibrateEncoder_R[f] + (512 * f - oldAngle) * (int)calibrateEncoder_R[s]) / 512;
      //Serial1.print("[6] 0<dAng<2000 ");
    }
    else if (oldAngle - angle > 2000) {diff = (angle * calibrateEncoder_R[0] + (4096 - oldAngle) * calibrateEncoder_R[7]) / 512;}
    if (diff > 0 && !inverse) {currentPosition += diff; oldAngle = angle;}
    else if (diff != 0) {currentPosition -= diff; oldAngle = angle;}
  }
  //Serial1.println("angle = " + String(angle) + "  oldAngle = " + String(oldAng) + "  diff = " + String(diff) + "  pos = " + String(currentPosition));
  if (currentPosition < 0) {if (lastPalletePosition) lastPalletePosition -= currentPosition; channelLength -= currentPosition; currentPosition = 0;}
  if (channelLength - shuttleLength < currentPosition) channelLength = currentPosition + shuttleLength;
}

void fifoLifo_Inverse()
{
  if (inverse) {inverse = 0; sensor_channel_f = &sens_chnl_f; sensor_channel_r = &sens_chnl_r; sensor_pallete_F = &sens_plt_F; sensor_pallete_R = &sens_plt_R; currentPosition = channelLength - currentPosition - 800;}
  else {inverse = 1; sensor_channel_f = &sens_chnl_r; sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F;}
}

void blink_Work() // Моргание светодиодом в режиме работы
{
  if (millis() - count2 > blinkTime) {
    counter++;
    Can1.read(CAN_RX_msg);
    if (counter == 10 && status != 25)
    {
      set_Position();      
      dataStr =  "Position = " + String(currentPosition);
      Serial.println(dataStr);
      Serial1.println(dataStr);
      int diff = abs(currentPosition - oldPosition);
      if (currentPosition == 0) diff = abs(channelLength - shuttleLength - oldPosition);
      //if ((motorStart && oldSpeed > 9 && diff < oldSpeed / 5) || mooveCount == 5) {motor_Stop(); status = 5; add_Error(10); return;}
      if (motorStart && oldSpeed && diff < 10) {if (!mooveCount) countCrush = millis(); mooveCount = 1;}
      else {oldPosition = currentPosition; mooveCount = 0; if (currentPosition == 0) oldPosition = channelLength - shuttleLength;}
      if (mooveCount && millis() - countCrush > 1500) {motor_Stop(); status = 5; add_Error(10); return;}
    }
    else if (counter == 1 || counter == 5) {digitalWrite(GREEN_LED, HIGH); digitalWrite(WHITE_LED, HIGH);}
    else if ((counter == 2 || counter == 12) && status != 25) 
    {
      if (motorStart) dataStr = "Speed = " + String(oldSpeed);
      else dataStr = "mooving lifter...";
      //dataStr = "Speed = " + String(oldSpeed) + " CF = " + String(distance[0]) + " CR = " + String(distance[1]) + " PF = " + String(distance[2]) + " PR = " + String(distance[3]);
      Serial.println(dataStr);
      Serial1.println(dataStr);
    }
    else if (counter == 3 || counter == 7) {digitalWrite(GREEN_LED, LOW); digitalWrite(WHITE_LED, LOW); IWatchdog.reload();}
    else if (counter == 4 && status != 25) {if (lifterCurrent) {Serial.println("LCrnt = " + String(lifterCurrent)); Serial1.println("LCrnt = " + String(lifterCurrent));}}
    else if (counter == 8 && status != 25) {uint8_t oldStatus = status; status = 16; send_Cmd(); status = oldStatus;}
    //else if ((counter == 6 || counter == 9 || counter == 10 || counter == 13 || counter == 15 || counter == 17) && status != 25) { Serial1.println("CB " + String(batteryCharge)); delay(20); Serial1.println("CV " + String(batteryVoltage));}
    else if ((counter == 11 || counter == 19) && status != 25) {send_Cmd();}
    else if (counter == 20) {counter = 0;}
    count2 = millis();      
  }
  return;
}

void blink_Error() // Моргание светодиодом в режиме ошибки
{}

void blink_Warning() // Моргание светодиодом на предупреждение
{
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(ZOOMER, HIGH);
  count = millis();
  while (millis() - count < 100) if (status = get_Cmd() == 5 || errorStatus[0]) {digitalWrite(RED_LED, LOW); digitalWrite(GREEN_LED, LOW); digitalWrite(ZOOMER, LOW); return;}
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(ZOOMER, LOW);
  count = millis();
  while (millis() - count < 100) if (status = get_Cmd() == 5 || errorStatus[0]) {digitalWrite(RED_LED, LOW); digitalWrite(GREEN_LED, LOW); digitalWrite(ZOOMER, LOW); return;}
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(ZOOMER, HIGH);
  count = millis();
  while (millis() - count < 100) if (status = get_Cmd() == 5 || errorStatus[0]) {digitalWrite(RED_LED, LOW); digitalWrite(GREEN_LED, LOW); digitalWrite(ZOOMER, LOW); return;}
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(ZOOMER, LOW);
  count = millis();
  while (millis() - count < 100) if (status = get_Cmd() == 5 || errorStatus[0]) {digitalWrite(RED_LED, LOW); digitalWrite(GREEN_LED, LOW); digitalWrite(ZOOMER, LOW); return;}
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(ZOOMER, HIGH);
  count = millis();
  while (millis() - count < 100) if (status = get_Cmd() == 5 || errorStatus[0]) {digitalWrite(RED_LED, LOW); digitalWrite(GREEN_LED, LOW); digitalWrite(ZOOMER, LOW); return;}
  digitalWrite(RED_LED, LOW);
  digitalWrite(ZOOMER, LOW);
  return;
}

#pragma endregion

#pragma region Рабочие функции шаттла

void stop_Before_Pallete_F() //Остановка перед паллетом к началу канала (к выгрузке)
{
  dataStr = "Start stopping before pallete F... at " + String(distance[3]);
  Serial.println(dataStr);
  Serial1.println(dataStr);
    moove_Before_Pallete_F();
  //if (lifterUp && (status == 21 || status == 22) && oldSpeed ==0) oldSpeed =20;
  if (status == 5 || errorStatus[0]) return;
  if (oldSpeed == 0 && distance[1] > 250) motor_Speed(20);
  if (distance[3] <= 900)
  {
    count = millis();
    while (distance[3] >= 550 && distance[1] > 85)
    {
      int spd = (distance[3] - 200) / 20;
      if (spd > distance[1] / 23) spd = distance[1] / 23;
      if (spd > oldSpeed) spd = oldSpeed;
      if (spd < 5) spd = 5;
      motor_Speed(spd);
      while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
      count = millis();
      set_Position();
      get_Distance();
    }
    int dist = distance[3];
    int diffP = currentPosition + startDiff;
    dataStr = "Speed = " + String(oldSpeed) + "  position = " + String(currentPosition);
    Serial.println(dataStr);
    Serial1.println(dataStr);
    int diff = distance[3];
    uint8_t i = 1;
    uint8_t j = 1;
    get_Distance();
    while (i < 4)
    {
      dataStr = "Distance F = " + String(distance[3]);
      Serial.println(dataStr);
      Serial1.println(dataStr);
      while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
      count = millis();
      get_Distance();
      if (distance[3] > 250 && distance[3] < 550) {dist += distance[3]; i++;}
      if (distance[3] > 550) {j++; if (j == 3) {dist = distance[3]; diffP = currentPosition + startDiff; diff = distance[3]; i = 1; j--;}}
      int spd = (distance[3] - 200) / 20;
      if (oldSpeed < spd && distance[3] < 700 && distance[1] > 700) motor_Speed(oldSpeed);
      else if (distance[3] >= 700 && distance[1] > 700) motor_Speed(30);
      else if (spd > 7 && distance[1] > 700) motor_Speed(spd);
      else if (distance[1] <= 700 && distance[1] > 85 + chnlOffset) {spd = distance[1] / 23; if (spd < 5) spd = 5; else if (spd > oldSpeed) spd = oldSpeed; motor_Speed(spd);}
      else if (distance[1] <= 85 + chnlOffset) {motor_Stop(); dataStr = "Stopped at the end of channel..."; Serial.println(dataStr); Serial1.println(dataStr); return;}
      else motor_Speed(7);
      set_Position();
    }
    dataStr = "Distance F = " + String(distance[3]);
    Serial.println(dataStr);
    Serial1.println(dataStr);
    diffP = abs(diffP - currentPosition) / 2;
    diff = abs(diff - distance[3]) * 3 / 5;
    dataStr = "Difference F =  " + String(diff) + "  diff by enc = " + String(diffP);
    if (lifterCurrent) diff = (lifterCurrent - 80) / 25;
    else diff = 0;
    if (diffP > 8 && shuttleLength == 800) diff += diffP - 9;
    if (diff < 0) diff = 0;
    Serial.println(dataStr);
    Serial1.println(dataStr);
    dist = dist / 4 - interPalleteDistance - 100 - diff - mprOffset;
    if (shuttleLength == 1000 || shuttleLength == 1200) dist -= 25;
    dist *= 0.96;
    moove_Distance_F(dist, oldSpeed, 7);
  }
  motor_Stop();
  if (distance[1] > 250) dataStr = "Stopped before pallete F...";
  else dataStr = "Stopped at the end of channel...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
}

void moove_Before_Pallete_F() //Остановка перед паллетом к началу канала (к выгрузке)
{
  dataStr = "Going before pallete F...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  uint8_t findPallete = 1;  
  get_Distance();
  if (motorStart) motor_Speed (oldSpeed);
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  if (motorStart) motor_Speed (oldSpeed);
  motor_Start_Forward();
  oldChannelDistanse = distance[1];
  oldPalleteDistanse = distance[3];
  count = millis();
  while (findPallete)
  {
    if (get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    blink_Work();
    if (millis() - count > timingBudget + 5)
    {
      get_Distance();
      set_Position();
      if (distance[1] >= 1500 && distance[3] >= 1500)
      {
        if (lifterUp && firstPalletePosition)
        {
          if (currentPosition - firstPalletePosition > 3000 ) motor_Speed(80);
          else if ((currentPosition - firstPalletePosition - 600) / 30 > 40) motor_Speed((currentPosition - firstPalletePosition - 600) / 30);
          else motor_Speed(40);
        }
        else if (lifterUp)
        {
          if (currentPosition > 3500 ) motor_Speed(80);
          else if (currentPosition / 50 > 40) motor_Speed(currentPosition / 50);
          else motor_Speed(40);
        }
        else if (currentPosition > 1800) motor_Speed(90);
        else if (currentPosition <= 100) {if (oldSpeed < 60) motor_Speed(oldSpeed + 5); else motor_Speed(60);}
        else if (currentPosition > 100 && currentPosition <= 1800) {int spd = currentPosition / 20; if (spd > oldSpeed) motor_Speed(oldSpeed); else motor_Speed(spd);}
        else motor_Speed(5);
      }
      else if (distance[1] >= distance[3] && distance[3] >= 750) {int spd = (int)((distance[3] - 40) / 25); if (lifterUp) spd = (distance[3] - 450) / 20; if (spd > oldSpeed) if (oldSpeed > 20) motor_Speed(oldSpeed); else motor_Speed(20); else motor_Speed(spd);}
      else if (distance[1] >= distance[3] && distance[3] < 750) findPallete = 0;
      else if (distance[1] > 150 + chnlOffset && distance[1] <= 1500)
      {
        int spd = (int)((distance[1] - 40) / 18);
        if (spd > oldSpeed) motor_Speed(oldSpeed + 5);
        else motor_Speed(spd);
        if (distance[3] < 750) findPallete = 0;
      }
      else if (distance[1] > 90 + chnlOffset && distance[1] <= 150 + chnlOffset) motor_Speed(5);
      else if (distance[1] <= 90 + chnlOffset) {findPallete = 0; currentPosition = 60;}
      else motor_Speed(oldSpeed);
      if (status == 5 || errorStatus[0]) return;
      count = millis();  
    }
  }  
  if (distance[1] > 150) dataStr = "Before at " + String(distance[3]) + " Pos = " + String(currentPosition);
  else {dataStr = "End of channel F... at " + String(distance[1]); currentPosition = distance[1] - 30;}
  Serial.println(dataStr);
  Serial1.println(dataStr);
}

void stop_Before_Pallete_R() //Остановка перед паллетом к концу канала (загрузка)
{
  dataStr = "Start stopping before pallete R... at " + String(distance[2]);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  moove_Before_Pallete_R();
  if (status == 5 || errorStatus[0]) return;
  if (oldSpeed == 0 && distance[0] > 250) motor_Speed(20);
  if (distance[2] <= 900) 
  {
    count = millis();
    while (distance[2] >= 550 && distance[0] > 80)
    {
      int spd = (distance[2] - 200) / 20;
      if (spd > distance[0] / 23) spd = distance[0] / 23;
      if (spd > oldSpeed) spd = oldSpeed;
      if (spd < 5) spd = 5;
      motor_Speed(spd);
      while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
      count = millis();
      set_Position();
      get_Distance();
    }
    int dist = distance[2];
    dataStr = "Speed = " + String(oldSpeed) + "  position = " + String(currentPosition);
    Serial.println(dataStr);
    Serial1.println(dataStr);
    int diff = distance[2];
    int diffP = currentPosition + startDiff;
    uint8_t i = 1;
    uint8_t j = 1;
    get_Distance();
    while (i < 4)
    {
      dataStr = "Distance R = " + String(distance[2]);
      Serial.println(dataStr);
      Serial1.println(dataStr);
      while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
      count = millis();
      get_Distance();
      if (distance[2] > 300 && distance[2] < 550) {dist += distance[2]; i++;}
      if (distance[2] > 550) {j++; if (j == 3) {dist = distance[2]; diffP = currentPosition +  + startDiff; diff = distance[2]; i = 1; j--;}}
      int spd = (distance[2] - 200) / 20;
      if (oldSpeed < spd && distance[2] < 700 && distance[0] > 700) motor_Speed(oldSpeed);
      else if (distance[2] >= 700 && distance[0] > 700) motor_Speed(30);
      else if (spd > 7 && distance[0] > 700) motor_Speed(spd);
      else if (distance[0] <= 700 && distance[0] > 85 + chnlOffset) {spd = distance[0] / 23; if (spd < 5) spd = 5; else if (spd > oldSpeed && oldSpeed > 5) spd = oldSpeed; motor_Speed(spd);}
      else if (distance[0] <= 85 + chnlOffset) {motor_Stop(); dataStr = "Stopped at the end of channel..."; Serial.println(dataStr); Serial1.println(dataStr); return;}
      else motor_Speed(7);
      set_Position();
    }
    dataStr = "Distance R = " + String(distance[2]);
    Serial.println(dataStr);
    Serial1.println(dataStr);
    diff = abs(diff - distance[2]) * 4 / 5;
    diffP = abs(diffP - currentPosition) / 2;
    dataStr = "Difference R =  " + String(diff) + "  diff by enc = " + String(diffP);
    if (lifterCurrent) diff = (lifterCurrent - 80) / 25;
    else diff = 0;
    if (diffP > 8 && shuttleLength == 800) diff += diffP - 9;
    if (diff < 0) diff = 0;
    Serial.println(dataStr);
    Serial1.println(dataStr);
    dist = dist / 4 + diffPallete - interPalleteDistance - 100 - diff - mprOffset;
    if (shuttleLength == 1000 || shuttleLength == 1200) dist -= 25;
    dist *= 0.96;
    moove_Distance_R(dist, oldSpeed, 7);
  }
  motor_Stop();
  if (distance[0] > 250) dataStr = "Stopped before pallete R...";
  else dataStr = "Stopped at the end of channel...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
}

void moove_Before_Pallete_R() //Остановка перед паллетом к концу канала (загрузка)
{
  dataStr = "Going before pallete R...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  uint8_t findPallete = 1;
  if (motorStart) motor_Speed (oldSpeed);
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  oldChannelDistanse = distance[0];
  oldPalleteDistanse = distance[2];
  if (motorStart) motor_Speed (oldSpeed);
  motor_Start_Reverse();
  count = millis(); 
  while (findPallete)
  {    
    if (get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    blink_Work();
    if (millis() - count > timingBudget + 5)
    {
      get_Distance();
      set_Position();
      if (distance[0] >= 1500 && distance[2] >= 1500)
      {
        if (lifterUp && lastPallete)
        {
          int diff = lastPalletePosition - currentPosition;
          if (diff > 3000) motor_Speed(80);
          else if ((diff - 600) / 30 > oldSpeed) {if (oldSpeed < 50) motor_Speed(50); else motor_Speed(oldSpeed);}
          else if ((diff - 600) / 30 > 40) motor_Speed((diff - 600) / 30);
          else motor_Speed(40);
        }
        else if (lifterUp && channelLength > 3000)
        {
          int diff = channelLength - shuttleLength - currentPosition;
          if (diff > 2000) motor_Speed(70);
          else if (diff < 100) {if (maxSpeed > 86) motor_Speed(60); else motor_Speed(66);}
          else if (diff / 25 > oldSpeed) {if (oldSpeed < 20) motor_Speed(20); else motor_Speed(oldSpeed);}
          else if (diff/ 25 > 50) motor_Speed(diff / 25);
          else motor_Speed(50);
        }
        else if (lifterUp) {if (maxSpeed > 86) motor_Speed(60); else motor_Speed(66);}
        else if (channelLength - shuttleLength - currentPosition > 1800) motor_Speed(90);
        else if (channelLength - shuttleLength - currentPosition < 100) {if (oldSpeed < 60 && endOfChannel) motor_Speed(oldSpeed + 5); else motor_Speed(60);}
        else if (channelLength - shuttleLength - currentPosition >= 100 && channelLength - shuttleLength - currentPosition <= 1800)
        {
          int spd = (channelLength - shuttleLength - currentPosition) / 20;
          if (!endOfChannel && spd < 50) spd = 50;
          if (endOfChannel && spd > oldSpeed) motor_Speed(oldSpeed);
          else motor_Speed(spd);
        }
        else motor_Speed(5);
      }
      else if (distance[0] >= distance[2] && distance[2] >= 750)
      {
        if ((distance[2] - 40) / 25 < oldSpeed) motor_Speed((distance[2] - 40) / 25);
        else if (oldSpeed >= 20) motor_Speed(oldSpeed);
        else motor_Speed(20);
      }
      else if (distance[0] >= distance[2] && distance[2] < 750) findPallete = 0;
      else if (distance[0] > 150 + chnlOffset && distance[0] <= 1500)
      {
        int spd = (int)((distance[0] - 40) / 18);
        if (spd > oldSpeed) motor_Speed(oldSpeed + 5);
        else motor_Speed(spd);
        if (distance[2] < 750) findPallete = 0;
      }
      else if (distance[0] > 90 + chnlOffset && distance[0] <= 150 + chnlOffset) motor_Speed(5);
      else if (distance[0] <= 90 + chnlOffset)  {findPallete = 0;}
      else motor_Speed(oldSpeed);
      if (status == 5 || errorStatus[0]) return;
      count = millis();
    }   
  }
  if (distance[0] > 150) dataStr = "Before at " + String(distance[2]) + " Pos = " + String(currentPosition);
  else {dataStr = "End of channel R... at " + String(distance[0]); channelLength = currentPosition + shuttleLength + distance[0] - 30; endOfChannel = 1;}
  Serial.println(dataStr);
  Serial1.println(dataStr);
}

void moove_Distance_F(int dist)  //Движение вперед на заданное расстояние
{
  moove_Distance_F(dist, 100, 10);
  motor_Stop();
}

void moove_Distance_F(int dist, int maxSpeed, int minSpeed)  //Движение вперед на заданное расстояние
{
  dataStr = "Moove F distance = " + String(dist) + " Pos = " + String(currentPosition);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  uint16_t startAngle = as5600.readAngle();
  while (startAngle > 4096 || startAngle < 0) startAngle = as5600.readAngle();
  uint16_t currentAngle = startAngle;
  uint8_t moove = 1;
  count = millis();
  if (maxSpeed < 2) maxSpeed = 2;
  if (minSpeed > maxSpeed) minSpeed = maxSpeed - 1;
  if (dist > 500 && dist <= 1000 && maxSpeed > 50) maxSpeed = 50;
  if (dist > 300 && dist <= 500 && maxSpeed > 30) maxSpeed = 30;
  if (dist <= 300 && maxSpeed > 20) maxSpeed = 20;
  if (dist > 50) dist -= 10;
  motor_Start_Forward();
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  int cnt = millis();
  count = millis();
  while (moove)
  {
    if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
    blink_Work();
    if (millis() - count > timingBudget + 5)
    {
      get_Distance();
      set_Position();
      currentAngle = as5600.readAngle();
      while (currentAngle > 4096 || currentAngle < 0) currentAngle = as5600.readAngle();
      int diff = 0;
      if (!inverse)
      {
        if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) {diff = (startAngle - currentAngle) * calibrateEncoder_F[s] / 512;}
          else {diff = ((512 * s - currentAngle) * calibrateEncoder_F[f] + (startAngle - 512 * s) * calibrateEncoder_F[s]) / 512;}
          //Serial1.print("[0] ");
        }
        else if (currentAngle - startAngle > 2000) {diff = (startAngle * calibrateEncoder_F[0] + (4096 - currentAngle) * calibrateEncoder_F[7]) / 512;/* Serial1.print("[1] ");*/}
        else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) diff = (currentAngle - startAngle) * calibrateEncoder_F[s] / 512;
          else diff = ((currentAngle - 512 * f) * calibrateEncoder_F[f] + (512 * f - startAngle) * calibrateEncoder_F[s]) / 512;
          //Serial1.print("[2] ");
        }
        else if (startAngle - currentAngle > 2000) {diff = (currentAngle * calibrateEncoder_F[0] + (4096 - startAngle) * calibrateEncoder_F[7]) / 512;}
      }
      else
      {
        if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) {diff = (startAngle - currentAngle) * (int)calibrateEncoder_R[s] / 512;}
          else {diff = ((512 * s - currentAngle) * (int)calibrateEncoder_R[f] + (startAngle - 512 * s) * (int)calibrateEncoder_R[s]) / 512;}
          //Serial1.print("[4] ");
        }
        else if (currentAngle - startAngle > 2000) {diff = (startAngle * calibrateEncoder_R[0] + (4096 - currentAngle) * calibrateEncoder_R[7]) / 512;}
        else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) diff = (currentAngle - startAngle) * (int)calibrateEncoder_R[s] / 512;
          else diff = ((currentAngle - 512 * f) * (int)calibrateEncoder_R[f] + (512 * f - startAngle) * (int)calibrateEncoder_R[s]) / 512;
          //Serial1.print("[6] ");
        }
        else if (startAngle - currentAngle > 2000) {diff = (currentAngle * calibrateEncoder_R[0] + (4096 - startAngle) * calibrateEncoder_R[7]) / 512;}
      }
      //Serial1.println("old angle = " + String(startAngle) + "  angle = " + String(currentAngle) + "  diff = " + String(diff));
      if (diff < 0) diff = 0;
      if (diff != 0) {dist -= diff; startAngle = currentAngle; cnt = millis();}
      
      if (distance[1] >= maxSpeed * 15 && dist >= maxSpeed * 15) motor_Speed(maxSpeed);
      else if (distance[1] <= 90 + chnlOffset) {moove = 0; currentPosition = 0;}
      else if (dist >= distance[1] && distance[1] < maxSpeed * 15) motor_Speed((int)(distance[1] / 15));
      else if (dist < maxSpeed * 15 && dist > minSpeed * 15) motor_Speed((int)(dist / 15));
      else if (dist > 0 && dist <= minSpeed * 15) motor_Speed(minSpeed);
      else if (dist <= 0)  moove = 0;
      if (lifterUp && millis() - cnt > 3000 && dist < 30) {motor_Stop(); return;}
      else if (millis() - cnt > 5000) {add_Error(12); motor_Stop(); if (lifterUp) lifter_Down(); status = 5; return;}
      count = millis(); 
    }
  }
  dataStr = "End mooving, position = " + String(currentPosition);
  Serial.println(dataStr);
  Serial1.println(dataStr);
}

void moove_Distance_R(int dist)  //Движение назад на заданное расстояние
{
  moove_Distance_R(dist, 100, 10);
  motor_Stop();
}

void moove_Distance_R(int dist, int maxSpeed, int minSpeed)  //Движение назад на заданное расстояние
{
  dataStr = "Moove R distance = " + String(dist) + " Pos = " + String(currentPosition);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  uint16_t startAngle = as5600.readAngle();
  while (startAngle > 4096 || startAngle < 0) startAngle = as5600.readAngle();
  uint16_t currentAngle = startAngle;
  uint8_t moove = 1;
  count = millis();
  if (dist > 500 && dist <= 1000 && maxSpeed > 50) maxSpeed = 50;
  if (dist > 300 && dist <= 500 && maxSpeed > 30) maxSpeed = 30;
  if (dist <= 300 && maxSpeed > 20) maxSpeed = 20;
  if (dist > 50) dist -= 10; 
  if (maxSpeed < 5) maxSpeed = 5;
  if (minSpeed > maxSpeed) minSpeed = maxSpeed - 1;
  motor_Start_Reverse();
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  int cnt = millis();
  count = millis();
  while (moove)
  {
    if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
    blink_Work();
    if (millis() - count > timingBudget + 5)
    {
      get_Distance();
      set_Position();
      currentAngle = as5600.readAngle();
      while (currentAngle > 4096 || currentAngle < 0) currentAngle = as5600.readAngle();
      int diff = 0;
      if (inverse)
      {
        if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) {diff = (startAngle - currentAngle) * (int)calibrateEncoder_F[s] / 512;}
          else {diff = ((512 * s - currentAngle) * (int)calibrateEncoder_F[f] + (startAngle - 512 * s) * (int)calibrateEncoder_F[s]) / 512;}
          //Serial1.print("[8] ");
        }
        else if (currentAngle - startAngle > 2000) {diff = (startAngle * (int)calibrateEncoder_F[0] + (4096 - currentAngle) * (int)calibrateEncoder_F[7]) / 512; /*Serial1.print("[9] ");*/}
        else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) diff = (currentAngle - startAngle) * (int)calibrateEncoder_F[s] / 512;
          else diff = ((currentAngle - 512 * f) * (int)calibrateEncoder_F[f] + (512 * f - startAngle) * (int)calibrateEncoder_F[s]) / 512;
          //Serial1.print("[10] ");
        }
        else if (startAngle - currentAngle > 2000) {diff = (currentAngle * calibrateEncoder_F[0] + (4096 - startAngle) * calibrateEncoder_F[7]) / 512;}
      }
      else
      {
        if (startAngle - currentAngle > 0 && startAngle - currentAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) {diff = (startAngle - currentAngle) * (int)calibrateEncoder_R[s] / 512;}
          else {diff = ((512 * s - currentAngle) * (int)calibrateEncoder_R[f] + (startAngle - 512 * s) * (int)calibrateEncoder_R[s]) / 512;}
          //Serial1.print("[12] ");
        }
        else if (currentAngle - startAngle > 2000) {diff = (startAngle * (int)calibrateEncoder_R[0] + (4096 - currentAngle) * (int)calibrateEncoder_R[7]) / 512;}
        else if (currentAngle - startAngle > 0 && currentAngle - startAngle <= 2000)
        {
          uint8_t s = startAngle / 512;
          uint8_t f = currentAngle / 512;
          if (s == f) diff = (currentAngle - startAngle) * (int)calibrateEncoder_R[s] / 512;
          else diff = ((currentAngle - 512 * f) * (int)calibrateEncoder_R[f] + (512 * f - startAngle) * (int)calibrateEncoder_R[s]) / 512;
          //Serial1.print("[14] ");
        }
        else if (startAngle - currentAngle > 2000) {diff = (currentAngle * calibrateEncoder_R[0] + (4096 - startAngle) * calibrateEncoder_R[7]) / 512;}
      }

      //Serial1.println("old angle = " + String(startAngle) + "  angle = " + String(currentAngle) + "  diff = " + String(diff));
      if (diff > 0) {dist -= diff; startAngle = currentAngle; cnt = millis();}
      
      if (distance[0] >= maxSpeed * 15 && dist >= maxSpeed * 15) motor_Speed(maxSpeed);
      else if (distance[0] <= 90 + chnlOffset) {moove = 0; channelLength = currentPosition + shuttleLength;}
      else if (dist >= distance[0] && distance[0] < maxSpeed * 15) motor_Speed((int)(distance[0] / 15));
      else if (dist < maxSpeed * 15 && dist > minSpeed * 15) motor_Speed((int)(dist / 15));      
      else if (dist > 0 && dist <= minSpeed * 15) motor_Speed(minSpeed);
      else if (dist <= 0) moove = 0;
      if (lifterUp && millis() - cnt > 3000 && dist < 30) {motor_Stop(); return;}
      else if (millis() - cnt > 5000) {add_Error(12); motor_Stop(); if (lifterUp) lifter_Down(); status = 5; return;}
      count = millis();
    } 
  }
  dataStr = "End mooving, position = " + String(currentPosition);
  Serial.println(dataStr);
  Serial1.println(dataStr);
}

void moove_Forward() //Движение вперед до конца канала
{
  dataStr = "Start moove forward... Status = " + String(status);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  motor_Start_Forward();
  detect_Pallete();
  if (lifterUp) {stop_Before_Pallete_F(); if (status != 5) lifter_Down(); return;}
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  oldChannelDistanse = distance[1];
  uint8_t moove = 1;
  count = millis();
  while (moove)
  {
    if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
    blink_Work();
    if (millis() - count > timingBudget + 5)
    {
      get_Distance();
      set_Position();
      if (currentPosition < 0) currentPosition = 0;
      if (distance[1] >= 1500 && currentPosition < 3000 && currentPosition > 1500) {speed = currentPosition / 50 + 40; if (speed < 70) speed = 70;}
      else if (distance[1] >= 1500 && currentPosition <= 1500) {speed = 70; if (oldSpeed > 50 && speed > oldSpeed) speed = oldSpeed;}
      else if (distance[1] >= 1500 && (currentPosition >= 3000)) speed = 100;
      else if (distance[1] > 90 + chnlOffset && distance[1] < 1500) {speed = distance[1] / 20; if (oldSpeed > 10 && speed > oldSpeed && currentPosition <= 1500) speed = oldSpeed; if (speed < 5) speed = 5; if (speed > 70) speed = 70; if (oldSpeed > 50 && speed > oldSpeed) speed = oldSpeed;}
      else if (distance[1] <= 90 + chnlOffset)
      {
        speed = 0;
        moove = 0;
        currentPosition = 60;
        Serial1.println("End of channel, stop moove forward...");
      }
      motor_Speed(speed);
      if (status == 5 || errorStatus[0]) return;
      count = millis();  
    }
  }
  motor_Stop();
}

void moove_Reverse() //Движение назад до конца канала
{
  dataStr = "Start moove reverse... Status = " + String(status);
  Serial.println(dataStr);
  Serial1.println(dataStr);
  motor_Start_Reverse();
  detect_Pallete();
  if (lifterUp) {stop_Before_Pallete_R(); if (status != 5) lifter_Down(); return;}
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  oldChannelDistanse = distance[0];
  uint8_t moove = 1;
  uint8_t maxSpd = 0;
  count = millis();
  while (moove)
  {
    if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
    blink_Work();
    if (millis() - count > timingBudget + 5)
    {
      get_Distance();
      set_Position();
      if (currentPosition > channelLength - shuttleLength) channelLength = currentPosition + shuttleLength;
      if (distance[0] >= 1500 && currentPosition > channelLength - shuttleLength - 3000 && currentPosition < channelLength - shuttleLength - 1500) {speed = 40 + (channelLength - shuttleLength - currentPosition) / 50;  if (speed < 70) speed = 70;}
      else if (distance[0] >= 1500 && currentPosition >= channelLength - shuttleLength - 1500) {speed = 70;  if (oldSpeed > 50 && speed > oldSpeed) speed = oldSpeed;}
      else if (distance[0] >= 1500 && currentPosition < channelLength - shuttleLength - 3000) speed = 100;
      else if (distance[0] > 90 + chnlOffset && distance[0] < 1500) {speed = distance[0] / 20; if (oldSpeed > 10 && speed > oldSpeed && currentPosition <= channelLength - shuttleLength - 100) speed = oldSpeed; if (speed < 5) speed = 5; if (speed > 70) speed = 70; if (oldSpeed > 50 && speed > oldSpeed) speed = oldSpeed;}
      else if (distance[0] <= 90 + chnlOffset)
      {
        speed = 0;
        startAngle = angle;
        turnCount = 0;
        moove = 0;
        channelLength = currentPosition + shuttleLength + distance[0] - 30;
        endOfChannel = 1;
        Serial1.println("End of channel, stop moove reverse...");
      }      
      motor_Speed(speed);
      if (status == 5 || errorStatus[0]) return;
      count = millis();
    }    
  }
  motor_Stop();
}

void unload_Pallete() //Выгрузка паллеты
{
  dataStr = "Start unloading pallete...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (fifoLifo) fifoLifo_Inverse();
  uint8_t moove = 0;
  uint8_t frontBoard = 0;
  int currentPalletePosition;
  int palleteLenght = 0;
  startDiff = 0;
  if (lifterUp) lifter_Down();
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 30) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); if (fifoLifo) fifoLifo_Inverse(); return;}}
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 30) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); if (fifoLifo) fifoLifo_Inverse(); return;}}
  get_Distance();
  if (distance[0] < 90 + chnlOffset) startDiff = 20;
  if (1)
  {
    //Подъезжаем к паллету
    if (distance[0] > shuttleLength && distance[2] > 750) {moove_Before_Pallete_R(); frontBoard = 0;} // Если свободен канал вперед, едем к паллету
    if (status == 5 || errorStatus[0]) {oldSpeed = 0; if (fifoLifo) fifoLifo_Inverse(); return;}
    moove = 1;
    motor_Start_Reverse();
    if (oldSpeed > 20 || distance[0] < 250 + chnlOffset) motor_Speed(oldSpeed);
    else motor_Speed(28);
    int cnt = millis();
    while (moove)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; if (fifoLifo) fifoLifo_Inverse(); return;}
      blink_Work();
      detect_Pallete();
      if (detectPalleteR1 && detectPalleteR2 && !frontBoard) {set_Position(); currentPalletePosition = currentPosition; int dst = 650; if (channelLength - currentPosition - shuttleLength < 1500 && shuttleLength == 800) dst = 500; moove_Distance_R(dst, oldSpeed, 10); frontBoard = 1;}
      delay(1);
      detect_Pallete();
      if (frontBoard && detectPalleteF1 && detectPalleteF2)
      {
        moove = 0;
        dataStr = "Back board detect...";
        Serial.println(dataStr);
        Serial1.println(dataStr);
        if (!(detectPalleteR1 || detectPalleteR2))
        {
          motor_Stop();
          Serial.println("Pallete error in BB...");
          Serial1.println("Pallete error in BB...");
          blink_Warning();
          Serial2.print(shuttleNums[shuttleNum] + "wc004!");
          moove_Forward(); status = 5;
          Serial2.print(shuttleNums[shuttleNum] + "wc000!");
          return;}
        int pause = 30 + (300 - 60 * oldSpeed / 2);
        while (pause > 100)
        {
          count = millis();
          while (millis() - count < 100) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
          motor_Speed(oldSpeed);
          pause -= 100;
        }
        count = millis();
        while (millis() - count < pause) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
        motor_Stop();
      }
      if (detectPalleteR1 && detectPalleteR2 && frontBoard && shuttleLength != 800) 
      {
        set_Position();
        palleteLenght = abs(currentPalletePosition - currentPosition);
        count = millis();
        while (millis() - count < 250) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
        motor_Speed(oldSpeed);
        count = millis();
        while (millis() - count < 150) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
        moove = 0; frontBoard = 0; motor_Stop();
      }
      if (moove && millis() - count > timingBudget + 5)
      {
        set_Position();
        get_Distance();
        count = millis();
        while (millis() - count < timingBudget + 30) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); if (fifoLifo) fifoLifo_Inverse(); return;}}
        get_Distance();
        int spd = distance[0] / 23;
        if (spd > oldSpeed) spd = oldSpeed;
        if (spd < 5) spd = 5;
        motor_Speed(spd);
        detect_Pallete();
        if (distance[0] < 90 + chnlOffset && (detectPalleteR1 || detectPalleteR2) && (detectPalleteF1 || detectPalleteF2)) {startDiff = 20; motor_Stop(); moove = 0;}
        if ((millis() - cnt > 20000 || distance[0] < 90 + chnlOffset))
        {
          Serial.println("Pallete error...");
          Serial1.println("Pallete error...");
          motor_Stop();
          Serial2.print(shuttleNums[shuttleNum] + "wc003!");
          blink_Warning();
          Serial2.print(shuttleNums[shuttleNum] + "wc000!");
          if (fifoLifo) fifoLifo_Inverse();
          return;
        }
        count = millis();
      }
    }
    if (frontBoard) moove = 1;
    detect_Pallete();
    if (detectPalleteF1 && detectPalleteF2) {moove = 0; delay(0); motor_Stop();}
    delay(10);
    detect_Pallete();
    if (detectPalleteF1 && detectPalleteF2) {moove = 0; delay(0); motor_Stop();}
    while (moove)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; if (fifoLifo) fifoLifo_Inverse(); return;}
      blink_Work();
      if (millis() - count > timingBudget + 5)
      {
        get_Distance();
        uint16_t dit = distance[0];
        set_Position();
        if (distance[0] < 90 + chnlOffset) {motor_Stop(); channelLength = currentPosition + shuttleLength + distance[0] - 30; status = 5; moove = 0; endOfChannel = 1;}
        else
        {
          uint8_t detect = 1;
          detect_Pallete();
          if (detectPalleteF1 && detectPalleteF2) {moove = 0; delay(0); motor_Stop(); detect = 0;}
          delay(5);
          detect_Pallete();
          if (detectPalleteF1 && detectPalleteF2) {moove = 0; delay(0); motor_Stop();}
          else if (detect) {set_Position(); motor_Speed(10);}
        }
        count = millis();
      }
    }
  }

  set_Position();
  Serial1.println("Pallete lenght = " + String(palleteLenght));
  if (status == 5 || errorStatus[0]) {if (fifoLifo) fifoLifo_Inverse(); return;}
  //Поднимаем паллет
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); if (fifoLifo) fifoLifo_Inverse(); return;}}
  get_Distance();
  if (distance[3] < 900) {if (lifterUp) lifter_Down(); Serial2.print(shuttleNums[shuttleNum] + "wc005!"); blink_Warning(); moove_Forward(); Serial2.print(shuttleNums[shuttleNum] + "wc000!"); status = 5; return;}
  if (!lifterUp) lifter_Up();
  int pstn = 0;
  if (distance[2] < 600) pstn = currentPosition;
  //Перехват если требуется
  if (!frontBoard && !(detectPalleteF1 && detectPalleteF2))
  {
    int dist = 250;
    if (shuttleLength == 1200) dist = 450;
    //if (shuttleLength - palleteLenght > 350) dist = 400;
    moove_Distance_F(dist, 12, 10);
    motor_Stop();
    if (lifterUp) lifter_Down();
    moove = 1;
    motor_Start_Reverse();
    while (moove)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; if (fifoLifo) fifoLifo_Inverse(); return;}
      blink_Work();
      if (millis() - count > timingBudget + 5)
      {
        get_Distance();
        if (distance[0] < 90 + chnlOffset) {motor_Stop(); channelLength = currentPosition + shuttleLength + distance[0] - 30; status = 5; moove = 0; endOfChannel = 1;}
        else
        {
          uint8_t detect = 1;
          detect_Pallete();
          if (detectPalleteF1 && detectPalleteF2) {moove = 0; motor_Stop(); detect = 0;}
          delay(5);
          if (detectPalleteF1 && detectPalleteF2) {moove = 0; motor_Stop(); detect = 0;}
          else if (detect) {set_Position(); motor_Speed(10);}
        }
        count = millis();
      }
    }    
    motor_Stop();
    if (!lifterUp) lifter_Up();
  }
  //Везем в начало канала
  if (status == 7 || status == 22 || status == 23) 
  {
    moove_Before_Pallete_F();
    motor_Stop();
    if (distance[1] > 150)
    {      
      while (distance[3] < 800)
      {
        if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; if (fifoLifo) fifoLifo_Inverse(); return;}
        blink_Work();
        if (millis() - count > timingBudget + 5) {get_Distance(); count = millis();}
      }
      while (millis() - count < waitTime) {blink_Work(); if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; if (fifoLifo) fifoLifo_Inverse(); return;}}
      motor_Start_Forward();
      motor_Speed(20);
      while (distance[1] > 90)
      {
        if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; if (fifoLifo) fifoLifo_Inverse(); return;}
        blink_Work();
        if(millis() - count > timingBudget + 5)
        {
          get_Distance();
          if (distance[1] > 400 + chnlOffset) motor_Speed(20);
          else if (distance[1] <= 400 + chnlOffset && distance[1] > 120 + chnlOffset) motor_Speed(distance[1] / 20);
          else motor_Speed(6);
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
  if (status == 5 || errorStatus[0]) {if (fifoLifo) fifoLifo_Inverse(); return;}
  //Опускаем паллет
  if (!longWork && lifterUp) lifter_Down();
  if (palleteLenght < 850 && pstn) lastPalletePosition = pstn + 800 + interPalleteDistance;
  else if (pstn) lastPalletePosition = pstn + 1000 + interPalleteDistance;
  if (pstn) lastPallete = 1;
  else lastPallete = 0;
  if (lastPallete)
  {
    dataStr = "Last pallete position after unload = " + String(lastPalletePosition);
    Serial.println(dataStr);
    Serial1.println(dataStr);
      }
  if (fifoLifo) fifoLifo_Inverse();
}

void load_Pallete() //Загрузка паллеты
{
  dataStr = "Start loading pallete...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  uint8_t moove = 1;
  uint8_t frontBoard = 1;
  int currentPalletePosition;
  int palleteLenght = 0;
  startDiff = 0;
  if (lifterUp) lifter_Down();
  if (lastPalletePosition && lastPalletePosition < shuttleLength + 300) {Serial2.print(shuttleNums[shuttleNum] + "wc003!"); blink_Warning(); Serial2.print(shuttleNums[shuttleNum] + "wc000!"); status = 5; return;}
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  if (distance[1] < 90 + chnlOffset) startDiff = 20;
  detect_Pallete();
  if (!((detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2) && distance[1] < 300 + chnlOffset && distance[3] > 400)) // Проверка шаттла на нахождение в начале канала, сработка паллетного датчика (паллет сверху) и свободное место впереди
  {
    //Подъезжаем к паллету
    if (distance[1] > shuttleLength && distance[3] > 750) {moove_Before_Pallete_F(); frontBoard = 0;} // Если есть куда ехать назад (канал свободен назад), едем к паллету
    else 
    {
      frontBoard = 0;
      //oldSpeed = 28;
    }
    if (status == 5 || errorStatus[0]) {oldSpeed = 0; return;}
    //Двигаемся под паллет
    if (distance[1] > 150)
    {
      moove = 1;
      motor_Start_Forward();
      if (oldSpeed > 20) motor_Speed(oldSpeed);
      else motor_Speed(20);
    }
    count = millis();
    uint8_t free = 0;
    int cnt = millis();
    while (moove)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
      detect_Pallete();
      if (detectPalleteF1 && detectPalleteF2 && !frontBoard) {frontBoard = 1; set_Position(); currentPalletePosition = currentPosition; moove_Distance_F(650, oldSpeed, 10); frontBoard = 1;}
      detect_Pallete();
      if (frontBoard && detectPalleteR1 && detectPalleteR2) 
      {
        moove = 0;
        dataStr = "Back board detect...";
        Serial.println(dataStr);
        Serial1.println(dataStr);
        if (!(detectPalleteF1 || detectPalleteF2))
        {
          motor_Stop();
          Serial.println("Pallete error in BB...");
          Serial1.println("Pallete error in BB...");
          Serial2.print(shuttleNums[shuttleNum] + "wc004!");
          blink_Warning();
          moove_Forward();
          Serial2.print(shuttleNums[shuttleNum] + "wc000!");
          status = 5;
          return;
        }
        int pause = 30 + (300 - 60 * oldSpeed / 2);
        while (pause > 100)
        {
          count = millis();
          while (millis() - count < 100) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
          motor_Speed(oldSpeed);
          pause -= 100;
        }
        count = millis();
        while (millis() - count < pause) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
        motor_Stop();
      }
      if (detectPalleteF1 && detectPalleteF2 && frontBoard && shuttleLength != 800)
      {
        set_Position();
        dataStr = "Channel dist = " + String(distance[1]) + "  pallete dist = " + String(distance[3]);
        Serial.println(dataStr);
        Serial1.println(dataStr);
        palleteLenght = abs(currentPalletePosition - currentPosition);
        //if (distance[1] < 300 || distance[3] < 700) {moove = 0; frontBoard = 0; motor_Stop();}
        count = millis();
        while (millis() - count < 250) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
        motor_Speed(oldSpeed);
        count = millis();
        while (millis() - count < 150) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
        moove = 0; frontBoard = 0; motor_Stop();
      }
      if (moove && millis() - count > timingBudget + 5)
      {
        set_Position();
        get_Distance();
        count = millis();
        while (millis() - count < timingBudget + 30) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); if (fifoLifo) fifoLifo_Inverse(); return;}}
        get_Distance();
        int spd = distance[1] / 23;
        if (spd > oldSpeed) spd = oldSpeed;
        if (spd < 5) spd = 5;
        motor_Speed(spd);
        detect_Pallete();
        if (distance[1] < 90 + chnlOffset && (detectPalleteR1 || detectPalleteR2) && (detectPalleteF1 || detectPalleteF2)) {startDiff = 20; motor_Stop(); moove = 0;}
        if ((millis() - cnt > 20000 || distance[1] < 90 + chnlOffset))
        {
          Serial.println("Pallete error...");
          Serial1.println("Pallete error...");
          motor_Stop();
          Serial2.print(shuttleNums[shuttleNum] + "wc003!");
          blink_Warning();
          Serial2.print(shuttleNums[shuttleNum] + "wc000!");
          return;
        }
        count = millis();
      }
    }
    if (frontBoard) moove = 1;
    detect_Pallete();
    if (detectPalleteR1 && detectPalleteR2) {moove = 0; motor_Stop();}
    delay(10);
    detect_Pallete();
    if (detectPalleteR1 && detectPalleteR2) {moove = 0; motor_Stop();}
    while (moove)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
      if (millis() - count > timingBudget + 5)
      {
        get_Distance();
        if (distance[1] < 90 + chnlOffset) {motor_Stop(); status = 5; moove = 0;}
        else
        {
          uint8_t detect = 1;
          detect_Pallete();
          if (detectPalleteR1 && detectPalleteR2) {moove = 0; motor_Stop(); detect = 0;}
          delay(5);
          detect_Pallete();
          if (detectPalleteR1 && detectPalleteR2) {moove = 0; motor_Stop();}
          else if (detect) {set_Position(); motor_Speed(10);}
        }
        count = millis();
        set_Position();
      }
    }
  }
  else if ((detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2) && distance[1] < 300 + chnlOffset && distance[2] <= interPalleteDistance + 600)
  {
    motor_Stop();
    Serial2.print(shuttleNums[shuttleNum] + "wc003!");
    blink_Warning();
    Serial2.print(shuttleNums[shuttleNum] + "wc000!");
    status = 5;
    return;
  }
  else if ((detectPalleteR1 || detectPalleteR2) && !(detectPalleteF1 || detectPalleteF2)) 
  {
    while (detectPalleteR1 || detectPalleteR2)
    {
      if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
      moove_Distance_R(10, 10, 10);
      detect_Pallete();
    }
    get_Distance();
    count = millis();
    while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
    get_Distance();
    if (distance[3] < 500) {motor_Stop(); Serial2.print(shuttleNums[shuttleNum] + "wc005!"); blink_Warning(); Serial2.print(shuttleNums[shuttleNum] + "wc000!"); status = 5; return;}
    frontBoard = 0;
  }
  else if (detectPalleteF1 && detectPalleteF2) frontBoard = 0;
  set_Position();
  
  
  if (status == 5 || errorStatus[0]) return;
  //Поднимаем паллет
  if (!lifterUp) lifter_Up();
  //Перехват если требуется
  detect_Pallete();
  if (!frontBoard && !(detectPalleteR1 && detectPalleteR2))
  {
    int dist = 100;
    if (shuttleLength == 1000) dist = 250;
    else if (shuttleLength == 1200) dist = 450;
    moove_Distance_R(dist, 15, 10);
    motor_Stop();
    if (lifterUp) lifter_Down();
    motor_Start_Forward();
    moove = 1;
    while (moove)
    {
      if (get_Cmd() == 5 || errorStatus[0] || status == 5) {motor_Stop(); status = 5; return;}
      blink_Work();
      if (millis() - count > timingBudget + 5)
      {
        uint8_t detect = 1;
        detect_Pallete();
        if (detectPalleteR1 && detectPalleteR2) {moove = 0; motor_Stop(); detect = 0;}
        delay(5);
        if (detectPalleteR1 && detectPalleteR2) {moove = 0; motor_Stop(); detect = 0;}
        else if (detect) {set_Position(); motor_Speed(10);}
        count = millis();
      }
    }
    motor_Stop();
    if (!lifterUp) lifter_Up();
    diffPallete = 0;
  }
  //Везем на выгрузку
  stop_Before_Pallete_R();
  if (status == 5 || errorStatus[0]) return;
  //Опускаем паллет  
  if (!longWork && lifterUp) {lifter_Down(); lastPallete = 1; lastPalletePosition = currentPosition;}
  if (lastPallete)
  {
    dataStr = "Last pallete position after load = " + String(lastPalletePosition);
    Serial.println(dataStr);
    Serial1.println(dataStr);
      }
}

void single_Load() // Единичная загрузка
{
  moove_Forward();
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  detect_Pallete();
  if (detectPalleteF1 && detectPalleteF2) load_Pallete();
  else
  {
    motor_Start_Reverse();
    motor_Speed(10);
    //Serial1.println("dist = " + String(distance[0]) + " " + String(distance[1]) + " plt: " + String(detectPalleteF1) + " " + String(detectPalleteF2));
    while (!(detectPalleteF1 && detectPalleteF2) && distance[1] < 260 + chnlOffset)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
      detect_Pallete();
      if (millis() - count > timingBudget + 5)
      {
        get_Distance();
        set_Position();
        motor_Speed(10);
        count = millis();
        //Serial1.println("dist = " + String(distance[0]) + " plt: " + String(detectPalleteF1) + " " + String(detectPalleteF2));
      }
    }
    motor_Stop();
    if (detectPalleteF1 && detectPalleteF2) {diffPallete = 10; load_Pallete();}
    else {Serial2.print(shuttleNums[shuttleNum] + "wc003!"); blink_Warning(); Serial2.print(shuttleNums[shuttleNum] + "wc000!");}
  }
  moove_Forward();
}

void pallete_Counting_F() // Пересчет паллет
{
  dataStr = "Start counting pallete forward...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (lifterUp) lifter_Down();
  palleteCount = 0;
  //Двигаемся в начало канала
  moove_Forward();
  detect_Pallete();
  uint8_t palleteOnStart = 0;
  if ((detectPalleteF1 && detectPalleteF2) || (detectPalleteR1 && detectPalleteR2)) palleteOnStart++;
  if (status == 5 || errorStatus[0]) return;  
  //Теперь к первому паллету в загрузке
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  detect_Pallete();
  if (distance[2] > 1000 && !(detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2)) {moove_Before_Pallete_R(); if (status == 5 || errorStatus[0]) return; if (distance[0] < 150 + chnlOffset) {motor_Stop(); return;}}
  //Запускаем цикл пересчета
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  motor_Start_Reverse();
  motor_Speed(28);
  uint8_t detect = 0;
  uint8_t detectBoard = 0;
  uint8_t boardCount = 0;
  int count = millis();
  int countBoard = count;
  int measuringCount = count;
  int boardPosition = 0;
  uint8_t moove = 1;
  while (moove)
  {    
    //Двигаемся и считаем паллеты
    if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    blink_Work();
    detect_Pallete();
    if (detectPalleteR1 && detectPalleteR2)
    {
      boardCount++;
      dataStr = "Find board, count = " + String(boardCount) + " Time between = " + String(millis() - countBoard) + " " + String(currentPosition);
      Serial.println(dataStr);
      Serial1.println(dataStr);
      if (boardCount - 3 * (int)(boardCount / 3) == 1) {set_Position(); boardPosition = currentPosition; palletePosition[palleteCount] = currentPosition; palleteCount++;}
      if (boardCount && boardCount - 3 * (int)(boardCount / 3) == 0) {set_Position(); Serial1.println("Pallete width = " + String(currentPosition - boardPosition) + "  " + String(currentPosition));}
      
      while (moove && (detectPalleteR1 || detectPalleteR2))
      {
        count = millis();
        blink_Work();
        if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}
        detect_Pallete();
        if (millis() - measuringCount > timingBudget + 5)
        {
          get_Distance();
          if (distance[0] <= 560 + chnlOffset && distance[0] > 120 + chnlOffset) motor_Speed((int)(distance[0] / 20));
          else if (distance[0] <= 120 + chnlOffset && distance[0] > 80 + chnlOffset) motor_Speed(6);
          else if (distance[0] <= 80 + chnlOffset) {motor_Stop(); moove = 0; Serial1.println("End channel on counting with pallete ...");}
          else motor_Speed(28);
          set_Position();
          measuringCount = millis();
        }
      }      
    }
    
    if (millis() - measuringCount > timingBudget + 5 && moove)
    {
      get_Distance();
      if (distance[0] + chnlOffset <= 560 && distance[0] > 120 + chnlOffset) motor_Speed((int)(distance[0] / 20));
      else if (distance[0] <= 120 + chnlOffset && distance[0] > 80 + chnlOffset) motor_Speed(6);
      else if (distance[0] <= 80 + chnlOffset) {motor_Stop(); moove = 0; Serial1.println("End channel on counting without pallete ...");}
      else motor_Speed(28);
      set_Position();
      measuringCount = millis();
    }
  }
  palleteCount = lrint((float)boardCount / 3) + palleteOnStart;
  set_Position();
  channelLength = currentPosition + shuttleLength;
  return;  
}

void pallete_Compacting_F() // Уплотнение вперед
{
  dataStr = "Start compacting pallete forward...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (!digitalRead(DL_DOWN)) lifter_Down();
  moove_Reverse();
  if (status == 5 || errorStatus[0]) return;  
  get_Distance();
  count = millis(); 
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
  get_Distance();
  status = 14;
  detect_Pallete();
  while (distance[3] < 1000 && (detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2))
  {
    moove_Distance_F(10, 25, 25);
    if (get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    status = 14;
    motor_Speed(oldSpeed);
  }
  while (status != 5) 
  {
    blink_Work();
    load_Pallete();
    if (get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    status = 14;
    sensor_Report();
  }
}

void pallete_Compacting_R() // Уплотнение назад
{
  dataStr = "Start compacting pallete reverse...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  moove_Forward();
  if (status == 5 || errorStatus[0]) return;
  get_Distance();
  count = millis(); 
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}}
  get_Distance();
  status = 15;
  detect_Pallete();
  while (distance[2] < 1000 && (detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2))
  {
    moove_Distance_R(10, 25, 25);
    if (get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    status = 15;
    motor_Speed(oldSpeed);
  }
  status = 15;
  while (status != 5) 
  {
    blink_Work();
    unload_Pallete();
    if (get_Cmd() == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
    status = 15;
    firstPalletePosition = currentPosition;
    sensor_Report();
  }
  firstPalletePosition = 0;
}

void long_Load() // Продолжительная загрузка
{
  dataStr = "Starting continuos load...";
  status = 21;
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (lifterUp) lifter_Down();
  get_Distance();
  count = millis();
  while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5 || errorStatus[0]) {status = 5; motor_Stop(); return;}}
  get_Distance();
  detect_Pallete();
  if (!(detectPalleteF1 && detectPalleteF2)) 
  {
    motor_Start_Reverse();
    motor_Speed(10);
    //Serial1.println("dist = " + String(distance[0]) + " " + String(distance[1]) + " plt: " + String(detectPalleteF1) + " " + String(detectPalleteF2));
    while (!(detectPalleteF1 && detectPalleteF2) && distance[1] < 260 + chnlOffset)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; return;}
      blink_Work();
      detect_Pallete();
      if (millis() - count > timingBudget + 5)
      {
        get_Distance();
        set_Position();
        motor_Speed(10);
        count = millis();
        //Serial1.println("dist = " + String(distance[0]) + " plt: " + String(detectPalleteF1) + " " + String(detectPalleteF2));
      }
    }
    motor_Stop();
    if (!(detectPalleteF1 && detectPalleteF2))
    {
      Serial2.print(shuttleNums[shuttleNum] + "wc003!");
      blink_Warning();
      uint8_t wait = 1;
      moove_Distance_R(shuttleLength + 100, 60, 30);
      motor_Stop();
      count = millis();
      while (wait)
      {
        blink_Work();
        if (get_Cmd() == 5 || errorStatus[0]) {status = 5; Serial2.print(shuttleNums[shuttleNum] + "wc000!"); return;}
        if (millis() - count > timingBudget + 5) {get_Distance(); count = millis();}
        detect_Pallete();
        if ((detectPalleteF1 && detectPalleteF2) || distance[3] < 1000)
        {
          Serial2.print(shuttleNums[shuttleNum] + "wc000!");
          count = millis();
          while (millis() - count < 10000) {if (get_Cmd() == 5 || errorStatus[0]) {status = 5; Serial2.print(shuttleNums[shuttleNum] + "wc000!"); return;} blink_Work();}
          wait = 0;
        }
      }
    }
    if (detectPalleteR1 && detectPalleteR2) diffPallete = 10;
  }
  while(1)
  {
    status = 21;
    load_Pallete();
    if (lastPalletePosition && lastPalletePosition < shuttleLength + 300) {Serial2.print(shuttleNums[shuttleNum] + "wc003!");blink_Warning(); Serial2.print(shuttleNums[shuttleNum] + "wc000!"); status = 0; return;}
    /*get_Distance();
    count = millis();
    while (millis() - count < timingBudget + 5) {blink_Work(); if (get_Cmd() == 5) {status = 5; motor_Stop(); return;}}
    get_Distance();
    if ((detectPalleteF1 || detectPalleteF2 || detectPalleteR1 || detectPalleteR2) && distance[1] < 300 && distance[2] <= interPalleteDistance + 600) return;*/
    uint8_t wait = 0;
    get_Distance();
    if (distance[1] < 90 + chnlOffset && !errorStatus[0]) {status = 21; moove_Distance_R(shuttleLength + 300, 60, 30); motor_Stop(); wait = 1;}
    else if (status == 5 || errorStatus[0]) return;
    else status = 21;
    count = millis();
    while (wait)
    {
      blink_Work();
      if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;}
      if (millis() - count > timingBudget + 5) {get_Distance(); count = millis();}
      detect_Pallete();
      if ((detectPalleteF1 && detectPalleteF2) || distance[3] < 1000)
      {
        count = millis();
        while (millis() - count < 10000) {if (get_Cmd() == 5 || errorStatus[0]) {status = 5; return;} blink_Work();}
        wait = 0;
      }
    }
  }
}

void long_Unload() // Продолжительная выгрузка
{
  uint16_t oldInterPalleteDistance = interPalleteDistance;
  interPalleteDistance = 700;
  dataStr = "Starting continuos unload...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (fifoLifo) fifoLifo_Inverse();
  moove_Forward();
  if (status == 5 || errorStatus[0]) {if (fifoLifo) fifoLifo_Inverse(); interPalleteDistance = oldInterPalleteDistance; return;}
  uint8_t detect = 1;
  while (detect)
  {
    if (fifoLifo) fifoLifo_Inverse();
    unload_Pallete();
    if (fifoLifo) fifoLifo_Inverse();
    if (distance[0] < 200 + chnlOffset && !errorStatus[0]) {detect = 0; status = 22; break;}
    else if (status == 5 || errorStatus[0]) {if (fifoLifo) fifoLifo_Inverse(); interPalleteDistance = oldInterPalleteDistance; return;}
    count = millis();
    while (distance[3] < 900 && distance[1] > 700)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; interPalleteDistance = oldInterPalleteDistance; if (fifoLifo) fifoLifo_Inverse(); return;}
      blink_Work();
      if(millis() - count > timingBudget + 5)
      {
        get_Distance();
        count = millis();
      }
    }
    if (distance[1] > 700) 
    {
      count = millis();
      while (distance[1] > 90)
      {
        if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; interPalleteDistance = oldInterPalleteDistance; if (fifoLifo) fifoLifo_Inverse(); return;}
        blink_Work();
        if(millis() - count > timingBudget + 5)
        {
          get_Distance();
          if (distance[1] > 400 + chnlOffset) motor_Speed(20);
          else if (distance[1] <= 400 + chnlOffset && distance[1] > 120 + chnlOffset) motor_Speed(distance[1] / 20);
          else motor_Speed(6);
          count = millis();
        }
      }
    }
    motor_Stop();
    if (lifterUp) lifter_Down();
    sensor_Report();
    moove_Distance_R(shuttleLength + 500, 80, 80);
  }
  interPalleteDistance = oldInterPalleteDistance;
  if (fifoLifo) fifoLifo_Inverse();
}

void long_Unload(uint8_t num) // Продолжительная выгрузка заданного количества паллет
{
  uint16_t oldInterPalleteDistance = interPalleteDistance;
  interPalleteDistance = 700;
  dataStr = "Starting continuos unload...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  if (fifoLifo) fifoLifo_Inverse();
  moove_Forward();
  if (status == 5 || errorStatus[0]) {if (fifoLifo) fifoLifo_Inverse(); interPalleteDistance = oldInterPalleteDistance; return;}
  uint8_t detect = 1;
  while (detect && num)
  {
    status = 23;
    send_Cmd();
    if (fifoLifo) fifoLifo_Inverse();
    unload_Pallete();
    if (fifoLifo) fifoLifo_Inverse();
    get_Distance();
    while (millis() - count < timingBudget + 5)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; interPalleteDistance = oldInterPalleteDistance; if (fifoLifo) fifoLifo_Inverse(); return;}
      blink_Work();
    }
    get_Distance();
    if (distance[0] < 200 + chnlOffset && !errorStatus[0]) {detect = 0; status = 23; break;}
    else if (status == 5 || errorStatus[0]) {if (fifoLifo) fifoLifo_Inverse(); interPalleteDistance = oldInterPalleteDistance; return;}
    status = 23;
    send_Cmd();
    count = millis();
    while (distance[3] < 900 && distance[1] > 700)
    {
      if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; interPalleteDistance = oldInterPalleteDistance; if (fifoLifo) fifoLifo_Inverse(); return;}
      blink_Work();
      if(millis() - count > timingBudget + 5)
      {
        get_Distance();
        count = millis();
      }
    }
    if (distance[1] > 700) 
    {
      count = millis();
      while (distance[1] > 90)
      {
        if (get_Cmd() == 5 || status == 5 || errorStatus[0]) {motor_Stop(); status = 5; interPalleteDistance = oldInterPalleteDistance; if (fifoLifo) fifoLifo_Inverse(); return;}
        blink_Work();
        if(millis() - count > timingBudget + 5)
        {
          get_Distance();
          if (distance[1] > 400 + chnlOffset) motor_Speed(20);
          else if (distance[1] <= 400 + chnlOffset && distance[1] > 120 + chnlOffset) motor_Speed(distance[1] / 20);
          else motor_Speed(6);
          count = millis();
        }
      }
    }
    
    motor_Stop();
    if (lifterUp) lifter_Down();
    num--;
    UPQuant--;
    status = 23;
    send_Cmd();
    if (num) moove_Distance_R(shuttleLength + 500, 80, 80);
  } 
  interPalleteDistance = oldInterPalleteDistance;
  if (fifoLifo) fifoLifo_Inverse();
}

void moove_Right() // Движение назад в ручном режиме
{
  Serial.println("Manual moove right...");
  Serial1.println("Manual moove right...");
  uint8_t manualCount = 6;
  uint8_t moove = 1;
  motor_Start_Reverse();
  int cnt = millis();
  int cnt2 = millis();
  get_Distance();
  pingCount = millis();
  motor_Speed(manualCount);
  while (moove)
  {
    uint8_t stTmp = 0;
    if (Serial2.available())
    {
      stTmp = get_Cmd_Manual();
      if (stTmp == 5 || errorStatus[0] || stTmp == 55 ) {motor_Stop(); Serial.println("Manual stop..."); Serial1.println("Manual stop..."); return;}
      else if (stTmp == 100) pingCount = millis();
    }
    if (millis() - cnt > timingBudget + 5)
    {
      get_Distance();
      if (manualCount < 60 && millis() - cnt2 > 500) manualCount += 3;
      cnt = millis();
      if (sensorOff) motor_Speed(manualCount);
      else
      {
        if (distance[0] >= 1500) motor_Speed(manualCount);
        else if (distance[0] >= 90 + chnlOffset && distance[0] < 1500) if (distance[0] / 25 < manualCount) motor_Speed(distance[0] / 25); else motor_Speed(manualCount);
        else if (distance[0] < 90 + chnlOffset) {motor_Stop(); return;}
      }
      set_Position();
      while (Serial1.available()) Serial1.read();
    }
    blink_Work();
    if (millis() - pingCount > 500)  
    {
      motor_Stop();
      moove = 0;
      Serial.println("End of cycle, stop...");
      Serial1.println("End of cycle, stop...");
      return;
    }
  }
  motor_Stop();
  oldSpeed = 0;
  return;
}

void moove_Left() // Движение назад в ручном режиме
{
  Serial.println("Manual moove left...");
  Serial1.println("Manual moove left...");
  uint8_t manualCount = 6;
  uint8_t moove = 1;
  motor_Start_Forward();
  int cnt = millis();
  int cnt2 = millis();
  get_Distance();
  pingCount = millis();
  motor_Speed(manualCount);
  while (moove)
  {
    uint8_t stTmp = 0;
    if (Serial2.available())
    {
      stTmp = get_Cmd_Manual();
      if (stTmp == 5 || errorStatus[0] || stTmp == 55 ) {motor_Stop(); Serial.println("Manual stop..."); Serial1.println("Manual stop..."); return;}
      else if (stTmp == 100) pingCount = millis();
    }
    if (millis() - cnt > timingBudget + 5)
    {
      get_Distance();
      if (manualCount < 60 && millis() - cnt2 > 500) manualCount += 3;
      cnt = millis();
      if (sensorOff) motor_Speed(manualCount);
      else
      {
        if (distance[1] >= 1500) motor_Speed(manualCount);
        else if (distance[1] >= 90 + chnlOffset && distance[1] < 1500) if (distance[1] / 25 < manualCount) motor_Speed(distance[1] / 25); else motor_Speed(manualCount);
        else if (distance[1] < 90 + chnlOffset) {motor_Stop(); return;}
      }
      set_Position();
      while (Serial1.available()) Serial1.read();
    }
    blink_Work();
    if (millis() - pingCount > 500) 
    {
      motor_Stop();
      moove = 0;
      Serial.println("End of cycle, stop...");
      Serial1.println("End of cycle, stop...");
      return;
    }
  }
  motor_Stop();
  oldSpeed = 0;
  return;
}

#pragma endregion

#pragma region Технические функции

void calibrate_Encoder_R() //Процедура калибровки энкодера вперед
{
  dataStr = "Start calibrating encoder to Reverse";
  Serial.println(dataStr);as5600.readAngle();
  Serial1.println(dataStr);
   
  if (inverse) {inverse = 0; sensor_channel_f = &sens_chnl_f; sensor_channel_r = &sens_chnl_r; sensor_pallete_F = &sens_plt_F; sensor_pallete_R = &sens_plt_R;}
  int summ = 0;
  
  //Двигаемся к концу канала
  moove_Forward();
  if (status == 5 || errorStatus[0]) {if (inverse) {sensor_channel_f = &sens_chnl_r; sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F;} return;}
  //Выставляем 0 на энкодере
  angle = as5600.readAngle();
  motor_Start_Reverse();
  motor_Speed(5);
  int cnt = millis();
  count = millis();
  while (!(angle > 4086 || angle < 10))
  {
    if (millis() - count > 100) {motor_Speed(5); count = millis(); }
    if (millis() - cnt > 5) {angle = as5600.readAngle(); cnt = millis();}
  }
  
  //Запускаем цикл измерений
  uint8_t i = 0;
  cnt = millis();
  while ( i < 8)
  {
    delay(5);
    if (millis() - count > 100) {motor_Speed(5); count = millis();}
    angle = as5600.readAngle();
    if (angle > (7 - i) * 512 - 10 && angle < (7 - i) * 512 + 10)
    {
      calibrateEncoder_R[7 - i] = millis() - cnt;
      summ += calibrateEncoder_R[7 - i];
      cnt = millis();
      i++;
    }
  }
  IWatchdog.reload();
  motor_Stop();
  
  //Выводим данные
  Serial1.print("Calibrate_R data: ");
  for (i = 0; i < 8; i++)
  {
    Serial1.print(String(calibrateEncoder_R[i]) + "/");
    calibrateEncoder_R[i] = lrint(40.5 + calibrateEncoder_R[i] * 320 / summ) / 2;
    Serial1.print(String(calibrateEncoder_R[i]) + " ");
    eepromData.calibrateEncoder_R[i] = calibrateEncoder_R[i];
  }
  Serial1.println(" ");
  if (inverse) {sensor_channel_f = &sens_chnl_r; sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F;}
}

void calibrate_Encoder_F() //Процедура калибровки энкодера вперед
{
  dataStr = "Start calibrating encoder to Reverse";
  Serial.println(dataStr);
  Serial1.println(dataStr);
    dataStr = "Current calibrate F data: ";
  for (uint8_t i = 0; i < 8; i++) dataStr += String(calibrateEncoder_F[i]) + " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
    dataStr = "Current calibrate R data: ";
  for (uint8_t i = 0; i < 8; i++) dataStr += String(calibrateEncoder_R[i]) + " ";
  Serial.println(dataStr);
  Serial1.println(dataStr);
    delay(50);
  if (inverse) {inverse = 0; sensor_channel_f = &sens_chnl_f; sensor_channel_r = &sens_chnl_r; sensor_pallete_F = &sens_plt_F; sensor_pallete_R = &sens_plt_R;}
  int summ = 0;
  
  //Двигаемся к концу канала
  moove_Distance_R(2000);
  if (status == 5 || errorStatus[0]) {if (inverse) {sensor_channel_f = &sens_chnl_r; sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F;} return;}
  //Выставляем 0 на энкодере
  angle = as5600.readAngle();
  motor_Start_Forward();
  motor_Speed(5);
  int cnt = millis();
  count = millis();
  while (!(angle > 4086 || angle < 10))
  {
    if (millis() - count > 100) {motor_Speed(5); count = millis();}
    if (millis() - cnt > 5) {angle = as5600.readAngle(); cnt = millis();}
  }
  
  //Запускаем цикл измерений
  uint8_t i = 0;
  cnt = millis();
  while ( i < 8)
  {
    delay(3);
    if (millis() - count > 100) {motor_Speed(5); count = millis();}
    angle = as5600.readAngle();
    if (angle > ((i + 1) * 512) - 10 && angle < (i + 1) * 512 + 10)
    {
      calibrateEncoder_F[i] = millis() - cnt;
      summ += calibrateEncoder_F[i];
      cnt = millis();
      i++;
    }
  }
  motor_Stop();
  
  //Выводим данные
  
  Serial1.print("Calibrate_R data: ");
  for (i = 0; i < 8; i++)
  {
    Serial1.print(String(calibrateEncoder_F[i]) + "/");
    calibrateEncoder_F[i] = lrint(40.5 + calibrateEncoder_F[i] * 320 / summ) / 2;
    Serial1.print(String(calibrateEncoder_F[i]) + " ");
    eepromData.calibrateEncoder_F[i] = calibrateEncoder_F[i];
  }
  Serial1.println(" ");
  if (inverse) {sensor_channel_f = &sens_chnl_r; sensor_channel_r = &sens_chnl_f; sensor_pallete_F = &sens_plt_R; sensor_pallete_R = &sens_plt_F;}
}

uint8_t initialise_Sensor(VL53L0X *sensor) //Инициализация датчиков VL53L0X
{
  dataStr = "Start initialise sensor VL53L0X...";
  Serial.println(dataStr);
  Serial1.println(dataStr);
  uint8_t st;
  Serial.println("New address = 0x" + String(sensor->dev, HEX));
  Serial1.println("New address = 0x" + String(sensor->dev, HEX));
  sensor->VL53L0X_Off();
  st = sensor->InitSensor(sensor->dev);  //инициируем датчик на новый адрес
  if(st)
  {
    dataStr = "Init sensor failed... status: " + String(st);
    Serial.println(dataStr);
    Serial1.println(dataStr);
        //add_Error(5);
  }
  else 
  {
    dataStr = "Init sensor success...";
    Serial.println(dataStr);
    Serial1.println(dataStr);
    sensor->Prepare();
    sensor->SetDeviceMode(VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); 
    sensor->SetMeasurementTimingBudgetMicroSeconds(timingBudget * 1000);
    sensor->StartMeasurement();
  }
  return st;
}

void reinitialise_Sensor()
{
  Serial.println("Start reinitialise sensor num = " + String(sensorNum));
  uint8_t st = 1;
  int tryes = 0;
  VL53L0X sensor1(&Wire, XSHUT_F);
  VL53L0X sensor2(&Wire, XSHUT_R);
  VL53L0X sensor3(&Wire, XSHUT_PF);
  VL53L0X sensor4(&Wire, XSHUT_PR);
  switch (sensorNum)
  {
    case 0:        
      while(st)
      {
        digitalWrite(XSHUT_F, LOW);
        delay(10);
        sensor1.dev = 0x60;
        st = initialise_Sensor(&sensor1);
        tryes++;
        if (tryes > 3) {Wire.end(); Wire.begin();}
        if (tryes > 50) 
        {
          dataStr = "Too mutch tryes, reboot MCU...";
          Serial.println(dataStr);
          Serial1.println(dataStr);
          delay(10);
          HAL_NVIC_SystemReset();
        }
        delay(timingBudget * 2);
        if (tryes < 4 && motorStart) motor_Speed(oldSpeed);
      }
      sens_chnl_f = sensor1;
      break;
    case 1:
      while(st)
      {
        digitalWrite(XSHUT_R, LOW);
        delay(10);
        sensor2.dev = 0x62;
        st = initialise_Sensor(&sensor2);
        tryes++;
        if (tryes > 3) {Wire.end(); Wire.begin();}
        if (tryes > 20) 
        {
          dataStr = "Too mutch tryes, reboot MCU...";
          Serial.println(dataStr);
          Serial1.println(dataStr);
          delay(10);
          HAL_NVIC_SystemReset();
        }
        delay(timingBudget * 2);
        if (tryes < 4 && motorStart) motor_Speed(oldSpeed);
      }
      sens_chnl_r = sensor2;
      break;
    case 2:
      while(st)
      {
        digitalWrite(XSHUT_PF, LOW);
        delay(10);
        sensor3.dev = 0x64;
        st = initialise_Sensor(&sensor3);
        tryes++;
        if (tryes > 3) {Wire.end(); Wire.begin();}
        if (tryes > 20) 
        {
          dataStr = "Too mutch tryes, reboot MCU...";
          Serial.println(dataStr);
          Serial1.println(dataStr);
          delay(10);
          HAL_NVIC_SystemReset();
        }
        delay(timingBudget + 10);
      }
      sens_plt_F = sensor3;
      break;
    case 3:
      while(st)
      {
        digitalWrite(XSHUT_PR, LOW);
        delay(10);
        sensor4.dev = 0x66;
        st = initialise_Sensor(&sensor4);
        tryes++;
        if (tryes > 3) {Wire.end(); Wire.begin();}
        if (tryes > 20) 
        {
          dataStr = "Too mutch tryes, reboot MCU...";
          Serial.println(dataStr);
          Serial1.println(dataStr);
          delay(10);
          HAL_NVIC_SystemReset();
        }
        delay(timingBudget + 10);
      }
      sens_plt_R = sensor4;
      break;
  }
  ITimer0.detachInterrupt();
  ITimer0.disableTimer();
}

int findActivePage_new() 
{
  int pageAddress = 0;
  int incrStat = 4 * (int)(1 + (sizeof(EEPROMStat) + 1) / 4);
  int incrData = 4 * (int)(1 + (sizeof(EEPROMData) + 1) / 4);
  uint8_t rd;
  while ((rd = EEPROM.read(pageAddress)) != 255)
  {
    if (rd) {pageAddressStat = pageAddress; pageAddress += incrStat;}
    else {pageAddressData = pageAddress; pageAddress += incrData;}
    if (pageAddress > 128 * 1024) return -1;
  }
  return pageAddress;
}

void saveEEPROMData_new(const EEPROMData& data) 
{
    int activePage = findActivePage();
    eeprom_buffered_write_byte(activePage, 0);
    activePage++;
    const uint8_t* dataPtr = (const uint8_t*)&data;
    int cnt = millis();
    int sz = sizeof(EEPROMData);
    for (int i = 0; i < sz; i++) eeprom_buffered_write_byte(activePage + i, dataPtr[i]);
    eeprom_buffer_flush();
    digitalWrite(ZOOMER, HIGH);
    delay(1000);
    digitalWrite(ZOOMER, LOW);
}

void saveEEPROMStat(const EEPROMStat& stat) 
{
    int activePage = findActivePage();
    int sz = sizeof(EEPROMStat);
    //if (activePage + 4 * (int)(1 + (sz + 1) / 4)) > 128 * 1024) {EEPROM.}
    eeprom_buffered_write_byte(activePage, 1);
    activePage++;
    const uint8_t* dataPtr = (const uint8_t*)&stat;
    int cnt = millis();
    for (int i = 0; i < sz; i++) eeprom_buffered_write_byte(activePage + i, dataPtr[i]);
    eeprom_buffer_flush();
    digitalWrite(ZOOMER, HIGH);
    delay(1000);
    digitalWrite(ZOOMER, LOW);
}

int findActivePage() 
{
    for (int i = 0; i < EEPROM_TOTAL_PAGES; i++) 
    {
        int pageAddress = i * EEPROM_PAGE_SIZE;
        if (EEPROM.read(pageAddress) == 1) return i;
    }
    return -1;
}

void clearPageHeader(int pageNum) 
{
    int pageAddress = pageNum * EEPROM_PAGE_SIZE;
    EEPROM.write(pageAddress, 0);
}

void setPageHeader(int pageNum) 
{
    int pageAddress = pageNum * EEPROM_PAGE_SIZE;
    EEPROM.write(pageAddress, 1);
}

void saveEEPROMData(const EEPROMData& data) 
{
    int activePage = findActivePage();
    int nextPage = (activePage + 1) % EEPROM_TOTAL_PAGES;

    clearPageHeader(activePage);

    int pageAddress = nextPage * EEPROM_PAGE_SIZE + EEPROM_HEADER_SIZE;
    const uint8_t* dataPtr = (const uint8_t*)&data;
    int cnt = millis();
    int sz = sizeof(EEPROMData);
    for (int i = 0; i < sz; i++) eeprom_buffered_write_byte(pageAddress + i, dataPtr[i]);
    eeprom_buffer_flush();
    setPageHeader(nextPage);
    digitalWrite(ZOOMER, HIGH);
    delay(1000);
    digitalWrite(ZOOMER, LOW);
}

bool readEEPROMData(EEPROMData& data) 
{
    int activePage = findActivePage();
    if (activePage == -1) return false;
    int pageAddress = activePage * EEPROM_PAGE_SIZE + EEPROM_HEADER_SIZE;
    uint8_t* dataPtr = (uint8_t*)&data;
    int cnt = millis();
    for (int i = 0; i < sizeof(EEPROMData); i++) 
    {
      cnt = millis();
      dataPtr[i] = EEPROM.read(pageAddress + i);
      Serial.println("Reading time byte eeprom №" + String(i) + " = " + String(millis() - cnt) + "ms, millis = " + String(millis()) + "ms");
    }
    return true;
}

void read_BatteryCharge()
{
  //Serial1.println("Read battery charge...");
  uint8_t datab[7] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
  if (!digitalRead(RS485)) pinMode(RS485, OUTPUT);
  else
  {
    dataStr = "RS485 collision...";
    Serial.println(dataStr);
    Serial1.println(dataStr);
    return;
  }
  digitalWrite(RS485, HIGH);
  delay(1);
  Serial3.write(datab, 7);
  delay(20);
  digitalWrite(RS485, LOW);
  delay(20);
  uint8_t dataRead[50] = {0};
  uint8_t data = 0;
  uint8_t i = 0;
  int cnt = millis();
  //Serial.print("Batt data: ");
  //Serial1.print("Batt data: ");
  while (Serial3.available()) // Получаем команду
  {
    if (status == 5 || errorStatus[0]) break;
    dataRead[i] = Serial3.read();
    //Serial.print(" " + String(dataRead[i], HEX));
    //Serial1.print(" " + String(dataRead[i], HEX));
    data = 1;
    delayMicroseconds(1750);
    i++;
    if (i > 5 && i == dataRead[3] + 6) {dataRead[i] = Serial3.read(); break;}
    if (millis() - cnt > 100) {data = 0; break;}
  }
  /*Serial.print(" " + String(dataRead[i], HEX));
  Serial1.print(" " + String(dataRead[i], HEX));
  Serial.println(" .");
  Serial1.println(" .");*/
  //i--;
  if (data && dataRead[0] == 0xDD)
  {
    float volt = (float)(dataRead[4] * 256 + dataRead[5]) / 100;
    float current = (float)(dataRead[6] * 256 + dataRead[7]);
    if (current > 32768) current = current - 65536;
    current /= 100;
    float capacity = (float)(dataRead[8] * 256 + dataRead[9]) / 100;
    //if (batteryCharge - lrint(capacity / 0.3) < 3 && lrint(capacity / 0.3) <= 100) batteryCharge = lrint(capacity / 0.3);
    uint16_t sum = 0;
    uint16_t res = dataRead[i - 2] * 256 + dataRead[i - 1];
    for (uint8_t j = 3; j < i - 2; j++) sum += dataRead[j];
    sum = ~sum + 1;
    if (sum == res) 
    {
      if (volt > 41 && volt < 60) batteryVoltage = volt;
      if ((batteryCharge - dataRead[23] < 3 && dataRead[23] - batteryCharge < 5) || batteryCharge == 0) {batteryCharge = dataRead[23]; oldBattCharge = batteryCharge; battCount = 0;}
      else if (oldBattCharge - dataRead[23] < 2 && dataRead[23] - oldBattCharge < 3) {battCount++; oldBattCharge = dataRead[23]; if (battCount > 2) {batteryCharge = oldBattCharge; battCount = 0;}}
      else oldBattCharge = dataRead[23];
      //countBatt = millis();
    }
    
    if (millis() - countBatt > 10000 || true)
    {
      /*dataStr = "Read battery fail. Data string: ";
      for (uint8_t j = 0; j < i + 1; j++) dataStr += String(dataRead[j]) + " ";
      Serial.println(dataStr);
      Serial1.println(dataStr);*/
      dataStr = "Calc checksumm = " + String(sum) + "  ; BMS checksumm = " + String(res);
      Serial.println(dataStr);
      Serial1.println(dataStr);
      countBatt = millis();
    } 
  }
  while(Serial3.available()) Serial3.read();
  pinMode(RS485, INPUT_PULLDOWN);
  if (batteryCharge < 20) Serial2.print(shuttleNums[shuttleNum] + "wc002!");
}

void crash()
{
  if (!(status == 0 || status == 5))
  {
    if (lifterUp) motor_Stop();
    else motor_Force_Stop();
    status = 5;
    add_Error(12);
    oldSpeed = 0;
  }
}

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
  }*stack_ptr; //Указатель на текущее значение стека(SP)

  asm(
      "TST lr, #4 \n" //Тестируем 3ий бит указателя стека(побитовое И)
      "ITE EQ \n"   //Значение указателя стека имеет бит 3?
      "MRSEQ %[ptr], MSP  \n"  //Да, сохраняем основной указатель стека
      "MRSNE %[ptr], PSP  \n"  //Нет, сохраняем указатель стека процесса
      : [ptr] "=r" (stack_ptr)
      );

  volatile uint32_t BFAR = 0xFFFFFFFF;
  
  asm(
    "MRS %[bfar], PSP  \n"
    : [bfar] "=r" (BFAR)
      );

  volatile uint32_t CFSR = 0xFFFFFFFF;
  
  asm(
    "MRS %[cfsr], PSP  \n"
    : [cfsr] "=r" (CFSR)
      );

  volatile uint32_t HFSR = 0xFFFFFFFF;
  
  asm(
    "MRS %[hfsr], PSP  \n"
    : [hfsr] "=r" (HFSR)
      );
  
  volatile uint32_t HFSR_R = SCB->HFSR;

  count = millis();
  uint8_t k = 0;
  while(1)
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
      dataStr = "R0 = " + String(stack_ptr->r0) + "\n R1 = " + String(stack_ptr->r1) + "\n R2 = " + String(stack_ptr->r2) + "\n R3 = " + String(stack_ptr->r3) + "\n R12 = " + String(stack_ptr->r12) + "\n LR = " + String(stack_ptr->lr) + 
      "\n PC = " + String(stack_ptr->pc) + "\n PSR = " + String(stack_ptr->psr) + "\n BFAR = " + String(BFAR) + String(stack_ptr->psr) + "\n CFSR = " + String(CFSR) + String(stack_ptr->psr) + "\n HFSR = " + String(HFSR);
      Serial.println(dataStr);
      Serial1.println(dataStr);
            k = 0;
      IWatchdog.reload();
    }
  }
}

bool isValidDateTime(int hour, int minute, int second, int day, int month, int year) {
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
    return false;
  }
  if (month < 1 || month > 12 || day < 1 || day > 31) {
    return false;
  }
  if (month == 2 && day > (isLeapYear(year) ? 29 : 28)) {
    return false;
  }
  if ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30) {
    return false;
  }
  return true;
}

uint8_t getWeekDay(int day, int month, int year) {
  if (month < 3) {
    month += 12;
    year -= 1;
  }
  int k = year % 100;
  int j = year / 100;
  int weekDay = (day + (13 * (month + 1)) / 5 + k + (k / 4) + (j / 4) - (2 * j)) % 7;
  return (weekDay + 5) % 7 + 1;
}

bool isLeapYear(int year) {
  return (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
}

#pragma endregion

#pragma endregion