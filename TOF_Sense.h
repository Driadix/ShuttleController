#ifndef _TOF_SENSE_H_
#define _TOF_SENSE_H_

#include "Arduino.h"
#include "Wire.h"

// Структура данных датчика
typedef struct {
  uint8_t id;              // ID датчика
  uint32_t system_time;    // Время с момента включения, мс
  uint32_t dis;            // Расстояние, мм
  uint8_t dis_status;      // Статус измерения
  uint16_t signal_strength;// Сила сигнала
  uint8_t range_precision; // Точность измерения, см (для TOFSense-F)
  uint8_t interface_mode;  // Режим интерфейса: 0-UART, 1-CAN, 2-I/O, 3-I2C
  uint32_t uart_baudrate;  // Бодрейт UART
} TOF_Parameter;

// Базовый адрес I2C: 0x08 + ID
#define TOF_BASE_I2C_ADDR 0x08

// Размеры и адреса регистров
#define TOF_REGISTER_TOTAL_SIZE 48

#define TOF_ADDR_MODE             0x0c
#define TOF_SIZE_MODE             1

#define TOF_ADDR_ID               0x0d
#define TOF_SIZE_ID               1

#define TOF_ADDR_UART_BAUDRATE    0x10
#define TOF_SIZE_UART_BAUDRATE    4

#define TOF_ADDR_SYSTEM_TIME      0x20
#define TOF_SIZE_SYSTEM_TIME      4

#define TOF_ADDR_DIS              0x24
#define TOF_SIZE_DIS              4

#define TOF_ADDR_DIS_STATUS       0x28
#define TOF_SIZE_DIS_STATUS       2

#define TOF_ADDR_SIGNAL_STRENGTH  0x2a
#define TOF_SIZE_SIGNAL_STRENGTH  2

#define TOF_ADDR_RANGE_PRECISION  0x2c
#define TOF_SIZE_RANGE_PRECISION  1

#define IIC_CHANGE_TO_UART_DATA   0x00

// Функции
void TOF_Inquire_I2C_Decoding_ByID(uint8_t id, TOF_Parameter *tof_data);
void IIC_Set_ID(uint8_t current_id, uint8_t new_id);
void IIC_Change_Mode_To_UART(uint8_t id);
bool TOF_Is_Device_Present(uint8_t id);

#endif