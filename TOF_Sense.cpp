#include "TOF_Sense.h"

/*****************************************************************************
 * | File       :   TOF_Sense.cpp
 * | Author      :   Waveshare team + Adapted for multi-device
 * | Function    :   TOF driver with multi-device support
 ******************************************************************************/

// Универсальная функция чтения данных с указанием адреса
static void I2C_Read_Nbyte_ByAddr(uint8_t slave_addr, uint8_t reg_addr, uint8_t *pdata, uint8_t len) {
  Wire.beginTransmission(slave_addr);
  Wire.write(reg_addr);
  Wire.endTransmission();

  Wire.requestFrom(slave_addr, len);
  uint8_t i = 0;
  while (Wire.available() && i < len) {
    pdata[i++] = Wire.read();
  }
}

// Функция опроса датчика по ID
void TOF_Inquire_I2C_Decoding_ByID(uint8_t id, TOF_Parameter *tof_data) {
  uint8_t slave_addr = TOF_BASE_I2C_ADDR + id;
  uint8_t read_buf[TOF_REGISTER_TOTAL_SIZE];

  // Чтение данных двумя частями
  I2C_Read_Nbyte_ByAddr(slave_addr, 0x00, read_buf, TOF_REGISTER_TOTAL_SIZE / 2);
  I2C_Read_Nbyte_ByAddr(slave_addr, TOF_REGISTER_TOTAL_SIZE / 2, &read_buf[TOF_REGISTER_TOTAL_SIZE / 2], TOF_REGISTER_TOTAL_SIZE / 2);

  // Парсинг данных
  tof_data->interface_mode = read_buf[TOF_ADDR_MODE] & 0x07;
  tof_data->id = read_buf[TOF_ADDR_ID];
  tof_data->uart_baudrate = (uint32_t)(
    ((uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE + 3] << 24) |
    ((uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE + 2] << 16) |
    ((uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE + 1] << 8) |
    (uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE]
  );
  tof_data->system_time = (uint32_t)(
    ((uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME + 3] << 24) |
    ((uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME + 2] << 16) |
    ((uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME + 1] << 8) |
    (uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME]
  );
  tof_data->dis = (uint32_t)(
    ((uint32_t)read_buf[TOF_ADDR_DIS + 3] << 24) |
    ((uint32_t)read_buf[TOF_ADDR_DIS + 2] << 16) |
    ((uint32_t)read_buf[TOF_ADDR_DIS + 1] << 8) |
    (uint32_t)read_buf[TOF_ADDR_DIS]
  );
  tof_data->dis_status = (uint16_t)(
    (uint16_t)read_buf[TOF_ADDR_DIS_STATUS] |
    ((uint16_t)read_buf[TOF_ADDR_DIS_STATUS + 1] << 8)
  );
  tof_data->signal_strength = (uint16_t)(
    (uint16_t)read_buf[TOF_ADDR_SIGNAL_STRENGTH] |
    ((uint16_t)read_buf[TOF_ADDR_SIGNAL_STRENGTH + 1] << 8)
  );
  tof_data->range_precision = read_buf[TOF_ADDR_RANGE_PRECISION];
}

// Функция изменения ID датчика
void IIC_Set_ID(uint8_t current_id, uint8_t new_id) {
  uint8_t slave_addr = TOF_BASE_I2C_ADDR + current_id;
  Wire.beginTransmission(slave_addr);
  Wire.write(TOF_ADDR_ID);
  Wire.write(new_id);
  Wire.endTransmission();
  delay(10); // Небольшая задержка для применения
}

// Функция переключения датчика в UART-режим
void IIC_Change_Mode_To_UART(uint8_t id) {
  uint8_t slave_addr = TOF_BASE_I2C_ADDR + id;
  Wire.beginTransmission(slave_addr);
  Wire.write(TOF_ADDR_MODE);
  Wire.write(IIC_CHANGE_TO_UART_DATA);
  Wire.endTransmission();
  delay(10);
}

bool TOF_Is_Device_Present(uint8_t id) {
  uint8_t slave_addr = TOF_BASE_I2C_ADDR + id;
  uint8_t dummy;

  Wire.beginTransmission(slave_addr);
  Wire.write(TOF_ADDR_ID); // Запрашиваем регистр ID
  if (Wire.endTransmission(false) != 0) {
    return false; // Нет ответа
  }

  Wire.requestFrom(slave_addr, (uint8_t)1);
  if (Wire.available()) {
    dummy = Wire.read();
    return true; // Есть ответ
  }

  return false;
}