#include "TOF_Sense.h"
#include <string.h>

/*****************************************************************************
 * | File       :   TOF_Sense.cpp
 * | Author      :   Waveshare team + Adapted for multi-device
 * | Function    :   TOF driver with multi-device support
 ******************************************************************************/

static constexpr uint32_t kTofI2cHalTimeoutMs = 30U;

static TofI2cStatus tofStatusFromHal(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:
        return TOF_I2C_OK;
    case HAL_BUSY:
        return TOF_I2C_HAL_BUSY;
    case HAL_TIMEOUT:
        return TOF_I2C_HAL_TIMEOUT;
    case HAL_ERROR:
    default:
        return TOF_I2C_HAL_ERROR;
    }
}

static void tofSetDiag(
    TofI2cDiagnostics *diag,
    TofI2cStatus       status,
    uint8_t            address,
    uint8_t            reg,
    uint8_t            expected,
    uint8_t            received,
    uint32_t           halStatus,
    uint32_t           halError)
{
    if (diag == nullptr)
    {
        return;
    }

    diag->status    = status;
    diag->address   = address;
    diag->reg       = reg;
    diag->expected  = expected;
    diag->received  = received;
    diag->halStatus = halStatus;
    diag->halError  = halError;
}

static I2C_HandleTypeDef *tofWireHalHandleFrom(I2C_HandleTypeDef *handle)
{
    return handle;
}

static I2C_HandleTypeDef *tofWireHalHandleFrom(i2c_t *wireHandle)
{
    if (wireHandle == nullptr)
    {
        return nullptr;
    }
    return &wireHandle->handle;
}

static I2C_HandleTypeDef *tofWireHalHandle()
{
    return tofWireHalHandleFrom(Wire.getHandle());
}

static bool I2C_Read_Nbyte_ByAddr(
    uint8_t slave_addr, uint8_t reg_addr, uint8_t *pdata, uint8_t len, TofI2cDiagnostics *diag)
{
    if (pdata == nullptr || len == 0)
    {
        tofSetDiag(diag, TOF_I2C_INVALID_ARG, slave_addr, reg_addr, len, 0, 0, 0);
        return false;
    }

    I2C_HandleTypeDef *handle = tofWireHalHandle();
    if (handle == nullptr)
    {
        tofSetDiag(diag, TOF_I2C_NO_HANDLE, slave_addr, reg_addr, len, 0, 0, 0);
        return false;
    }

    const uint16_t          deviceAddr = (uint16_t)slave_addr << 1;
    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(handle, deviceAddr, reg_addr, I2C_MEMADD_SIZE_8BIT, pdata, len, kTofI2cHalTimeoutMs);
    const uint32_t      halError  = HAL_I2C_GetError(handle);
    const TofI2cStatus tofStatus = tofStatusFromHal(status);
    tofSetDiag(diag, tofStatus, slave_addr, reg_addr, len, (tofStatus == TOF_I2C_OK) ? len : 0, status, halError);
    return tofStatus == TOF_I2C_OK;
}

static bool I2C_Write_Nbyte_ByAddr(uint8_t slave_addr, uint8_t reg_addr, const uint8_t *pdata, uint8_t len)
{
    if (pdata == nullptr || len == 0)
    {
        return false;
    }

    I2C_HandleTypeDef *handle = tofWireHalHandle();
    if (handle == nullptr)
    {
        return false;
    }

    const uint16_t          deviceAddr = (uint16_t)slave_addr << 1;
    const HAL_StatusTypeDef status     = HAL_I2C_Mem_Write(
        handle,
        deviceAddr,
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        const_cast<uint8_t *>(pdata),
        len,
        kTofI2cHalTimeoutMs);
    return status == HAL_OK;
}

const char *TOF_I2C_Status_Name(TofI2cStatus status)
{
    switch (status)
    {
    case TOF_I2C_OK:
        return "ok";
    case TOF_I2C_INVALID_ARG:
        return "arg";
    case TOF_I2C_NO_HANDLE:
        return "handle";
    case TOF_I2C_HAL_ERROR:
        return "hal_error";
    case TOF_I2C_HAL_BUSY:
        return "busy";
    case TOF_I2C_HAL_TIMEOUT:
        return "timeout";
    case TOF_I2C_SHORT_READ:
        return "short";
    case TOF_I2C_BUS_STUCK:
        return "stuck";
    default:
        return "unknown";
    }
}

bool TOF_Inquire_I2C_Decoding_ByID(uint8_t id, TOF_Parameter *tof_data, TofI2cDiagnostics *diag)
{
    if (tof_data == nullptr)
    {
        tofSetDiag(diag, TOF_I2C_INVALID_ARG, 0, 0, 0, 0, 0, 0);
        return false;
    }

    uint8_t slave_addr = TOF_BASE_I2C_ADDR + id;
    uint8_t read_buf[TOF_REGISTER_TOTAL_SIZE] = {};

    if (!I2C_Read_Nbyte_ByAddr(slave_addr, 0x00, read_buf, TOF_REGISTER_TOTAL_SIZE / 2, diag) ||
        !I2C_Read_Nbyte_ByAddr(
            slave_addr, TOF_REGISTER_TOTAL_SIZE / 2, &read_buf[TOF_REGISTER_TOTAL_SIZE / 2], TOF_REGISTER_TOTAL_SIZE / 2, diag))
    {
        memset(tof_data, 0, sizeof(TOF_Parameter));
        return false;
    }

    tof_data->interface_mode = read_buf[TOF_ADDR_MODE] & 0x07;
    tof_data->id             = read_buf[TOF_ADDR_ID];
    tof_data->uart_baudrate  = (uint32_t)(
        ((uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE + 3] << 24) | ((uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE + 2] << 16) |
        ((uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE + 1] << 8) | (uint32_t)read_buf[TOF_ADDR_UART_BAUDRATE]);
    tof_data->system_time = (uint32_t)(
        ((uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME + 3] << 24) | ((uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME + 2] << 16) |
        ((uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME + 1] << 8) | (uint32_t)read_buf[TOF_ADDR_SYSTEM_TIME]);
    tof_data->dis = (uint32_t)(
        ((uint32_t)read_buf[TOF_ADDR_DIS + 3] << 24) | ((uint32_t)read_buf[TOF_ADDR_DIS + 2] << 16) |
        ((uint32_t)read_buf[TOF_ADDR_DIS + 1] << 8) | (uint32_t)read_buf[TOF_ADDR_DIS]);
    tof_data->dis_status =
        (uint16_t)((uint16_t)read_buf[TOF_ADDR_DIS_STATUS] | ((uint16_t)read_buf[TOF_ADDR_DIS_STATUS + 1] << 8));
    tof_data->signal_strength =
        (uint16_t)((uint16_t)read_buf[TOF_ADDR_SIGNAL_STRENGTH] | ((uint16_t)read_buf[TOF_ADDR_SIGNAL_STRENGTH + 1] << 8));
    tof_data->range_precision = read_buf[TOF_ADDR_RANGE_PRECISION];
    return true;
}

void IIC_Set_ID(uint8_t current_id, uint8_t new_id)
{
    uint8_t slave_addr = TOF_BASE_I2C_ADDR + current_id;
    I2C_Write_Nbyte_ByAddr(slave_addr, TOF_ADDR_ID, &new_id, TOF_SIZE_ID);
    delay(10);
}

void IIC_Change_Mode_To_UART(uint8_t id)
{
    uint8_t data       = IIC_CHANGE_TO_UART_DATA;
    uint8_t slave_addr = TOF_BASE_I2C_ADDR + id;
    I2C_Write_Nbyte_ByAddr(slave_addr, TOF_ADDR_MODE, &data, TOF_SIZE_MODE);
    delay(10);
}

bool TOF_Is_Device_Present(uint8_t id, TofI2cDiagnostics *diag)
{
    uint8_t slave_addr = TOF_BASE_I2C_ADDR + id;
    uint8_t idReg      = 0;

    return I2C_Read_Nbyte_ByAddr(slave_addr, TOF_ADDR_ID, &idReg, TOF_SIZE_ID, diag);
}
