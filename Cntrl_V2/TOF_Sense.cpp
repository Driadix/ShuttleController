#include "TOF_Sense.h"
#include <string.h>

/*****************************************************************************
 * | File       :   TOF_Sense.cpp
 * | Author      :   Waveshare team + Adapted for multi-device
 * | Function    :   ToF driver with multi-device support
 ******************************************************************************/

// STM32duino Wire uses the standard Arduino return values: 2/3 are NACKs;
// every other non-zero write-phase result remains a generic Wire error here.
static constexpr uint8_t kWireNackAddress = 2U;
static constexpr uint8_t kWireNackData    = 3U;
static constexpr uint8_t kWireReadFailed  = 0xFEU;

static void tofSetDiag(
    TofI2cDiagnostics *diag,
    TofI2cStatus       status,
    uint8_t            address,
    uint8_t            reg,
    uint8_t            expected,
    uint8_t            received,
    uint8_t            wireStatus)
{
    if (diag == nullptr)
    {
        return;
    }

    diag->status     = status;
    diag->address    = address;
    diag->reg        = reg;
    diag->expected   = expected;
    diag->received   = received;
    diag->wireStatus = wireStatus;
}

static bool tofAddressForId(uint8_t id, uint8_t *address, TofI2cDiagnostics *diag)
{
    if (address == nullptr || id > (0x77U - TOF_BASE_I2C_ADDR))
    {
        tofSetDiag(diag, TOF_I2C_INVALID_ARG, 0, 0, id, 0, 0);
        return false;
    }

    *address = TOF_BASE_I2C_ADDR + id;
    return true;
}

static TofI2cStatus tofStatusFromWire(uint8_t wireStatus)
{
    if (wireStatus == 0U)
    {
        return TOF_I2C_OK;
    }
    if (wireStatus == kWireNackAddress || wireStatus == kWireNackData)
    {
        return TOF_I2C_NO_ACK;
    }
    return TOF_I2C_WIRE_ERROR;
}

// Reads one contiguous register block using only the Arduino Wire owner of
// this peripheral.  Both transactions finish with STOP, matching the
// Waveshare example path and avoiding direct HAL/Wire state-machine mixing.
static bool I2C_Read_Nbyte_ByAddr(
    uint8_t slave_addr, uint8_t reg_addr, uint8_t *pdata, uint8_t len, TofI2cDiagnostics *diag)
{
    if (pdata == nullptr || len == 0U)
    {
        tofSetDiag(diag, TOF_I2C_INVALID_ARG, slave_addr, reg_addr, len, 0, 0);
        return false;
    }

    Wire.beginTransmission(slave_addr);
    if (Wire.write(reg_addr) != 1U)
    {
        tofSetDiag(diag, TOF_I2C_WIRE_ERROR, slave_addr, reg_addr, len, 0, 1U);
        return false;
    }

    const uint8_t txStatus = Wire.endTransmission(true);
    const TofI2cStatus txResult = tofStatusFromWire(txStatus);
    if (txResult != TOF_I2C_OK)
    {
        tofSetDiag(diag, txResult, slave_addr, reg_addr, len, 0, txStatus);
        return false;
    }

    const uint8_t requested = Wire.requestFrom(slave_addr, len);
    uint8_t received = 0U;
    while (Wire.available() > 0 && received < len)
    {
        pdata[received++] = (uint8_t)Wire.read();
    }
    while (Wire.available() > 0)
    {
        (void)Wire.read();
    }

    if (requested == 0U)
    {
        // STM32duino deliberately collapses every read-phase failure to zero.
        // Preserve that uncertainty: one such result is not evidence that the
        // whole shared bus needs to be reset.
        tofSetDiag(diag, TOF_I2C_READ_FAILED_UNKNOWN, slave_addr, reg_addr, len, received, kWireReadFailed);
        return false;
    }

    if (requested != len)
    {
        tofSetDiag(diag, TOF_I2C_SHORT_READ, slave_addr, reg_addr, len, received, requested);
        return false;
    }

    if (received != len)
    {
        tofSetDiag(diag, TOF_I2C_SHORT_READ, slave_addr, reg_addr, len, received, requested);
        return false;
    }

    tofSetDiag(diag, TOF_I2C_OK, slave_addr, reg_addr, len, len, 0);
    return true;
}

static bool I2C_Write_Nbyte_ByAddr(uint8_t slave_addr, uint8_t reg_addr, const uint8_t *pdata, uint8_t len)
{
    if (pdata == nullptr || len == 0U)
    {
        return false;
    }

    Wire.beginTransmission(slave_addr);
    if (Wire.write(reg_addr) != 1U || Wire.write(pdata, len) != len)
    {
        return false;
    }
    return Wire.endTransmission(true) == 0U;
}

const char *TOF_I2C_Status_Name(TofI2cStatus status)
{
    switch (status)
    {
    case TOF_I2C_OK:
        return "ok";
    case TOF_I2C_INVALID_ARG:
        return "arg";
    case TOF_I2C_WIRE_ERROR:
        return "wire_err";
    case TOF_I2C_WIRE_TIMEOUT:
        return "wire_to";
    case TOF_I2C_SHORT_READ:
        return "short";
    case TOF_I2C_ID_MISMATCH:
        return "id_mis";
    case TOF_I2C_NO_ACK:
        return "noack";
    case TOF_I2C_BUS_STUCK:
        return "stuck";
    case TOF_I2C_READ_FAILED_UNKNOWN:
        return "read_unknown";
    default:
        return "unknown";
    }
}

bool TOF_Inquire_I2C_Decoding_ByID(uint8_t id, TOF_Parameter *tof_data, TofI2cDiagnostics *diag)
{
    if (tof_data == nullptr)
    {
        tofSetDiag(diag, TOF_I2C_INVALID_ARG, 0, 0, 0, 0, 0);
        return false;
    }

    uint8_t slave_addr = 0U;
    if (!tofAddressForId(id, &slave_addr, diag))
    {
        memset(tof_data, 0, sizeof(TOF_Parameter));
        return false;
    }

    uint8_t read_buf[TOF_MEASUREMENT_BLOCK_SIZE] = {};
    if (!I2C_Read_Nbyte_ByAddr(slave_addr, TOF_MEASUREMENT_REG_START, read_buf, TOF_MEASUREMENT_BLOCK_SIZE, diag))
    {
        memset(tof_data, 0, sizeof(TOF_Parameter));
        return false;
    }

    // read_buf[0..3]   = system_time       @0x20, uint32_t LE
    // read_buf[4..7]   = dis               @0x24, uint32_t LE
    // read_buf[8..9]   = dis_status        @0x28, uint16_t LE
    // read_buf[10..11] = signal_strength   @0x2A, uint16_t LE
    // read_buf[12]     = range_precision   @0x2C, uint8_t
    tof_data->system_time = (uint32_t)(
        ((uint32_t)read_buf[3] << 24) | ((uint32_t)read_buf[2] << 16) |
        ((uint32_t)read_buf[1] << 8) | (uint32_t)read_buf[0]);
    tof_data->dis = (uint32_t)(
        ((uint32_t)read_buf[7] << 24) | ((uint32_t)read_buf[6] << 16) |
        ((uint32_t)read_buf[5] << 8) | (uint32_t)read_buf[4]);
    tof_data->dis_status =
        (uint16_t)((uint16_t)read_buf[8] | ((uint16_t)read_buf[9] << 8));
    tof_data->signal_strength =
        (uint16_t)((uint16_t)read_buf[10] | ((uint16_t)read_buf[11] << 8));
    tof_data->range_precision = read_buf[12];

    tof_data->interface_mode = 3U;
    tof_data->id             = id;
    tof_data->uart_baudrate  = 0U;
    return true;
}

void IIC_Set_ID(uint8_t current_id, uint8_t new_id)
{
    uint8_t slave_addr = 0U;
    if (tofAddressForId(current_id, &slave_addr, nullptr))
    {
        (void)I2C_Write_Nbyte_ByAddr(slave_addr, TOF_ADDR_ID, &new_id, TOF_SIZE_ID);
    }
    delay(10);
}

bool TOF_Is_Device_Present(uint8_t id, TofI2cDiagnostics *diag)
{
    uint8_t slave_addr = 0U;
    uint8_t idReg      = 0U;
    if (!tofAddressForId(id, &slave_addr, diag))
    {
        return false;
    }

    if (!I2C_Read_Nbyte_ByAddr(slave_addr, TOF_ADDR_ID, &idReg, TOF_SIZE_ID, diag))
    {
        return false;
    }

    if (idReg != id)
    {
        tofSetDiag(diag, TOF_I2C_ID_MISMATCH, slave_addr, TOF_ADDR_ID, id, idReg, 0);
        return false;
    }
    return true;
}
