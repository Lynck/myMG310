#include "gw_grayscale_mspm0.h"

#define GW_GRAY_HW_I2C_FIFO_SIZE (8U)

static bool GW_Gray_HardwareI2C_WaitIdle(GW_GrayHardwareI2C *context)
{
    uint32_t remaining;

    if ((context == NULL) || (context->i2c == NULL)) {
        return false;
    }

    remaining = context->timeout_loops;
    while (remaining-- > 0U) {
        uint32_t status = DL_I2C_getControllerStatus(context->i2c);
        if ((status & DL_I2C_CONTROLLER_STATUS_ERROR) != 0U) {
            return false;
        }
        if (((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) &&
            ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U)) {
            return true;
        }
    }
    return false;
}

static bool GW_Gray_HardwareI2C_Transfer(
    GW_GrayHardwareI2C *context, uint8_t address7,
    DL_I2C_CONTROLLER_DIRECTION direction, const uint8_t *tx_data,
    uint8_t *rx_data, uint8_t length)
{
    uint8_t index;

    if ((context == NULL) || (context->i2c == NULL) || (length == 0U) ||
        (length > GW_GRAY_HW_I2C_FIFO_SIZE) ||
        ((direction == DL_I2C_CONTROLLER_DIRECTION_TX) && (tx_data == NULL)) ||
        ((direction == DL_I2C_CONTROLLER_DIRECTION_RX) && (rx_data == NULL)) ||
        !GW_Gray_HardwareI2C_WaitIdle(context)) {
        return false;
    }

    if (direction == DL_I2C_CONTROLLER_DIRECTION_TX) {
        DL_I2C_flushControllerTXFIFO(context->i2c);
        if (DL_I2C_fillControllerTXFIFO(context->i2c, tx_data, length) !=
            length) {
            return false;
        }
    } else {
        DL_I2C_flushControllerRXFIFO(context->i2c);
    }

    DL_I2C_startControllerTransfer(
        context->i2c, address7, direction, length);
    if (!GW_Gray_HardwareI2C_WaitIdle(context)) {
        return false;
    }

    if (direction == DL_I2C_CONTROLLER_DIRECTION_RX) {
        for (index = 0U; index < length; ++index) {
            uint32_t remaining = context->timeout_loops;
            while (DL_I2C_isControllerRXFIFOEmpty(context->i2c) &&
                   (remaining-- > 0U)) {
            }
            if (DL_I2C_isControllerRXFIFOEmpty(context->i2c)) {
                return false;
            }
            rx_data[index] = DL_I2C_receiveControllerData(context->i2c);
        }
    }
    return true;
}

static bool GW_Gray_HardwareI2C_ReadRegisters(
    void *bus_context, uint8_t address7, uint8_t reg,
    uint8_t *data, uint8_t length)
{
    GW_GrayHardwareI2C *context = (GW_GrayHardwareI2C *)bus_context;
    return GW_Gray_HardwareI2C_Transfer(
               context, address7, DL_I2C_CONTROLLER_DIRECTION_TX,
               &reg, NULL, 1U) &&
           GW_Gray_HardwareI2C_Transfer(
               context, address7, DL_I2C_CONTROLLER_DIRECTION_RX,
               NULL, data, length);
}

static bool GW_Gray_HardwareI2C_WriteRegisters(
    void *bus_context, uint8_t address7, uint8_t reg,
    const uint8_t *data, uint8_t length)
{
    uint8_t packet[GW_GRAY_HW_I2C_FIFO_SIZE];
    uint8_t index;

    if ((data == NULL) || (length == 0U) ||
        (length >= GW_GRAY_HW_I2C_FIFO_SIZE)) {
        return false;
    }
    packet[0] = reg;
    for (index = 0U; index < length; ++index) {
        packet[index + 1U] = data[index];
    }
    return GW_Gray_HardwareI2C_Transfer(
        (GW_GrayHardwareI2C *)bus_context, address7,
        DL_I2C_CONTROLLER_DIRECTION_TX, packet, NULL,
        (uint8_t)(length + 1U));
}

static bool GW_Gray_HardwareI2C_WriteRaw(
    void *bus_context, uint8_t address7, const uint8_t *data, uint8_t length)
{
    return GW_Gray_HardwareI2C_Transfer(
        (GW_GrayHardwareI2C *)bus_context, address7,
        DL_I2C_CONTROLLER_DIRECTION_TX, data, NULL, length);
}

const GW_GrayBusOps GW_Gray_HardwareI2C_Bus = {
    GW_Gray_HardwareI2C_ReadRegisters,
    GW_Gray_HardwareI2C_WriteRegisters,
    GW_Gray_HardwareI2C_WriteRaw
};

void GW_Gray_HardwareI2C_Init(GW_GrayHardwareI2C *context,
                              I2C_Regs *i2c, uint32_t timeout_loops)
{
    if (context == NULL) {
        return;
    }
    context->i2c = i2c;
    context->timeout_loops = timeout_loops;
}

static void GW_Gray_SoftwareI2C_Delay(GW_GraySoftwareI2C *context)
{
    context->delay_us(context->delay_context, context->half_period_us);
}

static void GW_Gray_SoftwareI2C_SCLLow(GW_GraySoftwareI2C *context)
{
    DL_GPIO_clearPins(context->scl_port, context->scl_pin);
    DL_GPIO_enableOutput(context->scl_port, context->scl_pin);
}

static void GW_Gray_SoftwareI2C_SCLRelease(GW_GraySoftwareI2C *context)
{
    DL_GPIO_disableOutput(context->scl_port, context->scl_pin);
}

static void GW_Gray_SoftwareI2C_SDALow(GW_GraySoftwareI2C *context)
{
    DL_GPIO_clearPins(context->sda_port, context->sda_pin);
    DL_GPIO_enableOutput(context->sda_port, context->sda_pin);
}

static void GW_Gray_SoftwareI2C_SDARelease(GW_GraySoftwareI2C *context)
{
    DL_GPIO_disableOutput(context->sda_port, context->sda_pin);
}

static bool GW_Gray_SoftwareI2C_SDARead(GW_GraySoftwareI2C *context)
{
    return DL_GPIO_readPins(context->sda_port, context->sda_pin) != 0U;
}

static void GW_Gray_SoftwareI2C_Start(GW_GraySoftwareI2C *context)
{
    GW_Gray_SoftwareI2C_SDARelease(context);
    GW_Gray_SoftwareI2C_SCLRelease(context);
    GW_Gray_SoftwareI2C_Delay(context);
    GW_Gray_SoftwareI2C_SDALow(context);
    GW_Gray_SoftwareI2C_Delay(context);
    GW_Gray_SoftwareI2C_SCLLow(context);
}

static void GW_Gray_SoftwareI2C_Stop(GW_GraySoftwareI2C *context)
{
    GW_Gray_SoftwareI2C_SDALow(context);
    GW_Gray_SoftwareI2C_Delay(context);
    GW_Gray_SoftwareI2C_SCLRelease(context);
    GW_Gray_SoftwareI2C_Delay(context);
    GW_Gray_SoftwareI2C_SDARelease(context);
    GW_Gray_SoftwareI2C_Delay(context);
}

static bool GW_Gray_SoftwareI2C_SendByte(
    GW_GraySoftwareI2C *context, uint8_t value)
{
    uint8_t index;
    for (index = 0U; index < 8U; ++index) {
        if ((value & 0x80U) != 0U) {
            GW_Gray_SoftwareI2C_SDARelease(context);
        } else {
            GW_Gray_SoftwareI2C_SDALow(context);
        }
        value <<= 1U;
        GW_Gray_SoftwareI2C_Delay(context);
        GW_Gray_SoftwareI2C_SCLRelease(context);
        GW_Gray_SoftwareI2C_Delay(context);
        GW_Gray_SoftwareI2C_SCLLow(context);
    }

    GW_Gray_SoftwareI2C_SDARelease(context);
    GW_Gray_SoftwareI2C_Delay(context);
    GW_Gray_SoftwareI2C_SCLRelease(context);
    GW_Gray_SoftwareI2C_Delay(context);
    {
        bool acknowledged = !GW_Gray_SoftwareI2C_SDARead(context);
        GW_Gray_SoftwareI2C_SCLLow(context);
        return acknowledged;
    }
}

static uint8_t GW_Gray_SoftwareI2C_ReceiveByte(
    GW_GraySoftwareI2C *context, bool acknowledge)
{
    uint8_t index;
    uint8_t value = 0U;

    GW_Gray_SoftwareI2C_SDARelease(context);
    for (index = 0U; index < 8U; ++index) {
        value <<= 1U;
        GW_Gray_SoftwareI2C_SCLRelease(context);
        GW_Gray_SoftwareI2C_Delay(context);
        if (GW_Gray_SoftwareI2C_SDARead(context)) {
            value |= 1U;
        }
        GW_Gray_SoftwareI2C_SCLLow(context);
        GW_Gray_SoftwareI2C_Delay(context);
    }

    if (acknowledge) {
        GW_Gray_SoftwareI2C_SDALow(context);
    } else {
        GW_Gray_SoftwareI2C_SDARelease(context);
    }
    GW_Gray_SoftwareI2C_Delay(context);
    GW_Gray_SoftwareI2C_SCLRelease(context);
    GW_Gray_SoftwareI2C_Delay(context);
    GW_Gray_SoftwareI2C_SCLLow(context);
    GW_Gray_SoftwareI2C_SDARelease(context);
    return value;
}

static bool GW_Gray_SoftwareI2C_WriteAddress(
    GW_GraySoftwareI2C *context, uint8_t address7, bool read)
{
    return GW_Gray_SoftwareI2C_SendByte(
        context, (uint8_t)((address7 << 1U) | (read ? 1U : 0U)));
}

static bool GW_Gray_SoftwareI2C_ReadRegisters(
    void *bus_context, uint8_t address7, uint8_t reg,
    uint8_t *data, uint8_t length)
{
    GW_GraySoftwareI2C *context = (GW_GraySoftwareI2C *)bus_context;
    uint8_t index;

    if ((context == NULL) || (data == NULL) || (length == 0U) ||
        (context->delay_us == NULL)) {
        return false;
    }
    GW_Gray_SoftwareI2C_Start(context);
    if (!GW_Gray_SoftwareI2C_WriteAddress(context, address7, false) ||
        !GW_Gray_SoftwareI2C_SendByte(context, reg)) {
        GW_Gray_SoftwareI2C_Stop(context);
        return false;
    }
    GW_Gray_SoftwareI2C_Start(context);
    if (!GW_Gray_SoftwareI2C_WriteAddress(context, address7, true)) {
        GW_Gray_SoftwareI2C_Stop(context);
        return false;
    }
    for (index = 0U; index < length; ++index) {
        data[index] = GW_Gray_SoftwareI2C_ReceiveByte(
            context, index < (uint8_t)(length - 1U));
    }
    GW_Gray_SoftwareI2C_Stop(context);
    return true;
}

static bool GW_Gray_SoftwareI2C_WriteRegisters(
    void *bus_context, uint8_t address7, uint8_t reg,
    const uint8_t *data, uint8_t length)
{
    GW_GraySoftwareI2C *context = (GW_GraySoftwareI2C *)bus_context;
    uint8_t index;

    if ((context == NULL) || (data == NULL) || (length == 0U) ||
        (context->delay_us == NULL)) {
        return false;
    }
    GW_Gray_SoftwareI2C_Start(context);
    if (!GW_Gray_SoftwareI2C_WriteAddress(context, address7, false) ||
        !GW_Gray_SoftwareI2C_SendByte(context, reg)) {
        GW_Gray_SoftwareI2C_Stop(context);
        return false;
    }
    for (index = 0U; index < length; ++index) {
        if (!GW_Gray_SoftwareI2C_SendByte(context, data[index])) {
            GW_Gray_SoftwareI2C_Stop(context);
            return false;
        }
    }
    GW_Gray_SoftwareI2C_Stop(context);
    return true;
}

static bool GW_Gray_SoftwareI2C_WriteRaw(
    void *bus_context, uint8_t address7, const uint8_t *data, uint8_t length)
{
    GW_GraySoftwareI2C *context = (GW_GraySoftwareI2C *)bus_context;
    uint8_t index;

    if ((context == NULL) || (data == NULL) || (length == 0U) ||
        (context->delay_us == NULL)) {
        return false;
    }
    GW_Gray_SoftwareI2C_Start(context);
    if (!GW_Gray_SoftwareI2C_WriteAddress(context, address7, false)) {
        GW_Gray_SoftwareI2C_Stop(context);
        return false;
    }
    for (index = 0U; index < length; ++index) {
        if (!GW_Gray_SoftwareI2C_SendByte(context, data[index])) {
            GW_Gray_SoftwareI2C_Stop(context);
            return false;
        }
    }
    GW_Gray_SoftwareI2C_Stop(context);
    return true;
}

const GW_GrayBusOps GW_Gray_SoftwareI2C_Bus = {
    GW_Gray_SoftwareI2C_ReadRegisters,
    GW_Gray_SoftwareI2C_WriteRegisters,
    GW_Gray_SoftwareI2C_WriteRaw
};

void GW_Gray_SoftwareI2C_Init(GW_GraySoftwareI2C *context,
                              GPIO_Regs *scl_port, uint32_t scl_pin,
                              GPIO_Regs *sda_port, uint32_t sda_pin,
                              GW_GrayDelayUs delay_us, void *delay_context,
                              uint32_t half_period_us)
{
    if (context == NULL) {
        return;
    }
    context->scl_port = scl_port;
    context->scl_pin = scl_pin;
    context->sda_port = sda_port;
    context->sda_pin = sda_pin;
    context->delay_us = delay_us;
    context->delay_context = delay_context;
    context->half_period_us = half_period_us;

    DL_GPIO_clearPins(scl_port, scl_pin);
    DL_GPIO_clearPins(sda_port, sda_pin);
    DL_GPIO_disableOutput(scl_port, scl_pin);
    DL_GPIO_disableOutput(sda_port, sda_pin);
}

void GW_Gray_Serial_Init(GW_GraySerial *context,
                         GPIO_Regs *clk_port, uint32_t clk_pin,
                         GPIO_Regs *dat_port, uint32_t dat_pin,
                         GW_GrayDelayUs delay_us, void *delay_context)
{
    if (context == NULL) {
        return;
    }
    context->clk_port = clk_port;
    context->clk_pin = clk_pin;
    context->dat_port = dat_port;
    context->dat_pin = dat_pin;
    context->delay_us = delay_us;
    context->delay_context = delay_context;
    DL_GPIO_setPins(clk_port, clk_pin);
}

uint8_t GW_Gray_Serial_Read(const GW_GraySerial *context)
{
    uint8_t index;
    uint8_t value = 0U;

    if ((context == NULL) || (context->delay_us == NULL)) {
        return 0U;
    }
    for (index = 0U; index < 8U; ++index) {
        DL_GPIO_clearPins(context->clk_port, context->clk_pin);
        context->delay_us(context->delay_context, 2U);
        if (DL_GPIO_readPins(context->dat_port, context->dat_pin) != 0U) {
            value |= (uint8_t)(1U << index);
        }
        DL_GPIO_setPins(context->clk_port, context->clk_pin);
        context->delay_us(context->delay_context, 5U);
    }
    return value;
}
