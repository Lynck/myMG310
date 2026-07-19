#include "gw_grayscale.h"

static const uint8_t g_address_reset_magic[8] = {
    0xB8U, 0xD0U, 0xCEU, 0xAAU, 0xBFU, 0xC6U, 0xBCU, 0xBCU
};

static bool GW_Gray_CanRead(const GW_GrayDevice *device)
{
    return (device != NULL) && (device->bus != NULL) &&
           (device->bus->read_registers != NULL);
}

static bool GW_Gray_CanWrite(const GW_GrayDevice *device)
{
    return (device != NULL) && (device->bus != NULL) &&
           (device->bus->write_registers != NULL);
}

void GW_Gray_Init(GW_GrayDevice *device, const GW_GrayBusOps *bus,
                  void *bus_context, uint8_t address7)
{
    if (device == NULL) {
        return;
    }

    device->bus = bus;
    device->bus_context = bus_context;
    device->delay_ms = NULL;
    device->delay_context = NULL;
    device->address7 = address7;
}

void GW_Gray_SetDelay(GW_GrayDevice *device, GW_GrayDelayMs delay_ms,
                      void *delay_context)
{
    if (device == NULL) {
        return;
    }
    device->delay_ms = delay_ms;
    device->delay_context = delay_context;
}

bool GW_Gray_ReadRegister(GW_GrayDevice *device, uint8_t reg, uint8_t *value)
{
    if (!GW_Gray_CanRead(device) || (value == NULL)) {
        return false;
    }
    return device->bus->read_registers(
        device->bus_context, device->address7, reg, value, 1U);
}

bool GW_Gray_WriteRegister(GW_GrayDevice *device, uint8_t reg, uint8_t value)
{
    if (!GW_Gray_CanWrite(device)) {
        return false;
    }
    return device->bus->write_registers(
        device->bus_context, device->address7, reg, &value, 1U);
}

bool GW_Gray_Ping(GW_GrayDevice *device)
{
    uint8_t response = 0U;
    return GW_Gray_ReadRegister(device, GW_GRAY_REG_PING, &response) &&
           (response == GW_GRAY_PING_RESPONSE);
}

bool GW_Gray_ReadDigital(GW_GrayDevice *device, uint8_t *digital)
{
    return GW_Gray_ReadRegister(device, GW_GRAY_REG_DIGITAL, digital);
}

bool GW_Gray_ReadAnalog(GW_GrayDevice *device,
                        uint8_t analog[GW_GRAY_CHANNEL_COUNT])
{
    if (!GW_Gray_CanRead(device) || (analog == NULL)) {
        return false;
    }
    return device->bus->read_registers(device->bus_context, device->address7,
                                        GW_GRAY_REG_ANALOG_BASE, analog,
                                        GW_GRAY_CHANNEL_COUNT);
}

bool GW_Gray_ReadAnalogChannel(GW_GrayDevice *device, uint8_t channel,
                               uint8_t *analog)
{
    if ((channel < 1U) || (channel > GW_GRAY_CHANNEL_COUNT)) {
        return false;
    }
    return GW_Gray_ReadRegister(
        device, (uint8_t)(GW_GRAY_REG_ANALOG_BASE + channel), analog);
}

bool GW_Gray_SetAnalogChannels(GW_GrayDevice *device, uint8_t channel_mask)
{
    return GW_Gray_WriteRegister(
        device, GW_GRAY_REG_CHANNEL_ENABLE, channel_mask);
}

bool GW_Gray_ReadNormalized(GW_GrayDevice *device, uint8_t channel_mask,
                            uint8_t analog[GW_GRAY_CHANNEL_COUNT])
{
    bool read_ok;
    bool disable_ok;

    if ((device == NULL) || (device->delay_ms == NULL) ||
        !GW_Gray_WriteRegister(
            device, GW_GRAY_REG_ANALOG_NORMALIZE, channel_mask)) {
        return false;
    }

    device->delay_ms(device->delay_context, 10U);
    read_ok = GW_Gray_ReadAnalog(device, analog);
    disable_ok = GW_Gray_WriteRegister(
        device, GW_GRAY_REG_ANALOG_NORMALIZE, 0U);
    return read_ok && disable_ok;
}

bool GW_Gray_ReadFirmware(GW_GrayDevice *device, uint8_t *firmware)
{
    return GW_Gray_ReadRegister(device, GW_GRAY_REG_FIRMWARE, firmware);
}

bool GW_Gray_ReadError(GW_GrayDevice *device, uint8_t *error)
{
    return GW_Gray_ReadRegister(device, GW_GRAY_REG_ERROR, error);
}

bool GW_Gray_ChangeAddress(GW_GrayDevice *device, uint8_t new_address7)
{
    if ((new_address7 == 0U) || (new_address7 > 0x7FU) ||
        !GW_Gray_WriteRegister(
            device, GW_GRAY_REG_CHANGE_ADDRESS, new_address7)) {
        return false;
    }
    device->address7 = new_address7;
    return true;
}

bool GW_Gray_ResetAllAddresses(GW_GrayDevice *device)
{
    if ((device == NULL) || (device->bus == NULL) ||
        (device->bus->write_raw == NULL)) {
        return false;
    }
    return device->bus->write_raw(device->bus_context, 0U,
                                  g_address_reset_magic,
                                  (uint8_t)sizeof(g_address_reset_magic));
}

uint8_t GW_Gray_ReverseBits(uint8_t value)
{
    value = (uint8_t)(((value & 0x55U) << 1U) | ((value & 0xAAU) >> 1U));
    value = (uint8_t)(((value & 0x33U) << 2U) | ((value & 0xCCU) >> 2U));
    return (uint8_t)((value << 4U) | (value >> 4U));
}

uint8_t GW_Gray_TransformDigital(uint8_t raw, bool invert, bool reverse)
{
    if (invert) {
        raw = (uint8_t)~raw;
    }
    return reverse ? GW_Gray_ReverseBits(raw) : raw;
}
