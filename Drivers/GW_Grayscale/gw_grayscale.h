#ifndef GW_GRAYSCALE_H_
#define GW_GRAYSCALE_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GW_GRAY_DEFAULT_ADDRESS          (0x4CU)
#define GW_GRAY_CHANNEL_COUNT            (8U)

#define GW_GRAY_REG_PING                 (0xAAU)
#define GW_GRAY_PING_RESPONSE            (0x66U)
#define GW_GRAY_REG_ANALOG_BASE          (0xB0U)
#define GW_GRAY_REG_REBOOT               (0xC0U)
#define GW_GRAY_REG_FIRMWARE             (0xC1U)
#define GW_GRAY_REG_CHANNEL_ENABLE       (0xCEU)
#define GW_GRAY_REG_ANALOG_NORMALIZE     (0xCFU)
#define GW_GRAY_REG_CALIBRATION_BLACK    (0xD0U)
#define GW_GRAY_REG_CALIBRATION_WHITE    (0xD1U)
#define GW_GRAY_REG_DIGITAL              (0xDDU)
#define GW_GRAY_REG_ERROR                (0xDEU)
#define GW_GRAY_REG_CHANGE_ADDRESS       (0xADU)

typedef bool (*GW_GrayReadRegisters)(void *context, uint8_t address7,
                                     uint8_t reg, uint8_t *data,
                                     uint8_t length);
typedef bool (*GW_GrayWriteRegisters)(void *context, uint8_t address7,
                                      uint8_t reg, const uint8_t *data,
                                      uint8_t length);
typedef bool (*GW_GrayWriteRaw)(void *context, uint8_t address7,
                                const uint8_t *data, uint8_t length);
typedef void (*GW_GrayDelayMs)(void *context, uint32_t milliseconds);

typedef struct {
    GW_GrayReadRegisters read_registers;
    GW_GrayWriteRegisters write_registers;
    GW_GrayWriteRaw write_raw;
} GW_GrayBusOps;

typedef struct {
    const GW_GrayBusOps *bus;
    void *bus_context;
    GW_GrayDelayMs delay_ms;
    void *delay_context;
    uint8_t address7;
} GW_GrayDevice;

void GW_Gray_Init(GW_GrayDevice *device, const GW_GrayBusOps *bus,
                  void *bus_context, uint8_t address7);
void GW_Gray_SetDelay(GW_GrayDevice *device, GW_GrayDelayMs delay_ms,
                      void *delay_context);

bool GW_Gray_ReadRegister(GW_GrayDevice *device, uint8_t reg, uint8_t *value);
bool GW_Gray_WriteRegister(GW_GrayDevice *device, uint8_t reg, uint8_t value);
bool GW_Gray_Ping(GW_GrayDevice *device);
bool GW_Gray_ReadDigital(GW_GrayDevice *device, uint8_t *digital);
bool GW_Gray_ReadAnalog(GW_GrayDevice *device,
                        uint8_t analog[GW_GRAY_CHANNEL_COUNT]);
bool GW_Gray_ReadAnalogChannel(GW_GrayDevice *device, uint8_t channel,
                               uint8_t *analog);
bool GW_Gray_SetAnalogChannels(GW_GrayDevice *device, uint8_t channel_mask);
bool GW_Gray_ReadNormalized(GW_GrayDevice *device, uint8_t channel_mask,
                            uint8_t analog[GW_GRAY_CHANNEL_COUNT]);
bool GW_Gray_ReadFirmware(GW_GrayDevice *device, uint8_t *firmware);
bool GW_Gray_ReadError(GW_GrayDevice *device, uint8_t *error);
bool GW_Gray_ChangeAddress(GW_GrayDevice *device, uint8_t new_address7);
bool GW_Gray_ResetAllAddresses(GW_GrayDevice *device);

uint8_t GW_Gray_ReverseBits(uint8_t value);
uint8_t GW_Gray_TransformDigital(uint8_t raw, bool invert, bool reverse);

#ifdef __cplusplus
}
#endif

#endif
