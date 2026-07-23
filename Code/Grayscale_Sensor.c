#include "Grayscale_Sensor.h"

#include "../Drivers/GW_Grayscale/gw_grayscale_mspm0.h"

uint8_t g_grayscale_digital[GRAYSCALE_SENSOR_COUNT];

static GW_GraySerial g_gray_serial;

static uint8_t Grayscale_RemapSerialBits(uint8_t serial_bits)
{
    /* 模块依次输出 CH8、CH1~CH7，统一为 bit7~bit0 对应 CH1~CH8。 */
    return (uint8_t)((serial_bits & 0x01U) |
                     ((serial_bits & 0x02U) << 6U) |
                     ((serial_bits & 0x04U) << 4U) |
                     ((serial_bits & 0x08U) << 2U) |
                     (serial_bits & 0x10U) |
                     ((serial_bits & 0x20U) >> 2U) |
                     ((serial_bits & 0x40U) >> 4U) |
                     ((serial_bits & 0x80U) >> 6U));
}

static void Grayscale_DelayUs(void *context, uint32_t microseconds)
{
    (void)context;
    delay_cycles((CPUCLK_FREQ / 1000000U) * microseconds);
}

void Grayscale_Sensor_Init(void)
{
    GW_Gray_Serial_Init(&g_gray_serial,
                        GRAYSCALE_SERIAL_CLK_PORT,
                        GRAYSCALE_SERIAL_CLK_PIN,
                        GRAYSCALE_SERIAL_DAT_PORT,
                        GRAYSCALE_SERIAL_DAT_PIN,
                        Grayscale_DelayUs, NULL);
}

uint8_t Grayscale_Sensor_Read(void)
{
    uint8_t index;
    uint8_t black_mask = Grayscale_RemapSerialBits(
        GW_Gray_Serial_Read(&g_gray_serial));

#if (GRAYSCALE_BLACK_LEVEL == 0U)
    black_mask = (uint8_t)~black_mask;
#endif

    for (index = 0U; index < GRAYSCALE_SENSOR_COUNT; ++index) {
        g_grayscale_digital[index] =
            (uint8_t)((black_mask >> (7U - index)) & 1U);
    }
    return black_mask;
}
