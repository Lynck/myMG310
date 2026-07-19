#include "Grayscale_Sensor.h"

#include "gw_grayscale_mspm0.h"

uint8_t g_grayscale_digital[GRAYSCALE_SENSOR_COUNT];

static GW_GraySerial g_gray_serial;
static uint8_t g_gray_serial_initialized;

static uint8_t Grayscale_RemapSerialBits(uint8_t serial_bits)
{
    /* 模块串行输出顺序为 CH8、CH1~CH7；统一转换为 bit7~bit0 对应 CH1~CH8。 */
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
    /* 给可移植驱动提供 MSPM0 平台的微秒延时回调。 */
    (void)context;
    delay_cycles((CPUCLK_FREQ / 1000000U) * microseconds);
}

uint8_t Grayscale_Sensor_Read(void)
{
    uint8_t index;
    uint8_t left_to_right;

    if (g_gray_serial_initialized == 0U) {
        /* 首次读取时初始化，避免在应用层重复配置 CLK/DAT 引脚。 */
        GW_Gray_Serial_Init(&g_gray_serial,
                            GRAYSCALE_SERIAL_CLK_PORT,
                            GRAYSCALE_SERIAL_CLK_PIN,
                            GRAYSCALE_SERIAL_DAT_PORT,
                            GRAYSCALE_SERIAL_DAT_PIN,
                            Grayscale_DelayUs, NULL);
        g_gray_serial_initialized = 1U;
    }

    left_to_right = Grayscale_RemapSerialBits(
        GW_Gray_Serial_Read(&g_gray_serial));
    for (index = 0U; index < GRAYSCALE_SENSOR_COUNT; ++index) {
        /* 数组下标 0~7 始终表示车体从左到右的 CH1~CH8。 */
        g_grayscale_digital[index] =
            (uint8_t)((left_to_right >> (7U - index)) & 1U);
    }
    return left_to_right;
}
