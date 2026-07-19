#ifndef GRAYSCALE_SENSOR_H_
#define GRAYSCALE_SENSOR_H_

#include "ti_msp_dl_config.h"

#define GRAYSCALE_SENSOR_COUNT        (8U)
/* 当前模块黑线输出低电平；如果实测黑线为高电平才改成 1U。 */
#define GRAYSCALE_BLACK_LEVEL         (0U)

/*
 * 应用适配层使用的 SysConfig GPIO 名称。
 * 保留现有 SCL/SDA 实例名，只需要在 SysConfig 中选择实际 CLK/DAT 引脚。
 */
#define GRAYSCALE_SERIAL_CLK_PORT     (SCL_PORT)
#define GRAYSCALE_SERIAL_CLK_PIN      (SCL_SCK_PIN_PIN)
#define GRAYSCALE_SERIAL_DAT_PORT     (SDA_PORT)
#define GRAYSCALE_SERIAL_DAT_PIN      (SDA_SDA_PIN_PIN)

/* 数组下标 0~7 对应车体从左到右的 CH1~CH8。 */
extern uint8_t g_grayscale_digital[GRAYSCALE_SENSOR_COUNT];

/* 返回原始电平：bit7=左侧 CH1，bit0=右侧 CH8。 */
uint8_t Grayscale_Sensor_Read(void);

#endif
