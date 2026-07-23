#ifndef GRAYSCALE_SENSOR_H_
#define GRAYSCALE_SENSOR_H_

#include "ti_msp_dl_config.h"

#define GRAYSCALE_SENSOR_COUNT (8U)

/* 当前实物原始电平为黑=0、白=1；读取函数取反后统一为黑=1。 */
#ifndef GRAYSCALE_BLACK_LEVEL
#define GRAYSCALE_BLACK_LEVEL (0U)
#endif

#define GRAYSCALE_SERIAL_CLK_PORT (SCL_PORT)
#define GRAYSCALE_SERIAL_CLK_PIN  (SCL_SCK_PIN_PIN)
#define GRAYSCALE_SERIAL_DAT_PORT (SDA_PORT)
#define GRAYSCALE_SERIAL_DAT_PIN  (SDA_SDA_PIN_PIN)

/* 下标 0~7 和 bit7~bit0 均对应车体从左到右的 CH1~CH8。 */
extern uint8_t g_grayscale_digital[GRAYSCALE_SENSOR_COUNT];

void Grayscale_Sensor_Init(void);
uint8_t Grayscale_Sensor_Read(void);

#endif
