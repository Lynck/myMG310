#ifndef __MYPID_H
#define __MYPID_H

#include <stdbool.h>

#include "ti_msp_dl_config.h"

/* 可通过蓝牙动态调整的循迹参数；g_base_speed 单位为 cm/s。 */
extern volatile int16_t g_base_speed;
extern volatile float g_left_wheel_scale;
extern volatile float g_right_wheel_scale;
extern volatile bool g_line_follow_enabled;

void Tracking_PID_Init(void);
void Tracking_PID_Reset(void);
void Tracking_Process(void);

void Tracking_PID2_Init(void);

#endif
