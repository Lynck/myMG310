#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "interrupt.h"

/* Main loop flags shared with timer interrupts. */
extern volatile bool start_100ms_timer;
extern volatile bool check_100ms_flag;
extern bool OLED_Flag;
extern bool timer_10ms_flag;

/* 读取最近一帧校验正确的航向角，尚未收到有效帧时返回 false。 */
bool Gyro_GetAngle(float *angle_deg);

#endif  /* _MAIN_H_ */
