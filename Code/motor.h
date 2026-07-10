#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "ti_msp_dl_config.h"

void Motor_Init(void);
void Motor_SetSpeed_A(int16_t speed);
void Motor_SetSpeed_B(int16_t speed);
void Motor_Brake(void);

#endif