#ifndef __MOTOR_SPEED_H
#define __MOTOR_SPEED_H

#include <stdint.h>

#define MOTOR_SPEED_PULSES_PER_REV      13
#define MOTOR_SPEED_GEAR_RATIO          30
#define MOTOR_SPEED_PULSES_PER_WHEEL    (MOTOR_SPEED_PULSES_PER_REV * MOTOR_SPEED_GEAR_RATIO)

/* ponytail: calibration knobs; measure the wheel and flip polarity here if the real car disagrees. */
#define MOTOR_SPEED_WHEEL_CIRCUMFERENCE_M   (0.1508f) /* 48 mm wheel diameter. */
#define MOTOR_SPEED_A_DIR_HIGH_FORWARD      1
#define MOTOR_SPEED_B_DIR_HIGH_FORWARD      1

/* 速度闭环参数：输出是 Motor_SetSpeed 的 -100..100 命令值。 */
#define MOTOR_SPEED_PID_KP                  10.0f
#define MOTOR_SPEED_PID_KI                  0.10f
#define MOTOR_SPEED_PID_KD                  0.0f
#define MOTOR_SPEED_FEEDFORWARD_CMD_PER_MPS 77.0f
#define MOTOR_SPEED_MIN_FORWARD_CMD         18.0f
#define MOTOR_SPEED_BASE_CMD_MAX            80.0f
#define MOTOR_SPEED_PID_OUT_MAX             25.0f
#define MOTOR_SPEED_PID_OUT_MIN             -25.0f
#define MOTOR_SPEED_PID_INT_MAX             40.0f
#define MOTOR_SPEED_PID_INT_MIN             -40.0f

#define MOTOR_SPEED_DIR_STOP       0
#define MOTOR_SPEED_DIR_FORWARD    1
#define MOTOR_SPEED_DIR_REVERSE   -1

extern volatile int32_t motor_speed_count_A;
extern volatile int32_t motor_speed_count_B;
extern volatile float motor_speed_A_mps;
extern volatile float motor_speed_B_mps;
extern volatile int8_t motor_speed_dir_A;
extern volatile int8_t motor_speed_dir_B;

void MotorSpeed_Init(void);
void MotorSpeed_OnPulseA(void);
void MotorSpeed_OnPulseB(void);
void MotorSpeed_Update(float dt_s);
void MotorSpeed_ResetControl(void);
void MotorSpeed_Control(float target_mps, int16_t turn_cmd, float left_scale, float right_scale);
char MotorSpeed_DirChar(int8_t dir);

#endif
