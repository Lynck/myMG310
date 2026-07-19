#ifndef __MOTOR_SPEED_H
#define __MOTOR_SPEED_H

#include <stdint.h>

#define MOTOR_SPEED_PULSES_PER_REV      13
#define MOTOR_SPEED_GEAR_RATIO          20
#define MOTOR_SPEED_PULSES_PER_WHEEL    (MOTOR_SPEED_PULSES_PER_REV * MOTOR_SPEED_GEAR_RATIO)

/* 编码器标定参数：轮径或方向不一致时只在这里调整。 */
#define MOTOR_SPEED_WHEEL_CIRCUMFERENCE_M   (0.1508f) /* 48 mm wheel diameter. */
#define MOTOR_SPEED_A_DIR_HIGH_FORWARD      1
#define MOTOR_SPEED_B_DIR_HIGH_FORWARD      1

/*
 * 双轮速度PI参数，控制周期为10 ms，输出叠加到前馈PWM命令上。
 * 调参顺序：先调 KP 获得响应速度，再小幅增加 KI 消除稳态误差，KD 保持 0。
 */
#define MOTOR_SPEED_PID_KP                  2.2f
#define MOTOR_SPEED_PID_KI                  0.05f
#define MOTOR_SPEED_PID_KD                  0.0f
/* 速度到PWM的前馈系数；0.5 m/s 时初始命令约为 38.5。 */
#define MOTOR_SPEED_FEEDFORWARD_CMD_PER_MPS 56.0f
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
void MotorSpeed_ControlWheels(float target_A_mps, float target_B_mps,
                              float scale_A, float scale_B);
char MotorSpeed_DirChar(int8_t dir);

#endif
