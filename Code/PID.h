#ifndef __PID_H
#define __PID_H

#include "ti_msp_dl_config.h"

/**
 * @brief PID控制算法结构体
 */
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float Target;   // 目标值
    float Actual;   // 实际测量值
    float Out;      // 输出值

    float Error0;   // 本次误差
    float Error1;   // 上次误差
    float Error2;   // 上上次误差 (用于增量式)
    float ErrorInt; // 积分项

    float OutMax;   // 输出上限
    float OutMin;   // 输出下限
    float IntMax;   // 积分上限 (设为0则默认绑定OutMax)
    float IntMin;   // 积分下限 (设为0则默认绑定OutMin)
    float Deadband; // 死区阈值 (不需使用时设为0)
} PID_t;

/* 函数声明 */
void PID_Init(PID_t *p);
void PID_Update(PID_t *p);
void PID_Update_Incremental(PID_t *p);

#endif /* __PID_H */