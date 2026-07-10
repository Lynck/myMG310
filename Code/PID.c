#include "PID.h"

/**
 * @brief  初始化PID结构体变量（完美兼容单参数调用）
 * @param  p: PID结构体指针
 */
void PID_Init(PID_t *p)
{
    // 初始化基础参数
    p->Kp = 0.0f;
    p->Ki = 0.0f;
    p->Kd = 0.0f;

    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->Out = 0.0f;

    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->Error2 = 0.0f;
    p->ErrorInt = 0.0f;

    // 初始化限幅与死区默认值
    p->OutMax = 100.0f;
    p->OutMin = -100.0f;
    p->IntMax = 0.0f;    // 设为0表示未显式设置，后面会自动适配 OutMax
    p->IntMin = 0.0f;
    p->Deadband = 0.0f;  // 默认不开启死区
}

/**
 * @brief  位置式 PID 计算 (针对循迹转向进行了抗饱和优化)
 * @param  p: PID结构体指针
 */
void PID_Update(PID_t *p)
{
    // 1. 保存上次误差，计算当前误差
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    // 2. 传感器死区处理（防止在目标值附近由于噪声导致小车微小震荡抖动）
    if (p->Deadband > 0.0f) {
        if (p->Error0 > -p->Deadband && p->Error0 < p->Deadband) {
            p->Error0 = 0.0f;
        }
    }

    // 3. 积分项累加与抗饱和处理
    if (p->Ki != 0.0f) {
        p->ErrorInt += p->Error0;
        
        // 智能适配：若用户未单独设置积分限幅(即为0)，则自动限制在输出限幅 [OutMin, OutMax] 内
        float real_int_max = (p->IntMax != 0.0f) ? p->IntMax : p->OutMax;
        float real_int_min = (p->IntMin != 0.0f) ? p->IntMin : p->OutMin;

        if (p->ErrorInt > real_int_max) p->ErrorInt = real_int_max;
        else if (p->ErrorInt < real_int_min) p->ErrorInt = real_int_min;
    } else {
        p->ErrorInt = 0.0f;
    }

    // 4. 标准位置式 PID 计算公式
    p->Out = (p->Kp * p->Error0)
           + (p->Ki * p->ErrorInt)
           + (p->Kd * (p->Error0 - p->Error1));

    // 5. 输出限幅
    if (p->Out > p->OutMax) p->Out = p->OutMax;
    else if (p->Out < p->OutMin) p->Out = p->OutMin;
}

/**
 * @brief  增量式 PID 计算 (备用：如果你后续需要做电机速度闭环控制)
 * @param  p: PID结构体指针
 */
void PID_Update_Incremental(PID_t *p)
{
    p->Error2 = p->Error1;
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    if (p->Deadband > 0.0f) {
        if (p->Error0 > -p->Deadband && p->Error0 < p->Deadband) {
            p->Error0 = 0.0f;
        }
    }

    float delta_out = p->Kp * (p->Error0 - p->Error1)
                    + p->Ki * p->Error0
                    + p->Kd * (p->Error0 - 2.0f * p->Error1 + p->Error2);

    p->Out += delta_out;

    if (p->Out > p->OutMax) p->Out = p->OutMax;
    else if (p->Out < p->OutMin) p->Out = p->OutMin;
}