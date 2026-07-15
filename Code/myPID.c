#include "myPID.h"
#include "PID.h"
#include "motor.h"
#include "grayscale_uart.h"
#include "myTask.h"

/* 循迹参数：支持蓝牙动态调整。 */
volatile int16_t g_base_speed = 22;          /* 基础直行速度，可通过 S:xx 修改。 */
volatile float g_left_wheel_scale = 1.00f;   /* 左轮补偿系数，可通过 L:xx 修改。 */
volatile float g_right_wheel_scale = 1.00f;  /* 右轮补偿系数，可通过 R:xx 修改。 */
volatile bool g_line_follow_enabled = false; /* 循迹总开关。 */

/* 电机/传感器方向适配宏，只在接线或传感器方向相反时调整。 */
#define SWAP_MOTORS       0
#define REVERSE_PID_DIR   0

/* 循迹直接输出 Motor_SetSpeed 命令值，不走速度闭环。 */
#define TRACKING_SEARCH_CMD           (g_base_speed)
#define TRACKING_LOST_CENTER_BAND     2.0f

PID_t tracking_pid;

/* 上一次有效黑线位置；完全丢线时用它判断该往哪边找线。 */
static float last_actual_pos = 0.0f;

static void Tracking_SetMotorSpeeds(int16_t left_cmd, int16_t right_cmd)
{
    int16_t left = (int16_t)((float)left_cmd * g_left_wheel_scale);
    int16_t right = (int16_t)((float)right_cmd * g_right_wheel_scale);

#if (SWAP_MOTORS == 1)
    Motor_SetSpeed_A(right);
    Motor_SetSpeed_B(left);
#else
    Motor_SetSpeed_A(left);
    Motor_SetSpeed_B(right);
#endif
}

void Tracking_PID_Init(void)
{
    PID_Init(&tracking_pid);

    /* 该 PID 只修正左右轮差速，目标是黑线位于传感器中心。 */
    tracking_pid.Kp = 6.0f;
    tracking_pid.Ki = 0.0f;
    tracking_pid.Kd = 20.0f;

    tracking_pid.Target = 0.0f;

    tracking_pid.OutMax = 100.0f;
    tracking_pid.OutMin = -100.0f;

    tracking_pid.Deadband = 0.6f;
}

void Tracking_PID2_Init(void)
{
    PID_Init(&tracking_pid);

    /* 该 PID 只修正左右轮差速，目标是黑线位于传感器中心。 */
    tracking_pid.Kp = 12.0f;
    tracking_pid.Ki = 0.0f;
    tracking_pid.Kd = 30.0f;

    tracking_pid.Target = 0.0f;

    tracking_pid.OutMax = 100.0f;
    tracking_pid.OutMin = -100.0f;

    tracking_pid.Deadband = 0.6f;
}

void Tracking_PID_Reset(void)
{
    /* 重启循迹时清控制器状态，但保留用户调好的 Kp/Kd/速度参数。 */
    tracking_pid.Out      = 0.0f;
    tracking_pid.Error0   = 0.0f;
    tracking_pid.Error1   = 0.0f;
    tracking_pid.Error2   = 0.0f;
    tracking_pid.ErrorInt = 0.0f;
}

void Tracking_Process(void)
{
    uint8_t black_mask;

    if (!Grayscale_UART_GetBlackMask(&black_mask)) {
        return;
    }

    /* bit7 is leftmost, bit0 is rightmost, and 1 means black line. */
    uint8_t d8 = (black_mask >> 7U) & 1U; /* leftmost */
    uint8_t d7 = (black_mask >> 6U) & 1U;
    uint8_t d6 = (black_mask >> 5U) & 1U;
    uint8_t d5 = (black_mask >> 4U) & 1U;
    uint8_t d4 = (black_mask >> 3U) & 1U;
    uint8_t d3 = (black_mask >> 2U) & 1U;
    uint8_t d2 = (black_mask >> 1U) & 1U;
    uint8_t d1 = black_mask & 1U;         /* rightmost */

    int32_t sum_bits = d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;
    float actual_pos = 0.0f;

    //任务二0.5m/s
    if(is_speed_50)
    {
        g_base_speed = 37;
    }

    /* 3. 正常循迹状态：至少有一个传感器压到黑线。 */
    if (sum_bits > 0)
    {
        /*
         * 加权平均计算黑线位置：
         *   D8~D5 在车体左侧，权重为正；D4~D1 在车体右侧，权重为负。
         *   线在左侧时 actual_pos 为正，PID error = 0 - actual_pos 为负。
         *   out_val 为负后左轮减速、右轮加速，车头向左修正。
         */
        float sum_weight = (d8 * 4.0f) + (d7 * 3.0f) + (d6 * 2.0f) + (d5 * 1.0f) +
                           (d4 * -1.0f) + (d3 * -2.0f) + (d2 * -3.0f) + (d1 * -4.0f);

        actual_pos = sum_weight / sum_bits;
        last_actual_pos = actual_pos;
    }
    else
    {
        if (last_actual_pos > TRACKING_LOST_CENTER_BAND) {
            Tracking_SetMotorSpeeds(-TRACKING_SEARCH_CMD, TRACKING_SEARCH_CMD);
        } else if (last_actual_pos < -TRACKING_LOST_CENTER_BAND) {
            // Tracking_SetMotorSpeeds(TRACKING_SEARCH_CMD, -TRACKING_SEARCH_CMD);
        } else {
            Tracking_SetMotorSpeeds(g_base_speed, g_base_speed);
        }
        return;
    }

    /* 5. 全黑一般是路口/宽黑带：不做差速修正，按基础速度直行通过。 */
    if (d1 && d2 && d3 && d4 && d5 && d6 && d7 && d8)
    {
        Tracking_SetMotorSpeeds(g_base_speed, g_base_speed);
        return;
    }

    /* 6. 执行 PID，输出值作为左右轮差速修正量。 */
    tracking_pid.Actual = actual_pos;
    PID_Update(&tracking_pid);

    int16_t out_val = (int16_t)tracking_pid.Out;

    #if (REVERSE_PID_DIR == 1)
        out_val = -out_val;
    #endif

    Tracking_SetMotorSpeeds((int16_t)(g_base_speed + out_val),
                            (int16_t)(g_base_speed - out_val));

}
