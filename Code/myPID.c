#include "myPID.h"
#include "PID.h"
#include "motor.h"
#include "motor_speed.h"
#include "Grayscale_Sensor.h"

/* 循迹参数：支持蓝牙动态调整。 */
volatile int16_t g_base_speed = 30;          /* 基础直行速度，可通过 S:xx 修改。 */
volatile float g_left_wheel_scale = 1.00f;   /* 左轮补偿系数，可通过 L:xx 修改。 */
volatile float g_right_wheel_scale = 1.00f;  /* 右轮补偿系数，可通过 R:xx 修改。 */
volatile bool g_line_follow_enabled = false; /* 循迹总开关。 */

/* 电机/传感器方向适配宏，只在接线或传感器方向相反时调整。 */
#define SWAP_MOTORS       0
#define REVERSE_PID_DIR   0

/* g_base_speed 用作目标车速，单位约为 cm/s；30 => 0.30m/s。 */
#define TRACKING_SPEED_CMD_TO_MPS     0.01f
#define TRACKING_TARGET_SPEED_MPS     ((float)g_base_speed * TRACKING_SPEED_CMD_TO_MPS)
#define TRACKING_SEARCH_CMD           20

extern define_Data data_1;

extern volatile bool start_100ms_timer;
extern volatile bool check_100ms_flag;

/* 丢线状态机：0=正常，1=刚丢线等待确认，2=确认丢线后按最后位置找线。 */
static uint8_t lost_line_state = 0;

PID_t tracking_pid;

/* 上一次有效黑线位置；完全丢线时用它判断该往哪边找线。 */
static float last_actual_pos = 0.0f;

void Tracking_PID_Init(void)
{
    PID_Init(&tracking_pid);

    /* 该 PID 只修正左右轮差速，目标是黑线位于传感器中心。 */
    tracking_pid.Kp = 6.0f;
    tracking_pid.Ki = 0.0f;
    tracking_pid.Kd = 2.0f;

    tracking_pid.Target = 0.0f;

    tracking_pid.OutMax = 50.0f;
    tracking_pid.OutMin = -50.0f;

    tracking_pid.Deadband = 0.0f;
}

void Tracking_PID_Reset(void)
{
    /* 重启循迹时清控制器状态，但保留用户调好的 Kp/Kd/速度参数。 */
    tracking_pid.Out      = 0.0f;
    tracking_pid.Error0   = 0.0f;
    tracking_pid.Error1   = 0.0f;
    tracking_pid.ErrorInt = 0.0f;
    lost_line_state = 0;
    MotorSpeed_ResetControl();
}

void Tracking_Process(void)
{
    /* 1. 刷新灰度传感器数据。 */
    Read_data_1_GPIO();

    /*
     * 2. 传感器原始值为 0 表示黑线。
     *    这里取反成 1=检测到黑线，便于后面加权求和。
     */
    uint8_t d8 = !data_1.D8; /* 最左端 */
    uint8_t d7 = !data_1.D7;
    uint8_t d6 = !data_1.D6;
    uint8_t d5 = !data_1.D5;
    uint8_t d4 = !data_1.D4;
    uint8_t d3 = !data_1.D3;
    uint8_t d2 = !data_1.D2;
    uint8_t d1 = !data_1.D1; /* 最右端 */

    int32_t sum_bits = d1 + d2 + d3 + d4 + d5 + d6 + d7 + d8;
    float actual_pos = 0.0f;

    /* 3. 正常循迹状态：至少有一个传感器压到黑线。 */
    if (sum_bits > 0)
    {
        lost_line_state = 0;

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
        /*
         * 4. 丢线保护：
         *    先等待 100ms 去抖；如果仍然没有黑线，再根据最后一次位置原地找线。
         */
        if (lost_line_state == 0)
        {
            start_100ms_timer = true;
            check_100ms_flag = false;
            lost_line_state = 1;
        }
        else if (lost_line_state == 1)
        {
            if (check_100ms_flag == true)
            {
                lost_line_state = 2;
                start_100ms_timer = false;
            }
        }

        if (lost_line_state == 2)
        {
            if (last_actual_pos > 4.0f)
            {
                Motor_SetSpeed_A(-TRACKING_SEARCH_CMD);
                Motor_SetSpeed_B(TRACKING_SEARCH_CMD);
                return;
            }
            else if (last_actual_pos < -4.0f)
            {
                Motor_SetSpeed_A(TRACKING_SEARCH_CMD);
                Motor_SetSpeed_B(-TRACKING_SEARCH_CMD);
                return;
            }
            else if (last_actual_pos >= -4.0f && last_actual_pos <= 4.0f)
            {
                MotorSpeed_Control(TRACKING_TARGET_SPEED_MPS, 0, g_left_wheel_scale, g_right_wheel_scale);
                return;
            }
            else
            {
                Motor_Brake();
                return;
            }
        }
    }

    /* 5. 全黑一般是路口/宽黑带：不做差速修正，按基础速度直行通过。 */
    if (d1 && d2 && d3 && d4 && d5 && d6 && d7 && d8)
    {
        MotorSpeed_Control(TRACKING_TARGET_SPEED_MPS, 0, g_left_wheel_scale, g_right_wheel_scale);
        return;
    }

    /* 6. 执行 PID，输出值作为左右轮差速修正量。 */
    tracking_pid.Actual = actual_pos;
    PID_Update(&tracking_pid);

    int16_t out_val = (int16_t)tracking_pid.Out;

    #if (REVERSE_PID_DIR == 1)
        out_val = -out_val;
    #endif

    // /* 限制差速量，保证内侧轮仍有最小前进速度。 */
    // #define MIN_FWD_SPEED  10
    // int16_t max_diff = g_base_speed - MIN_FWD_SPEED;
    // if (out_val >  max_diff) out_val =  max_diff;
    // if (out_val < -max_diff) out_val = -max_diff;

    /* 7. 速度闭环只控制左右平均速度；循迹 PID 的 out_val 只负责左右差速。 */
    #if (SWAP_MOTORS == 1)
        MotorSpeed_Control(TRACKING_TARGET_SPEED_MPS, (int16_t)-out_val, g_right_wheel_scale, g_left_wheel_scale);
    #else
        MotorSpeed_Control(TRACKING_TARGET_SPEED_MPS, out_val, g_left_wheel_scale, g_right_wheel_scale);
    #endif
}
