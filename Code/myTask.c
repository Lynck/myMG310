#include "myTask.h"

#include "Grayscale_Sensor.h"
#include "clock.h"
#include "motor.h"
#include "motor_speed.h"

#define SWAP_MOTORS                   0
#define TRACKING_LOST_CENTER_BAND     2.0f
#define TASK1_STOP_GUARD_SAMPLES      800U  /* 800×10 ms：任务一前 8 秒禁止停车。 */
#define TASK2_STOP_GUARD_SAMPLES      1500U /* 1000×10 ms：任务二前 15 秒禁止计圈。14-17s跑完两圈 */
#define TASK1_TRACKING_BASE_CMD       20
#define TASK2_TRACKING_BASE_CMD       27
#define TASK2_REQUIRED_LAPS           2U
#define TASK1_TARGET_SPEED_MPS         0.30f
/* 任务二车体中心目标速度；左右轮目标速度的平均值始终保持为该值。 */
#define TASK2_TARGET_SPEED_MPS         0.50f
/* 将循迹PID输出换算为左右轮目标速度差，并限制急弯最大差速。 */
#define TRACKING_STEERING_MPS_PER_CMD  0.005f
#define TRACKING_STEERING_MAX_MPS      0.20f

/* 一次灰度采样的统一表示：d7/CH1 在最左，d0/CH8 在最右。 */
typedef struct {
    uint8_t mask;
    uint8_t count;
    uint8_t d0;
    uint8_t d1;
    uint8_t d2;
    uint8_t d3;
    uint8_t d4;
    uint8_t d5;
    uint8_t d6;
    uint8_t d7;
} TrackingFrame_t;

Task_t current_task = TASK_ID_1;
uint8_t able_stop;
uint8_t is_speed_50;

TrackingPID_t tracking_pid;
volatile int16_t g_base_speed = TASK1_TRACKING_BASE_CMD;
volatile float g_left_wheel_scale = 1.00f;
volatile float g_right_wheel_scale = 1.00f;
volatile bool g_line_follow_enabled = false;

static Task_t previous_task = TASK_MAX;
/* 任务运行状态：计圈数、停止线锁存和起步保护计数。 */
static uint8_t task2_lap_count = 0U;
static bool stop_pattern_latched = false;
static uint16_t stop_guard_count = 0U;
static uint16_t stop_guard_limit = TASK1_STOP_GUARD_SAMPLES;

static void Tracking_RunState_Reset(void)
{
    task2_lap_count = 0U;
    stop_pattern_latched = false;
    stop_guard_count = 0U;
    MotorSpeed_ResetControl();
}

static void Tracking_SetMotorSpeeds(int16_t left_cmd, int16_t right_cmd)
{
    /* 集中处理左右轮补偿以及电机 A/B 的接线对应关系。 */
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

static void Tracking_ControlWheels(float target_speed_mps, float steering_output)
{
    float delta_mps = steering_output * TRACKING_STEERING_MPS_PER_CMD;
    float left_target_mps;
    float right_target_mps;

    if (delta_mps > TRACKING_STEERING_MAX_MPS) {
        delta_mps = TRACKING_STEERING_MAX_MPS;
    } else if (delta_mps < -TRACKING_STEERING_MAX_MPS) {
        delta_mps = -TRACKING_STEERING_MAX_MPS;
    }

    /* 一侧加速、另一侧等量减速，左右轮目标速度的平均值保持不变。 */
    left_target_mps = target_speed_mps + delta_mps;
    right_target_mps = target_speed_mps - delta_mps;

#if (SWAP_MOTORS == 1)
    MotorSpeed_ControlWheels(right_target_mps, left_target_mps,
                             g_right_wheel_scale, g_left_wheel_scale);
#else
    MotorSpeed_ControlWheels(left_target_mps, right_target_mps,
                             g_left_wheel_scale, g_right_wheel_scale);
#endif
}

static TrackingFrame_t Tracking_ReadFrame(void)
{
    TrackingFrame_t frame;
    uint8_t raw = Grayscale_Sensor_Read();

    if (GRAYSCALE_BLACK_LEVEL == 0U) {
        /* 感为模块黑线为低电平，统一转换为 1 表示黑线。 */
        raw = (uint8_t)~raw;
    }

    /* Grayscale_Sensor_Read() 已完成 CH1~CH8 到 d7~d0 的位序适配。 */
    frame.mask = raw;
    frame.d7 = (uint8_t)((raw >> 7U) & 1U);
    frame.d6 = (uint8_t)((raw >> 6U) & 1U);
    frame.d5 = (uint8_t)((raw >> 5U) & 1U);
    frame.d4 = (uint8_t)((raw >> 4U) & 1U);
    frame.d3 = (uint8_t)((raw >> 3U) & 1U);
    frame.d2 = (uint8_t)((raw >> 2U) & 1U);
    frame.d1 = (uint8_t)((raw >> 1U) & 1U);
    frame.d0 = (uint8_t)(raw & 1U);
    frame.count = (uint8_t)(frame.d0 + frame.d1 + frame.d2 + frame.d3 +
                            frame.d4 + frame.d5 + frame.d6 + frame.d7);
    return frame;
}

static bool Tracking_IsStopPattern(const TrackingFrame_t *frame)
{
    /* 当前停止线条件：检测到四路及以上黑线。 */
    return (frame->count >= 4U);
}

static void Tracking_RunLineController(const TrackingFrame_t *frame)
{
    /* PID 不直接读取硬件，只处理已经归一化的八位黑线掩码。 */
    TrackingPID_Output_t output;
    TrackingPID_Status_t status = TrackingPID_Update(
        &tracking_pid, frame->mask, g_base_speed, &output);

    if (status == TRACKING_PID_LINE_LOST) {
        /* 丢线时仅保留已经实车验证稳定的找线方向。 */
        /* 找线可能需要倒转，暂时退出只支持前进的速度环。 */
        MotorSpeed_ResetControl();
        if (output.Position > TRACKING_LOST_CENTER_BAND) {
            Tracking_SetMotorSpeeds(-g_base_speed, g_base_speed);
        } else if (output.Position < -TRACKING_LOST_CENTER_BAND) {
            /* Keep the stable backup's right-side lost-line behaviour. */
        } else {
            Tracking_SetMotorSpeeds(g_base_speed, g_base_speed);
        }
        return;
    }

    if (is_speed_50) {
        Tracking_ControlWheels(TASK2_TARGET_SPEED_MPS, tracking_pid.Out);
    } else {
        /* 任务一复用任务二的双轮速度环，车体中心目标速度为 0.3 m/s。 */
        Tracking_ControlWheels(TASK1_TARGET_SPEED_MPS, tracking_pid.Out);
    }
}

static void Tracking_UpdateStopGuard(void)
{
    /* 本函数每 10 ms 调用一次，计数达到上限后才允许识别停止线。 */
    if (stop_guard_count < stop_guard_limit) {
        stop_guard_count++;
    }
}

static void Tracking_ProcessTask1(void)
{
    TrackingFrame_t frame = Tracking_ReadFrame();

    /* 任务一先执行循迹，再判断是否满足最终停车条件。 */
    Tracking_UpdateStopGuard();
    Tracking_RunLineController(&frame);

    if (stop_guard_count < stop_guard_limit) {
        return;
    }

    if (!Tracking_IsStopPattern(&frame)) {
        stop_pattern_latched = false;
        return;
    }

    if (stop_pattern_latched) {
        /* 同一片宽黑区域只触发一次，离开后才解除锁存。 */
        return;
    }
    stop_pattern_latched = true;

    if (able_stop) {
        /* 任务一完成：主动刹车、关闭循迹，并鸣笛 100 ms。 */
        able_stop = 0U;
        Motor_Brake();
        g_line_follow_enabled = false;
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_9_PIN);
        mspm0_delay_ms(100U);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_9_PIN);
    }
}

static void Tracking_ProcessTask2(void)
{
    /* 任务二沿用相同灰度入口，但使用独立 PID 参数和原有计圈逻辑。 */
    TrackingFrame_t frame = Tracking_ReadFrame();

    Tracking_UpdateStopGuard();
    Tracking_RunLineController(&frame);

    if (stop_guard_count < stop_guard_limit) {
        return;
    }

    if (!Tracking_IsStopPattern(&frame)) {
        stop_pattern_latched = false;
        return;
    }

    if (stop_pattern_latched) {
        return;
    }
    stop_pattern_latched = true;
    task2_lap_count++;

    if (task2_lap_count >= TASK2_REQUIRED_LAPS) {
        task2_lap_count = 0U;
        is_speed_50 = 0U;
        MotorSpeed_ResetControl();
        Motor_Brake();
        g_line_follow_enabled = false;
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_9_PIN);
        mspm0_delay_ms(100U);
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_9_PIN);
    }
}

void Tracking_PID_Init(void)
{
    TrackingPID_Init(&tracking_pid);
    Tracking_RunState_Reset();
}

void Tracking_PID2_Init(void)
{
    /* Task 2 starts from exactly the same tracking controller as task 1. */
    TrackingPID2_Init(&tracking_pid);
    Tracking_RunState_Reset();
}

void Tracking_PID_Reset(void)
{
    TrackingPID_Reset(&tracking_pid);
    Tracking_RunState_Reset();
}

void Tracking_SetLane(TrackingPID_Lane_t lane)
{
    /* 设置内/外圈模式；真正的多黑线屏蔽判断位于 TrackingPID_Update()。 */
    tracking_pid.Config.Lane = lane;
}

void Tracking_Process(void)
{
    if (is_speed_50) {
        Tracking_ProcessTask2();
    } else {
        Tracking_ProcessTask1();
    }
}

static void Task_Enter(Task_t task)
{
    /* 任务切换时统一设置基础速度、保护时间、PID参数和赛道模式。 */
    switch (task) {
        case TASK_ID_1:
            g_base_speed = TASK1_TRACKING_BASE_CMD;
            stop_guard_limit = TASK1_STOP_GUARD_SAMPLES;
            Tracking_PID_Init();
            /* 当前任务一按外圈规则运行；走内圈时改为 INNER。 */
            Tracking_SetLane(TRACKING_PID_LANE_OUTER);
            is_speed_50 = 0U;
            break;

        case TASK_ID_2:
            g_base_speed = TASK2_TRACKING_BASE_CMD;
            stop_guard_limit = TASK2_STOP_GUARD_SAMPLES;
            Tracking_PID2_Init();
            /* 当前任务二也配置为外圈，本次只补注释，不修改任务二逻辑。 */
            Tracking_SetLane(TRACKING_PID_LANE_OUTER);
            is_speed_50 = 1U;
            break;

        case TASK_ID_3:
            g_base_speed = TASK1_TRACKING_BASE_CMD;
            stop_guard_limit = TASK1_STOP_GUARD_SAMPLES;
            Tracking_PID_Init();
            Tracking_SetLane(TRACKING_PID_LANE_OUTER);
            is_speed_50 = 0U;
            break;

        default:
            break;
    }
}

void ExecuteTask(Task_t task)
{
    if (task != previous_task) {
        Task_Enter(task);
        previous_task = task;
    }

    switch (task) {
        case TASK_ID_1:
            able_stop = 1U;
            Tracking_ProcessTask1();
            break;

        case TASK_ID_2:
            is_speed_50 = 1U;
            Tracking_ProcessTask2();
            break;

        case TASK_ID_3:
        default:
            break;
    }
}
