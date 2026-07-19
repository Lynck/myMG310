#include "myTask.h"

#include "grayscale_uart.h"
#include "leader_distance.h"
#include "distance_pid.h"
#include "clock.h"
#include "motor.h"

#define SWAP_MOTORS                  0
#define TRACKING_LOST_CENTER_BAND    2.0f
#define FOLLOW_SUCCESS_MIN_CM        18.0f
#define FOLLOW_SUCCESS_MAX_CM        22.0f
#define FOLLOW_ADJUST_MIN_SPEED      12
#define FOLLOW_ADJUST_MAX_SPEED      18
#define FOLLOW_ADJUST_SPEED_GAIN     0.7f
#define FOLLOW_SETTLE_TIME_MS        250UL
#define TASK2_NO_DISTANCE_SPEED      37

Task_t current_task = TASK_ID_1;
uint8_t able_stop;
uint8_t is_speed_50;

TrackingPID_t tracking_pid;
volatile int16_t g_base_speed = 22;
volatile float g_left_wheel_scale = 1.00f;
volatile float g_right_wheel_scale = 1.00f;
volatile bool g_line_follow_enabled = false;
volatile bool g_leader_stop_requested = false;
volatile int16_t g_distance_control_speed = 27;

DistancePID_t distance_pid;

static Task_t previous_task = TASK_MAX;
static bool stop_adjust_settling = false;
static unsigned long stop_adjust_settle_start_ms = 0UL;

static float Tracking_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

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

static bool Tracking_DistanceInSuccessRange(float distance_cm)
{
    return (distance_cm >= FOLLOW_SUCCESS_MIN_CM) &&
           (distance_cm <= FOLLOW_SUCCESS_MAX_CM);
}

static int16_t Tracking_GetStopAdjustSpeed(float distance_cm)
{
    int16_t speed = (int16_t)(
        Tracking_Abs(distance_cm - distance_pid.Config.TargetCm) *
        FOLLOW_ADJUST_SPEED_GAIN);

    if (speed < FOLLOW_ADJUST_MIN_SPEED) {
        speed = FOLLOW_ADJUST_MIN_SPEED;
    } else if (speed > FOLLOW_ADJUST_MAX_SPEED) {
        speed = FOLLOW_ADJUST_MAX_SPEED;
    }
    return speed;
}

void Tracking_PID_Init(void)
{
    TrackingPID_Init(&tracking_pid);
    DistancePID_Init(&distance_pid);
    g_distance_control_speed = g_base_speed;
}

void Tracking_PID2_Init(void)
{
    TrackingPID_Init(&tracking_pid);
    DistancePID_Init(&distance_pid);
    /* 任务二基础速度为45，距离环必须能够把速度最低修正到0。 */
    distance_pid.Config.OutMin = -(float)TASK2_NO_DISTANCE_SPEED;
    tracking_pid.Config.Kp = 13.0f;
    tracking_pid.Config.Kd = 35.0f;
    g_distance_control_speed = g_base_speed;
}

void Tracking_PID_Reset(void)
{
    TrackingPID_Reset(&tracking_pid);
    DistancePID_Reset(&distance_pid);
    stop_adjust_settling = false;
    stop_adjust_settle_start_ms = 0UL;
    g_distance_control_speed = g_base_speed;
}

void Tracking_SetLane(TrackingPID_Lane_t lane)
{
    tracking_pid.Config.Lane = lane;
}

void Tracking_SetLeaderStopRequested(bool requested)
{
    if (!requested || !g_leader_stop_requested) {
        stop_adjust_settling = false;
        stop_adjust_settle_start_ms = 0UL;
    }
    g_leader_stop_requested = requested;
}

void Tracking_Process(void)
{
    TrackingPID_Output_t output;
    TrackingPID_Status_t status;
    uint8_t black_mask;
    float distance_cm;
    bool distance_valid;
    int16_t controlled_base_speed;

    if (is_speed_50) {
        /* 红外距离暂时无效时，任务二使用该速度继续循迹等待测距恢复。 */
        g_base_speed = TASK2_NO_DISTANCE_SPEED;
    }

    distance_valid = LeaderDistance_Get(&distance_cm);
    if (g_leader_stop_requested && (!is_speed_50 || distance_valid)) {
        /*
         * 任务二只有红外距离有效时才进入最终停车调整；没有距离时继续循迹。
         * 任务一仍保持原来的失去距离立即刹车行为。
         */
        DistancePID_Reset(&distance_pid);
        g_distance_control_speed = 0;

        if (!distance_valid) {
            Motor_Brake();
            return;
        }

        if (stop_adjust_settling) {
            Motor_Brake();
            if ((tick_ms - stop_adjust_settle_start_ms) <
                FOLLOW_SETTLE_TIME_MS) {
                return;
            }
            stop_adjust_settling = false;

            if (Tracking_DistanceInSuccessRange(distance_cm)) {
                g_leader_stop_requested = false;
                g_line_follow_enabled = false;
                DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_9_PIN);
                mspm0_delay_ms(100U);
                DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_9_PIN);
                return;
            }
        }

        if (Tracking_DistanceInSuccessRange(distance_cm)) {
            Motor_Brake();
            stop_adjust_settling = true;
            stop_adjust_settle_start_ms = tick_ms;
            return;
        }

        controlled_base_speed = Tracking_GetStopAdjustSpeed(distance_cm);
        if (distance_cm < FOLLOW_SUCCESS_MIN_CM) {
            g_distance_control_speed = -controlled_base_speed;
            Tracking_SetMotorSpeeds(
                -controlled_base_speed, -controlled_base_speed);
            return;
        }

        /* Forward adjustment keeps using the line-following controller. */
    } else if (distance_valid) {
        controlled_base_speed = DistancePID_Update(
            &distance_pid, distance_cm, g_base_speed);
    } else {
        DistancePID_Reset(&distance_pid);
        controlled_base_speed = g_base_speed;
    }
    g_distance_control_speed = controlled_base_speed;

    if (!Grayscale_UART_GetBlackMask(&black_mask)) {
        return;
    }

    status = TrackingPID_Update(
        &tracking_pid, black_mask, controlled_base_speed, &output);

    if (status == TRACKING_PID_LINE_LOST) {
        if (output.Position > TRACKING_LOST_CENTER_BAND) {
            Tracking_SetMotorSpeeds(
                -controlled_base_speed, controlled_base_speed);
        } else if (output.Position < -TRACKING_LOST_CENTER_BAND) {
            /* Keep the existing right-side lost-line behaviour unchanged. */
        } else {
            Tracking_SetMotorSpeeds(
                controlled_base_speed, controlled_base_speed);
        }
        return;
    }

    Tracking_SetMotorSpeeds(output.LeftSpeed, output.RightSpeed);
}

static void Task_Enter(Task_t task)
{
    switch (task) {
        case TASK_ID_1:
            Tracking_PID_Init();
            Tracking_SetLane(TRACKING_PID_LANE_OUTER);
            is_speed_50 = 0;
            break;

        case TASK_ID_2:
            Tracking_PID2_Init();
            Tracking_SetLane(TRACKING_PID_LANE_OUTER);
            is_speed_50 = 1;
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
            able_stop = 1;
            is_speed_50 = 0;
            g_base_speed = 27;
            Tracking_Process();
            break;

        case TASK_ID_2:
            is_speed_50 = 1;
            Tracking_Process();
            break;

        case TASK_ID_3:
        default:
            break;
    }
}
