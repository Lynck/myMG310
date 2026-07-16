#include "myTask.h"

#include "grayscale_uart.h"
#include "motor.h"

#define SWAP_MOTORS                  0
#define TRACKING_LOST_CENTER_BAND    2.0f

Task_t current_task = TASK_ID_1;
uint8_t able_stop;
uint8_t is_speed_50;

TrackingPID_t tracking_pid;
volatile int16_t g_base_speed = 22;
volatile float g_left_wheel_scale = 1.00f;
volatile float g_right_wheel_scale = 1.00f;
volatile bool g_line_follow_enabled = false;

static Task_t previous_task = TASK_MAX;

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
    TrackingPID_Init(&tracking_pid);
}

void Tracking_PID2_Init(void)
{
    TrackingPID_Init(&tracking_pid);
    tracking_pid.Config.Kp = 12.0f;
    tracking_pid.Config.Kd = 30.0f;
}

void Tracking_PID_Reset(void)
{
    TrackingPID_Reset(&tracking_pid);
}

void Tracking_Process(void)
{
    TrackingPID_Output_t output;
    TrackingPID_Status_t status;
    uint8_t black_mask;

    if (!Grayscale_UART_GetBlackMask(&black_mask)) {
        return;
    }

    if (is_speed_50) {
        g_base_speed = 37;
    }

    status = TrackingPID_Update(
        &tracking_pid, black_mask, g_base_speed, &output);

    if (status == TRACKING_PID_LINE_LOST) {
        if (output.Position > TRACKING_LOST_CENTER_BAND) {
            Tracking_SetMotorSpeeds(-g_base_speed, g_base_speed);
        } else if (output.Position < -TRACKING_LOST_CENTER_BAND) {
            /* Keep the existing right-side lost-line behaviour unchanged. */
        } else {
            Tracking_SetMotorSpeeds(g_base_speed, g_base_speed);
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
            is_speed_50 = 0;
            break;

        case TASK_ID_2:
            Tracking_PID2_Init();
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
