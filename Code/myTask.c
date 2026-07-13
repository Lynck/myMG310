#include "myTask.h"

#include "PID.h"
#include "clock.h"
#include "leader_distance.h"
#include "myPID.h"

#define TASK1_DISTANCE_KP          15.0f
#define TASK1_DISTANCE_KI          0.0f
#define TASK1_DISTANCE_KD          0.0f
#define TASK1_DISTANCE_DEADBAND_CM 1.0f
#define TASK1_SPEED_LIMIT          35.0f
#define TASK1_DISTANCE_TIMEOUT_MS  500UL
#define TASK1_NO_TARGET_SPEED      22

Task_t current_task = TASK_ID_1;
uint8_t able_stop;
uint8_t is_speed_50;

static PID_t task1_distance_pid;
static Task_t previous_task = TASK_MAX;

static void Task1_DistancePID_Init(void)
{
    PID_Init(&task1_distance_pid);
    task1_distance_pid.Kp = TASK1_DISTANCE_KP;
    task1_distance_pid.Ki = TASK1_DISTANCE_KI;
    task1_distance_pid.Kd = TASK1_DISTANCE_KD;
    task1_distance_pid.Target = TASK1_DISTANCE_TARGET_CM;
    task1_distance_pid.OutMax = TASK1_SPEED_LIMIT;
    task1_distance_pid.OutMin = -TASK1_SPEED_LIMIT;
    task1_distance_pid.Deadband = TASK1_DISTANCE_DEADBAND_CM;
    g_base_speed = TASK1_NO_TARGET_SPEED;
}

static void Task1_UpdateBaseSpeed(void)
{
    if (!g_leader_distance_valid ||
        ((tick_ms - g_leader_distance_last_update_ms) >
         TASK1_DISTANCE_TIMEOUT_MS)) {
        g_base_speed = TASK1_NO_TARGET_SPEED;
        return;
    }

    if (g_leader_distance_updated) {
        g_leader_distance_updated = false;
        task1_distance_pid.Actual = g_leader_distance_cm;
        PID_Update(&task1_distance_pid);
        g_base_speed = (int16_t)(-task1_distance_pid.Out);
    }

}

static void Task_Enter(Task_t task)
{
    switch (task) {
        case TASK_ID_1:
            Tracking_PID_Init();
            Task1_DistancePID_Init();
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
            Task1_UpdateBaseSpeed();
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
