#include "myTask.h"

#include "myPID.h"

Task_t current_task = TASK_ID_1;
uint8_t able_stop;
uint8_t is_speed_50;

static Task_t previous_task = TASK_MAX;

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
