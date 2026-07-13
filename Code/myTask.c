/**
 * 切换不同任务的函数
 **/
#include "myPID.h"

typedef enum Type_e{
    TASK_ID_1 = 0,
    TASK_ID_2,
    TASK_ID_3,
    TASK_MAX
}Task_t;

Task_t current_task = TASK_ID_1;
uint8_t able_stop;
uint8_t is_speed_50;

void ExecuteTask(Task_t current_task)
{
    switch (current_task) {
        //任务一：普通循迹
        case TASK_ID_1:
            able_stop = 1;
            Tracking_Process();//循迹
            break;

        //任务二：
        case TASK_ID_2:
            is_speed_50 = 1;
            Tracking_PID2_Init();//任务二单独PID
            Tracking_Process();
            break;

        case TASK_ID_3:

            break; 

        default:
            break;
    }
}
