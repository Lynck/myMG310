/**
 * 切换不同任务的函数
 **/
#include "myPID.h"

typedef enum Type_e{
    TASK_ID_1 = 0,
    TASK_ID_2,
    TASK_MAX
}Task_t;

Task_t current_task = TASK_ID_1;

void ExecuteTask(Task_t current_task)
{
    switch (current_task) {
        //任务一：普通循迹
        case TASK_ID_1:
            Tracking_Process();//循迹
            break;

        //任务二：
        case TASK_ID_2:

            break;

        default:
            break;
    }
}
