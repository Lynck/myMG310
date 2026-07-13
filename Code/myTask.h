#ifndef __MYTASK_H
#define __MYTASK_H

#include "ti_msp_dl_config.h"

typedef enum Type_e{
    TASK_ID_1 = 0,
    TASK_ID_2,
    TASK_ID_3,
    TASK_MAX
}Task_t;

extern Task_t current_task;
extern uint8_t able_stop;
extern uint8_t is_speed_50;

void ExecuteTask(Task_t current_task);

#endif