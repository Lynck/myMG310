#ifndef __MYTASK_H
#define __MYTASK_H

#include <stdbool.h>
#include <stdint.h>

#include "myPID.h"
#include "distance_pid.h"

typedef enum Type_e {
    TASK_ID_1 = 0,
    TASK_ID_2,
    TASK_ID_3,
    TASK_MAX
} Task_t;

extern Task_t current_task;
extern uint8_t able_stop;
extern uint8_t is_speed_50;

/* Project adapter state used by Bluetooth, OLED and the task scheduler. */
extern TrackingPID_t tracking_pid;
extern volatile int16_t g_base_speed;
extern volatile float g_left_wheel_scale;
extern volatile float g_right_wheel_scale;
extern volatile bool g_line_follow_enabled;
extern volatile bool g_leader_stop_requested;
extern volatile int16_t g_distance_control_speed;
extern DistancePID_t distance_pid;

void Tracking_PID_Init(void);
void Tracking_PID2_Init(void);
void Tracking_PID_Reset(void);
void Tracking_SetLane(TrackingPID_Lane_t lane);
void Tracking_SetLeaderStopRequested(bool requested);
void Tracking_Process(void);
void ExecuteTask(Task_t current_task);

#endif
