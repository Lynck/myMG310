#ifndef LEADER_DISTANCE_H
#define LEADER_DISTANCE_H

#include <stdbool.h>
#include <stdint.h>

#define LEADER_DISTANCE_SCALE_CM_PER_UNIT (1.0f)
#define LEADER_DISTANCE_TIMEOUT_MS        (500UL)

extern volatile float g_leader_distance_raw;
extern volatile float g_leader_distance_cm;
extern volatile bool g_leader_distance_updated;
extern volatile bool g_leader_distance_valid;
extern volatile unsigned long g_leader_distance_last_update_ms;

void LeaderDistance_Init(void);
bool LeaderDistance_Process(void);
bool LeaderDistance_Get(float *distance_cm);
void LeaderDistance_ReceiveByte(uint8_t byte);

#endif
