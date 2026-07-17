#ifndef DISTANCE_PID_H
#define DISTANCE_PID_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float Kp;
    float Kd;
    float TargetCm;
    float DeadbandCm;
    float OutMin;
    float OutMax;
    float FastFilterError;
    float FastFilterDelta;
    float FastFilterAlpha;
    float SlowFilterAlpha;
} DistancePID_Config_t;

typedef struct {
    DistancePID_Config_t Config;
    float FilteredDistance;
    float Error;
    float LastError;
    float Out;
    bool FilterReady;
} DistancePID_t;

void DistancePID_Init(DistancePID_t *pid);
void DistancePID_Reset(DistancePID_t *pid);
int16_t DistancePID_Update(DistancePID_t *pid, float distance_cm,
                           int16_t nominal_speed);

#endif
