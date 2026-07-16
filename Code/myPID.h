#ifndef __MYPID_H
#define __MYPID_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Hardware-independent eight-channel line-following PD module.
 * black_mask bit7 is the leftmost sensor, bit0 is the rightmost sensor,
 * and a set bit means that the sensor sees the black line.
 */

typedef enum {
    TRACKING_PID_LINE_FOUND = 0,
    TRACKING_PID_LINE_LOST,
    TRACKING_PID_ALL_BLACK
} TrackingPID_Status_t;

typedef struct {
    float Kp;
    float Kd;
    float Deadband;
    float OutMax;
    float OutMin;
    float FastFilterError;
    float FastFilterDelta;
    float FastFilterAlpha;
    float SlowFilterAlpha;
    float CurveSlowdownGain;
    int16_t MinCurveSpeed;
} TrackingPID_Config_t;

typedef struct {
    TrackingPID_Config_t Config;

    /* Runtime values are exposed for tuning displays and diagnostics. */
    float Out;
    float Position;
    float LastPosition;
    float Error;
    float LastError;

    /* Internal filter state; clear it through TrackingPID_Reset(). */
    float FilteredPosition;
    bool FilterReady;
} TrackingPID_t;

typedef struct {
    int16_t LeftSpeed;
    int16_t RightSpeed;
    float Position;
} TrackingPID_Output_t;

void TrackingPID_Init(TrackingPID_t *pid);
void TrackingPID_Reset(TrackingPID_t *pid);
TrackingPID_Status_t TrackingPID_Update(TrackingPID_t *pid,
                                        uint8_t black_mask,
                                        int16_t base_speed,
                                        TrackingPID_Output_t *output);

#endif
