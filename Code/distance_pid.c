#include "distance_pid.h"

#define DISTANCE_SPEED_MIN (0)
#define DISTANCE_SPEED_MAX (80)

static float DistancePID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float DistancePID_Clamp(float value, float min, float max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

void DistancePID_Init(DistancePID_t *pid)
{
    pid->Config.Kp = 0.4f;
    pid->Config.Kd = 20.0f;
    pid->Config.TargetCm = 20.0f;
    pid->Config.DeadbandCm = 0.0f;
    pid->Config.OutMin = -37.0f;
    pid->Config.OutMax = 8.0f;
    pid->Config.FastFilterError = 8.0f;
    pid->Config.FastFilterDelta = 3.0f;
    pid->Config.FastFilterAlpha = 0.4f;
    pid->Config.SlowFilterAlpha = 0.25f;
    DistancePID_Reset(pid);
}

void DistancePID_Reset(DistancePID_t *pid)
{
    pid->FilteredDistance = 0.0f;
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->Out = 0.0f;
    pid->FilterReady = false;
}

int16_t DistancePID_Update(DistancePID_t *pid, float distance_cm,
                           int16_t nominal_speed)
{
    float alpha;
    float raw_error = distance_cm - pid->Config.TargetCm;
    float speed;

    if (!pid->FilterReady) {
        pid->FilteredDistance = distance_cm;
        pid->FilterReady = true;
    } else {
        alpha = ((DistancePID_Abs(raw_error) >= pid->Config.FastFilterError) ||
                 (DistancePID_Abs(distance_cm - pid->FilteredDistance) >=
                  pid->Config.FastFilterDelta)) ?
                    pid->Config.FastFilterAlpha :
                    pid->Config.SlowFilterAlpha;
        pid->FilteredDistance +=
            alpha * (distance_cm - pid->FilteredDistance);
    }

    pid->LastError = pid->Error;
    pid->Error = pid->FilteredDistance - pid->Config.TargetCm;
    if (pid->Error > pid->Config.DeadbandCm) {
        pid->Error -= pid->Config.DeadbandCm;
    } else if (pid->Error < -pid->Config.DeadbandCm) {
        pid->Error += pid->Config.DeadbandCm;
    } else {
        pid->Error = 0.0f;
    }

    pid->Out = (pid->Config.Kp * pid->Error) +
               (pid->Config.Kd * (pid->Error - pid->LastError));
    pid->Out = DistancePID_Clamp(
        pid->Out, pid->Config.OutMin, pid->Config.OutMax);

    speed = DistancePID_Clamp((float)nominal_speed + pid->Out,
                              (float)DISTANCE_SPEED_MIN,
                              (float)DISTANCE_SPEED_MAX);
    return (int16_t)speed;
}
