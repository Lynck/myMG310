#include "myPID.h"

static float TrackingPID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float TrackingPID_Filter(TrackingPID_t *pid, float raw_position)
{
    float alpha;

    if (!pid->FilterReady) {
        pid->FilteredPosition = raw_position;
        pid->FilterReady = true;
        return raw_position;
    }

    if ((TrackingPID_Abs(raw_position) >= pid->Config.FastFilterError) ||
        (TrackingPID_Abs(raw_position - pid->FilteredPosition) >=
         pid->Config.FastFilterDelta)) {
        alpha = pid->Config.FastFilterAlpha;
    } else {
        alpha = pid->Config.SlowFilterAlpha;
    }

    pid->FilteredPosition += alpha * (raw_position - pid->FilteredPosition);
    return pid->FilteredPosition;
}

static float TrackingPID_ApplyDeadband(float position, float deadband)
{
    if (position > deadband) {
        return position - deadband;
    }
    if (position < -deadband) {
        return position + deadband;
    }
    return 0.0f;
}

void TrackingPID_Init(TrackingPID_t *pid)
{
    pid->Config.Kp = 6.0f;
    pid->Config.Kd = 0.0f;
    pid->Config.Deadband = 0.6f;
    pid->Config.OutMax = 100.0f;
    pid->Config.OutMin = -100.0f;
    pid->Config.FastFilterError = 2.0f;
    pid->Config.FastFilterDelta = 0.8f;
    pid->Config.FastFilterAlpha = 0.70f;
    pid->Config.SlowFilterAlpha = 0.25f;
    pid->Config.CurveSlowdownGain = 2.5f;
    pid->Config.MinCurveSpeed = 12;
    pid->Config.Lane = TRACKING_PID_LANE_OUTER;

    TrackingPID_Reset(pid);
}

void TrackingPID_Reset(TrackingPID_t *pid)
{
    pid->Out = 0.0f;
    pid->Position = 0.0f;
    pid->LastPosition = 0.0f;
    pid->Error = 0.0f;
    pid->LastError = 0.0f;
    pid->FilteredPosition = 0.0f;
    pid->FilterReady = false;
}

TrackingPID_Status_t TrackingPID_Update(TrackingPID_t *pid,
                                        uint8_t black_mask,
                                        int16_t base_speed,
                                        TrackingPID_Output_t *output)
{
    static const int8_t weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
    int32_t weighted_sum = 0;
    int32_t sensor_sum = 0;
    int32_t raw_sensor_sum = 0;
    uint8_t i;

    output->Position = pid->LastPosition;
    output->LeftSpeed = base_speed;
    output->RightSpeed = base_speed;

    for (i = 0U; i < 8U; i++) {
        if ((black_mask & (uint8_t)(1U << i)) != 0U) {
            raw_sensor_sum++;
        }
    }

    if (black_mask == 0xFFU) {
        pid->Out = 0.0f;
        pid->LastError = pid->Error;
        pid->Error = 0.0f;
        return TRACKING_PID_ALL_BLACK;
    }

    if (raw_sensor_sum >= 2) {
        if (pid->Config.Lane == TRACKING_PID_LANE_OUTER) {
            /* Outer lane: ignore left sensors 1..3 (bits 7..5). */
            black_mask &= 0x1FU;
        } else {
            /* Inner lane: ignore right sensors 6..8 (bits 2..0). */
            black_mask &= 0xF8U;
        }

        if (black_mask == 0U) {
            pid->Out = 0.0f;
            return TRACKING_PID_BRANCH_IGNORED;
        }
    }

    for (i = 0U; i < 8U; i++) {
        if ((black_mask & (uint8_t)(1U << i)) != 0U) {
            weighted_sum += weights[i];
            sensor_sum++;
        }
    }

    if (sensor_sum == 0) {
        return TRACKING_PID_LINE_LOST;
    }

    pid->Position = TrackingPID_Filter(
        pid, (float)weighted_sum / (float)sensor_sum);
    pid->LastPosition = pid->Position;
    output->Position = pid->Position;

    pid->LastError = pid->Error;
    pid->Error = -TrackingPID_ApplyDeadband(
        pid->Position, pid->Config.Deadband);
    pid->Out = (pid->Config.Kp * pid->Error) +
               (pid->Config.Kd * (pid->Error - pid->LastError));

    if (pid->Out > pid->Config.OutMax) {
        pid->Out = pid->Config.OutMax;
    } else if (pid->Out < pid->Config.OutMin) {
        pid->Out = pid->Config.OutMin;
    }

    int16_t min_curve_speed =
        (base_speed < pid->Config.MinCurveSpeed) ?
        base_speed : pid->Config.MinCurveSpeed;
    int16_t current_base_speed = (int16_t)(
        (float)base_speed -
        TrackingPID_Abs(pid->Position) * pid->Config.CurveSlowdownGain);

    if (current_base_speed < min_curve_speed) {
        current_base_speed = min_curve_speed;
    }

    output->LeftSpeed = (int16_t)(current_base_speed + pid->Out);
    output->RightSpeed = (int16_t)(current_base_speed - pid->Out);
    return TRACKING_PID_LINE_FOUND;
}
