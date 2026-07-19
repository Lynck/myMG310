#include "myPID.h"

static float TrackingPID_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float TrackingPID_Filter(TrackingPID_t *pid, float raw_position)
{
    float alpha;

    if (!pid->FilterReady) {
        /* 第一次采样直接建立滤波初值，避免从 0 缓慢追到真实位置。 */
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
    /* 任务一循迹参数；任务层另外负责设置基础速度和赛道模式。 */
    pid->Config.Kp = 7.0f;
    pid->Config.Kd = 35.0f;
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

/* 任务二使用独立的循迹参数，调节时不要误改任务一参数。 */
void TrackingPID2_Init(TrackingPID_t *pid)
{
    pid->Config.Kp = 8.0f;
    pid->Config.Kd = 38.0f;
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
    /* bit0~bit7 对应右~左，权重正负决定黑线在车体中心的哪一侧。 */
    static const int8_t weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};
    int32_t weighted_sum = 0;
    int32_t sensor_sum = 0;
    int32_t raw_sensor_sum = 0;
    uint8_t i;

    output->Position = pid->LastPosition;
    output->LeftSpeed = base_speed;
    output->RightSpeed = base_speed;

    /* 先统计原始黑线数量，内外圈岔路过滤必须使用过滤前的数量。 */
    for (i = 0U; i < 8U; i++) {
        if ((black_mask & (uint8_t)(1U << i)) != 0U) {
            raw_sensor_sum++;
        }
    }

    if (black_mask == 0xFFU) {
        /* 全黑通常是宽黑带或特殊标志，保持基础速度直行，不做差速修正。 */
        pid->Out = 0.0f;
        pid->LastError = pid->Error;
        pid->Error = 0.0f;
        return TRACKING_PID_ALL_BLACK;
    }

    if (raw_sensor_sum >= 3) {
        /*
         * 内外圈判断的核心位置：
         * 外圈屏蔽左侧 CH1~CH3，内圈屏蔽右侧 CH6~CH8。
         */
        if (pid->Config.Lane == TRACKING_PID_LANE_OUTER) {
            black_mask &= 0x1FU;
        } else {
            black_mask &= 0xF8U;
        }

        if (black_mask == 0U) {
            /* 被屏蔽后没有有效黑线，本周期保持基础速度，不继承旧的差速量。 */
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
        /* 完全丢线时由任务适配层根据上一次位置决定如何找线。 */
        return TRACKING_PID_LINE_LOST;
    }

    pid->Position = TrackingPID_Filter(
        pid, (float)weighted_sum / (float)sensor_sum);
    pid->LastPosition = pid->Position;
    output->Position = pid->Position;

    pid->LastError = pid->Error;
    /* 目标位置为 0；取负号后，左右黑线会产生方向相反的差速输出。 */
    pid->Error = -TrackingPID_ApplyDeadband(
        pid->Position, pid->Config.Deadband);
    pid->Out = (pid->Config.Kp * pid->Error) +
               (pid->Config.Kd * (pid->Error - pid->LastError));

    if (pid->Out > pid->Config.OutMax) {
        pid->Out = pid->Config.OutMax;
    } else if (pid->Out < pid->Config.OutMin) {
        pid->Out = pid->Config.OutMin;
    }

    /* 偏离中心越远，先降低共同基础速度，再叠加左右差速，减少急弯冲出。 */
    int16_t min_curve_speed =
        (base_speed < pid->Config.MinCurveSpeed) ?
        base_speed : pid->Config.MinCurveSpeed;
    int16_t current_base_speed = (int16_t)(
        (float)base_speed -
        TrackingPID_Abs(pid->Position) * pid->Config.CurveSlowdownGain);

    if (current_base_speed < min_curve_speed) {
        current_base_speed = min_curve_speed;
    }

    /* pid->Out<0 时左轮减速右轮加速；pid->Out>0 时方向相反。 */
    output->LeftSpeed = (int16_t)(current_base_speed + pid->Out);
    output->RightSpeed = (int16_t)(current_base_speed - pid->Out);
    return TRACKING_PID_LINE_FOUND;
}
