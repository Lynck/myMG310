#ifndef __MYPID_H
#define __MYPID_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 与硬件无关的八路灰度循迹 PD 模块。
 * black_mask 的 bit7 是最左侧 CH1，bit0 是最右侧 CH8，置 1 表示检测到黑线。
 */

typedef enum {
    TRACKING_PID_LINE_FOUND = 0,
    TRACKING_PID_LINE_LOST,
    /* 岔路黑线按当前内/外圈规则被全部忽略。 */
    TRACKING_PID_BRANCH_IGNORED,
    TRACKING_PID_ALL_BLACK
} TrackingPID_Status_t;

typedef enum {
    /* 外圈：多路黑线时忽略最左侧三路，防止岔路口误左转。 */
    TRACKING_PID_LANE_OUTER = 0,
    /* 内圈：多路黑线时忽略最右侧三路，防止岔路口误右转。 */
    TRACKING_PID_LANE_INNER
} TrackingPID_Lane_t;

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
    TrackingPID_Lane_t Lane;
} TrackingPID_Config_t;

typedef struct {
    TrackingPID_Config_t Config;

    /* 运行值对外开放，供 OLED 显示和蓝牙调参观察。 */
    float Out;
    float Position;
    float LastPosition;
    float Error;
    float LastError;

    /* 滤波内部状态必须通过 TrackingPID_Reset() 清零。 */
    float FilteredPosition;
    bool FilterReady;
} TrackingPID_t;

typedef struct {
    int16_t LeftSpeed;
    int16_t RightSpeed;
    float Position;
} TrackingPID_Output_t;

void TrackingPID_Init(TrackingPID_t *pid);
void TrackingPID2_Init(TrackingPID_t *pid);
void TrackingPID_Reset(TrackingPID_t *pid);
TrackingPID_Status_t TrackingPID_Update(TrackingPID_t *pid,
                                        uint8_t black_mask,
                                        int16_t base_speed,
                                        TrackingPID_Output_t *output);

#endif
