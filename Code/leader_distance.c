#include "leader_distance.h"

#include "clock.h"
#include "vl53l0x.h"

#include <stddef.h>

#define LEADER_DISTANCE_MIN_MM (10U)
#define LEADER_DISTANCE_MAX_MM (2000U)

volatile float g_leader_distance_raw = 0.0f;
volatile float g_leader_distance_cm = 0.0f;
volatile bool g_leader_distance_updated = false;
volatile bool g_leader_distance_valid = false;
volatile unsigned long g_leader_distance_last_update_ms = 0UL;

void LeaderDistance_Init(void)
{
    g_leader_distance_raw = 0.0f;
    g_leader_distance_cm = 0.0f;
    g_leader_distance_updated = false;
    g_leader_distance_valid = false;
    g_leader_distance_last_update_ms = 0UL;

    /* 初始化失败时保持距离无效，避免使用伪造的测距数据。 */
    (void)VL53L0X_Init();
}

bool LeaderDistance_Process(void)
{
    uint16_t distance_mm;
    float corrected_distance_cm;

    g_leader_distance_updated = false;

    /* VL53L0X连续测量约33 ms产生一帧；10 ms调用只做非阻塞轮询。 */
    if (VL53L0X_Process() && VL53L0X_GetDistance(&distance_mm) &&
        (distance_mm >= LEADER_DISTANCE_MIN_MM) &&
        (distance_mm <= LEADER_DISTANCE_MAX_MM)) {
        corrected_distance_cm =
            (float)distance_mm * 0.1f - LEADER_DISTANCE_INSTALL_OFFSET_CM;
        if (corrected_distance_cm < 0.0f) {
            corrected_distance_cm = 0.0f;
        }

        g_leader_distance_raw = (float)distance_mm;
        g_leader_distance_cm = corrected_distance_cm;
        g_leader_distance_last_update_ms = tick_ms;
        g_leader_distance_valid = true;
        g_leader_distance_updated = true;
        return true;
    }

    if (g_leader_distance_valid &&
        ((tick_ms - g_leader_distance_last_update_ms) >
         LEADER_DISTANCE_TIMEOUT_MS)) {
        g_leader_distance_valid = false;
        g_leader_distance_raw = 0.0f;
        g_leader_distance_cm = 0.0f;
    }
    return false;
}

bool LeaderDistance_Get(float *distance_cm)
{
    if ((distance_cm == NULL) || !g_leader_distance_valid) {
        return false;
    }

    *distance_cm = g_leader_distance_cm;
    return true;
}
