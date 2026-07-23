/* ================================================================
 *  myOLED.c - basic chassis debug page, refreshed every 100 ms
 * ================================================================ */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "oled_hardware_i2c.h"
#include "Grayscale_Sensor.h"
#include "motor_speed.h"
#include "myPID.h"
#include "myTask.h"
#include "leader_distance.h"
#include "main.h"

void MainInterface_Show(void)
{
    char text[64];
    uint8_t black_mask;
    float distance_cm;
    float gyro_angle_deg;

    sprintf(text, "LINE:%s", g_line_follow_enabled ? "RUN " : "STOP");
    OLED_ShowString(0, 0, (uint8_t *)text, 8);

    sprintf(text, "A:%4.2f%c m/s", motor_speed_A_mps < 0 ? -motor_speed_A_mps : motor_speed_A_mps,
            MotorSpeed_DirChar(motor_speed_dir_A));
    OLED_ShowString(0, 1, (uint8_t *)text, 8);

    black_mask = Grayscale_Sensor_Read();
    sprintf(text, "L:%c%c%c%c%c%c%c%c:R",
            (black_mask & 0x80U) ? '1' : '0',
            (black_mask & 0x40U) ? '1' : '0',
            (black_mask & 0x20U) ? '1' : '0',
            (black_mask & 0x10U) ? '1' : '0',
            (black_mask & 0x08U) ? '1' : '0',
            (black_mask & 0x04U) ? '1' : '0',
            (black_mask & 0x02U) ? '1' : '0',
            (black_mask & 0x01U) ? '1' : '0');
    OLED_ShowString(0, 2, (uint8_t *)text, 8);

    sprintf(text, "B:%4.2f%c m/s", motor_speed_B_mps < 0 ? -motor_speed_B_mps : motor_speed_B_mps,
            MotorSpeed_DirChar(motor_speed_dir_B));
    OLED_ShowString(0, 3, (uint8_t *)text, 8);

    sprintf(text, "V:%d/%d P:%4.1f", g_distance_control_speed,
            g_base_speed, tracking_pid.Out);
    OLED_ShowString(0, 4, (uint8_t *)text, 8);

    sprintf(text, "Task:%d", (uint8_t)current_task);
    OLED_ShowString(0, 5, (uint8_t *)text, 8);

    if (LeaderDistance_Get(&distance_cm)) {
        sprintf(text, "TOF:%6.1fcm", distance_cm);
    } else {
        sprintf(text, "TOF:----.-cm");
    }
    OLED_ShowString(0, 6, (uint8_t *)text, 8);

    if (Gyro_GetAngle(&gyro_angle_deg)) {
        sprintf(text, "Yaw:%7.2f deg", gyro_angle_deg);
    } else {
        sprintf(text, "Yaw:---.-- deg");
    }
    OLED_ShowString(0, 7, (uint8_t *)text, 8);
}
