/* ================================================================
 *  myOLED.c - basic chassis debug page, refreshed every 100 ms
 * ================================================================ */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "oled_software_i2c.h"
#include "imu660rb.h"
#include "PID.h"
#include "Grayscale_Sensor.h"
#include "encoder.h"
#include "myPID.h"
#include "myTask.h"

void MainInterface_Show(void)
{
    float yaw = euler.angle.yaw;
    char text[64];

    Read_data_1_GPIO();

    sprintf(text, "LINE:%s", g_line_follow_enabled ? "RUN " : "STOP");
    OLED_ShowString(0, 0, (uint8_t *)text, 8);

    sprintf(text, "Yaw:%7.2f", yaw);
    OLED_ShowString(0, 1, (uint8_t *)text, 8);

    sprintf(text, "VA:%4.1f VB:%4.1f", enc_speed_A, enc_speed_B);
    OLED_ShowString(0, 2, (uint8_t *)text, 8);

    sprintf(text, "L:%d%d%d%d%d%d%d%d:R",
            data_1.D8, data_1.D7, data_1.D6, data_1.D5,
            data_1.D4, data_1.D3, data_1.D2, data_1.D1);
    OLED_ShowString(0, 3, (uint8_t *)text, 8);

    extern PID_t tracking_pid;
    sprintf(text, "Spd:%d PID:%4.1f", g_base_speed, tracking_pid.Out);
    OLED_ShowString(0, 4, (uint8_t *)text, 8);

    sprintf(text, "Task:%d", (uint8_t)current_task);
    OLED_ShowString(0, 5, (uint8_t *)text, 8);
}
