/* ================================================================
 *  myOLED.c - basic chassis debug page, refreshed every 100 ms
 * ================================================================ */

#include "ti_msp_dl_config.h"
#include <stdio.h>
#include "oled_software_i2c.h"
#include "grayscale_uart.h"
#include "motor_speed.h"
#include "myPID.h"
#include "myTask.h"
#include "vl53l0x.h"

void MainInterface_Show(void)
{
    char text[64];
    uint8_t black_mask;
    uint16_t distance_mm;

    sprintf(text, "LINE:%s", g_line_follow_enabled ? "RUN " : "STOP");
    OLED_ShowString(0, 0, (uint8_t *)text, 8);

    sprintf(text, "A:%4.2f%c m/s", motor_speed_A_mps < 0 ? -motor_speed_A_mps : motor_speed_A_mps,
            MotorSpeed_DirChar(motor_speed_dir_A));
    OLED_ShowString(0, 1, (uint8_t *)text, 8);

    if (Grayscale_UART_GetBlackMask(&black_mask)) {
        sprintf(text, "L:%c%c%c%c%c%c%c%c:R",
                (black_mask & 0x80U) ? 'B' : 'W',
                (black_mask & 0x40U) ? 'B' : 'W',
                (black_mask & 0x20U) ? 'B' : 'W',
                (black_mask & 0x10U) ? 'B' : 'W',
                (black_mask & 0x08U) ? 'B' : 'W',
                (black_mask & 0x04U) ? 'B' : 'W',
                (black_mask & 0x02U) ? 'B' : 'W',
                (black_mask & 0x01U) ? 'B' : 'W');
    } else {
        sprintf(text, "L:--------:R");
    }
    OLED_ShowString(0, 2, (uint8_t *)text, 8);

    sprintf(text, "B:%4.2f%c m/s", motor_speed_B_mps < 0 ? -motor_speed_B_mps : motor_speed_B_mps,
            MotorSpeed_DirChar(motor_speed_dir_B));
    OLED_ShowString(0, 3, (uint8_t *)text, 8);

    sprintf(text, "T:%dcm P:%4.1f", g_base_speed, tracking_pid.Out);
    OLED_ShowString(0, 4, (uint8_t *)text, 8);

    sprintf(text, "Task:%d", (uint8_t)current_task);
    OLED_ShowString(0, 5, (uint8_t *)text, 8);

    if (VL53L0X_GetDistance(&distance_mm)) {
        sprintf(text, "TOF:%4umm", distance_mm);
    } else {
        sprintf(text, "TOF:----mm");
    }
    OLED_ShowString(0, 6, (uint8_t *)text, 8);
}
