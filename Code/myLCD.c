#include "myLCD.h"

#include <stdint.h>
#include <stdio.h>

#include "Grayscale_Sensor.h"
#include "leader_distance.h"
#include "main.h"
#include "motor_speed.h"
#include "myPID.h"
#include "myTask.h"
#include "../Drivers/LCD_SPI/lcd_spi.h"

#define MY_LCD_COLUMNS (20U)
#define MY_LCD_ROW_STEP (19U)

static void MyLCD_ShowLine(uint8_t row, const char *text, uint16_t color)
{
    char padded[MY_LCD_COLUMNS + 1U];

    snprintf(padded, sizeof(padded), "%-20.20s", text);
    LCD_SPI_DrawString(4U, (uint16_t)(4U + row * MY_LCD_ROW_STEP),
                       padded, color, LCD_COLOR_BLACK);
}

void MyLCD_Init(void)
{
    LCD_SPI_Init();
    LCD_SPI_Fill(LCD_COLOR_BLACK);
}

void MyLCD_Show(void)
{
    char text[32];
    uint8_t black_mask;
    float distance_cm;
    float gyro_angle_deg;

    snprintf(text, sizeof(text), "LINE:%s",
             g_line_follow_enabled ? "RUN" : "STOP");
    MyLCD_ShowLine(0U, text, g_line_follow_enabled ?
                   LCD_COLOR_GREEN : LCD_COLOR_RED);

    snprintf(text, sizeof(text), "A:%4.2f%c m/s",
             motor_speed_A_mps < 0 ? -motor_speed_A_mps : motor_speed_A_mps,
             MotorSpeed_DirChar(motor_speed_dir_A));
    MyLCD_ShowLine(1U, text, LCD_COLOR_CYAN);

    black_mask = Grayscale_Sensor_Read();
    snprintf(text, sizeof(text), "L:%c%c%c%c%c%c%c%c:R",
             (black_mask & 0x80U) ? '1' : '0',
             (black_mask & 0x40U) ? '1' : '0',
             (black_mask & 0x20U) ? '1' : '0',
             (black_mask & 0x10U) ? '1' : '0',
             (black_mask & 0x08U) ? '1' : '0',
             (black_mask & 0x04U) ? '1' : '0',
             (black_mask & 0x02U) ? '1' : '0',
             (black_mask & 0x01U) ? '1' : '0');
    MyLCD_ShowLine(2U, text, LCD_COLOR_YELLOW);

    snprintf(text, sizeof(text), "B:%4.2f%c m/s",
             motor_speed_B_mps < 0 ? -motor_speed_B_mps : motor_speed_B_mps,
             MotorSpeed_DirChar(motor_speed_dir_B));
    MyLCD_ShowLine(3U, text, LCD_COLOR_CYAN);

    snprintf(text, sizeof(text), "V:%d/%d P:%4.1f",
             g_distance_control_speed, g_base_speed, tracking_pid.Out);
    MyLCD_ShowLine(4U, text, LCD_COLOR_MAGENTA);

    snprintf(text, sizeof(text), "Task:%d", (uint8_t)current_task);
    MyLCD_ShowLine(5U, text, LCD_COLOR_WHITE);

    if (LeaderDistance_Get(&distance_cm)) {
        snprintf(text, sizeof(text), "TOF:%6.1fcm", distance_cm);
    } else {
        snprintf(text, sizeof(text), "TOF:----.-cm");
    }
    MyLCD_ShowLine(6U, text, LCD_COLOR_BLUE);

    if (Gyro_GetAngle(&gyro_angle_deg)) {
        snprintf(text, sizeof(text), "Yaw:%7.2f deg", gyro_angle_deg);
    } else {
        snprintf(text, sizeof(text), "Yaw:---.-- deg");
    }
    MyLCD_ShowLine(7U, text, LCD_COLOR_GREEN);
}
