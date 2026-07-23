#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "main.h"
#include "myOLED.h"
#include "myLCD.h"
#include "myPID.h"
#include "motor.h"
#include "myBluetooth.h"
#include "motor_speed.h"
#include "oled_hardware_i2c.h"
#include "myTask.h"
#include "Grayscale_Sensor.h"
#include "run_button.h"
#include "Drivers/SingleAxisGyro/single_axis_gyro.h"

bool OLED_Flag;
bool timer_10ms_flag;

volatile bool start_100ms_timer = false;
volatile bool check_100ms_flag  = false;

#define STARTUP_DELAY_TICKS   100U

static uint16_t startup_tick = 0;
static bool startup_done = false;

/* 陀螺仪协议实例由主程序持有，底层驱动本身不依赖 MSPM0。 */
static SingleAxisGyro_Device gyro;

bool Gyro_GetAngle(float *angle_deg)
{
    return SingleAxisGyro_GetAngle(&gyro, angle_deg);
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    /* 当前只读取角度，不发送配置命令，因此发送和延时回调传 NULL。 */
    SingleAxisGyro_Init(&gyro, NULL, NULL, NULL);
    NVIC_EnableIRQ(UART_GYRO_INST_INT_IRQN);//陀螺仪
    NVIC_ClearPendingIRQ(UART_GYRO_INST_INT_IRQN);

    OLED_Init();
    MyLCD_Init();
    Motor_Init();
    MotorSpeed_Init();
    Motor_Brake();
    Tracking_PID_Init();
    Grayscale_Sensor_Init();
    RunButton_Init();

    NVIC_EnableIRQ(TIMER_100MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_10MS_INST_INT_IRQN);
    Interrupt_Init();
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    while (1)
    {
        if (bt_cmd_ready_flag) {
            Bluetooth_ParseCommand(bt_rx_buffer);
            bt_cmd_ready_flag = false;
        }

        if (timer_10ms_flag)
        {
            timer_10ms_flag = false;

            MotorSpeed_Update(0.01f);

            if (!startup_done) {
                startup_tick++;
                if (startup_tick >= STARTUP_DELAY_TICKS) {
                    startup_done = true;
                }
            }

            RunButton_Update();
            Bluetooth_CheckSyncTimeout();

            if (startup_done && g_line_follow_enabled) {
                ExecuteTask(current_task);
            }
        }

        if (OLED_Flag) {
            MainInterface_Show();
            MyLCD_Show();
            OLED_Flag = false;
        }
    }
}

void TIMER_100MS_INST_IRQHandler(void)
{
    switch (DL_TimerA_getPendingInterrupt(TIMER_100MS_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            OLED_Flag = true;
            if (start_100ms_timer) {
                check_100ms_flag = true;
            }
            break;
        default:
            break;
    }
}

void TIMER_10MS_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_10MS_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            timer_10ms_flag = true;
            break;
        default:
            break;
    }
}

void UART_GYRO_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_GYRO_INST)) {
        case DL_UART_IIDX_RX:
            /* 中断中只取字节和解析帧，OLED 显示留在主循环执行。 */
            while (!DL_UART_isRXFIFOEmpty(UART_GYRO_INST)) {
                SingleAxisGyro_ReceiveByte(
                    &gyro, (uint8_t)DL_UART_receiveData(UART_GYRO_INST));
            }
            break;

        default:
            break;
    }
}
