#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "myOLED.h"
#include "myPID.h"
#include "motor.h"
#include "myBluetooth.h"
#include "motor_speed.h"
#include "oled_software_i2c.h"
#include "myTask.h"
#include "grayscale_uart.h"

bool OLED_Flag;
volatile uint16_t ADC_Val;
bool ADC_Flag;
bool timer_10ms_flag;

volatile bool start_100ms_timer = false;
volatile bool check_100ms_flag  = false;

#define STARTUP_DELAY_TICKS   300U
#define ADC_MIDDLE_MIN        3000U
#define ADC_MIDDLE_MAX        3100U
#define ADC_UP_MIN            2000U
#define ADC_UP_MAX            2100U

static uint16_t startup_tick = 0;
static bool startup_done = false;
static bool middle_pressed = false;
static bool up_pressed = false;

static void LineFollow_Start(void)
{
    Tracking_PID_Reset();
    check_100ms_flag = false;
    start_100ms_timer = false;
    g_line_follow_enabled = true;
}

static void LineFollow_Stop(void)
{
    g_line_follow_enabled = false;
    Motor_Brake();
}

static void Key_Scan(uint16_t adc)
{
    bool middle_now = (adc >= ADC_MIDDLE_MIN) && (adc <= ADC_MIDDLE_MAX);
    bool up_now = (adc >= ADC_UP_MIN) && (adc <= ADC_UP_MAX);

    if (middle_now) {
        middle_pressed = true;
        return;
    }

    if (middle_pressed) {
        middle_pressed = false;
        Bluetooth_EnterLocalDebugMode();
        if (g_line_follow_enabled) {
            LineFollow_Stop();
        } else {
            LineFollow_Start();
        }
    }

    if (up_now) {
        up_pressed = true;
        return;
    }

    if (up_pressed) {
        up_pressed = false;
        Bluetooth_EnterLocalDebugMode();
        current_task++;
        if (current_task == TASK_MAX) {
            current_task = TASK_ID_1;
        }
    }
}

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    OLED_Init();
    Motor_Init();
    MotorSpeed_Init();
    Motor_Brake();
    Tracking_PID_Init();
    Grayscale_UART_Init();

    NVIC_EnableIRQ(TIMER_100MS_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_10MS_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC_BUTTON_INST_INT_IRQN);
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

            Bluetooth_CheckSyncTimeout();

            if (startup_done && g_line_follow_enabled) {
                ExecuteTask(current_task);
            }
        }

        if (ADC_Flag) {
            ADC_Val = DL_ADC12_getMemResult(ADC_BUTTON_INST, DL_ADC12_MEM_IDX_0);
            DL_ADC12_enableConversions(ADC_BUTTON_INST);
            ADC_Flag = false;
            Key_Scan(ADC_Val);
        } else {
            DL_ADC12_startConversion(ADC_BUTTON_INST);
        }

        if (OLED_Flag) {
            MainInterface_Show();
            OLED_Flag = false;
        }
    }
}

void ADC_BUTTON_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC_BUTTON_INST))
    {
        case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
            ADC_Flag = true;
            break;
        default:
            break;
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
    switch (DL_TimerA_getPendingInterrupt(TIMER_10MS_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            timer_10ms_flag = true;
            break;
        default:
            break;
    }
}
