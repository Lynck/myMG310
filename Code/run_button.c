#include "run_button.h"

#include <stdbool.h>
#include <stdint.h>

#include "ti_msp_dl_config.h"
#include "motor.h"
#include "myBluetooth.h"
#include "myTask.h"

#define RUN_BUTTON_DEBOUNCE_TICKS 3U

static uint8_t press_ticks;
static bool press_handled;

static bool RunButton_IsPressed(void)
{
    /* PB21 使用内部上拉，板载按键按下后把输入拉低。 */
    return DL_GPIO_readPins(RUN_BUTTON_PORT, RUN_BUTTON_PIN_21_PIN) == 0U;
}

void RunButton_Init(void)
{
    press_ticks = 0U;

    /* 上电时若按键正被按住，先等待释放，避免误触发启动。 */
    press_handled = RunButton_IsPressed();
}

void RunButton_Update(void)
{
    if (!RunButton_IsPressed()) {
        press_ticks = 0U;
        press_handled = false;
        return;
    }

    if (press_handled) {
        return;
    }

    if (press_ticks < RUN_BUTTON_DEBOUNCE_TICKS) {
        press_ticks++;
    }
    if (press_ticks < RUN_BUTTON_DEBOUNCE_TICKS) {
        return;
    }

    /* 一次按压只切换一次，必须释放后才允许下一次切换。 */
    press_handled = true;
    if (g_line_follow_enabled) {
        g_line_follow_enabled = false;
        Motor_Brake();
    } else {
        /* 本地按键启动不依赖蓝牙心跳，否则会被同步超时立即停掉。 */
        Bluetooth_EnterLocalDebugMode();
        Tracking_PID_Reset();
        Tracking_SetLeaderStopRequested(false);
        g_line_follow_enabled = true;
    }
}
