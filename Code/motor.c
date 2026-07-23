#include "ti_msp_dl_config.h"

/* SysConfig 中 PWM Period Count 保持为 1000。 */
#define MOTOR_PWM_PERIOD (1000U)

/**
 * @brief 启动 TB6612FNG 双通道电机 PWM。
 *
 * 64 引脚驱动板已经把 STBY 硬件上拉到 +5V，因此不再占用 MCU GPIO。
 */
void Motor_Init(void)
{
    DL_Timer_startCounter(PWM_MOTOR_INST);
}

/**
 * @brief 设置 TB6612FNG A 通道速度。
 * @param speed 有符号百分比，范围 -100 到 100。
 */
void Motor_SetSpeed_A(int16_t speed)
{
    speed = -speed;

    if (speed > 100) {
        speed = 100;
    } else if (speed < -100) {
        speed = -100;
    }

    /* 方向由 PB17/PB19 控制；本项目的 PWM 比较值与有效占空比反向。 */
    if (speed > 0) {
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_17_PIN);
        DL_GPIO_setPins(AIN2_PORT, AIN2_PIN_19_PIN);
        speed = 100 - speed;
    } else if (speed < 0) {
        DL_GPIO_setPins(AIN1_PORT, AIN1_PIN_17_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_19_PIN);
        speed = 100 + speed;
    } else {
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_17_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_19_PIN);
    }

    DL_TimerG_setCaptureCompareValue(
        PWM_MOTOR_INST,
        (uint32_t)speed * MOTOR_PWM_PERIOD / 100U,
        DL_TIMER_CC_0_INDEX);
}

/**
 * @brief 设置 TB6612FNG B 通道速度。
 * @param speed 有符号百分比，范围 -100 到 100。
 */
void Motor_SetSpeed_B(int16_t speed)
{
    /* 保留原车 B 通道的安装方向补偿。 */
    speed = -speed;

    if (speed > 100) {
        speed = 100;
    } else if (speed < -100) {
        speed = -100;
    }

    /* 方向由 PA16/PB24 控制；CC1 对应新板 PA13/PWMB。 */
    if (speed < 0) {
        DL_GPIO_setPins(BIN1_PORT, BIN1_PIN_16_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_24_PIN);
        speed = 100 + speed;
    } else if (speed > 0) {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_16_PIN);
        DL_GPIO_setPins(BIN2_PORT, BIN2_PIN_24_PIN);
        speed = 100 - speed;
    } else {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_16_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_24_PIN);
    }

    DL_TimerG_setCaptureCompareValue(
        PWM_MOTOR_INST,
        (uint32_t)speed * MOTOR_PWM_PERIOD / 100U,
        DL_TIMER_CC_1_INDEX);
}

/**
 * @brief 两个 H 桥都切到短刹状态，并关闭 PWM 输出。
 */
void Motor_Brake(void)
{
    DL_GPIO_setPins(AIN1_PORT, AIN1_PIN_17_PIN);
    DL_GPIO_setPins(AIN2_PORT, AIN2_PIN_19_PIN);
    DL_GPIO_setPins(BIN1_PORT, BIN1_PIN_16_PIN);
    DL_GPIO_setPins(BIN2_PORT, BIN2_PIN_24_PIN);

    DL_TimerG_setCaptureCompareValue(
        PWM_MOTOR_INST, 0U, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(
        PWM_MOTOR_INST, 0U, DL_TIMER_CC_1_INDEX);
}
