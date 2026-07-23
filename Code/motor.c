/*  PWM配置��?
*       1.添加PWM，命名为"PWM_MOTOR"
*       2.Timer Clock Divider设置��?分频
*       3.PWM Period Count设置��?000（默��?000不用改）
*       4.PinMux分别设置��?PA26""PA24"
*   GPIO:
*       1.分别设置五个GPIO��?AIN1""AIN2""BIN1""BIN2""STBY"
*       2.引脚分别对应命名"PIN_2""PIN_24""PIN_20""PIN_7""PIN_8"
*       3.指定对应引脚，分别为PA2,PB24,PB20,PB7,PB8
*
*/
#include "ti_msp_dl_config.h"
// #include <ti/driverlib/dl_timer.h>

//PWM Period Count
#define MOTOR_PWM_PERIOD  (1000)

/**
 * @brief 电机初始��?
 */
void Motor_Init(void) {
    // 1. 拉高 STBY 引脚（使能端��?
    /* TB6612 �� STBY �������ߺ� H �ŲŹ�����PWM ������Ҳ������������ */
    DL_GPIO_setPins(STBY_PORT, STBY_PIN_8_PIN);
    
    // 2. 启动 PWM 定时��?
    DL_Timer_startCounter(PWM_MOTOR_INST);
}

/**
 * @brief 电机 A 控制 (PWMA)
 */
void Motor_SetSpeed_A(int16_t speed) {
    /* speed �ķ��ž�������ת������ֵ�����ٷֱȣ���Χ������ -100~100�� */
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    /* A ͨ����ת������ AIN1/AIN2 ���Ծ����������ַ������ˣ��Ȳ������ͽ��ߡ� */
    if (speed > 0) {
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_2_PIN);
        DL_GPIO_setPins(AIN2_PORT, AIN2_PIN_24_PIN);
        speed = 100 - speed;
    } 
    else if (speed < 0) {
        DL_GPIO_setPins(AIN1_PORT, AIN1_PIN_2_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_24_PIN);
        speed = -speed;
        speed = 100 - speed;
    } 
    else {
        /* speed=0 ʱ�ر����������ţ���ͨ��ֹͣ������������ */
        DL_GPIO_clearPins(AIN1_PORT, AIN1_PIN_2_PIN);
        DL_GPIO_clearPins(AIN2_PORT, AIN2_PIN_24_PIN);
    }

    // 计算占空比，公式为：$duty = speed \times \frac{PERIOD}{100}$
    /* ������ PWM Ϊ����ռ�ձ�д����speed Խ����д���Ƚ�ֵԽС�� */
    uint32_t duty = (uint32_t)(speed * (MOTOR_PWM_PERIOD / 100.0f));
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, duty, DL_TIMER_CC_0_INDEX);
}

/**
 * @brief 电机 B 控制 (PWMB)
 */
void Motor_SetSpeed_B(int16_t speed) {
    speed = -speed;

    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    if (speed < 0) {
        DL_GPIO_setPins(BIN1_PORT, BIN1_PIN_20_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_7_PIN);
        speed = -speed;
        speed = 100 - speed;
    } 
    else if (speed > 0) {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_20_PIN);
        DL_GPIO_setPins(BIN2_PORT, BIN2_PIN_7_PIN);
        speed = 100 - speed;
    } 
    else {
        DL_GPIO_clearPins(BIN1_PORT, BIN1_PIN_20_PIN);
        DL_GPIO_clearPins(BIN2_PORT, BIN2_PIN_7_PIN);
    }

    /* B ͨ��д�� PWM_MOTOR �� CC1���� A ͨ�� CC0 �ֿ��� */
    uint32_t duty = (uint32_t)(speed * (MOTOR_PWM_PERIOD / 100.0f));
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, duty, DL_TIMER_CC_1_INDEX);
}

/**
 * @brief 急刹��?
 */
void Motor_Brake(void) {
    // AIN/BIN 全部拉高实现急刹
    /* ��ɲ�� speed=0 ��ͬ���ĸ�������ȫ���ߣ��� TB6612 ������ɲ�� */
    DL_GPIO_setPins(AIN1_PORT, AIN1_PIN_2_PIN);
    DL_GPIO_setPins(AIN2_PORT, AIN2_PIN_24_PIN);
    DL_GPIO_setPins(BIN1_PORT, BIN1_PIN_20_PIN);
    DL_GPIO_setPins(BIN2_PORT, BIN2_PIN_7_PIN);

    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, 0, DL_TIMER_CC_1_INDEX);
}