#include "encoder.h"
#include "ti_msp_dl_config.h"

/* CCS generated makefiles already build encoder.c; keep the new module linked without editing generated files. */
#define MOTOR_SPEED_IMPLEMENTATION
#include "motor_speed.c"

/* 全局编码器脉冲计数器, 在 GROUP1 中断中累加 */
volatile int32_t enc_count_A = 0;   /* 左轮: PA8 上升沿触发 */
volatile int32_t enc_count_B = 0;   /* 右轮: PA16 上升沿触发 */

/* 滤波后的编码器增量 (每 10ms 更新), 参考 pid_speed 的低通滤波 */
volatile float enc_speed_A = 0.0f;
volatile float enc_speed_B = 0.0f;

/* 差速匹配 PI 增益 (蓝牙 E:xx 调 Kp) */
volatile float g_enc_kp = 3.0f;
volatile float g_enc_kd = 0.5f;

/* 上一次读取的计数值 */
static int32_t last_count_A = 0;
static int32_t last_count_B = 0;

void Encoder_Init(void)
{
    enc_count_A = 0;
    enc_count_B = 0;
    last_count_A = 0;
    last_count_B = 0;
    enc_speed_A = 0.0f;
    enc_speed_B = 0.0f;
}

void Encoder_GetDeltas(int16_t *delta_A, int16_t *delta_B)
{
    int32_t cur_A = enc_count_A;
    int32_t cur_B = enc_count_B;

    *delta_A = (int16_t)(cur_A - last_count_A);
    *delta_B = (int16_t)(cur_B - last_count_B);

    last_count_A = cur_A;
    last_count_B = cur_B;
}

/**
 * @brief  低通滤波更新轮速 (与参考 pid_speed 一致: real = real*0.2 + raw*0.8)
 */
void Encoder_UpdateSpeeds(int16_t delta_A, int16_t delta_B)
{
    enc_speed_A = enc_speed_A * 0.2f + (float)delta_A * 0.8f;
    enc_speed_B = enc_speed_B * 0.2f + (float)delta_B * 0.8f;
}
