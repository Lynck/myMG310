#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>

/* MG310 编码器参数：电机轴每圈 13 脉冲，减速比 1:30。 */
#define ENC_PULSES_PER_REV     13
#define ENC_GEAR_RATIO         30
#define ENC_PULSES_PER_WHEEL   (ENC_PULSES_PER_REV * ENC_GEAR_RATIO)

/* 编码器累计脉冲计数。 */
extern volatile int32_t enc_count_A;
extern volatile int32_t enc_count_B;

/* 10ms 周期更新后的滤波速度，用于 OLED 显示和后续速度闭环。 */
extern volatile float enc_speed_A;
extern volatile float enc_speed_B;

/* 左右轮速度匹配增益，可通过蓝牙 E:xx 调整。 */
extern volatile float g_enc_kp;
extern volatile float g_enc_kd;

void Encoder_Init(void);
void Encoder_GetDeltas(int16_t *delta_A, int16_t *delta_B);
void Encoder_UpdateSpeeds(int16_t delta_A, int16_t delta_B);

#endif
