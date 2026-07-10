#include "motor_speed.h"
#include "ti_msp_dl_config.h"

#ifdef MOTOR_SPEED_IMPLEMENTATION

volatile int32_t motor_speed_count_A = 0;
volatile int32_t motor_speed_count_B = 0;
volatile float motor_speed_A_mps = 0.0f;
volatile float motor_speed_B_mps = 0.0f;
volatile int8_t motor_speed_dir_A = MOTOR_SPEED_DIR_STOP;
volatile int8_t motor_speed_dir_B = MOTOR_SPEED_DIR_STOP;

static int32_t motor_last_count_A = 0;
static int32_t motor_last_count_B = 0;

static int8_t MotorSpeed_ReadDirA(void)
{
    int8_t dir = (DL_GPIO_readPins(ENC_A_DIR_PORT, ENC_A_DIR_PIN_18_PIN) != 0) ?
                 MOTOR_SPEED_DIR_FORWARD : MOTOR_SPEED_DIR_REVERSE;

    return MOTOR_SPEED_A_DIR_HIGH_FORWARD ? dir : (int8_t)-dir;
}

static int8_t MotorSpeed_ReadDirB(void)
{
    int8_t dir = (DL_GPIO_readPins(ENC_B_DIR_PORT, ENC_B_DIR_PIN_25_PIN) != 0) ?
                 MOTOR_SPEED_DIR_FORWARD : MOTOR_SPEED_DIR_REVERSE;

    return MOTOR_SPEED_B_DIR_HIGH_FORWARD ? dir : (int8_t)-dir;
}

void MotorSpeed_Init(void)
{
    motor_speed_count_A = 0;
    motor_speed_count_B = 0;
    motor_speed_A_mps = 0.0f;
    motor_speed_B_mps = 0.0f;
    motor_speed_dir_A = MOTOR_SPEED_DIR_STOP;
    motor_speed_dir_B = MOTOR_SPEED_DIR_STOP;
    motor_last_count_A = 0;
    motor_last_count_B = 0;
}

void MotorSpeed_OnPulseA(void)
{
    motor_speed_dir_A = MotorSpeed_ReadDirA();
    motor_speed_count_A += motor_speed_dir_A;
}

void MotorSpeed_OnPulseB(void)
{
    motor_speed_dir_B = MotorSpeed_ReadDirB();
    motor_speed_count_B += motor_speed_dir_B;
}

void MotorSpeed_Update(float dt_s)
{
    if (dt_s <= 0.0f) {
        return;
    }

    int32_t cur_A;
    int32_t cur_B;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    cur_A = motor_speed_count_A;
    cur_B = motor_speed_count_B;
    if (primask == 0U) {
        __enable_irq();
    }

    int32_t delta_A = cur_A - motor_last_count_A;
    int32_t delta_B = cur_B - motor_last_count_B;

    motor_last_count_A = cur_A;
    motor_last_count_B = cur_B;

    float scale = MOTOR_SPEED_WHEEL_CIRCUMFERENCE_M /
                  ((float)MOTOR_SPEED_PULSES_PER_WHEEL * dt_s);

    motor_speed_A_mps = motor_speed_A_mps * 0.2f + ((float)delta_A * scale) * 0.8f;
    motor_speed_B_mps = motor_speed_B_mps * 0.2f + ((float)delta_B * scale) * 0.8f;

    if (delta_A == 0) {
        motor_speed_dir_A = MOTOR_SPEED_DIR_STOP;
    }
    if (delta_B == 0) {
        motor_speed_dir_B = MOTOR_SPEED_DIR_STOP;
    }
}

char MotorSpeed_DirChar(int8_t dir)
{
    if (dir > 0) {
        return 'F';
    }
    if (dir < 0) {
        return 'R';
    }
    return 'S';
}

#endif
