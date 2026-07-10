#include "motor_speed.h"
#include "PID.h"
#include "motor.h"
#include "ti_msp_dl_config.h"

#ifdef MOTOR_SPEED_IMPLEMENTATION

volatile int32_t motor_speed_count_A = 0;
volatile int32_t motor_speed_count_B = 0;
volatile float motor_speed_A_mps = 0.0f;
volatile float motor_speed_B_mps = 0.0f;
volatile int8_t motor_speed_dir_A = MOTOR_SPEED_DIR_STOP;
volatile int8_t motor_speed_dir_B = MOTOR_SPEED_DIR_STOP;

static PID_t motor_speed_pid;
static int32_t motor_last_count_A = 0;
static int32_t motor_last_count_B = 0;

static int16_t MotorSpeed_ClampCmd(int16_t cmd)
{
    if (cmd > 100) {
        return 100;
    }
    if (cmd < -100) {
        return -100;
    }
    return cmd;
}

static int16_t MotorSpeed_AbsCmd(int16_t cmd)
{
    return (cmd < 0) ? (int16_t)-cmd : cmd;
}

static float MotorSpeed_AbsMps(float speed_mps)
{
    return (speed_mps < 0.0f) ? -speed_mps : speed_mps;
}

static float MotorSpeed_ClampFloat(float value, float min, float max)
{
    if (value > max) {
        return max;
    }
    if (value < min) {
        return min;
    }
    return value;
}

static float MotorSpeed_FeedforwardCmd(float target_mps)
{
    float cmd = target_mps * MOTOR_SPEED_FEEDFORWARD_CMD_PER_MPS;

    if (cmd > 0.0f && cmd < MOTOR_SPEED_MIN_FORWARD_CMD) {
        cmd = MOTOR_SPEED_MIN_FORWARD_CMD;
    }

    return MotorSpeed_ClampFloat(cmd, 0.0f, MOTOR_SPEED_BASE_CMD_MAX);
}

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
    PID_Init(&motor_speed_pid);
    motor_speed_pid.Kp = MOTOR_SPEED_PID_KP;
    motor_speed_pid.Ki = MOTOR_SPEED_PID_KI;
    motor_speed_pid.Kd = MOTOR_SPEED_PID_KD;
    motor_speed_pid.OutMax = MOTOR_SPEED_PID_OUT_MAX;
    motor_speed_pid.OutMin = MOTOR_SPEED_PID_OUT_MIN;
    motor_speed_pid.IntMax = MOTOR_SPEED_PID_INT_MAX;
    motor_speed_pid.IntMin = MOTOR_SPEED_PID_INT_MIN;

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

void MotorSpeed_ResetControl(void)
{
    motor_speed_pid.Out = 0.0f;
    motor_speed_pid.Error0 = 0.0f;
    motor_speed_pid.Error1 = 0.0f;
    motor_speed_pid.Error2 = 0.0f;
    motor_speed_pid.ErrorInt = 0.0f;
}

void MotorSpeed_Control(float target_mps, int16_t turn_cmd, float left_scale, float right_scale)
{
    if (target_mps <= 0.0f) {
        Motor_Brake();
        MotorSpeed_ResetControl();
        return;
    }

    motor_speed_pid.Target = target_mps;
    motor_speed_pid.Actual = (MotorSpeed_AbsMps(motor_speed_A_mps) +
                              MotorSpeed_AbsMps(motor_speed_B_mps)) * 0.5f;
    PID_Update(&motor_speed_pid);

    float base_cmd_f = MotorSpeed_FeedforwardCmd(target_mps) + motor_speed_pid.Out;
    int16_t base_cmd = (int16_t)MotorSpeed_ClampFloat(base_cmd_f, 0.0f, MOTOR_SPEED_BASE_CMD_MAX);
    int16_t turn_limit = (int16_t)(100 - MotorSpeed_AbsCmd(base_cmd));

    if (turn_cmd > turn_limit) {
        turn_cmd = turn_limit;
    } else if (turn_cmd < -turn_limit) {
        turn_cmd = (int16_t)-turn_limit;
    }

    int16_t left_cmd = (int16_t)((base_cmd + turn_cmd) * left_scale);
    int16_t right_cmd = (int16_t)((base_cmd - turn_cmd) * right_scale);

    Motor_SetSpeed_A(MotorSpeed_ClampCmd(left_cmd));
    Motor_SetSpeed_B(MotorSpeed_ClampCmd(right_cmd));
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
