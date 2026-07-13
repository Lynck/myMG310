#include "single_axis_gyro.h"

#define GYRO_RX_HEADER              0x5AU
#define GYRO_TYPE_ANGULAR_VELOCITY  0xAAU
#define GYRO_TYPE_YAW               0xBBU
#define GYRO_TYPE_STATUS            0xCCU

#define GYRO_REG_SAVE               0x00U
#define GYRO_REG_OUTPUT_RATE        0x02U
#define GYRO_REG_BAUD               0x03U
#define GYRO_REG_CALIBRATION        0x0AU
#define GYRO_REG_KEY                0x13U
#define GYRO_REG_YAW_ZERO           0x15U

#define GYRO_KEY_UNLOCK             0x8E5FU
#define GYRO_SAVE_SETTINGS          0x0000U
#define GYRO_RESTORE_FACTORY        0x0001U
#define GYRO_REBOOT                 0x00FFU
#define GYRO_BIAS_CALIBRATION       0x0001U
#define GYRO_SCALE_CALIBRATION      0x0003U
#define GYRO_COMMAND_DELAY_MS       100U

static int16_t SingleAxisGyro_DecodeInt16(uint8_t low, uint8_t high)
{
    /* 模块按低字节在前发送，组合后按有符号 16 位数解释。 */
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static bool SingleAxisGyro_CanConfigure(const SingleAxisGyro *gyro)
{
    return (gyro != NULL) && (gyro->write != NULL) &&
           (gyro->delay_ms != NULL);
}

static void SingleAxisGyro_Delay(SingleAxisGyro *gyro)
{
    gyro->delay_ms(gyro->user, GYRO_COMMAND_DELAY_MS);
}

static bool SingleAxisGyro_UnlockAndWrite(SingleAxisGyro *gyro,
                                          uint8_t address,
                                          uint16_t value)
{
    if (!SingleAxisGyro_CanConfigure(gyro)) {
        return false;
    }

    /* 数据手册要求所有设置先解锁，并在相邻命令之间延时 100 ms。 */
    SingleAxisGyro_Unlock(gyro);
    SingleAxisGyro_Delay(gyro);
    SingleAxisGyro_WriteRegister(gyro, address, value);
    SingleAxisGyro_Delay(gyro);
    return true;
}

void SingleAxisGyro_Init(SingleAxisGyro *gyro,
                         SingleAxisGyro_WriteFn write,
                         SingleAxisGyro_DelayMsFn delay_ms,
                         void *user)
{
    if (gyro == NULL) {
        return;
    }

    *gyro = (SingleAxisGyro){0};
    gyro->write = write;
    gyro->delay_ms = delay_ms;
    gyro->user = user;
}

void SingleAxisGyro_ResetParser(SingleAxisGyro *gyro)
{
    if (gyro != NULL) {
        gyro->frame_index = 0U;
    }
}

SingleAxisGyro_FeedResult SingleAxisGyro_FeedByte(SingleAxisGyro *gyro,
                                                   uint8_t byte)
{
    uint8_t checksum;
    int16_t raw;

    if (gyro == NULL) {
        return SINGLE_AXIS_GYRO_FEED_NONE;
    }

    if (gyro->frame_index == 0U) {
        if (byte != GYRO_RX_HEADER) {
            return SINGLE_AXIS_GYRO_FEED_NONE;
        }
        gyro->frame[gyro->frame_index++] = byte;
        return SINGLE_AXIS_GYRO_FEED_NONE;
    }

    gyro->frame[gyro->frame_index++] = byte;
    if (gyro->frame_index < SINGLE_AXIS_GYRO_FRAME_SIZE) {
        return SINGLE_AXIS_GYRO_FEED_NONE;
    }

    gyro->frame_index = 0U;
    checksum = (uint8_t)(gyro->frame[0] + gyro->frame[1] +
                         gyro->frame[2] + gyro->frame[3]);
    /*
     * 手册给出的零偏成功帧以 0x96 结尾，但按累加公式应为 0x26。
     * 这里只兼容手册明确给出的这一帧，普通测量帧仍执行严格校验。
     */
    if ((checksum != gyro->frame[4]) &&
        !((gyro->frame[1] == GYRO_TYPE_STATUS) &&
          (gyro->frame[2] == 0x00U) && (gyro->frame[3] == 0x00U) &&
          (gyro->frame[4] == 0x96U))) {
        if (byte == GYRO_RX_HEADER) {
            gyro->frame[0] = byte;
            gyro->frame_index = 1U;
        }
        return SINGLE_AXIS_GYRO_FEED_CHECKSUM_ERROR;
    }

    raw = SingleAxisGyro_DecodeInt16(gyro->frame[2], gyro->frame[3]);
    switch (gyro->frame[1]) {
        case GYRO_TYPE_ANGULAR_VELOCITY:
            gyro->angular_velocity_raw = raw;
            gyro->angular_velocity_dps = (float)raw * (2000.0f / 32768.0f);
            gyro->angular_velocity_updates++;
            return SINGLE_AXIS_GYRO_FEED_ANGULAR_VELOCITY;

        case GYRO_TYPE_YAW:
            gyro->yaw_raw = raw;
            gyro->yaw_deg = (float)raw * (180.0f / 32768.0f);
            gyro->yaw_updates++;
            return SINGLE_AXIS_GYRO_FEED_YAW;

        case GYRO_TYPE_STATUS:
            gyro->status_raw = raw;
            gyro->status_updates++;
            return SINGLE_AXIS_GYRO_FEED_STATUS;

        default:
            return SINGLE_AXIS_GYRO_FEED_UNKNOWN_TYPE;
    }
}

SingleAxisGyro_FeedResult SingleAxisGyro_FeedBuffer(SingleAxisGyro *gyro,
                                                     const uint8_t *data,
                                                     size_t length)
{
    SingleAxisGyro_FeedResult result = SINGLE_AXIS_GYRO_FEED_NONE;
    size_t i;

    if ((gyro == NULL) || (data == NULL)) {
        return result;
    }

    for (i = 0U; i < length; ++i) {
        SingleAxisGyro_FeedResult current =
            SingleAxisGyro_FeedByte(gyro, data[i]);
        if (current != SINGLE_AXIS_GYRO_FEED_NONE) {
            result = current;
        }
    }
    return result;
}

bool SingleAxisGyro_WriteRegister(SingleAxisGyro *gyro,
                                  uint8_t address,
                                  uint16_t value)
{
    uint8_t command[SINGLE_AXIS_GYRO_FRAME_SIZE];

    if ((gyro == NULL) || (gyro->write == NULL)) {
        return false;
    }

    command[0] = 0x55U;
    command[1] = 0xAAU;
    command[2] = address;
    command[3] = (uint8_t)(value & 0x00FFU);
    command[4] = (uint8_t)(value >> 8);
    gyro->write(gyro->user, command, sizeof(command));
    return true;
}

bool SingleAxisGyro_Unlock(SingleAxisGyro *gyro)
{
    return SingleAxisGyro_WriteRegister(gyro, GYRO_REG_KEY, GYRO_KEY_UNLOCK);
}

bool SingleAxisGyro_Save(SingleAxisGyro *gyro)
{
    return SingleAxisGyro_WriteRegister(gyro, GYRO_REG_SAVE,
                                        GYRO_SAVE_SETTINGS);
}

bool SingleAxisGyro_Reboot(SingleAxisGyro *gyro)
{
    if (!SingleAxisGyro_UnlockAndWrite(gyro, GYRO_REG_SAVE, GYRO_REBOOT)) {
        return false;
    }
    return SingleAxisGyro_Save(gyro);
}

bool SingleAxisGyro_FactoryReset(SingleAxisGyro *gyro)
{
    if (!SingleAxisGyro_UnlockAndWrite(gyro, GYRO_REG_SAVE,
                                       GYRO_RESTORE_FACTORY)) {
        return false;
    }
    return SingleAxisGyro_Save(gyro);
}

bool SingleAxisGyro_ZeroYaw(SingleAxisGyro *gyro)
{
    if (!SingleAxisGyro_UnlockAndWrite(gyro, GYRO_REG_YAW_ZERO, 0x0000U)) {
        return false;
    }
    return SingleAxisGyro_Save(gyro);
}

bool SingleAxisGyro_SetOutputRate(SingleAxisGyro *gyro,
                                  SingleAxisGyro_OutputRate rate)
{
    if ((uint16_t)rate > (uint16_t)SINGLE_AXIS_GYRO_RATE_1000_HZ) {
        return false;
    }
    if (!SingleAxisGyro_UnlockAndWrite(gyro, GYRO_REG_OUTPUT_RATE,
                                       (uint16_t)rate)) {
        return false;
    }
    return SingleAxisGyro_Save(gyro);
}

bool SingleAxisGyro_BeginBaudChange(SingleAxisGyro *gyro,
                                    SingleAxisGyro_Baud baud)
{
    if ((uint16_t)baud > (uint16_t)SINGLE_AXIS_GYRO_BAUD_115200) {
        return false;
    }
    return SingleAxisGyro_UnlockAndWrite(gyro, GYRO_REG_BAUD,
                                         (uint16_t)baud);
}

bool SingleAxisGyro_StartBiasCalibration(SingleAxisGyro *gyro)
{
    return SingleAxisGyro_UnlockAndWrite(gyro, GYRO_REG_CALIBRATION,
                                         GYRO_BIAS_CALIBRATION);
}

bool SingleAxisGyro_RequestBiasStatus(SingleAxisGyro *gyro)
{
    return SingleAxisGyro_WriteRegister(gyro, 0x04U, 0x000AU);
}

bool SingleAxisGyro_StartScaleCalibration(SingleAxisGyro *gyro)
{
    return SingleAxisGyro_UnlockAndWrite(gyro, GYRO_REG_CALIBRATION,
                                         GYRO_SCALE_CALIBRATION);
}

bool SingleAxisGyro_FinishScaleCalibration(SingleAxisGyro *gyro)
{
    return SingleAxisGyro_Save(gyro);
}
