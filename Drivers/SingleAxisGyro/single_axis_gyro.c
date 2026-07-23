#include "single_axis_gyro.h"

#include <string.h>

#define SINGLE_AXIS_GYRO_RX_HEADER              (0x5AU)
#define SINGLE_AXIS_GYRO_TYPE_ANGULAR_VELOCITY  (0xAAU)
#define SINGLE_AXIS_GYRO_TYPE_ANGLE             (0xBBU)
#define SINGLE_AXIS_GYRO_VALID_ANGLE            (0x01U)
#define SINGLE_AXIS_GYRO_VALID_ANGULAR_VELOCITY (0x02U)

#define SINGLE_AXIS_GYRO_REG_SAVE       (0x00U)
#define SINGLE_AXIS_GYRO_REG_OUTPUT_RATE (0x02U)
#define SINGLE_AXIS_GYRO_REG_YAW_ZERO   (0x15U)

#define SINGLE_AXIS_GYRO_COMMAND_DELAY_MS (100U)

static bool SingleAxisGyro_IsDataType(uint8_t type)
{
    return (type == SINGLE_AXIS_GYRO_TYPE_ANGULAR_VELOCITY) ||
           (type == SINGLE_AXIS_GYRO_TYPE_ANGLE);
}

static int16_t SingleAxisGyro_DecodeInt16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

static bool SingleAxisGyro_Send(SingleAxisGyro_Device *device, const uint8_t command[5])
{
    return (device != NULL) && (device->write != NULL) &&
           device->write(device->user_data, command, 5U);
}

static bool SingleAxisGyro_Unlock(SingleAxisGyro_Device *device)
{
    /*
     * 手册通用格式为低字节在前，但解锁示例反复明确写成 55 AA 13 8E 5F。
     * 这里严格发送手册给出的字节序，不用 SingleAxisGyro_WriteRegister() 拼装。
     */
    static const uint8_t command[5] = {0x55U, 0xAAU, 0x13U, 0x8EU, 0x5FU};
    return SingleAxisGyro_Send(device, command);
}

static bool SingleAxisGyro_RunSavedCommand(
    SingleAxisGyro_Device *device, uint8_t address, uint16_t value)
{
    if ((device == NULL) || (device->delay_ms == NULL) ||
        !SingleAxisGyro_Unlock(device)) {
        return false;
    }

    device->delay_ms(device->user_data, SINGLE_AXIS_GYRO_COMMAND_DELAY_MS);
    if (!SingleAxisGyro_WriteRegister(device, address, value)) {
        return false;
    }

    device->delay_ms(device->user_data, SINGLE_AXIS_GYRO_COMMAND_DELAY_MS);
    if (!SingleAxisGyro_WriteRegister(device, SINGLE_AXIS_GYRO_REG_SAVE, 0x0000U)) {
        return false;
    }

    device->delay_ms(device->user_data, SINGLE_AXIS_GYRO_COMMAND_DELAY_MS);
    return true;
}

void SingleAxisGyro_Init(SingleAxisGyro_Device *device, SingleAxisGyro_WriteFn write,
    SingleAxisGyro_DelayMsFn delay_ms, void *user_data)
{
    if (device == NULL) {
        return;
    }

    memset(device, 0, sizeof(*device));
    device->write = write;
    device->delay_ms = delay_ms;
    device->user_data = user_data;
}

void SingleAxisGyro_ReceiveByte(SingleAxisGyro_Device *device, uint8_t byte)
{
    uint8_t checksum;
    int16_t raw;

    if (device == NULL) {
        return;
    }

    if (device->rx_index == 0U) {
        if (byte == SINGLE_AXIS_GYRO_RX_HEADER) {
            device->rx_frame[0] = byte;
            device->rx_index = 1U;
        }
        return;
    }

    if (device->rx_index == 1U && !SingleAxisGyro_IsDataType(byte)) {
        /* 当前字节仍可能是下一帧的帧头，保留它可更快恢复同步。 */
        device->rx_index = (byte == SINGLE_AXIS_GYRO_RX_HEADER) ? 1U : 0U;
        return;
    }

    device->rx_frame[device->rx_index++] = byte;
    if (device->rx_index < SINGLE_AXIS_GYRO_FRAME_SIZE) {
        return;
    }

    checksum = (uint8_t)(device->rx_frame[0] + device->rx_frame[1] +
                         device->rx_frame[2] + device->rx_frame[3]);
    if (checksum != device->rx_frame[4]) {
        device->checksum_error_count++;
        device->rx_index = (byte == SINGLE_AXIS_GYRO_RX_HEADER) ? 1U : 0U;
        return;
    }

    raw = SingleAxisGyro_DecodeInt16(device->rx_frame[2], device->rx_frame[3]);
    if (device->rx_frame[1] == SINGLE_AXIS_GYRO_TYPE_ANGLE) {
        device->angle_raw = raw;
        device->valid_mask |= SINGLE_AXIS_GYRO_VALID_ANGLE;
    } else {
        device->angular_velocity_raw = raw;
        device->valid_mask |= SINGLE_AXIS_GYRO_VALID_ANGULAR_VELOCITY;
    }

    device->good_frame_count++;
    device->rx_index = 0U;
}

bool SingleAxisGyro_GetAngle(const SingleAxisGyro_Device *device, float *angle_deg)
{
    if ((device == NULL) || (angle_deg == NULL) ||
        ((device->valid_mask & SINGLE_AXIS_GYRO_VALID_ANGLE) == 0U)) {
        return false;
    }

    *angle_deg = (float)device->angle_raw * (180.0f / 32768.0f);
    return true;
}

bool SingleAxisGyro_GetAngularVelocity(
    const SingleAxisGyro_Device *device, float *angular_velocity_dps)
{
    if ((device == NULL) || (angular_velocity_dps == NULL) ||
        ((device->valid_mask & SINGLE_AXIS_GYRO_VALID_ANGULAR_VELOCITY) == 0U)) {
        return false;
    }

    *angular_velocity_dps =
        (float)device->angular_velocity_raw * (2000.0f / 32768.0f);
    return true;
}

bool SingleAxisGyro_WriteRegister(
    SingleAxisGyro_Device *device, uint8_t address, uint16_t value)
{
    uint8_t command[5] = {
        0x55U,
        0xAAU,
        address,
        (uint8_t)(value & 0xFFU),
        (uint8_t)(value >> 8)
    };

    return SingleAxisGyro_Send(device, command);
}

bool SingleAxisGyro_SetOutputRate(SingleAxisGyro_Device *device, SingleAxisGyro_OutputRate rate)
{
    if ((uint32_t)rate > (uint32_t)SINGLE_AXIS_GYRO_RATE_1000_HZ) {
        return false;
    }

    return SingleAxisGyro_RunSavedCommand(
        device, SINGLE_AXIS_GYRO_REG_OUTPUT_RATE, (uint16_t)rate);
}

bool SingleAxisGyro_ZeroAngle(SingleAxisGyro_Device *device)
{
    return SingleAxisGyro_RunSavedCommand(device, SINGLE_AXIS_GYRO_REG_YAW_ZERO, 0x0000U);
}


