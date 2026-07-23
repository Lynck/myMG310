#ifdef SINGLE_AXIS_GYRO_DRIVER_TEST

#include "single_axis_gyro.h"

#include <assert.h>
#include <math.h>
#include <string.h>

typedef struct {
    uint8_t bytes[32];
    size_t length;
    uint32_t delay_calls;
} TestIO;

static bool TestWrite(void *user_data, const uint8_t *data, size_t length)
{
    TestIO *io = (TestIO *)user_data;
    assert(io->length + length <= sizeof(io->bytes));
    memcpy(&io->bytes[io->length], data, length);
    io->length += length;
    return true;
}

static void TestDelay(void *user_data, uint32_t milliseconds)
{
    TestIO *io = (TestIO *)user_data;
    assert(milliseconds == 100U);
    io->delay_calls++;
}

static void Feed(SingleAxisGyro_Device *device, const uint8_t frame[5])
{
    size_t index;
    for (index = 0U; index < 5U; index++) {
        SingleAxisGyro_ReceiveByte(device, frame[index]);
    }
}

int main(void)
{
    SingleAxisGyro_Device device;
    TestIO io = {0};
    float value;
    const uint8_t angle_frame[5] = {0x5A, 0xBB, 0x00, 0x40, 0x55};
    const uint8_t speed_frame[5] = {0x5A, 0xAA, 0x00, 0xC0, 0xC4};
    const uint8_t bad_frame[5] = {0x5A, 0xBB, 0x00, 0x20, 0x00};
    const uint8_t expected_commands[15] = {
        0x55, 0xAA, 0x13, 0x8E, 0x5F,
        0x55, 0xAA, 0x02, 0x08, 0x00,
        0x55, 0xAA, 0x00, 0x00, 0x00
    };

    SingleAxisGyro_Init(&device, TestWrite, TestDelay, &io);
    assert(!SingleAxisGyro_GetAngle(&device, &value));

    SingleAxisGyro_ReceiveByte(&device, 0x00);
    Feed(&device, angle_frame);
    assert(SingleAxisGyro_GetAngle(&device, &value));
    assert(fabsf(value - 90.0f) < 0.001f);

    Feed(&device, speed_frame);
    assert(SingleAxisGyro_GetAngularVelocity(&device, &value));
    assert(fabsf(value + 1000.0f) < 0.001f);

    Feed(&device, bad_frame);
    assert(device.good_frame_count == 2U);
    assert(device.checksum_error_count == 1U);

    assert(SingleAxisGyro_SetOutputRate(&device, SINGLE_AXIS_GYRO_RATE_50_HZ));
    assert(io.delay_calls == 3U);
    assert(io.length == sizeof(expected_commands));
    assert(memcmp(io.bytes, expected_commands, sizeof(expected_commands)) == 0);

    return 0;
}

#endif /* SINGLE_AXIS_GYRO_DRIVER_TEST */


