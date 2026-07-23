#ifndef SINGLE_AXIS_GYRO_H
#define SINGLE_AXIS_GYRO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * 单轴陀螺仪独立驱动
 *
 * 本文件只依赖 C 标准类型，不依赖 MSPM0、DriverLib、SysConfig 或操作系统。
 * 移植时只需要实现串口发送和毫秒延时两个回调；只读数据时两个回调都可传 NULL。
 *
 * MSPM0 + SysConfig 配置流程：
 * 1. 新增一个未被占用的 UART，建议命名 UART_GYRO，配置为 8 数据位、无校验、
 *    1 停止位、开启 RX 中断。手册首页写默认 115200bps，但寄存器页又写默认
 *    9600bps；建议先试 115200bps，无数据时再试 9600bps。
 * 2. 接线：模块 TX -> MCU UART_RX，模块 RX -> MCU UART_TX，GND 必须共地；
 *    VCC 可接 3.3V~16V，手册推荐 5V。连接前还应确认双方 TTL 电平兼容。
 * 3. 应用文件包含 "Drivers/SingleAxisGyro/single_axis_gyro.h"。调用 SYSCFG_DL_init() 后，
 *    必须先调用 SingleAxisGyro_Init() 并配置陀螺仪 UART RX 中断，然后才能初始化
 *    或使能其他 UART。
 * 4. 在 UART 中断中读空 RX FIFO，并对每个字节调用 SingleAxisGyro_ReceiveByte()。
 * 5. 主循环调用 SingleAxisGyro_GetAngle() 和 SingleAxisGyro_GetAngularVelocity() 读取最新值。
 *
 * 【重要：初始化顺序（本项目实物验证）】
 * 陀螺仪串口的软件初始化必须放在所有其他串口初始化函数之前。推荐顺序：
 *
 *   SYSCFG_DL_init();
 *   SingleAxisGyro_Init(&gyro, write, delay_ms, user_data);
 *   // 在这里清除/使能 UART_GYRO 的 RX 中断
 *   // 然后才初始化蓝牙等其他 UART 功能
 *
 * 如果顺序颠倒，可能出现“引脚和波特率配置正确、串口助手能看到模块数据，
 * 但 MCU 中的角度数据始终不更新，OLED 一直显示无有效数据”的现象。
 *
 * 典型的 MSPM0 接收中断写法（UART 名称以实际 SysConfig 生成结果为准）：
 *
 *   static SingleAxisGyro_Device gyro;
 *
 *   static bool GyroWrite(void *user, const uint8_t *data, size_t length)
 *   {
 *       size_t i;
 *       (void)user;
 *       for (i = 0; i < length; ++i) {
 *           DL_UART_transmitDataBlocking(UART_GYRO_INST, data[i]);
 *       }
 *       return true;
 *   }
 *
 *   static void GyroDelayMs(void *user, uint32_t milliseconds)
 *   {
 *       (void)user;
 *       delay_cycles((CPUCLK_FREQ / 1000U) * milliseconds);
 *   }
 *
 *   // 必须放在 SYSCFG_DL_init() 之后。
 *   SingleAxisGyro_Init(&gyro, GyroWrite, GyroDelayMs, NULL);
 *   NVIC_EnableIRQ(UART_GYRO_INST_INT_IRQN);
 *
 *   void UART_GYRO_INST_IRQHandler(void)
 *   {
 *       if (DL_UART_getPendingInterrupt(UART_GYRO_INST) == DL_UART_IIDX_RX) {
 *           while (!DL_UART_isRXFIFOEmpty(UART_GYRO_INST)) {
 *               SingleAxisGyro_ReceiveByte(&gyro,
 *                   (uint8_t)DL_UART_receiveData(UART_GYRO_INST));
 *           }
 *       }
 *   }
 *
 * 注意：手册只支持读取角度/角速度，并支持将当前 Z 轴角度归零；没有“写入任意
 * 角度”或“写入角速度”的命令。
 */

#define SINGLE_AXIS_GYRO_FRAME_SIZE (5U)

typedef bool (*SingleAxisGyro_WriteFn)(
    void *user_data, const uint8_t *data, size_t length);
typedef void (*SingleAxisGyro_DelayMsFn)(void *user_data, uint32_t milliseconds);

typedef enum {
    SINGLE_AXIS_GYRO_RATE_0_1_HZ = 0x00,
    SINGLE_AXIS_GYRO_RATE_0_2_HZ = 0x01,
    SINGLE_AXIS_GYRO_RATE_0_5_HZ = 0x02,
    SINGLE_AXIS_GYRO_RATE_1_HZ   = 0x03,
    SINGLE_AXIS_GYRO_RATE_2_HZ   = 0x04,
    SINGLE_AXIS_GYRO_RATE_5_HZ   = 0x05,
    SINGLE_AXIS_GYRO_RATE_10_HZ  = 0x06,
    SINGLE_AXIS_GYRO_RATE_20_HZ  = 0x07,
    SINGLE_AXIS_GYRO_RATE_50_HZ  = 0x08,
    SINGLE_AXIS_GYRO_RATE_100_HZ = 0x09,
    SINGLE_AXIS_GYRO_RATE_125_HZ = 0x0A,
    SINGLE_AXIS_GYRO_RATE_200_HZ = 0x0B,
    SINGLE_AXIS_GYRO_RATE_250_HZ = 0x0C,
    SINGLE_AXIS_GYRO_RATE_500_HZ = 0x0D,
    SINGLE_AXIS_GYRO_RATE_1000_HZ = 0x0E
} SingleAxisGyro_OutputRate;

typedef struct {
    SingleAxisGyro_WriteFn write;
    SingleAxisGyro_DelayMsFn delay_ms;
    void *user_data;

    uint8_t rx_frame[SINGLE_AXIS_GYRO_FRAME_SIZE];
    uint8_t rx_index;

    volatile int16_t angle_raw;
    volatile int16_t angular_velocity_raw;
    volatile uint8_t valid_mask;
    volatile uint32_t good_frame_count;
    volatile uint32_t checksum_error_count;
} SingleAxisGyro_Device;

/* 初始化解析状态和平台回调。只接收数据时 write、delay_ms 可以为 NULL。 */
void SingleAxisGyro_Init(SingleAxisGyro_Device *device, SingleAxisGyro_WriteFn write,
    SingleAxisGyro_DelayMsFn delay_ms, void *user_data);

/*
 * 输入一个 UART 接收字节。函数内部自动找帧头、校验并处理粘包/错位；
 * 适合直接在 UART RX 中断中调用，不会阻塞。
 */
void SingleAxisGyro_ReceiveByte(SingleAxisGyro_Device *device, uint8_t byte);

/* 读取最近一次校验正确的航向角，单位为度，范围约为 [-180, 180)。 */
bool SingleAxisGyro_GetAngle(const SingleAxisGyro_Device *device, float *angle_deg);

/*
 * 读取最近一次校验正确的 Z 轴角速度，单位为度/秒。
 * 手册参数页写量程 ±400°/s，但协议页明确给出 raw/32768*2000；驱动按协议公式换算。
 */
bool SingleAxisGyro_GetAngularVelocity(
    const SingleAxisGyro_Device *device, float *angular_velocity_dps);

/* 按手册通用写格式发送：55 AA ADDR DATAL DATAH（低字节在前）。 */
bool SingleAxisGyro_WriteRegister(
    SingleAxisGyro_Device *device, uint8_t address, uint16_t value);

/* 解锁 -> 延时 -> 设置输出速率 -> 延时 -> 保存 -> 延时。 */
bool SingleAxisGyro_SetOutputRate(SingleAxisGyro_Device *device, SingleAxisGyro_OutputRate rate);

/* 解锁 -> 延时 -> 将当前 Z 轴角度设为 0 -> 延时 -> 保存 -> 延时。 */
bool SingleAxisGyro_ZeroAngle(SingleAxisGyro_Device *device);

#ifdef __cplusplus
}
#endif

#endif /* SINGLE_AXIS_GYRO_H */
