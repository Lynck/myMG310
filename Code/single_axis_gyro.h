#ifndef SINGLE_AXIS_GYRO_H
#define SINGLE_AXIS_GYRO_H

/*
 * 单轴陀螺仪 UART 驱动
 *
 * 本驱动不绑定具体 MCU、UART 外设或中断函数。移植时只需提供串口发送和毫秒
 * 延时回调：
 *   1. 调用 SingleAxisGyro_Init() 初始化对象；
 *   2. UART 每收到一个字节，就调用 SingleAxisGyro_FeedByte()；
 *   3. 主循环通过 angular_velocity_dps、yaw_deg 和更新计数读取最新数据。
 *
 * 模块输出的是 5 字节二进制数据，不是 ASCII 字符串。
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SINGLE_AXIS_GYRO_FRAME_SIZE 5U

/*
 * 串口发送回调必须在返回前完成数据发送或将数据复制到自己的发送缓冲区，
 * 不能在回调返回后继续引用 data 指针。
 */
typedef void (*SingleAxisGyro_WriteFn)(void *user,
                                       const uint8_t *data,
                                       size_t length);

/* 配置寄存器时使用的毫秒延时回调；如果只接收数据，可以传入 NULL。 */
typedef void (*SingleAxisGyro_DelayMsFn)(void *user, uint32_t delay_ms);

/* 每喂入一个字节后返回的解析结果。 */
typedef enum {
    SINGLE_AXIS_GYRO_FEED_NONE = 0,          /* 尚未收到完整帧 */
    SINGLE_AXIS_GYRO_FEED_ANGULAR_VELOCITY, /* 已更新 Z 轴角速度 */
    SINGLE_AXIS_GYRO_FEED_YAW,              /* 已更新航向角 */
    SINGLE_AXIS_GYRO_FEED_STATUS,           /* 已更新标定状态 */
    SINGLE_AXIS_GYRO_FEED_UNKNOWN_TYPE,     /* 校验正确，但数据类型未知 */
    SINGLE_AXIS_GYRO_FEED_CHECKSUM_ERROR    /* 数据帧校验失败 */
} SingleAxisGyro_FeedResult;

/*
 * 数据手册对上电默认波特率的描述冲突：参数表写 115200，BAUD 寄存器章节的
 * 默认值 0x0002 则代表 9600。首次连接时需要根据实际模块确认。
 */
typedef enum {
    SINGLE_AXIS_GYRO_BAUD_2400 = 0x0000,
    SINGLE_AXIS_GYRO_BAUD_4800 = 0x0001,
    SINGLE_AXIS_GYRO_BAUD_9600 = 0x0002,
    SINGLE_AXIS_GYRO_BAUD_19200 = 0x0003,
    SINGLE_AXIS_GYRO_BAUD_38400 = 0x0004,
    SINGLE_AXIS_GYRO_BAUD_57600 = 0x0005,
    SINGLE_AXIS_GYRO_BAUD_115200 = 0x0006
} SingleAxisGyro_Baud;

/* 模块输出频率对应的 RRATE 寄存器值。 */
typedef enum {
    SINGLE_AXIS_GYRO_RATE_0_1_HZ = 0x0000,
    SINGLE_AXIS_GYRO_RATE_0_2_HZ = 0x0001,
    SINGLE_AXIS_GYRO_RATE_0_5_HZ = 0x0002,
    SINGLE_AXIS_GYRO_RATE_1_HZ = 0x0003,
    SINGLE_AXIS_GYRO_RATE_2_HZ = 0x0004,
    SINGLE_AXIS_GYRO_RATE_5_HZ = 0x0005,
    SINGLE_AXIS_GYRO_RATE_10_HZ = 0x0006,
    SINGLE_AXIS_GYRO_RATE_20_HZ = 0x0007,
    SINGLE_AXIS_GYRO_RATE_50_HZ = 0x0008,
    SINGLE_AXIS_GYRO_RATE_100_HZ = 0x0009,
    SINGLE_AXIS_GYRO_RATE_125_HZ = 0x000A,
    SINGLE_AXIS_GYRO_RATE_200_HZ = 0x000B,
    SINGLE_AXIS_GYRO_RATE_250_HZ = 0x000C,
    SINGLE_AXIS_GYRO_RATE_500_HZ = 0x000D,
    SINGLE_AXIS_GYRO_RATE_1000_HZ = 0x000E
} SingleAxisGyro_OutputRate;

typedef struct {
    /* 平台适配回调及用户上下文指针。 */
    SingleAxisGyro_WriteFn write;
    SingleAxisGyro_DelayMsFn delay_ms;
    void *user;

    /* 接收解析状态，由驱动内部维护。 */
    uint8_t frame[SINGLE_AXIS_GYRO_FRAME_SIZE];
    uint8_t frame_index;

    /*
     * 最新接收数据。raw 是模块原始有符号 16 位值；dps 和 deg 是换算后的
     * 物理量。volatile 用于 UART 中断写入、主循环读取的常见用法。
     */
    volatile int16_t angular_velocity_raw;
    volatile int16_t yaw_raw;
    volatile int16_t status_raw;
    volatile float angular_velocity_dps;
    volatile float yaw_deg;

    /* 每成功解析一帧，对应计数加一；可用于判断是否收到新数据。 */
    volatile uint32_t angular_velocity_updates;
    volatile uint32_t yaw_updates;
    volatile uint32_t status_updates;
} SingleAxisGyro;

/* 初始化驱动对象。user 会原样传给 write 和 delay_ms 回调。 */
void SingleAxisGyro_Init(SingleAxisGyro *gyro,
                         SingleAxisGyro_WriteFn write,
                         SingleAxisGyro_DelayMsFn delay_ms,
                         void *user);

/* 丢弃尚未接收完整的数据帧，不清除已经解析出的测量值。 */
void SingleAxisGyro_ResetParser(SingleAxisGyro *gyro);

/* UART 接收中断中每收到一个字节调用一次。 */
SingleAxisGyro_FeedResult SingleAxisGyro_FeedByte(SingleAxisGyro *gyro,
                                                   uint8_t byte);

/* 一次输入一段 UART 数据，返回这段数据中最后一个非 NONE 的解析结果。 */
SingleAxisGyro_FeedResult SingleAxisGyro_FeedBuffer(SingleAxisGyro *gyro,
                                                     const uint8_t *data,
                                                     size_t length);

/* 发送原始寄存器写命令：55 AA ADDR DATAL DATAH，不会自动解锁或保存。 */
bool SingleAxisGyro_WriteRegister(SingleAxisGyro *gyro,
                                  uint8_t address,
                                  uint16_t value);

/* 基础控制命令。需要延时的组合操作会自动按手册插入 100 ms 延时。 */
bool SingleAxisGyro_Unlock(SingleAxisGyro *gyro);
bool SingleAxisGyro_Save(SingleAxisGyro *gyro);
bool SingleAxisGyro_Reboot(SingleAxisGyro *gyro);
bool SingleAxisGyro_FactoryReset(SingleAxisGyro *gyro);
bool SingleAxisGyro_ZeroYaw(SingleAxisGyro *gyro);

/* 自动执行：解锁 -> 设置输出频率 -> 保存。 */
bool SingleAxisGyro_SetOutputRate(SingleAxisGyro *gyro,
                                  SingleAxisGyro_OutputRate rate);

/*
 * 本函数只修改模块端波特率。调用后必须先把 MCU UART 切换到新波特率，再调用
 * SingleAxisGyro_Save() 保存。手册将 115200 和 230400 都写成 0x0006，无法
 * 确认 230400 的真实寄存器值，因此本驱动不提供 230400 枚举。
 */
bool SingleAxisGyro_BeginBaudChange(SingleAxisGyro *gyro,
                                    SingleAxisGyro_Baud baud);

/*
 * 零偏标定：启动后保持模块静止至少 20 秒，再调用 RequestBiasStatus 查询结果。
 * 收到状态帧后，status_updates 会增加；手册规定 status_raw == 0 表示成功。
 */
bool SingleAxisGyro_StartBiasCalibration(SingleAxisGyro *gyro);
bool SingleAxisGyro_RequestBiasStatus(SingleAxisGyro *gyro);

/* 标度标定：启动后将模块准确旋转 360 度，再调用 Finish 保存并结束标定。 */
bool SingleAxisGyro_StartScaleCalibration(SingleAxisGyro *gyro);
bool SingleAxisGyro_FinishScaleCalibration(SingleAxisGyro *gyro);

#ifdef __cplusplus
}
#endif

#endif
