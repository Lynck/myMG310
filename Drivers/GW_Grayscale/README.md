# 感为八路灰度模块驱动（MSPM0）

本目录只包含传感器协议和 MSPM0 通信适配，不依赖领头车的 PID、OLED、任务状态或具体引脚名，复制整个目录即可移植。

## 支持的通信模式

| 模式 | 可读取内容 | 接线 | 备注 |
|---|---|---|---|
| CLK/DAT（例程名 Serial） | 8 位黑白量 | VCC、GND、CLK、DAT | 不是 UART；速度快、占 2 个 GPIO，当前领头车默认使用 |
| 硬件 I2C | 数字量、8 路模拟量及寄存器 | VCC、GND、SCL、SDA | 需要在 SysConfig 新建 I2C Controller |
| 软件 I2C | 数字量、8 路模拟量及寄存器 | VCC、GND、SCL、SDA | 任意 2 个 GPIO，SCL/SDA 均需外接上拉 |

I2C 地址在本驱动中始终填写 **7 位地址 `0x4C`**，调用者不要左移。软件 I2C 适配层会在发送地址时自行左移。

## 当前领头车的 CLK/DAT 用法

领头车应用层使用 SysConfig 中现有的 `SCL`、`SDA` 两个 GPIO 实例：

- 模块 `CLK` -> `SCL` 实例的 `SCK_PIN`（输出，初始高电平）
- 模块 `DAT` -> `SDA` 实例的 `SDA_PIN`（输入，无上下拉）
- 模块 `GND` -> 小车 GND
- 模块 `VCC` -> 按模块标注供电
- 原来的 `PL` 实例不再使用，可以稍后在 SysConfig 删除

在 SysConfig 中只需要重新选择 `SCL/SCK_PIN` 和 `SDA/SDA_PIN` 的物理引脚，不要改实例名和引脚名。代码调用 `Grayscale_Sensor_Read()`；返回值规定为 bit7=最左侧1号、bit0=最右侧8号，`g_grayscale_digital[0..7]` 则直接对应从左到右的1号到8号。

循迹层默认 `0=黑线`。上电后可先看 OLED：正常白底应显示 `11111111`，某个探头压到黑线时对应位变成 `0`。如果实测电平相反，把 `Code/Grayscale_Sensor.h` 中的 `GRAYSCALE_BLACK_LEVEL` 改为 `1U`。

## 独立调用 CLK/DAT

```c
static void delay_us_cb(void *ctx, uint32_t us)
{
    (void)ctx;
    delay_cycles((CPUCLK_FREQ / 1000000U) * us);
}

GW_GraySerial serial;
GW_Gray_Serial_Init(&serial, GPIOB, DL_GPIO_PIN_2,
                    GPIOA, DL_GPIO_PIN_31,
                    delay_us_cb, NULL);

uint8_t raw = GW_Gray_Serial_Read(&serial);
```

例程协议先返回 bit0，最终 `raw` 的 bit0...bit7 对应模块输出的第 1...8 路。模块安装方向相反时可调用 `GW_Gray_ReverseBits(raw)`；黑白电平反相时可调用 `GW_Gray_TransformDigital(raw, true, false)`。驱动本身不擅自规定 1 是黑还是白。

## 硬件 I2C 用法

先在 SysConfig 中建立一个 I2C Controller（建议 100 kHz），为 SCL/SDA 配置上拉或在板外各接约 4.7 kΩ 上拉到模块逻辑电压。领头车当前没有空闲硬件 I2C 引脚配置，因此本次未擅自分配引脚。

```c
GW_GrayHardwareI2C hw;
GW_GrayDevice sensor;
uint8_t digital;
uint8_t analog[8];

GW_Gray_HardwareI2C_Init(&hw, I2C0, 100000U);
GW_Gray_Init(&sensor, &GW_Gray_HardwareI2C_Bus,
             &hw, GW_GRAY_DEFAULT_ADDRESS);

if (GW_Gray_Ping(&sensor)) {
    GW_Gray_ReadDigital(&sensor, &digital);
    GW_Gray_ReadAnalog(&sensor, analog);
}
```

把示例中的 `I2C0` 换成 SysConfig 生成的实例宏。驱动带有限等待，模块没接好时不会像原例程一样永久卡在 while 循环。

## 软件 I2C 用法

SCL 和 SDA 必须各有一个外部上拉电阻（常用 4.7 kΩ），两个 GPIO 在 SysConfig 中配置为普通数字 GPIO。软件 I2C 通过“输出低 / 释放为输入”模拟开漏，不能和 OLED 或 CLK/DAT 同时占用同一对引脚。

```c
GW_GraySoftwareI2C sw;
GW_GrayDevice sensor;

GW_Gray_SoftwareI2C_Init(&sw,
    GPIOB, DL_GPIO_PIN_2,       /* SCL */
    GPIOA, DL_GPIO_PIN_31,      /* SDA */
    delay_us_cb, NULL, 5U);     /* 约 100 kHz */
GW_Gray_Init(&sensor, &GW_Gray_SoftwareI2C_Bus,
             &sw, GW_GRAY_DEFAULT_ADDRESS);
```

之后 `GW_Gray_Ping`、`GW_Gray_ReadDigital`、`GW_Gray_ReadAnalog` 的调用与硬件 I2C 完全相同。

## 归一化模拟量

```c
static void delay_ms_cb(void *ctx, uint32_t ms)
{
    (void)ctx;
    delay_cycles((CPUCLK_FREQ / 1000U) * ms);
}

uint8_t normalized[8];
GW_Gray_SetDelay(&sensor, delay_ms_cb, NULL);
GW_Gray_ReadNormalized(&sensor, 0xFFU, normalized);
```

归一化寄存器要求模块固件 v3.6 或更高。`GW_Gray_ReadFirmware()` 可读取固件版本原始字节。

## 移植清单

1. 复制 `Drivers/GW_Grayscale`。
2. 把该目录加入编译器 include path，并编译两个 `.c` 文件。
3. 配置所选模式的 GPIO 或 I2C 外设。
4. 提供微秒延时回调；使用归一化时再提供毫秒延时回调。
5. 在白底、黑线实测一次位序和极性，再决定是否反转/取反。

`No_Mcu_Ganv_Grayscale_Sensor.*` 是另一款无 MCU、通过模拟多路复用器和 ADC 采样的模块，不是本模块的第四种通信模式，因此没有混入此驱动。
