# myMG310 工程交接摘要

## 工程目标

当前工程 `C:\Users\Lenovo\workspace_ccstheia\myMG310` 是 TI MSPM0G3507 + MG310 电机的小车基础框架。目标不是完整比赛任务，而是保留后续任务可复用的底盘能力：

- 黑线循迹函数
- PID
- TB6612 电机驱动
- MG310 编码器计数/转速
- IMU660RB 陀螺仪 yaw
- OLED 调试显示
- 蓝牙调参/启停循迹

## 已删除/清理

- 已删除旧 H 题导航任务状态机：
  - `Code/car_nav.c`
  - `Code/car_nav.h`
- `.cproject` 中也移除了 `Code/car_nav.c` 的排除残留。
- 搜索确认已无 `car_nav`、`TASK_`、`NAV_`、`T:2/T:3/T:4` 等旧任务入口。

## 当前保留的主要模块

- `main.c`
  - 初始化 SysConfig、SysTick、OLED、IMU660RB、电机、编码器、循迹 PID。
  - 10ms 周期中读取 IMU、更新编码器速度、处理蓝牙命令、在使能后调用 `Tracking_Process()`。
  - ADC 中间按键用于切换循迹启停。
  - 100ms 定时器用于 OLED 刷新和丢线确认。

- `Code/myPID.c`
  - `Tracking_PID_Init()`
  - `Tracking_PID_Reset()`
  - `Tracking_Process()`
  - 当前循迹使用单 IO 灰度读取 `Read_data_1_GPIO()`。
  - 传感器原始值含义：`0 = 黑线`，`1 = 白底/没压线`。
  - 内部取反后用 `1 = 检测到黑线` 做加权平均。

- `Code/motor.c`
  - `Motor_Init()`
  - `Motor_SetSpeed_A(int16_t speed)`
  - `Motor_SetSpeed_B(int16_t speed)`
  - `Motor_Brake()`
  - speed 范围为 `-100..100`。

- `Code/encoder.c`
  - `Encoder_Init()`
  - `Encoder_GetDeltas()`
  - `Encoder_UpdateSpeeds()`
  - `enc_speed_A / enc_speed_B` 已在 `main.c` 的 10ms 周期更新。

- `Code/myOLED.c`
  - OLED 显示：
    - `LINE:RUN/STOP`
    - `Yaw`
    - `VA/VB` 编码器速度
    - `L:xxxxxxxx:R` 灰度数据
    - `Spd/PID`
  - 当前 OLED 每次刷新前会主动调用 `Read_data_1_GPIO()`，所以即使循迹没启动，也应该能看到灰度数据变化。

- `Code/myBluetooth.c`
  - `G` 启动循迹
  - `T:0` 停止循迹
  - `T:非0` 启动循迹
  - `P/D/B/S/L/R/E` 用于 PID、速度、补偿、编码器参数调节

## 编码状态

已将当前参与编译的主要应用源码转为严格 UTF-8：

- `main.c`
- `main.h`
- `Code/myPID.c`
- `Code/myPID.h`
- `Code/encoder.h`
- `Drivers/MSPM0/interrupt.c`
- 以及当前主要编译源码检查通过

注意：`.cproject` 已排除的第三方 `Drivers/VL53L0X` 里仍可能有非 UTF-8 文件，之前没有改，避免无关大改。

## 当前问题：OLED 灰度显示全 0

用户现象：

- OLED 显示的灰度数据全是 `0`
- 变化黑线/白底时不改变
- 用户感觉“灰度传感器数据没有传过来”

当前 OLED 显示格式：

```text
L:xxxxxxxx:R
```

8 位顺序为从左到右：

```text
D8 D7 D6 D5 D4 D3 D2 D1
```

当前代码调用的是：

```c
Read_data_1_GPIO();
```

也就是单 IO 串行模式，使用 `PL/SCK/SDA`，不是 8 路并行 OUT0~OUT7 模式。

## 最重要怀疑点

`Code/Grayscale_Sensor.h` 中写的是：

```c
#define PL_PORT  (GPIOB)
#define PL_PIN   (DL_GPIO_PIN_3)

#define SCK_PORT (GPIOB)
#define SCK_PIN  (DL_GPIO_PIN_2)

#define SDA_PORT (GPIOA)
#define SDA_PIN  (DL_GPIO_PIN_17)
```

但是 `mspm0-modules.syscfg` 和 `Debug/ti_msp_dl_config.h` 显示 SysConfig 里灰度 `SDA` 配的是 `PA22`：

```text
GPIO10.$name                          = "SDA"
GPIO10.associatedPins[0].direction    = "INPUT"
GPIO10.associatedPins[0].assignedPort = "PORTA"
GPIO10.associatedPins[0].assignedPin  = "22"
GPIO10.associatedPins[0].$name        = "SDA_PIN"
GPIO10.associatedPins[0].pin.$suggestSolution = "PA22"
```

生成头文件里对应：

```c
#define SDA_PORT        (GPIOA)
#define SDA_SDA_PIN_PIN (DL_GPIO_PIN_22)
```

因此最大问题可能是：

- 代码实际读 `PA17`
- SysConfig 初始化的是 `PA22`
- 实际硬件可能接的是 `PA22`

这会导致代码一直读错脚，OLED 全 0。

## 另一个可能原因

灰度模块可能接的是 8 路独立输出 `OUT0~OUT7`，但当前代码使用的是单 IO 串行接口。

如果硬件接法是 8 路 OUT 模式，就应该使用：

```c
Read_data_8_GPIO();
data_8.D8 ... data_8.D1
```

而不是：

```c
Read_data_1_GPIO();
data_1.D8 ... data_1.D1
```

## 建议新对话优先确认

先问/查用户实际接线：

1. 灰度模块是否是单 IO 串行模式？
   - 接线应为 `PL / SCK / SDA`
2. 还是 8 路并行模式？
   - 接线应为 `OUT0~OUT7`
3. 如果是单 IO 模式，SDA 实际接在 MSPM0 哪个脚？
   - `PA17` 还是 `PA22`

如果实际接的是 `PA22`，应改 `Code/Grayscale_Sensor.h`：

```c
#define SDA_PIN (DL_GPIO_PIN_22)
```

如果实际接的是 `PA17`，应改 `.syscfg` 中 SDA 到 PA17，并重新生成 SysConfig。

如果实际接的是 8 路 OUT 模式，应改 OLED 和循迹读取链路使用 `Read_data_8_GPIO()` / `data_8`。

## 最近编译命令

在 `Debug` 目录执行：

```powershell
D:\TI\ccs\utils\bin\gmake.exe -k -j 8 all -r -O
```

最近一次编译通过，生成：

```text
Debug/myMG310.out
```

## 注意事项

- 不要手改 `Debug/ti_msp_dl_config.c/h`，它们是 SysConfig 生成文件。
- 如果要改引脚配置，优先改 `mspm0-modules.syscfg`，然后重新生成/编译。
- 用户当前明确要求过“不要修改代码”来分析原因；下一步若要改，需要先明确实际接线。
