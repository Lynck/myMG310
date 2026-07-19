# 八路灰度循迹 PID 使用说明

## 文件与职责

循迹代码分成算法模块和硬件适配层：

- `Code/myPID.c`、`Code/myPID.h`：纯算法，只依赖标准整数和布尔类型。
- `Code/myTask.c`、`Code/myTask.h`：读取灰度传感器、设置电机、处理丢线、任务和蜂鸣器。

把算法移植到其他工程时，只需复制 `myPID.c/.h`。新工程负责把八路黑白数据组成 `black_mask`，再将算法输出交给自己的电机驱动。

## 传感器约定

算法规定：

- `black_mask` 的 1 表示检测到黑线。
- bit7 是最左侧 `L0`。
- bit0 是最右侧 `L7`。

领头车灰度模块为低电平表示黑线，因此适配层先取反：

```text
L0 = !data_1.D8 -> bit7
L1 = !data_1.D7 -> bit6
L2 = !data_1.D6 -> bit5
L3 = !data_1.D5 -> bit4
L4 = !data_1.D4 -> bit3
L5 = !data_1.D3 -> bit2
L6 = !data_1.D2 -> bit1
L7 = !data_1.D1 -> bit0
```

正确的组包形式应为：

```c
black_mask = (uint8_t)((l0 << 7U) | (l1 << 6U) |
                       (l2 << 5U) | (l3 << 4U) |
                       (l4 << 3U) | (l5 << 2U) |
                       (l6 << 1U) | l7);
```

务必用 OLED 或串口逐路遮挡确认左右顺序。顺序相反会造成小车向错误方向修正。

## 算法流程

每次调用 `TrackingPID_Update()` 会依次执行：

1. 统计原始黑点数量。
2. 根据内外圈状态过滤分叉处的干扰黑线。
3. 对剩余黑点做加权平均，得到黑线位置。
4. 使用自适应低通滤波平滑位置。
5. 对中心误差应用可减式死区。
6. 计算 PD 差速修正量。
7. 根据偏离中心的程度降低弯道基础速度。
8. 输出左右轮速度命令。

控制关系为：

```text
error = -deadband(filtered_position)
correction = Kp * error + Kd * (error - last_error)
left  = curve_base_speed + correction
right = curve_base_speed - correction
```

## 位置权重

从 bit0 到 bit7 的权重为：

```c
{-4, -3, -2, -1, 1, 2, 3, 4}
```

因此：

- 黑线偏左：位置为正，算法让左轮减速、右轮加速。
- 黑线偏右：位置为负，算法让左轮加速、右轮减速。
- 黑线居中：位置接近 0，两轮速度接近一致。

## 自适应滤波

当前参数：

```c
FastFilterError = 2.0f;
FastFilterDelta = 0.8f;
FastFilterAlpha = 0.70f;
SlowFilterAlpha = 0.25f;
```

偏差较大或位置突然变化时使用较大的 `alpha`，提高进弯响应；中心附近使用较小的 `alpha`，降低传感器跳动造成的左右摆动。

## 中心死区

当前死区为：

```c
Deadband = 0.6f;
```

死区内误差为 0。超出死区后会先扣掉死区宽度，再计算 P、D 项，使纠偏从 0 平滑增加。

## 内外圈分叉过滤

只有原始黑点数量不少于 2 时才启用分叉过滤：

- 外圈：忽略最左侧 `L0-L2`，防止误向左进入内圈。
- 内圈：忽略最右侧 `L5-L7`，防止误向右留在外圈。

设置外圈：

```c
Tracking_SetLane(TRACKING_PID_LANE_OUTER);
```

设置内圈：

```c
Tracking_SetLane(TRACKING_PID_LANE_INNER);
```

如果过滤后没有剩余黑点，算法返回 `TRACKING_PID_BRANCH_IGNORED`，输出保持为左右轮基础速度，即直行通过该次采样。

按照 C 题规则：

- 任务一：外圈。
- 任务二：外圈。
- 任务三领头车：第一、二圈外圈，第三圈内圈。

当前任务三圈次自动切换尚未实现；进入第三圈时需要调用一次内圈设置函数。

## 状态返回值

`TrackingPID_Update()` 返回：

- `TRACKING_PID_LINE_FOUND`：正常得到循迹输出。
- `TRACKING_PID_LINE_LOST`：没有检测到黑线；模块保留上次位置，由适配层决定如何找线。
- `TRACKING_PID_BRANCH_IGNORED`：分叉干扰被全部过滤，本次直行。
- `TRACKING_PID_ALL_BLACK`：检测到全黑图案。

领头车的丢线搜索、终点横线和圈数判断均在 `Code/myTask.c`，不属于纯 PID 模块。

## 当前参数

领头车任务一当前本地参数为：

```c
Kp = 6.0f;
Kd = 8.0f;
Deadband = 0.6f;
OutMax = 100.0f;
OutMin = -100.0f;
CurveSlowdownGain = 2.5f;
MinCurveSpeed = 12;
```

任务一基础速度命令为 22。任务二适配层使用：

```c
Kp = 12.0f;
Kd = 30.0f;
base_speed = 37;
```

这些数值是当前工程参数，不是其他车或其他电机的通用参数。

## 蓝牙调参

领头车支持以回车或换行结尾的文本命令：

```text
P:6       设置 Kp
D:8       设置 Kd
B:0.6     设置中心死区
S:22      设置基础速度命令
L:1.00    设置左轮补偿系数
R:1.00    设置右轮补偿系数
```

建议调参顺序：

1. 先用较低基础速度。
2. 将 `Kd` 设小，逐渐增加 `Kp`，直到能跟住弯道。
3. 再逐渐增加 `Kd`，抑制左右摆动和过冲。
4. 最后调整死区、弯道降速和左右轮补偿。

## 最小移植示例

```c
#include "myPID.h"

static TrackingPID_t line_pid;

void Line_Init(void)
{
    TrackingPID_Init(&line_pid);
    line_pid.Config.Lane = TRACKING_PID_LANE_OUTER;
}

void Line_Update(uint8_t black_mask)
{
    TrackingPID_Output_t output;
    TrackingPID_Status_t status;

    status = TrackingPID_Update(&line_pid, black_mask, 22, &output);

    if (status == TRACKING_PID_LINE_LOST) {
        /* 使用 output.Position 判断上次丢线方向。 */
        return;
    }

    Motor_SetLeft(output.LeftSpeed);
    Motor_SetRight(output.RightSpeed);
}
```

## 复位与调用周期

开始新一次循迹时调用：

```c
TrackingPID_Reset(&line_pid);
```

复位只清除误差、输出和滤波状态，不覆盖已经调好的参数。领头车当前在 10 ms 主循环中调用循迹更新。不要在每个周期重新调用 `TrackingPID_Init()`，否则滤波和 D 项历史会一直被清零。

## 当前代码检查提示

领头车工作区最近把灰度变量改名为 `d0-d7`。当前 `Code/myTask.c` 的 `black_mask` 最低位疑似误写为 `d1`，应结合硬件确认是否需要改为 `d0`。否则最右侧 L7 不会进入 PID，倒数第二路会被重复使用。
