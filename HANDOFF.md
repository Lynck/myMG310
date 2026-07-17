# myMG310slave Handoff

Updated: 2026-07-17

## Project

- Follower workspace: `C:\Users\Lenovo\workspace_ccstheia\myMG310slave`
- Leader workspace: `C:\Users\Lenovo\workspace_ccstheia\myMG310`
- Remote: `https://github.com/Lynck/myMG310.git`
- Branch: `codex/follower-car-backup`
- Target: MSPM0G3507, CCS Theia, SysConfig, TI Arm Clang 4.0.4 LTS

`mspm0-modules.syscfg` is the peripheral and pin source of truth. Do not edit generated files under `Debug/`.

## Current UART Hardware

All three UARTs use 115200 baud and RX FIFO interrupts.

- CH9141K Bluetooth: generated `UART_0`, hardware UART2
  - TX PA21, RX PA22
- MCU-based 8-channel grayscale module: generated `UART_GRAY`, hardware UART3
  - TX PB2, RX PB3
  - Module TX connects to PB3; share GND.
- MaixCAM distance input: generated `UART_DISTANCE`, hardware UART0
  - TX PA10, RX PA11
  - MaixCAM TX connects to PA11; share GND.

PA10/PA11 previously belonged to the VL53L0X I2C interface. That I2C instance, runtime initialization, include path, and active VL53L0X calls have been removed so UART0 can own these pins.

CH9141K follower module:

- Slave mode: `AT+BLEMODE=2`
- Slave address: `AF:39:63:E4:C2:84`
- Leader pairing command: `AT+CONADD=AF:39:63:E4:C2:84,000000`
- Exit AT mode: `AT+EXIT`

## Distance Reception

Reusable files:

- `Code/leader_distance.c`
- `Code/leader_distance.h`

Expected ASCII frame: `Z47.70\r\n`. Parsing runs from the UART0 RX interrupt plus `LeaderDistance_Process()` every 10 ms. A valid value is exposed in centimetres and displayed on OLED row 6. Data becomes invalid after 500 ms without a complete frame; OLED then displays `DIST:----cm`.

Current conversion and validation:

- Scale: `1.0 cm` per received unit
- Accepted range: 1 to 500 cm
- Invalid distance during normal following: use nominal base speed
- Invalid distance while processing a leader stop request: brake and wait; do not move blindly

## Distance Controller

Reusable files:

- `Code/distance_pid.c`
- `Code/distance_pid.h`

Current parameters in `DistancePID_Init()`:

```c
Kp = 0.4f;
Kd = 20.0f;
TargetCm = 20.0f;
DeadbandCm = 0.0f;
OutMin = -37.0f;
OutMax = 8.0f;
FastFilterAlpha = 0.4f;
SlowFilterAlpha = 0.25f;
```

The output adjusts the line-following base speed. Current nominal task-1 speed is 27; task 2 changes it to 37. The first valid sample can still produce a derivative kick, but `OutMax=8` limits the forward surge. `Kd=20` is the current tested source value and should be rechecked on hardware if following oscillates.

OLED row 4 shows controlled/nominal speed as `V:controlled/nominal`.

## Leader Stop and Soft Final Adjustment

Bluetooth `C:t:0` now requests a controlled final adjustment instead of immediately disabling the motors.

Constants are at the top of `Code/myTask.c`:

```c
FOLLOW_SUCCESS_MIN_CM = 18.0f
FOLLOW_SUCCESS_MAX_CM = 22.0f
FOLLOW_ADJUST_MIN_SPEED = 12
FOLLOW_ADJUST_MAX_SPEED = 18
FOLLOW_ADJUST_SPEED_GAIN = 0.7f
FOLLOW_SETTLE_TIME_MS = 250
```

Behaviour after the leader stop request:

- Distance greater than 22 cm: approach at speed 12 to 18 while continuing grayscale line following.
- Distance less than 18 cm: reverse straight at speed 12 to 18.
- Distance from 18 to 22 cm: brake for 250 ms, then measure again.
- Still within 18 to 22 cm after settling: success; keep braking, disable line following, and sound the buzzer for 100 ms.
- Outside the range after settling: resume forward or reverse adjustment.
- Missing distance data: remain braked until valid data returns.

Repeated Bluetooth stop packets do not reset the 250 ms settling period. Bluetooth synchronization timeout remains an immediate safety brake rather than entering distance adjustment.

## Grayscale and Line Following

UART grayscale driver:

- `Code/grayscale_uart.c`
- `Code/grayscale_uart.h`
- Binary frame used by tracking: `AA 81 xx`
- bit7 is the leftmost sensor and bit0 is the rightmost sensor
- normalized mask uses `1 = black`
- OLED row 2 shows left-to-right black/white state

Current line PID defaults:

- Task 1: Kp 6, Kd 0
- Task 2 override: Kp 12, Kd 30
- Weights correspond to left-to-right `+4,+3,+2,+1,-1,-2,-3,-4` after bit mapping.

Lane/branch handling is implemented in `Code/myPID.c`:

- When two or more sensors are black, outer-lane mode ignores left sensors 1 to 3.
- Inner-lane mode ignores right sensors 6 to 8.
- Both current task-entry paths select outer-lane mode. Call `Tracking_SetLane(TRACKING_PID_LANE_INNER)` where the course logic determines the inner lane.

## Bluetooth Commands

Commands are ASCII and require CR or LF:

- `C:t:r`: synchronize task and run state
- `P:value`: line Kp
- `D:value`: line Kd
- `B:value`: line deadband
- `S:value`: nominal base speed
- `L:value`, `R:value`: wheel scale
- `E:value`: encoder speed-match gain

The parser does not send acknowledgements. Task initialization overwrites line PID gains, so send tuning commands after starting the task.

## Validation

Latest complete clean build passed on 2026-07-17:

```powershell
cd C:\Users\Lenovo\workspace_ccstheia\myMG310slave\Debug
D:\TI\ccs\utils\bin\gmake.exe clean
D:\TI\ccs\utils\bin\gmake.exe -j4 all
```

Result: `Debug\myMG310slave.out`, zero compiler/linker errors. SysConfig emitted informational retention and ADC/flash notes only. No hardware flashing or track validation was performed by Codex.

## Suggested Skills

- `mspm0-ccs`: SysConfig, DriverLib, CCS builds, flashing, and pin ownership.
- `diagnosing-bugs`: intermittent UART, distance latency, or physical stop behaviour.
- `ponytail`: keep tuning fixes small and preserve the reusable driver boundaries.
- `handoff`: refresh this document after the next hardware-tested change.

## Recommended Next Test

Run the follower behind a stationary leader and record the distance when adjustment first brakes and the final settled distance. If the car oscillates around 18/22 cm, tune only `FOLLOW_ADJUST_MIN_SPEED`, `FOLLOW_ADJUST_MAX_SPEED`, and `FOLLOW_SETTLE_TIME_MS` first. Preserve the wide safety behaviour for missing distance data.
