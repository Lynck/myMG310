# myMG310 Handoff

Date: 2026-07-11

Project path: `C:\Users\Lenovo\workspace_ccstheia\myMG310`

Remote: `https://github.com/Lynck/myMG310.git`

Branch: `main`

Last pushed commit:

```text
0de619f Tune motor speed control
d8cc6f3 Add motor speed GPIO capture
3ee3d53 Initial MSPM0 MG310 project
```

## Project

TI MSPM0G3507 / CCS Theia project using SysConfig.

Do not hand-edit generated files under `Debug/`, especially:

- `Debug/ti_msp_dl_config.c`
- `Debug/ti_msp_dl_config.h`
- `Debug/device_linker.cmd`
- object/map/out files

Edit `mspm0-modules.syscfg` and source files instead.

## Build And Check

Build from `Debug`:

```powershell
D:\TI\ccs\utils\bin\gmake.exe -k -j 8 all -r -O
```

Clean build:

```powershell
D:\TI\ccs\utils\bin\gmake.exe -k clean all -r -O
```

SysConfig static check:

```powershell
python C:\Users\Lenovo\.codex\skills\mspm0-ccs\scripts\check_syscfg.py C:\Users\Lenovo\workspace_ccstheia\myMG310
```

Latest validation before this handoff:

- `gmake all` passed
- `gmake clean all` passed
- `check_syscfg.py` passed
- `Debug\myMG310.out` generated

## Current Git Status

Current worktree is not clean.

Modified files:

- `Code/motor_speed.h`
- `Code/myPID.c`
- `main.c`

Untracked:

- `tmp\gyro_datasheet.pdf`

Do not assume these local changes are pushed. The last pushed commit is still `0de619f`.

## Current User Goal

The user no longer wants to use the speed closed loop for line following. They want only the line-following loop to control left/right motor differential.

Recent symptom:

- With speed loop removed, car could not turn reliably in curves.
- Then after aggressive lost-line recovery, car became unstable and kept swinging left/right.
- Latest user request was to change lost-line recovery so it only starts searching when `last_actual_pos > 2.0` or `< -2.0`.

This has already been changed in local `Code/myPID.c`:

```c
#define TRACKING_LOST_CENTER_BAND     2.0f
```

## Current Line-Following Logic

Main file: `Code/myPID.c`

Important current values:

```c
volatile int16_t g_base_speed = 22;

#define SWAP_MOTORS       0
#define REVERSE_PID_DIR   0

#define TRACKING_SEARCH_CMD           (g_base_speed)
#define TRACKING_LOST_CENTER_BAND     2.0f

tracking_pid.Kp = 6.0f;
tracking_pid.Ki = 0.0f;
tracking_pid.Kd = 20.0f;
tracking_pid.OutMax = 100.0f;
tracking_pid.OutMin = -100.0f;
tracking_pid.Deadband = 0.6f;
```

The line sensor convention in `Tracking_Process()` is:

- Raw `data_1.Dx == 0` means black line.
- Code inverts the raw values: `uint8_t d8 = !data_1.D8;`, so internal `d8..d1 == 1` means black detected.
- Weighted position:
  - `D8..D5` are positive weights: left side
  - `D4..D1` are negative weights: right side
  - `actual_pos` range is approximately `-4.0` to `4.0`

Important bug that was fixed locally:

- Old lost-line code tested `last_actual_pos > 4.0f` or `< -4.0f`.
- Because `actual_pos` cannot normally exceed that range, the car almost never entered search-turn mode and often drove straight after losing the line.
- Current threshold is `2.0f`.

The current code no longer calls `MotorSpeed_Control()` from `myPID.c`.

Current control path is:

```text
Read_data_1_GPIO()
  -> compute actual_pos
  -> PID_Update(&tracking_pid)
  -> out_val
  -> Tracking_SetMotorSpeeds(g_base_speed + out_val,
                             g_base_speed - out_val)
  -> Motor_SetSpeed_A/B()
```

`Tracking_SetMotorSpeeds()` also applies:

- `g_left_wheel_scale`
- `g_right_wheel_scale`
- `SWAP_MOTORS`

If the car corrects in the wrong direction, first try:

```c
#define REVERSE_PID_DIR   1
```

## Current Speed Capture / Speed Loop State

Speed capture module still exists and is still built.

Files:

- `Code/motor_speed.c`
- `Code/motor_speed.h`
- `Code/encoder.c`
- `Drivers/MSPM0/interrupt.c`

Encoder pins:

- Motor A capture: `PA8`
- Motor A direction level: `PB18`
- Motor B capture: `PA16`
- Motor B direction level: `PA25`

Wheel diameter:

```c
#define MOTOR_SPEED_WHEEL_CIRCUMFERENCE_M   (0.1508f) /* 48 mm wheel diameter. */
```

Current local speed-loop parameters in `Code/motor_speed.h`:

```c
#define MOTOR_SPEED_PID_KP                  250.f
#define MOTOR_SPEED_PID_KI                  0.1f
#define MOTOR_SPEED_PID_KD                  10.0f
#define MOTOR_SPEED_FEEDFORWARD_CMD_PER_MPS 77.0f
```

Note: the user is not currently using this speed loop for line following, but `main.c` still calls:

```c
MotorSpeed_Init();
MotorSpeed_Update(0.01f);
```

OLED still displays motor speeds.

## Main Loop / Buttons

Main file: `main.c`

Relevant behavior:

- 10 ms timer updates IMU and motor speed measurement.
- If `startup_done && g_line_follow_enabled`, it calls `ExecuteTask(current_task)`.
- `ExecuteTask(TASK_ID_1)` calls `Tracking_Process()`.
- Middle key toggles line following.
- Up key changes task.

Current local diff added:

```c
while(middle_pressed);
while(up_pressed);
```

These are probably ineffective because the variables are set to `false` immediately before the `while`. They are harmless but suspicious. Consider removing them before committing unless the user intended a blocking debounce.

## Bluetooth / OLED

Bluetooth command parser: `Code/myBluetooth.c`

Useful commands:

- `G`: start line following
- `T:0`: stop
- `T:1`: start
- `P:xx`: line PID Kp
- `D:xx`: line PID Kd
- `B:xx`: line PID deadband
- `S:xx`: set `g_base_speed`
- `L:xx`: left wheel scale
- `R:xx`: right wheel scale
- `E:xx`: old encoder speed-match Kp variable

OLED: `Code/myOLED.c`

Displays:

- line run/stop
- yaw
- A/B speed in m/s and direction
- grayscale sensor bit pattern
- target/base speed and line PID output
- task id

## Important Hardware / Control Notes

`Motor_SetSpeed_A/B(int16_t speed)` takes roughly `-100..100`.

Current `motor.c` maps PWM using `100 - speed`, and previous real test confirmed:

- Fixed `Motor_SetSpeed_A/B(20, 30, 40, 50)` gave monotonically increasing speed.

So PWM polarity likely works for command magnitude.

Line-following instability can come from:

- Lost-line threshold too small
- `tracking_pid.Kd = 20.0f` too large
- `tracking_pid.OutMax = 100.0f` too large
- `g_base_speed` too high for current curve radius
- Wrong correction direction (`REVERSE_PID_DIR`)
- Sensor bit order mismatch

Current likely next tuning steps:

1. Test with `TRACKING_LOST_CENTER_BAND = 2.0f`.
2. If still swinging, reduce `tracking_pid.Kd` from `20.0f` to `5.0f` or `10.0f`.
3. If turns are too weak, keep `Kd` lower and increase `Kp` gradually, or reduce `g_base_speed`.
4. If it turns the wrong way, flip `REVERSE_PID_DIR`.
5. If curves fail only after total line loss, tune `TRACKING_LOST_CENTER_BAND` and `TRACKING_SEARCH_CMD`.

## Encoding Warning

Several existing source comments are mojibake in terminal output. Avoid large comment rewrites unless necessary. Keep code edits small.

## Do Not Forget

Before committing or pushing, run:

```powershell
D:\TI\ccs\utils\bin\gmake.exe -k -j 8 all -r -O
python C:\Users\Lenovo\.codex\skills\mspm0-ccs\scripts\check_syscfg.py C:\Users\Lenovo\workspace_ccstheia\myMG310
```

If doing a clean verification:

```powershell
D:\TI\ccs\utils\bin\gmake.exe -k clean all -r -O
```

