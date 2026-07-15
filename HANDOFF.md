# myMG310slave Handoff

Updated: 2026-07-15

## Project

- Workspace: `C:\Users\Lenovo\workspace_ccstheia\myMG310slave`
- Remote: `https://github.com/Lynck/myMG310.git`
- Branch: `main`
- Target: MSPM0G3507, CCS Theia, SysConfig, TI Arm Clang

`mspm0-modules.syscfg` is the source of truth for peripherals and pins. Do not edit generated files under `Debug/`.

## Current Hardware Configuration

Both UARTs use 115200 8N1 with RX FIFO interrupts.

- CH9141K Bluetooth: generated name `UART_0`, hardware UART2
  - TX: PA21
  - RX: PA22
- MCU-based 8-channel grayscale module: generated name `UART_GRAY`, hardware UART3
  - TX: PB2
  - RX: PB3
  - Module TX must connect to PB3; common ground is required.

CH9141K follower module:

- Role: slave (`AT+BLEMODE=2`)
- Address: `AF:39:63:E4:C2:84`
- Leader pairing command: `AT+CONADD=AF:39:63:E4:C2:84,000000`
- Exit AT mode with `AT+EXIT`.

## Grayscale UART Driver

Files:

- `Code/grayscale_uart.c`
- `Code/grayscale_uart.h`

Supported frames:

- `AA 81 xx`: 8-channel binary state used by line following
- `AA 80 ...` and `AA 82 ... AA 86 ...`: eight 16-bit channel values

Line-following convention:

- bit7 is the leftmost sensor; bit0 is the rightmost sensor.
- `Grayscale_UART_GetBlackMask()` normalizes the result to `1 = black`.
- Current `GRAYSCALE_UART_BLACK_LEVEL` is `1U`; change it if OLED black/white is inverted.

OLED row 2 displays `L:BBBBWWWW:R`; `B` means black, `W` means white, and `-` means no valid `AA 81 xx` frame has arrived.

The previous GPIO grayscale driver files remain in the tree but active line following and OLED code no longer call them.

## Line Following

Main control file: `Code/myPID.c`.

- Default task 1 gains: Kp 6, Ki 0, Kd 20
- Task 2 gains: Kp 12, Ki 0, Kd 30
- Sensor weights remain `+4,+3,+2,+1,-1,-2,-3,-4`.
- Lost-line threshold remains `2.0f`.
- `leader_distance` is no longer used and its source files were removed.

## Bluetooth Commands

Parser: `Code/myBluetooth.c`.

Commands are ASCII and are executed only after CR or LF:

- `C:t:r` - synchronize task/run state
- `P:value` - set line Kp
- `D:value` - set line Kd
- `B:value` - set deadband
- `S:value` - set base speed
- `L:value`, `R:value` - wheel scale
- `E:value` - encoder speed-match gain

Example `D:1\r\n` bytes: `44 3A 31 0D 0A`.

Important current behavior:

- The firmware does not echo command success to the phone.
- OLED shows PID output, not Kp/Kd.
- `Tracking_PID_Init()` overwrites gains when a task is entered. Send tuning commands after line following has started, or change initialization behavior.
- Use `S:10\r\n` as a visible receive test; OLED `T:` should change to 10 in task 1.

## Removed / Inactive Features

- `Code/leader_distance.c/.h` removed.
- `Drivers/Ultrasonic_GPIO/ultrasonic_gpio.c/.h` removed.
- `TIMER_ULTRASONIC` still exists in SysConfig but currently has no runtime consumer; it can be cleaned up later.

## Validation

Latest clean build passed on 2026-07-15:

```powershell
D:\TI\ccs\utils\bin\gmake.exe clean -r -O
D:\TI\ccs\utils\bin\gmake.exe -j 32 all -r -O
```

Output: `Debug\myMG310slave.out`.

Static check:

```powershell
python C:\Users\Lenovo\.codex\skills\mspm0-ccs\scripts\check_syscfg.py C:\Users\Lenovo\workspace_ccstheia\myMG310slave
```

## Suggested Skills

- `mspm0-ccs` for SysConfig, UART, build, flash, and DriverLib work.
- `diagnosing-bugs` for missing UART data or intermittent hardware behavior.
- `ponytail` for minimal firmware changes.

## Recommended Next Work

Add a minimal Bluetooth acknowledgement or show Kp/Kd on OLED so phone tuning can be verified. Preserve CR/LF parsing and test with `S:10` before changing the parser.
