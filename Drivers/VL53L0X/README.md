# GY-VL53L0XV2 driver

This directory contains the ST VL53L0X API plus a small MSPM0 wrapper. Application code only needs `vl53l0x.h`.

## Wiring in this project

| Module | MSPM0G3507 |
| --- | --- |
| VIN | 3.3 V |
| GND | GND |
| SCL | PA11 (`I2C0 SCL`) |
| SDA | PA10 (`I2C0 SDA`) |
| GPIO1 | Not connected |
| XSHUT | Not connected |

Use 3.3 V for VIN. The GY-VL53L0XV2 schematic pulls the host side of SCL and SDA up to VIN, so powering VIN from 5 V would also pull the MSPM0 pins toward 5 V.

The module already pulls XSHUT high. This port polls measurement-ready state, so GPIO1 is optional.

## SysConfig contract

Add a controller-mode I2C instance with:

- name: `I2C_VL53L0X`
- peripheral: `I2C0`
- speed: 400 kHz
- SDA: PA10
- SCL: PA11

## Application API

```c
VL53L0X_Init();

/* Call periodically, for example every 10 ms. */
VL53L0X_Process();

uint16_t distance_mm;
if (VL53L0X_GetDistance(&distance_mm)) {
    /* Use distance_mm. */
}
```

To move the driver to another MSPM0 project, copy this directory, add its include path, create the same SysConfig I2C instance, and compile the `.c` files. Only `vl53l0x_platform.c` is MSPM0-specific.

The sensor core files retain STMicroelectronics' BSD-style license notices.
