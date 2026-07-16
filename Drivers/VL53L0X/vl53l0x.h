#ifndef VL53L0X_H_
#define VL53L0X_H_

#include <stdbool.h>
#include <stdint.h>

/* DriverLib uses the 7-bit address. The datasheet writes it as 0x52 (8-bit). */
#define VL53L0X_I2C_ADDRESS_7BIT (0x29U)

/* Call once after SYSCFG_DL_init() and SysTick_Init(). */
bool VL53L0X_Init(void);

/* Non-blocking poll; returns true only when a new valid sample was stored. */
bool VL53L0X_Process(void);

/* Return the latest valid distance snapshot in millimeters. */
bool VL53L0X_GetDistance(uint16_t *distance_mm);

#endif
