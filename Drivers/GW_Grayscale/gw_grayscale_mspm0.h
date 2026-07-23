#ifndef GW_GRAYSCALE_MSPM0_H_
#define GW_GRAYSCALE_MSPM0_H_

#include "gw_grayscale.h"
#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*GW_GrayDelayUs)(void *context, uint32_t microseconds);

typedef struct {
    I2C_Regs *i2c;
    uint32_t timeout_loops;
} GW_GrayHardwareI2C;

typedef struct {
    GPIO_Regs *scl_port;
    uint32_t scl_pin;
    GPIO_Regs *sda_port;
    uint32_t sda_pin;
    GW_GrayDelayUs delay_us;
    void *delay_context;
    uint32_t half_period_us;
} GW_GraySoftwareI2C;

typedef struct {
    GPIO_Regs *clk_port;
    uint32_t clk_pin;
    GPIO_Regs *dat_port;
    uint32_t dat_pin;
    GW_GrayDelayUs delay_us;
    void *delay_context;
} GW_GraySerial;

extern const GW_GrayBusOps GW_Gray_HardwareI2C_Bus;
extern const GW_GrayBusOps GW_Gray_SoftwareI2C_Bus;

void GW_Gray_HardwareI2C_Init(GW_GrayHardwareI2C *context,
                              I2C_Regs *i2c, uint32_t timeout_loops);
void GW_Gray_SoftwareI2C_Init(GW_GraySoftwareI2C *context,
                              GPIO_Regs *scl_port, uint32_t scl_pin,
                              GPIO_Regs *sda_port, uint32_t sda_pin,
                              GW_GrayDelayUs delay_us, void *delay_context,
                              uint32_t half_period_us);
void GW_Gray_Serial_Init(GW_GraySerial *context,
                         GPIO_Regs *clk_port, uint32_t clk_pin,
                         GPIO_Regs *dat_port, uint32_t dat_pin,
                         GW_GrayDelayUs delay_us, void *delay_context);
uint8_t GW_Gray_Serial_Read(const GW_GraySerial *context);

#ifdef __cplusplus
}
#endif

#endif
