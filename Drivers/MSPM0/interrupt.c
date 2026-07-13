#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "vl53l0x.h"
#include "Code/motor_speed.h"

uint8_t enable_group1_irq = 0;

void Interrupt_Init(void)
{
#if defined GPIO_ENCODER_INT_IRQN
    NVIC_EnableIRQ(GPIO_ENCODER_INT_IRQN);
    enable_group1_irq = 1;
#endif

    if (enable_group1_irq) {
        NVIC_EnableIRQ(1);
    }
}

void SysTick_Handler(void)
{
    tick_ms++;
}

void GROUP1_IRQHandler(void)
{
#if defined GPIO_ENCODER_PIN_A_IIDX && defined GPIO_ENCODER_PIN_B_IIDX
    switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
        case GPIO_ENCODER_PIN_A_IIDX:
            MotorSpeed_OnPulseA();
            return;

        case GPIO_ENCODER_PIN_B_IIDX:
            MotorSpeed_OnPulseB();
            return;

        default:
            break;
    }
#endif

    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
#if defined GPIO_MULTIPLE_GPIOA_INT_IIDX
        case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOA)) {
#if defined(GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && \
    (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOA)
                case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                    Read_VL53L0X();
                    break;
#endif
                default:
                    break;
            }
            break;
#endif

#if defined GPIO_MULTIPLE_GPIOB_INT_IIDX
        case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
#if defined(GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT) && \
    (GPIO_VL53L0X_PIN_VL53L0X_GPIO1_PORT == GPIOB)
                case GPIO_VL53L0X_PIN_VL53L0X_GPIO1_IIDX:
                    Read_VL53L0X();
                    break;
#endif
                default:
                    break;
            }
            break;
#endif

#if defined GPIO_VL53L0X_INT_IIDX
        case GPIO_VL53L0X_INT_IIDX:
            Read_VL53L0X();
            break;
#endif

        default:
            break;
    }
}
