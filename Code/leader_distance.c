#include "leader_distance.h"

#include "ti_msp_dl_config.h"
#include "clock.h"

#include <stdint.h>
#include <stdlib.h>

#define LEADER_DISTANCE_RX_SIZE 24U

volatile float g_leader_distance_raw = 0.0f;
volatile float g_leader_distance_cm = 0.0f;
volatile bool g_leader_distance_updated = false;
volatile bool g_leader_distance_valid = false;
volatile unsigned long g_leader_distance_last_update_ms = 0UL;

static volatile char rx_frame[LEADER_DISTANCE_RX_SIZE];
static volatile uint8_t rx_index = 0U;
static volatile bool rx_frame_ready = false;

void LeaderDistance_Init(void)
{
    rx_index = 0U;
    rx_frame_ready = false;
    g_leader_distance_updated = false;
    g_leader_distance_valid = false;
    g_leader_distance_last_update_ms = 0UL;
    NVIC_ClearPendingIRQ(UART_3_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);
}

bool LeaderDistance_Process(void)
{
    char frame[LEADER_DISTANCE_RX_SIZE];
    char *end;
    float raw;
    uint8_t i;

    if (!rx_frame_ready) {
        return false;
    }

    NVIC_DisableIRQ(UART_3_INST_INT_IRQN);
    for (i = 0U; i < LEADER_DISTANCE_RX_SIZE; i++) {
        frame[i] = rx_frame[i];
        if (frame[i] == '\0') {
            break;
        }
    }
    frame[LEADER_DISTANCE_RX_SIZE - 1U] = '\0';
    rx_frame_ready = false;
    NVIC_EnableIRQ(UART_3_INST_INT_IRQN);

    raw = strtof(&frame[1], &end);
    if ((frame[0] != 'Z') || (end == &frame[1]) || (*end != '\0')) {
        return false;
    }

    g_leader_distance_raw = raw;
    g_leader_distance_cm = raw * LEADER_DISTANCE_SCALE_CM_PER_UNIT;
    g_leader_distance_last_update_ms = tick_ms;
    g_leader_distance_valid = true;
    g_leader_distance_updated = true;
    return true;
}

void UART_3_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_3_INST)) {
        case DL_UART_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_3_INST)) {
                char byte = (char)DL_UART_receiveData(UART_3_INST);

                if (rx_frame_ready || (byte == '\r')) {
                    continue;
                }

                if (byte == '\n') {
                    if (rx_index > 1U) {
                        rx_frame[rx_index] = '\0';
                        rx_frame_ready = true;
                    }
                    rx_index = 0U;
                } else if (byte == 'Z') {
                    rx_frame[0] = byte;
                    rx_index = 1U;
                } else if ((rx_index > 0U) &&
                           (rx_index < (LEADER_DISTANCE_RX_SIZE - 1U))) {
                    rx_frame[rx_index++] = byte;
                } else {
                    rx_index = 0U;
                }
            }
            break;

        default:
            break;
    }
}
