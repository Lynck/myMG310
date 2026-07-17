#include "leader_distance.h"

#include "clock.h"
#include "ti_msp_dl_config.h"

#include <stdlib.h>

#define LEADER_DISTANCE_RX_SIZE        (24U)
#define LEADER_DISTANCE_MIN_CM         (1.0f)
#define LEADER_DISTANCE_MAX_CM         (500.0f)

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
    g_leader_distance_raw = 0.0f;
    g_leader_distance_cm = 0.0f;
    g_leader_distance_updated = false;
    g_leader_distance_valid = false;
    g_leader_distance_last_update_ms = 0UL;
    NVIC_ClearPendingIRQ(UART_DISTANCE_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_DISTANCE_INST_INT_IRQN);
}

void LeaderDistance_ReceiveByte(uint8_t byte)
{
    if (rx_frame_ready || (byte == (uint8_t)'\r')) {
        return;
    }

    if (byte == (uint8_t)'\n') {
        if (rx_index > 1U) {
            rx_frame[rx_index] = '\0';
            rx_frame_ready = true;
        }
        rx_index = 0U;
    } else if (byte == (uint8_t)'Z') {
        rx_frame[0] = 'Z';
        rx_index = 1U;
    } else if ((rx_index > 0U) &&
               (rx_index < (LEADER_DISTANCE_RX_SIZE - 1U))) {
        rx_frame[rx_index++] = (char)byte;
    } else {
        rx_index = 0U;
    }
}

bool LeaderDistance_Process(void)
{
    char frame[LEADER_DISTANCE_RX_SIZE];
    char *end;
    float raw;
    float distance_cm;
    uint8_t i;

    g_leader_distance_updated = false;

    if (!rx_frame_ready) {
        if (g_leader_distance_valid &&
            ((tick_ms - g_leader_distance_last_update_ms) >
             LEADER_DISTANCE_TIMEOUT_MS)) {
            g_leader_distance_valid = false;
            g_leader_distance_raw = 0.0f;
            g_leader_distance_cm = 0.0f;
        }
        return false;
    }

    NVIC_DisableIRQ(UART_DISTANCE_INST_INT_IRQN);
    for (i = 0U; i < LEADER_DISTANCE_RX_SIZE; ++i) {
        frame[i] = rx_frame[i];
        if (frame[i] == '\0') {
            break;
        }
    }
    frame[LEADER_DISTANCE_RX_SIZE - 1U] = '\0';
    rx_frame_ready = false;
    NVIC_EnableIRQ(UART_DISTANCE_INST_INT_IRQN);

    raw = strtof(&frame[1], &end);
    distance_cm = raw * LEADER_DISTANCE_SCALE_CM_PER_UNIT;
    if ((frame[0] != 'Z') || (end == &frame[1]) || (*end != '\0') ||
        (distance_cm < LEADER_DISTANCE_MIN_CM) ||
        (distance_cm > LEADER_DISTANCE_MAX_CM)) {
        return false;
    }

    g_leader_distance_raw = raw;
    g_leader_distance_cm = distance_cm;
    g_leader_distance_last_update_ms = tick_ms;
    g_leader_distance_valid = true;
    g_leader_distance_updated = true;
    return true;
}

bool LeaderDistance_Get(float *distance_cm)
{
    if ((distance_cm == NULL) || !g_leader_distance_valid) {
        return false;
    }
    *distance_cm = g_leader_distance_cm;
    return true;
}

void UART_DISTANCE_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_DISTANCE_INST)) {
        case DL_UART_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_DISTANCE_INST)) {
                LeaderDistance_ReceiveByte(
                    (uint8_t)DL_UART_receiveData(UART_DISTANCE_INST));
            }
            break;

        default:
            break;
    }
}
