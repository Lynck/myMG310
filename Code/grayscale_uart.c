#include "grayscale_uart.h"

#include <stddef.h>

#include "ti_msp_dl_config.h"

#define GRAYSCALE_FRAME_HEADER       (0xAAU)
#define GRAYSCALE_PAYLOAD_SIZE       (GRAYSCALE_UART_CHANNEL_COUNT * 2U)

typedef enum {
    GRAYSCALE_RX_WAIT_HEADER = 0,
    GRAYSCALE_RX_WAIT_COMMAND,
    GRAYSCALE_RX_WAIT_CHANNELS,
    GRAYSCALE_RX_WAIT_BINARY
} Grayscale_RxState;

static Grayscale_RxState rx_state;
static uint8_t rx_command;
static uint8_t rx_index;
static uint8_t rx_payload[GRAYSCALE_PAYLOAD_SIZE];

static volatile uint16_t latest_channels[GRAYSCALE_UART_CHANNEL_COUNT];
static volatile uint8_t latest_channel_command;
static volatile uint8_t latest_binary;
static volatile bool channels_ready;
static volatile bool binary_ready;
static volatile bool binary_valid;

static bool Grayscale_UART_IsChannelCommand(uint8_t command)
{
    return (command == 0x80U) ||
           ((command >= 0x82U) && (command <= 0x86U));
}

static void Grayscale_UART_PublishChannels(void)
{
    uint8_t channel;

    for (channel = 0U; channel < GRAYSCALE_UART_CHANNEL_COUNT; channel++) {
        uint8_t offset = (uint8_t)(channel * 2U);
        latest_channels[channel] =
            ((uint16_t)rx_payload[offset] << 8U) |
            (uint16_t)rx_payload[offset + 1U];
    }

    latest_channel_command = rx_command;
    channels_ready = true;
}

void Grayscale_UART_Init(void)
{
    rx_state = GRAYSCALE_RX_WAIT_HEADER;
    rx_command = 0U;
    rx_index = 0U;
    channels_ready = false;
    binary_ready = false;
    binary_valid = false;

    NVIC_ClearPendingIRQ(UART_GRAY_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_GRAY_INST_INT_IRQN);
}

void Grayscale_UART_ReceiveByte(uint8_t byte)
{
    switch (rx_state) {
        case GRAYSCALE_RX_WAIT_HEADER:
            if (byte == GRAYSCALE_FRAME_HEADER) {
                rx_state = GRAYSCALE_RX_WAIT_COMMAND;
            }
            break;

        case GRAYSCALE_RX_WAIT_COMMAND:
            rx_command = byte;
            rx_index = 0U;

            if (Grayscale_UART_IsChannelCommand(byte)) {
                rx_state = GRAYSCALE_RX_WAIT_CHANNELS;
            } else if (byte == 0x81U) {
                rx_state = GRAYSCALE_RX_WAIT_BINARY;
            } else {
                rx_state = GRAYSCALE_RX_WAIT_HEADER;
            }
            break;

        case GRAYSCALE_RX_WAIT_CHANNELS:
            /* Preserve the channel/byte order used by the original project. */
            rx_payload[GRAYSCALE_PAYLOAD_SIZE - 1U - rx_index] = byte;
            rx_index++;

            if (rx_index >= GRAYSCALE_PAYLOAD_SIZE) {
                Grayscale_UART_PublishChannels();
                rx_state = GRAYSCALE_RX_WAIT_HEADER;
                rx_index = 0U;
            }
            break;

        case GRAYSCALE_RX_WAIT_BINARY:
            latest_binary = byte;
            binary_ready = true;
            binary_valid = true;
            rx_state = GRAYSCALE_RX_WAIT_HEADER;
            break;

        default:
            rx_state = GRAYSCALE_RX_WAIT_HEADER;
            break;
    }
}

bool Grayscale_UART_ReadChannels(
    uint16_t channels[GRAYSCALE_UART_CHANNEL_COUNT], uint8_t *command)
{
    uint8_t channel;
    bool ready;
    uint32_t irq_enabled;

    if (channels == NULL) {
        return false;
    }

    irq_enabled = NVIC_GetEnableIRQ(UART_GRAY_INST_INT_IRQN);
    NVIC_DisableIRQ(UART_GRAY_INST_INT_IRQN);

    ready = channels_ready;
    if (ready) {
        for (channel = 0U; channel < GRAYSCALE_UART_CHANNEL_COUNT; channel++) {
            channels[channel] = latest_channels[channel];
        }
        if (command != NULL) {
            *command = latest_channel_command;
        }
        channels_ready = false;
    }

    if (irq_enabled != 0U) {
        NVIC_EnableIRQ(UART_GRAY_INST_INT_IRQN);
    }
    return ready;
}

bool Grayscale_UART_ReadBinary(uint8_t *binary)
{
    bool ready;
    uint32_t irq_enabled;

    if (binary == NULL) {
        return false;
    }

    irq_enabled = NVIC_GetEnableIRQ(UART_GRAY_INST_INT_IRQN);
    NVIC_DisableIRQ(UART_GRAY_INST_INT_IRQN);

    ready = binary_ready;
    if (ready) {
        *binary = latest_binary;
        binary_ready = false;
    }

    if (irq_enabled != 0U) {
        NVIC_EnableIRQ(UART_GRAY_INST_INT_IRQN);
    }
    return ready;
}

bool Grayscale_UART_GetBlackMask(uint8_t *black_mask)
{
    bool valid;
    uint32_t irq_enabled;

    if (black_mask == NULL) {
        return false;
    }

    irq_enabled = NVIC_GetEnableIRQ(UART_GRAY_INST_INT_IRQN);
    NVIC_DisableIRQ(UART_GRAY_INST_INT_IRQN);

    valid = binary_valid;
    if (valid) {
#if (GRAYSCALE_UART_BLACK_LEVEL == 0U)
        *black_mask = (uint8_t)~latest_binary;
#else
        *black_mask = latest_binary;
#endif
    }

    if (irq_enabled != 0U) {
        NVIC_EnableIRQ(UART_GRAY_INST_INT_IRQN);
    }
    return valid;
}

void UART_GRAY_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_GRAY_INST)) {
        case DL_UART_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_GRAY_INST)) {
                Grayscale_UART_ReceiveByte(
                    (uint8_t)DL_UART_receiveData(UART_GRAY_INST));
            }
            break;

        default:
            break;
    }
}
