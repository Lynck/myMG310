#ifndef GRAYSCALE_UART_H
#define GRAYSCALE_UART_H

#include <stdbool.h>
#include <stdint.h>

#define GRAYSCALE_UART_CHANNEL_COUNT (8U)

/* Change to 1U only if the module reports black as logic high. */
#ifndef GRAYSCALE_UART_BLACK_LEVEL
#define GRAYSCALE_UART_BLACK_LEVEL (1U)
#endif

/* Enable UART reception after SYSCFG_DL_init(). */
void Grayscale_UART_Init(void);

/* Feed one protocol byte. Kept public so the parser is easy to port/test. */
void Grayscale_UART_ReceiveByte(uint8_t byte);

/*
 * Return the newest 8-channel frame once.
 * Commands 0x80 and 0x82..0x86 carry eight 16-bit channel values.
 */
bool Grayscale_UART_ReadChannels(
    uint16_t channels[GRAYSCALE_UART_CHANNEL_COUNT], uint8_t *command);

/* Return the newest 0x81 binary frame once; bit order is unchanged. */
bool Grayscale_UART_ReadBinary(uint8_t *binary);

/*
 * Non-consuming snapshot for line following and display.
 * bit7 = leftmost sensor, bit0 = rightmost sensor, 1 = black line.
 */
bool Grayscale_UART_GetBlackMask(uint8_t *black_mask);

#endif
