/* ================================================================
 *  myBluetooth.c - UART command parser for the reusable car frame
 * ================================================================
 *  Commands end with CR/LF:
 *    G       start line following
 *    T:0     stop line following
 *    T:1     start line following
 *    P:xx    set line PID Kp
 *    D:xx    set line PID Kd
 *    B:xx    set line PID deadband
 *    S:xx    set base speed, 0-100
 *    L:xx    set left wheel scale
 *    R:xx    set right wheel scale
 *    E:xx    set encoder speed-match Kp
 * ================================================================ */

#include "ti_msp_dl_config.h"
#include "myPID.h"
#include "PID.h"
#include "encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "myBluetooth.h"

extern PID_t tracking_pid;

#define BT_RX_BUF_SIZE  64
char bt_rx_buffer[BT_RX_BUF_SIZE];
uint16_t bt_rx_index = 0;
volatile bool bt_cmd_ready_flag = false;

void Bluetooth_SendString(char *str)
{
    while (*str)
    {
        DL_UART_transmitDataBlocking(UART_0_INST, *str++);
    }
}

void Bluetooth_ParseCommand(char *packet)
{
    char cmd_type = packet[0];
    float val = 0.0f;
    char reply_buf[64];

    if ((cmd_type == 'G' || cmd_type == 'g') && packet[1] == '\0') {
        Tracking_PID_Reset();
        g_line_follow_enabled = true;
        Bluetooth_SendString("OK! Line follow started!\r\n");
        return;
    }

    if (strlen(packet) < 3 || packet[1] != ':') {
        Bluetooth_SendString("Error: Invalid Format!\r\n");
        return;
    }

    val = atof(&packet[2]);

    switch (cmd_type)
    {
        case 'P':
        case 'p':
            tracking_pid.Kp = val;
            sprintf(reply_buf, "OK! Kp set to %.2f\r\n", tracking_pid.Kp);
            Bluetooth_SendString(reply_buf);
            break;

        case 'D':
        case 'd':
            tracking_pid.Kd = val;
            sprintf(reply_buf, "OK! Kd set to %.2f\r\n", tracking_pid.Kd);
            Bluetooth_SendString(reply_buf);
            break;

        case 'B':
        case 'b':
            tracking_pid.Deadband = val;
            sprintf(reply_buf, "OK! Deadband set to %.2f\r\n", tracking_pid.Deadband);
            Bluetooth_SendString(reply_buf);
            break;

        case 'S':
        case 's':
            g_base_speed = (int16_t)val;
            sprintf(reply_buf, "OK! BaseSpeed set to %d\r\n", g_base_speed);
            Bluetooth_SendString(reply_buf);
            break;

        case 'L':
        case 'l':
            g_left_wheel_scale = val;
            sprintf(reply_buf, "OK! LeftScale set to %.2f\r\n", g_left_wheel_scale);
            Bluetooth_SendString(reply_buf);
            break;

        case 'R':
        case 'r':
            g_right_wheel_scale = val;
            sprintf(reply_buf, "OK! RightScale set to %.2f\r\n", g_right_wheel_scale);
            Bluetooth_SendString(reply_buf);
            break;

        case 'E':
        case 'e':
            g_enc_kp = val;
            sprintf(reply_buf, "OK! EncoderKP set to %.2f\r\n", g_enc_kp);
            Bluetooth_SendString(reply_buf);
            break;

        case 'T':
        case 't':
            if ((int8_t)val == 0) {
                g_line_follow_enabled = false;
                Bluetooth_SendString("OK! Line follow stopped!\r\n");
            } else {
                Tracking_PID_Reset();
                g_line_follow_enabled = true;
                Bluetooth_SendString("OK! Line follow started!\r\n");
            }
            break;

        default:
            Bluetooth_SendString("Error: Unknown Command!\r\n");
            break;
    }
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
            {
                char rx_data = (char)DL_UART_receiveData(UART_0_INST);

                if (rx_data == '\n' || rx_data == '\r')
                {
                    if (bt_rx_index > 0)
                    {
                        bt_rx_buffer[bt_rx_index] = '\0';
                        bt_cmd_ready_flag = true;
                        bt_rx_index = 0;
                    }
                }
                else
                {
                    if (bt_rx_index < (BT_RX_BUF_SIZE - 1))
                    {
                        bt_rx_buffer[bt_rx_index++] = rx_data;
                    }
                    else
                    {
                        bt_rx_index = 0;
                    }
                }
            }
            break;

        default:
            break;
    }
}