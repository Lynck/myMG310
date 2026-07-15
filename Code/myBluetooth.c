/* ================================================================
 *  myBluetooth.c - UART command parser for the reusable car frame
 * ================================================================
 *  Commands end with CR/LF:
 *    C:t:r   synchronize task t and run state r from the leader
 *    P:xx    set line PID Kp
 *    D:xx    set line PID Kd
 *    B:xx    set line PID deadband
 *    S:xx    set target speed in cm/s
 *    L:xx    set left wheel scale
 *    R:xx    set right wheel scale
 *    E:xx    set encoder speed-match Kp
 * ================================================================ */

#include "ti_msp_dl_config.h"
#include "myPID.h"
#include "PID.h"
#include "encoder.h"
#include "motor.h"
#include "myTask.h"
#include "clock.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "myBluetooth.h"

extern PID_t tracking_pid;

#define BT_RX_BUF_SIZE  64
#define BT_SYNC_TIMEOUT_MS 500UL
char bt_rx_buffer[BT_RX_BUF_SIZE];
uint16_t bt_rx_index = 0;
volatile bool bt_cmd_ready_flag = false;
static unsigned long bt_last_sync_ms = 0UL;
static bool bt_sync_received = false;

static void Bluetooth_SetLineFollowEnabled(bool enabled)
{
    if (enabled) {
        if (!g_line_follow_enabled) {
            Tracking_PID_Reset();
            g_line_follow_enabled = true;
        }
    } else {
        g_line_follow_enabled = false;
        Motor_Brake();
    }
}

void Bluetooth_ParseCommand(char *packet)
{
    char cmd_type = packet[0];
    float val = 0.0f;
    size_t packet_len = strlen(packet);

    if (cmd_type == 'C') {
        if ((packet_len == 5U) &&
            (packet[1] == ':') &&
            (packet[2] >= '0') &&
            (packet[2] < ('0' + TASK_MAX)) &&
            (packet[3] == ':') &&
            ((packet[4] == '0') || (packet[4] == '1'))) {
            bt_last_sync_ms = tick_ms;
            bt_sync_received = true;
            current_task = (Task_t)(packet[2] - '0');
            Bluetooth_SetLineFollowEnabled(packet[4] == '1');
        }
        return;
    }

    if (packet_len < 3U || packet[1] != ':') {
        return;
    }

    val = atof(&packet[2]);

    switch (cmd_type)
    {
        case 'P':
        case 'p':
            tracking_pid.Kp = val;
            break;

        case 'D':
        case 'd':
            tracking_pid.Kd = val;
            break;

        case 'B':
        case 'b':
            tracking_pid.Deadband = val;
            break;

        case 'S':
        case 's':
            g_base_speed = (int16_t)val;
            break;

        case 'L':
        case 'l':
            g_left_wheel_scale = val;
            break;

        case 'R':
        case 'r':
            g_right_wheel_scale = val;
            break;

        case 'E':
        case 'e':
            g_enc_kp = val;
            break;

        default:
            break;
    }
}

void Bluetooth_CheckSyncTimeout(void)
{
    if (bt_sync_received &&
        g_line_follow_enabled &&
        ((tick_ms - bt_last_sync_ms) > BT_SYNC_TIMEOUT_MS)) {
        Bluetooth_SetLineFollowEnabled(false);
    }
}

void Bluetooth_EnterLocalDebugMode(void)
{
    bt_sync_received = false;
}

void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:
            while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
            {
                char rx_data = (char)DL_UART_receiveData(UART_0_INST);

                if (bt_cmd_ready_flag) {
                    continue;
                }

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
