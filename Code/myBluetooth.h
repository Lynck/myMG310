#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>

void Bluetooth_ParseCommand(char *packet);
void Bluetooth_SendString(const char *str);
void Bluetooth_SendSyncState(uint8_t task, bool running);

extern volatile bool bt_cmd_ready_flag;
extern char bt_rx_buffer[];

#endif
