#ifndef __MYBLUETOOTH_H
#define __MYBLUETOOTH_H

#include <stdbool.h>

void Bluetooth_ParseCommand(char *packet);
void Bluetooth_SendString(char *str);

extern volatile bool bt_cmd_ready_flag;
extern char bt_rx_buffer[];

#endif