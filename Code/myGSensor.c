#include "Grayscale_Sensor.h"

uint8_t sensor_val;//1字节灰度传感器数据

void Get_Sensor_State(void)
{
    sensor_val = Grayscale_Sensor_Read();
}
