#include "ti_msp_dl_config.h"
#include "Grayscale_Sensor.h"

uint8_t sensor_val;

void Get_Sensor_State(void)
{
    /* bit7 = left sensor 1, bit0 = right sensor 8. */
    sensor_val = Grayscale_Sensor_Read();
}
