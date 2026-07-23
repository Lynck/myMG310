#include "ti_msp_dl_config.h"
#include "Grayscale_Sensor.h"

extern define_Data data_1;

uint8_t sensor_val;//1字节灰度传感器数据

// 定义传感器状态，将结构体转换为1个字节，方便使用 switch-case 或位运算
void Get_Sensor_State(void)
{
    // 调用你写的串行读取函数，读取后数据会存放在 data_1 结构体中
    Read_data_1_GPIO(); 
    
    // 将 data_1 的8个位拼接成一个8位无符号整数 (0x00 ~ 0xFF)
    // 假设 D1 是最左边的探头，D8 是最右边的探头 (具体物理排序请根据你的模块实际测试调整)
    sensor_val = (data_1.D8 << 7) |
                 (data_1.D7 << 6) |
                 (data_1.D6 << 5) |
                 (data_1.D5 << 4) |
                 (data_1.D4 << 3) |
                 (data_1.D3 << 2) |
                 (data_1.D2 << 1) |
                 (data_1.D1 << 0);
}