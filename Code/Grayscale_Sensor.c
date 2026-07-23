#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "string.h"

#include "Grayscale_Sensor.h"

/*
2026/1/14
适用于： 无MCU版 8路通道循迹模块Ver1.2
使用前请到 Grayscale_Sensor.h 中更改 IO 口总表
*/

define_Data data_1; //单IO口读取到的数据
define_Data data_8; //8IO口读取到的数据

char message_1[12];
char message_8[12];
uint8_t target_1;
uint8_t target_8;

/**
 * @brief  单IO口读取数据
 * @param  无参数(所有参数请直接修改IO口总表)
 * @note   PL 为置位端，低电平数据流入寄存器，高电平锁定数据
 *         SCK 为时钟信号
 *         SDA 为数据输出端口
 *         data_1 存储数据
 * @retval 无返回值
 */
void Read_data_1_GPIO(void)
{
    uint8_t i;
    DL_GPIO_clearPins(SCK_PORT, SCK_PIN); // 确保 SCK 初始是低电平
    //并行加载
    DL_GPIO_clearPins(PL_PORT, PL_PIN);  //拉低PL，数据进入寄存器
    delay_cycles(8000);   //延时1ms
    DL_GPIO_setPins(PL_PORT, PL_PIN);    //拉高PL，锁定数据

    //读取Q7数据
    if(DL_GPIO_readPins(SDA_PORT, SDA_PIN) > 0)
    {
        data_1.D8 = 1;
    }
    else {
        data_1.D8 = 0;
    }
    //读取Q6-Q0
    for(i=0;i < 7;i++)
    {
        DL_GPIO_setPins(SCK_PORT, SCK_PIN);        //时钟上升沿，数据移位
        delay_cycles(8000);
        if(DL_GPIO_readPins(SDA_PORT,SDA_PIN) > 0)     //读取成功
        {
            switch(i)
            {
                case 0 :data_1.D7 = 1;break;
                case 1 :data_1.D6 = 1;break;
                case 2 :data_1.D5 = 1;break;
                case 3 :data_1.D4 = 1;break;
                case 4 :data_1.D3 = 1;break;
                case 5 :data_1.D2 = 1;break;
                case 6 :data_1.D1 = 1;break;
                default: break;
            }
        }
        else {
             switch(i)
            {
                case 0 :data_1.D7 = 0;break;
                case 1 :data_1.D6 = 0;break;
                case 2 :data_1.D5 = 0;break;
                case 3 :data_1.D4 = 0;break;
                case 4 :data_1.D3 = 0;break;
                case 5 :data_1.D2 = 0;break;
                case 6 :data_1.D1 = 0;break;
                default: break;
            }
        
        }
        DL_GPIO_clearPins(SCK_PORT,SCK_PIN);   //时钟下降沿，准备下一个数据位
        delay_cycles(8000);    //延时1ms
    }
}


/**
 * @brief  //8IO口读取数据
 * @param  无参数(所有参数请直接修改IO口总表)
 * @note   OUT0~OUT7 为各数据输出口
 *         data_8 存储数据
 * @retval 无返回值
 */
void Read_data_8_GPIO(void)
{
    //OUT0~OUT7的IO口直接接到了比较器输出，绕过了并转串芯片
        //data_8.D1
    if(DL_GPIO_readPins(OUT0_PORT,OUT0_PIN) > 0)   
        {
            data_8.D1 = 1;
        }
    else {
            data_8.D1 = 0;
        }
        //data_8.D2
    if(DL_GPIO_readPins(OUT1_PORT,OUT1_PIN) > 0)
        {
            data_8.D2 = 1;
        }
    else {
            data_8.D2 = 0;
        }
        //data_8.D3
    if(DL_GPIO_readPins(OUT2_PORT,OUT2_PIN) > 0)
        {
            data_8.D3 = 1;
        }
    else {
            data_8.D3 = 0;
        }
        //data_8.D4
    if(DL_GPIO_readPins(OUT3_PORT,OUT3_PIN) > 0)
        {
            data_8.D4 = 1;
        }
    else {
            data_8.D4 = 0;
        }
        //data_8.D5
    if(DL_GPIO_readPins(OUT4_PORT,OUT4_PIN) > 0)
        {
            data_8.D5 = 1;
        }
    else {
            data_8.D5 = 0;
        }
        //data_8.D6
    if(DL_GPIO_readPins(OUT5_PORT,OUT5_PIN) > 0)
        {
            data_8.D6 = 1;
        }
    else {
            data_8.D6 = 0;
        }
        //data_8.D7
    if(DL_GPIO_readPins(OUT6_PORT,OUT6_PIN) > 0)
        {
            data_8.D7 = 1;
        }
    else {
            data_8.D7 = 0;
        }
        //data_8.D8
    if(DL_GPIO_readPins(OUT7_PORT,OUT7_PIN) > 0)
        {
            data_8.D8 = 1;
        }
    else {
            data_8.D8 = 0;
        }
}

/**
 * @brief  单IO口发送数据
 * @param  uart  指向UART寄存器配置结构体的指针，包含串口GPIO引脚/波特率/时序等配置信息
 *         取值为：UART1,UART2,UART3...
 * @note   函数中的'1_GPIO'代指单IO口
 *         数据发送格式为：
 *         GD_SFF
 * @retval 无返回值
 */
void Send_data_1_GPIO(UART_Regs *uart)
{
    Read_data_1_GPIO();
    target_1 =    data_1.D8 << 7 |
                 data_1.D7 << 6 |
                 data_1.D6 << 5 |
                 data_1.D5 << 4 |
                 data_1.D4 << 3 |
                 data_1.D3 << 2 |
                 data_1.D2 << 1 |
                 data_1.D1 << 0;
    // sprintf(message_1,"GD_S%x\r\n",target_1);
    // uart1_send_string(uart , message_1);

}

/**
 * @brief  8IO口发送数据
 * @param  uart  指向UART寄存器配置结构体的指针，包含串口GPIO引脚/波特率/时序等配置信息
 *         取值为：UART1,UART2,UART3...
 * @note   函数中的'8_GPIO'代指8IO口
 *         数据发送格式为
 *         GD_PFF
 * @retval 无返回值
 */
void Send_data_8_GPIO(UART_Regs *uart)
{
    Read_data_8_GPIO();
    target_8 =    data_8.D8 << 7 |
                 data_8.D7 << 6 |
                 data_8.D6 << 5 |
                 data_8.D5 << 4 |
                 data_8.D4 << 3 |
                 data_8.D3 << 2 |
                 data_8.D2 << 1 |
                 data_8.D1 << 0;
    // sprintf(message_8,"GD_P%x\r\n",target_8);
    // uart1_send_string(uart , message_8);

}

