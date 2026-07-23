#ifndef __GRAYSCALE_SENSOR_H_
#define __GRAYSCALE_SENSOR_H_

#include "ti_msp_dl_config.h"

//使用前请根据实际使用IO口更改此处的IO口总表

//灰度传感器单IO总接口与8IO口部分接口
//置位端，低电平数据流入寄存器，高电平锁定
#define PL_PIN                              (PL_PL_PIN_PIN)
//时钟信号
#define SCK_PORT                            (SCL_PORT)
#define SCK_PIN                             (SCL_SCK_PIN_PIN)
//数据输出端
#define SDA_PIN                             (SDA_SDA_PIN_PIN)

//灰度传感器8IO数据接口
//OUT0数据输出
#define OUT0_PORT                           (GPIOB)
#define OUT0_PIN                            (DL_GPIO_PIN_24)
//OUT1数据输出
#define OUT1_PORT                           (GPIOB)
#define OUT1_PIN                            (DL_GPIO_PIN_20)
//OUT2数据输出
#define OUT2_PORT                           (GPIOB)
#define OUT2_PIN                            (DL_GPIO_PIN_19)
//OUT3数据输出
#define OUT3_PORT                           (GPIOB)
#define OUT3_PIN                            (DL_GPIO_PIN_18)
//OUT4数据输出
#define OUT4_PORT                           (GPIOA)
#define OUT4_PIN                            (DL_GPIO_PIN_7)
//OUT5数据输出
#define OUT5_PORT                           (GPIOB)
#define OUT5_PIN                            (DL_GPIO_PIN_2)
//OUT6数据输出
#define OUT6_PORT                           (GPIOB)
#define OUT6_PIN                            (DL_GPIO_PIN_3)
//OUT7数据输出
#define OUT7_PORT                           (GPIOA)
#define OUT7_PIN                            (DL_GPIO_PIN_8)


typedef struct
{
    uint8_t D1;
    uint8_t D2;
    uint8_t D3;
    uint8_t D4;
    uint8_t D5;
    uint8_t D6;
    uint8_t D7;
    uint8_t D8;
}define_Data;

extern define_Data data_1;
extern define_Data data_8; 

//单IO口读取数据
void Read_data_1_GPIO(void);
//8IO口读取数据
void Read_data_8_GPIO(void);
//单IO口发送数据
void Send_data_1_GPIO(UART_Regs *uart);
//8IO口发送数据
void Send_data_8_GPIO(UART_Regs *uart);

#endif

