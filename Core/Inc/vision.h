#ifndef __VISION_H
#define __VISION_H

#include "stm32f4xx_hal.h"

//接收数据缓冲区
uint8_t usart2_rx_buf[4];
uint8_t usart3_rx_buf[10];

//视觉数据存储变量
uint8_t QRPacket[2];        //USART2：二维码数值
uint8_t ColorPacket[5];     //USART3：物料颜色顺序，取每种颜色首字母，b蓝色和黑色重复，黑色对应k
uint8_t LetterPacket[3];    //USART3：字母物料顺序
uint8_t LocatePacket[6];    //USART3：圆心坐标

//接收标志位
uint8_t qr_flag;
uint8_t color_flag;
uint8_t letter_flag;
uint8_t locate_flag;

void MX_USART2_UART_Init(void);    //USART2+DMA初始化
void MX_USART3_UART_Init(void);    //USART3+DMA初始化
void Vision_Parse_Packet(void);    //视觉数据包解析函数

#endif
