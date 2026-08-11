#ifndef __GRAY_H
#define __GRAY_H

#define GRAY_LOW     1   //低电平对应黑线
#define GRAY_HIGH    0   //高电平对应白色

#include "stm32f4xx_hal.h"
#include <stdint.h>


void Gray_Trace_GPIO_Init(void);          //引脚初始化
uint8_t Gray_Get_Status(void);            //获取8路灰度8位状态值
float Gray_Trace_Get_Dir(void);   //循迹函数
void XUNji(void);
void Task2_Start(void);           //修改：右移寻找目标侧黑线
#endif /* __GRAY_H */
