#include "gray.h"
#include "move.H"
#include "main.h"
#include "duoji.h"
#include "oled.h"
#include "math.h"
#include <stdio.h>
//八路引脚初始化
void Gray_Trace_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  //配置为上拉输入
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7|GPIO_PIN_6|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

//单路灰度电平读取函数，1=白色，0=黑色
uint8_t Gray_Read_Pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  return (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET) ? GRAY_LOW : GRAY_HIGH;
}

//获取8路灰度8位状态值
//最高位对应左4(PG8)，最低位对应右4(PD15)
uint8_t Gray_Get_Status(void)
{
  uint8_t gray_status = 0;
  gray_status |= (Gray_Read_Pin(GPIOG, GPIO_PIN_5) << 7);
  gray_status |= (Gray_Read_Pin(GPIOG, GPIO_PIN_6) << 6);
  gray_status |= (Gray_Read_Pin(GPIOG, GPIO_PIN_7) << 5);
  gray_status |= (Gray_Read_Pin(GPIOG, GPIO_PIN_8) << 4);
  gray_status |= (Gray_Read_Pin(GPIOD, GPIO_PIN_15) << 3);
  gray_status |= (Gray_Read_Pin(GPIOG, GPIO_PIN_2) << 2);
  gray_status |= (Gray_Read_Pin(GPIOG, GPIO_PIN_3) << 1);
  gray_status |= (Gray_Read_Pin(GPIOG, GPIO_PIN_4) << 0);
  return gray_status;
}

//返回值：+2.0(大左转)、+1.0(中左转)、+0.5(小左转)、0.0(直行)、-0.5(小右转)、-1.0(中右转)、-2.0(大右转)
float Gray_Trace_Get_Dir(void)
{
  uint8_t stat = Gray_Get_Status();
  float dir = 0; //默认直行

  switch(stat)
  {
    //直行模式
  	case 0xFF: dir = 0; break;  //无黑线
    case 0xE7: dir = 0;  break;  //左1右1均触线

	//左转模式
    case 0x7F: dir = 0.08; break;  //仅左4触线
	  case 0x3F: dir = 0.27; break;   //左4+左3触线
    case 0xBF: dir = 0.4; break;  //仅左3触线
	  case 0x9F: dir = 0.6; break;   //左3+左2触线
    case 0xDF: dir = 0.8; break;  //仅左2触线
	  case 0xCF: dir = 2; break;   //左2+左1触线
    case 0xEF: dir = 3; break;   //仅左1触线
    
    //右转模式
    case 0xFE: dir = -0.08; break;  //仅右4触线
	  case 0xFC: dir = -0.27; break;   //右4+右3触线
    case 0xFD: dir = -0.4; break;  //仅右3触线
    case 0xF9: dir = -0.6; break;   //右3+右2触线
    case 0xFB: dir = -0.8; break;  //仅右2触线
	  case 0xF3: dir = -2; break;   //右2+右1触线
    case 0xF7: dir = -3; break;   //仅右1触线

    default:   
		dir = 0;break;
  }
  return dir;
}

void XUNji(void)
{
  
	  float gray = Gray_Trace_Get_Dir();
	   if(gray != 0)
	  {
		if(gray > 0)
		{
			Move_Circle(gray, 0.35, 1);
			HAL_Delay(8);
		}
		else
		{ 
			Move_Circle(fabsf(gray), 0.35, 0);
			HAL_Delay(8);
		}
	  }
	  else
	  {
		Move_TranslateContinuous(0,0.45);
		HAL_Delay(8);
	  }
}

// 修改：迁移右移寻线函数；检测到左一（gray = 3.0f）后退出
extern uint8_t g_motionActive;
void Task2_Start(void)
{
  float gray = Gray_Trace_Get_Dir();

  while (gray < 3.0f)
  {
    gray = Gray_Trace_Get_Dir();
    if (gray >= 3.0f)
    {
      break;
    }

    Move_StartTranslateForTime(280.0f, 0.7f, 50U);
    while (1)
    {
      Move_Update();
      if (g_motionActive != 0U)
      {
        break;
      }
    }
  }
}

