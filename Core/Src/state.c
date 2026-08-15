#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "move.h"
#include "gray.h"
#include "shijue.h"
#include "duoji.h"
#include "oled.h"
#include <math.h>
#include <stdio.h>
uint8_t stat=0;

#define Shaoma1 1
#define Zuoquan 2
#define TASK1   3
#define Shaoma2 4
#define Youquan 5
#define TASk2   6
#define HOME    7

#define VISION_CMD_STOP      0x00U
#define VISION_CMD_QR_TASK1  0x01U
#define VISION_CMD_CIRCLE    0x02U
#define VISION_CMD_QR_TASK2  0x03U
#define VISION_CMD_COLOR     0x04U

extern uint8_t Coulor[5];
extern uint8_t QRPacke[2];
extern uint8_t QRPacke1[2];
extern uint8_t place[7];
extern uint8_t txBuffer2[10];
extern volatile uint8_t vision_place_valid;
extern volatile uint8_t vision_current_cmd;
extern uint8_t g_motionActive;
extern uint8_t count1;
extern int ball_1[5];
extern int ball_2[5];
extern uint8_t outLetter[4]; // 修改：用于输出任务二二维码解析顺序
extern uint8_t get_vision_xy(uint16_t *vision_x, uint16_t *vision_y, uint16_t max_x, uint16_t max_y);
extern void Vision_XY_PID_Then_Forward_Backward_Reset(void);
extern uint8_t Vision_XY_PID_Then_Forward_Backward(uint16_t vision_x,
                                                   uint16_t vision_y,
                                                   uint8_t valid,
                                                   uint16_t center_x,
                                                   uint16_t max_x,
                                                   uint16_t center_y,
                                                   uint16_t max_y,
                                                   uint16_t deadband_x_px,
                                                   uint16_t deadband_y_px,
                                                   float kp_x,
                                                   float ki_x,
                                                   float kd_x,
                                                   float kp_y,
                                                   float ki_y,
                                                   float kd_y,
                                                   float min_speed_mps,
                                                   float max_speed_mps,
                                                   float integral_limit_x,
                                                   float integral_limit_y,
                                                   uint32_t update_period_ms,
                                                   float angle_when_x_low,
                                                   float angle_when_x_high,
                                                   float angle_when_y_low,
                                                   float angle_when_y_high,
                                                   uint32_t stable_required_ms,
                                                   float forward_distance_m,
                                                   float backward_distance_m,
                                                   float move_speed_mps,
                                                   float forward_angle_deg,
                                                   float backward_angle_deg);
extern uint8_t Vision_XY_PID_Then_Forward_Backward2(uint16_t vision_x,
                                                   uint16_t vision_y,
                                                   uint8_t valid,
                                                   uint16_t center_x,
                                                   uint16_t max_x,
                                                   uint16_t center_y,
                                                   uint16_t max_y,
                                                   uint16_t deadband_x_px,
                                                   uint16_t deadband_y_px,
                                                   float kp_x,
                                                   float ki_x,
                                                   float kd_x,
                                                   float kp_y,
                                                   float ki_y,
                                                   float kd_y,
                                                   float min_speed_mps,
                                                   float max_speed_mps,
                                                   float integral_limit_x,
                                                   float integral_limit_y,
                                                   uint32_t update_period_ms,
                                                   float angle_when_x_low,
                                                   float angle_when_x_high,
                                                   float angle_when_y_low,
                                                   float angle_when_y_high,
                                                   uint32_t stable_required_ms,
                                                   float forward_distance_m,
                                                   float backward_distance_m,
                                                   float move_speed_mps,
                                                   float forward_angle_deg,
                                                   float backward_angle_deg);		
extern uint8_t Vision_XY_PID_Then_Forward_Backward3(uint16_t vision_x,
                                                   uint16_t vision_y,
                                                   uint8_t valid,
                                                   uint16_t center_x,
                                                   uint16_t max_x,
                                                   uint16_t center_y,
                                                   uint16_t max_y,
                                                   uint16_t deadband_x_px,
                                                   uint16_t deadband_y_px,
                                                   float kp_x,
                                                   float ki_x,
                                                   float kd_x,
                                                   float kp_y,
                                                   float ki_y,
                                                   float kd_y,
                                                   float min_speed_mps,
                                                   float max_speed_mps,
                                                   float integral_limit_x,
                                                   float integral_limit_y,
                                                   uint32_t update_period_ms,
                                                   float angle_when_x_low,
                                                   float angle_when_x_high,
                                                   float angle_when_y_low,
                                                   float angle_when_y_high,
                                                   uint32_t stable_required_ms,
                                                   float forward_distance_m,
                                                   float backward_distance_m,
                                                   float move_speed_mps,
                                                   float forward_angle_deg,
                                                   float backward_angle_deg);

uint32_t timstart;
uint32_t timnow;
uint8_t con=1;
uint8_t cot=1;

uint8_t duo1=1;
uint8_t duo2=1;
uint8_t duo3=1;
uint8_t duo4=1;
uint8_t duo5=1;
uint8_t duo6=1;



void CHANge(uint8_t i)
{
	txBuffer2[0]=0xAA;
	txBuffer2[1]=0x00;
	txBuffer2[2]=i;
	txBuffer2[3]=0x55;
	vision_current_cmd = i;
	HAL_UART_Transmit_DMA(&huart1,txBuffer2,4);
}

static uint8_t State_IsTask1Color(uint8_t color)
{
	return (color == 'k') || (color == 'w') || (color == 'r') || (color == 'g') || (color == 'b');
}

static uint8_t State_ColorExists(uint8_t color)
{
	uint8_t i = 0;
	for(i = 0; i < 5; i++)
	{
		if(Coulor[i] == color)
		{
			return 1;
		}
	}
	return 0;
}

static void State_FillTaskData(void)
{
	const uint8_t color_order[5] = {'k', 'w', 'r', 'g', 'b'};
	uint8_t i = 0;
	uint8_t j = 0;

	// 修改：二维码未完整收到时，任务1和任务2都默认补为编号1（ASCII "01"）
	if((QRPacke[0] == 0U) || (QRPacke[1] == 0U))
	{
		QRPacke[0] = '0';
		QRPacke[1] = '1';
	}
	if((QRPacke1[0] == 0U) || (QRPacke1[1] == 0U))
	{
		QRPacke1[0] = '0';
		QRPacke1[1] = '1';
	}

	for(i = 0; i < 5; i++)
	{
		if(State_IsTask1Color(Coulor[i]))
		{
			continue;
		}

		for(j = 0; j < 5; j++)
		{
			if(State_ColorExists(color_order[j]) == 0)
			{
				Coulor[i] = color_order[j];
				break;
			}
		}
	}
}

static void State_RunTimedTranslate(float angle_deg, float velocity_mps, uint32_t duration_ms)
{
	Move_StartTranslateForTime(angle_deg, velocity_mps, duration_ms);
	while(1)
	{
		Move_Update();
		if(g_motionActive)
		{
			break;
		}
		HAL_Delay(5);
	}
}
//任务一放置函数；center_x_px 用于调整视觉横向目标位置
static void State_RunTask1VisionPoint(float forward_distance_m, uint16_t center_x_px)
{
	uint16_t vision_x = 0U;
	uint16_t vision_y = 0U;
	uint8_t vision_valid = 0U;

	Vision_XY_PID_Then_Forward_Backward_Reset();
	while(1)
	{
		vision_valid = get_vision_xy(&vision_x, &vision_y, 960U, 720U);
		if(Vision_XY_PID_Then_Forward_Backward(vision_x, vision_y, vision_valid,
																   center_x_px, 960U,
											   360U, 720U,
											   2U, 2U,
											   0.0008f, 0.00000f, 0.00000f,
											   0.0008f, 0.00000f, 0.00000f,
											   0.01f, 0.14f,
											   400.0f, 400.0f,
											   15U,
											   90.0f, 270.0f,
											   0.0f, 180.0f,
											   100U,
											   forward_distance_m, 0.180f,
											   0.15f,
											   0.0f, 180.0f))
		{
			break;
		}
		HAL_Delay(5);
	}
}
//任务二放置函数；center_x_px 用于调整视觉横向目标位置
static void State_RunTask2VisionPoint(float forward_distance_m, float backward_distance_m, uint16_t center_x_px){
	uint16_t vision_x = 0U;
	uint16_t vision_y = 0U;
	uint8_t vision_valid = 0U;

	Vision_XY_PID_Then_Forward_Backward_Reset();
	while(1)
	{
		vision_valid = get_vision_xy(&vision_x, &vision_y, 960U, 720U);
		if(Vision_XY_PID_Then_Forward_Backward2(vision_x, vision_y, vision_valid,
											   center_x_px, 960U,
											   360U, 720U,
											   2U, 2U,
											   0.0008f, 0.00000f, 0.00000f,
											   0.0008f, 0.00000f, 0.00000f,
											   0.01f, 0.14f,
											   400.0f, 400.0f,
											   15U,
											   90.0f, 270.0f,
											   0.0f, 180.0f,
											   200U,
											   forward_distance_m, backward_distance_m,
											   0.20f,
											   0.0f, 180.0f))
		{
			break;
		}
		HAL_Delay(5);
	}
}
static void State_RunTask2VisionPoint2(void){
	uint16_t vision_x = 0U;
	uint16_t vision_y = 0U;
	uint8_t vision_valid = 0U;

	Vision_XY_PID_Then_Forward_Backward_Reset();
	while(1)
	{
		vision_valid = get_vision_xy(&vision_x, &vision_y, 640U, 480U);
		if(Vision_XY_PID_Then_Forward_Backward3(vision_x, vision_y, vision_valid,
											   526U, 960U,
											   360U, 720U,
											   2U, 2U,
											   0.0008f, 0.00000f, 0.00000f,
											   0.0008f, 0.00000f, 0.00000f,
											   0.01f, 0.14f,
											   400.0f, 400.0f,
											   15U,
											   90.0f, 270.0f,
											   0.0f, 180.0f,
											   200U,
											   0.1775f, 0.200f,
											   0.20f,
											   0.0f, 180.0f))
		{
			break;
		}
		HAL_Delay(5);
	}
}
uint8_t qwqwq = 1;
void State(void)
{
	uint16_t vision_x = 0U;
	uint16_t vision_y = 0U;
	 //goto aaa1;

	//CHANge(3);
	//CHANge(VISION_CMD_QR_TASK1);
	if(qwqwq)
	{		    
    	   //降低
		CHANge(3);
		qwqwq=0;
		// 修改：PB9 按键未按下时保持等待，按下低电平后退出
		 while(1)
     	{
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET)
			{
				break;
			}
			HAL_Delay(10);
  	    }
		duoji_tc(); 
	}
	switch(stat)
	{
		case 1:
			Move_StartTranslateForTime(230,0.3, 1550);
			while(1)
			{
				Move_Update();
				if(g_motionActive)
				{

					Move_StartTranslateForTime(180,0.3, 1815);

					while(1)
					{
						Move_Update();
						if(g_motionActive)
						{
							HAL_Delay(300);
							Move_StartTranslateForTime(1.1,0.4, 3600);
							uint32_t task1_qr_start_tick = HAL_GetTick();
							uint8_t task1_qr_sent = 0U;
							while(1)
							{
								Move_Update();
								// 修改：运动开始1500ms后发送一次任务1二维码识别指令
								if((task1_qr_sent == 0U) &&
								   ((HAL_GetTick() - task1_qr_start_tick) >= 1500U))
								{
									CHANge(1);
									task1_qr_sent = 1U;
								}
								if(g_motionActive)
								{	

									HAL_Delay(100);
									Move_RotateCW_FromInitialYaw(0.0f);
									HAL_Delay(100);
						
					// while(1)
					// {

					// }
									
									State_RunTimedTranslate(90.0f, 0.5f, 500);
									CHANge(4);
									{
						char qr_debug[40];
						int qr_debug_len = snprintf(qr_debug, sizeof(qr_debug),
							"TASK1_QR=%c%c,TASK2_QR=%c%c\r\n",
							(QRPacke[0] != 0U) ? QRPacke[0] : '0',
							(QRPacke[1] != 0U) ? QRPacke[1] : '0',
							(QRPacke1[0] != 0U) ? QRPacke1[0] : '0',
							(QRPacke1[1] != 0U) ? QRPacke1[1] : '0');

						if (qr_debug_len > 0)
						{
							for (uint8_t i = 0U; i < 3U; i++)
							{
								HAL_UART_Transmit(&huart2, (uint8_t *)qr_debug,
									(uint16_t)qr_debug_len, 100U);
								HAL_Delay(10);
							}
						}
					}
									Move_RotateCW_FromInitialYaw(7.0f);
									HAL_Delay(200);
									GetTask1SequenceHexArray();
									timstart=HAL_GetTick();
									while(1)
									{
										timnow=HAL_GetTick();
										if((timnow-timstart)>=7200)
										{
											CHANge(2);
											goto EXIT_CASE1; 
										}
										else if((timnow-timstart)>=6800)
										{
											if(duo5)
											{
												task1_1_finish();
												duo5=0;
											}
										}
										else if((timnow-timstart)>=5500)
										{
											if(duo4)
											{
												task1_1_step4();
												duo4=0;
											}
										}
										else if((timnow-timstart)>=4200)
										{
											if(duo3)
											{
												task1_1_step3();
												duo3=0;
											}
										}
										else if((timnow-timstart)>=3100)
										{
											if(duo2)
											{
												task1_1_step2();
												duo2=0;
											}
											
		//									//��ת
											//{
											// for (int i = 0; i < 5; i++) 
											// {
											// 	set_servo_angle_direction(2, 72, CLOCKWISE);
											// 	HAL_Delay(5000);
											// }
											// set_servo_angle_direction(2, 36, CLOCKWISE);
											// HAL_Delay(5000);
											// reset_ball_output_state();
											// g_first_ball5_ccw_compensation_pending = 1U;
											//}	
											
										}
										else if((timnow-timstart)>=1690)
										{
											if(duo1)
											{
												task1_1_step1();
												duo1=0;
											}
										}
										XUNji();
									}
								}
							}
						}
					}
				}
			}
			EXIT_CASE1:
			//CHANge(VISION_CMD_CIRCLE);
			Move_StopAll();
			stat=3;
			BuildBall1OrderFromQr();
			HAL_Delay(500);

			break;
		case 2:
			break;
		case 3:
		//111 
			State_FillTaskData(); // 修改：在原位置统一补全二维码和任务1颜色
			BuildBall1OrderFromQr();
			//	CHANge(VISION_CMD_CIRCLE);

			State_RunTimedTranslate(235.0f, 0.5f, 2530U);
			State_RunTimedTranslate(0.0f, 0.3f, 500U);
			HAL_Delay(200);
			//output_ball_alternating((uint8_t)ball_1[0]);
			duoji_Turntable_Set_Port((uint8_t)ball_1[0]);

			HAL_Delay(200);

			// 修改：未收到有效视觉坐标时持续等待
			while (get_vision_xy(&vision_x, &vision_y, 960U, 720U) == 0U)
			{
				HAL_Delay(5);
			}

			State_RunTask1VisionPoint(0.156f, 528U);
			
			//set_servo_angle_direction(2U, 36.0f, 1U);
			
			
			
			
			State_RunTimedTranslate(278.0f, 0.35f, 1400U);
			//set_servo_angle_direction(2U, 72.0f, 1U);
			//output_ball_alternating((uint8_t)ball_1[1]);
			duoji_Turntable_Set_Port((uint8_t)ball_1[1]);
			HAL_Delay(300);
			State_RunTask1VisionPoint(0.156f, 527U);


			
			// State_RunTask1VisionPoint1();
			State_RunTimedTranslate(270.0f, 0.35f, 1500U);
			Move_RotateCW_FromInitialYaw(135.0f);
			HAL_Delay(100);
			State_RunTimedTranslate(5.0f, 0.4f, 2250U);    
			//set_servo_angle_direction(2U, 72.0f, 1U);
			//output_ball_alternating((uint8_t)ball_1[2]);
			duoji_Turntable_Set_Port((uint8_t)ball_1[2]);
			HAL_Delay(300);
			State_RunTask1VisionPoint(0.156f, 527U);
			

			
			
			State_RunTimedTranslate(245.0f, 0.4f, 1360U);
			HAL_Delay(100);
			//set_servo_angle_direction(2U, 72.0f, 1U);
			//output_ball_alternating((uint8_t)ball_1[3]);
			duoji_Turntable_Set_Port((uint8_t)ball_1[3]);
			HAL_Delay(300);
			State_RunTask1VisionPoint(0.155f, 525U);
			

			
			State_RunTimedTranslate(270.0f, 0.35f, 1100U);
			State_RunTimedTranslate(-1.0f, 0.45f, 1450U);
			//set_servo_angle_direction(2U, 72.0f, 1U);
			//output_ball_alternating((uint8_t)ball_1[4]);
			duoji_Turntable_Set_Port((uint8_t)ball_1[4]);
			
			HAL_Delay(300);
			State_RunTask1VisionPoint(0.156f, 527U);
			
            duoji_Turntable_Set_Start_Position();
			GetTask1SequenceHexArray();
			stat=5;
			Move_RotateCW_FromInitialYaw(135.0f);
			HAL_Delay(10);
			
			break;
		case 4:
			break;
		case 5:
		//aaa1:
			Move_StartTranslateForTime(280,0.4,2300);

			// while(1)
			// {
			// 	Move_Update();
			// }
			//aaa1:
			while(1)
			{
				Move_Update();
				if(g_motionActive)
				{
					Task2_Start();
					HAL_Delay(200);
					//Move_RotateAngle(3,1,50,50);
					//HAL_Delay(200);
					timstart=HAL_GetTick();
					uint8_t case5_turn1 = 1U;
					uint8_t case5_turn2 = 1U;
					uint8_t case5_turn3 = 1U;
					while(1)
					{
						timnow=HAL_GetTick();
						if((timnow-timstart)>=4800)
						{
							goto EXIT_CASE6; 
						}
						else if((timnow-timstart)>=4300)
						{
							if(case5_turn3)
							{
								task2_1_step3();
								case5_turn3=0;
							}
						}
						else if((timnow-timstart)>=3300)
						{
							if(case5_turn2)
							{
								task2_1_step2();
								case5_turn2=0;
							}
						}
						else if((timnow-timstart)>=2000)
						{
							if(case5_turn1)
							{
								task2_1_step1();
								case5_turn1=0;
							}
						}
						XUNji();
					}
				}
			}
			
			Move_StopAll();
			stat=6;
			break;
		case 6:
			EXIT_CASE6:
			//HAL_Delay(500);
			Move_RotateCW_FromInitialYaw(90.0f);
			HAL_Delay(300);
			Move_StopAll();
			HAL_Delay(100);
			Move_StartTranslateForTime(104,0.49,2650);	
			yajun_1();			
			while(1)
			{
				Move_Update();
				if(g_motionActive)
				{
					//Move_RotateCW_FromInitialYaw(89.5f);
					HAL_Delay(100);
					//  aaa1:
					//  HAL_Delay(1000);
					//  CHANge(2);
					//  yajun_1();///记得注释！！！！！！！！！！！
					vision_x = 0U;
					vision_y = 0U;
					for(uint8_t i = 0U; i < 6U; i++)
					{
						place[i] = '0';
					}
					place[6] = '\0';
					vision_place_valid = 0U;
					while(1)
					{
						//vision_valid = get_vision_xy(&vision_x, &vision_y, 1280U, 960U);
						if(vision_x == 0U || vision_y == 0U)
						{
							(void)get_vision_xy(&vision_x, &vision_y, 1280U, 960U);
							Move_StartTranslateForTime(0,0.07,100);
							while(1)
							{
								Move_Update();
								if(g_motionActive)
								{
									break;
								}
							}
						}
						else
						{
							break;
						}
					}
					HAL_Delay(300);
					//Move_RotateAngle(42,0,50,50);
					// //Move_StartTranslateForTime(0,0.33,1080); //�Ǿ�
					// while(1)
					// {
					// 	Move_Update();
					// 	if(g_motionActive)
					// 	{
					// 		HAL_Delay(3000);
					// 		Move_StartTranslateForTime(90,0.29,1285);	//�ھ�
					// 		while(1)
					// 		{
					// 			Move_Update();
					// 			if(g_motionActive)
					// 			{
					// 				HAL_Delay(3000);
					// 				Move_StartTranslateForTime(90,0.29,1285);	 //����
					// 				while(1)
					// 				{
					// 						Move_Update();
					// 						if(g_motionActive)
					// 						{
					// 							HAL_Delay(3000);
					// 							goto EXIT_CASE7; 
												
					// 						}
					// 				}
					// 			}
					// 		}	
					// 	}	
					// }
					// Move_StartTranslateForTime(0,0.49,255);	
					// while(1)
					// {
					// Move_Update();
					// if(g_motionActive)
					// {
					// stat=3;
					// GetTask1SequenceHexArray();
					// 修改：ball_2[0..2]依次保存亚军b、冠军a、季军c所在的逻辑仓号
					BuildBall2OrderFromQr();
					/* 修改：任务二顺序调试完成，暂时关闭串口输出
					{
						// 修改：任务二放置前输出二维码、映射顺序和端口，每次仅发送2遍
						char task2_debug[64];
						int task2_debug_len = snprintf(task2_debug, sizeof(task2_debug),
							"TASK2_QR=%c%c,OUT=%c%c%c,BALL=%d%d%d\r\n",
							(QRPacke1[0] != 0U) ? QRPacke1[0] : '0',
							(QRPacke1[1] != 0U) ? QRPacke1[1] : '0',
							(outLetter[0] != 0U) ? outLetter[0] : '0',
							(outLetter[1] != 0U) ? outLetter[1] : '0',
							(outLetter[2] != 0U) ? outLetter[2] : '0',
							ball_2[0], ball_2[1], ball_2[2]);

						if(task2_debug_len > 0)
						{
							for(uint8_t i = 0U; i < 2U; i++)
							{
								HAL_UART_Transmit(&huart2, (uint8_t *)task2_debug,
									(uint16_t)task2_debug_len, 100U);
								HAL_Delay(10);
							}
						}
					}
					*/
					// 修改：实际转盘1号口和3号口反向，使用4-逻辑仓号转换为物理端口
					duoji_Turntable_Set_Port(4U - (uint8_t)ball_2[0]); // 第一次：亚军位置放b
					State_RunTask2VisionPoint(0.164f, 0.275f, 528U); // 修改：第三个参数为中心坐标
					Move_StartTranslateForTime(90,0.29,1240);
					guanjun_1();
					while(1)
					{
						Move_Update();
						if(g_motionActive)
						{
							duoji_Turntable_Set_Port(4U - (uint8_t)ball_2[1]); // 第二次：冠军位置放a
							State_RunTask2VisionPoint(0.181f, 0.290f, 529U); // 修改：第二次额外后退2厘米，第三个参数为中心坐标
							HAL_Delay(10);
							Move_StartTranslateForTime(90,0.29,1200);
							duoji_tc_2();
							//Move_StartTranslateForTime(90,0.29,1170);
							while(1)
							{
								Move_Update();
				if(g_motionActive)
				{
					duoji_Turntable_Set_Port(4U - (uint8_t)ball_2[2]); // 第三次：季军位置放c
					// 修改：靠近转盘的 ID1 舵机下降 2°（-122° -> -120°）
					duoji_Set_ID1_Angle_yajun(-112.0f);
					duoji_Set_ID2_Angle(15.0f);
					State_RunTask1VisionPoint(0.158f, 524U); // 第三次放置前移2厘米
									HAL_Delay(100);
					 				goto EXIT_CASE7; 									
								}
							}
						}
					}
				//}
			//}
					
					
				}
			}
			EXIT_CASE7:
			stat=7;
			break;
		case 7:
			Move_RotateCW_FromInitialYaw(89.5f);
			HAL_Delay(100);
			Move_StartTranslateForTime(190.5,0.5,3910);
			duoji_tc_1(); 
			while(1)
			{
				Move_Update();
			}
			break;
		}			
}






