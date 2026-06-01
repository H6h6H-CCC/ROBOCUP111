#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "move.h"
#include "gray.h"
#include "shijue.h"
#include "duoji.h"
#include "oled.h"
uint8_t stat=0;

#define Shaoma1 1
#define Zuoquan 2
#define TASK1   3
#define Shaoma2 4
#define Youquan 5
#define TASk2   6
#define HOME    7
extern uint8_t Coulor[5];
extern uint8_t txBuffer2[10];
extern uint8_t g_motionActive;
extern uint8_t count1;
extern int ball_1[5];
extern int ball_2[5];
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
	txBuffer2[0]=0xAE;txBuffer2[1]=i;txBuffer2[2]=0x55;
	HAL_UART_Transmit_DMA(&huart2,txBuffer2,3);
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

static void State_RunTask1VisionPoint(void)
{
	uint16_t vision_x = 0U;
	uint16_t vision_y = 0U;
	uint8_t vision_valid = 0U;

	Vision_XY_PID_Then_Forward_Backward_Reset();
	while(1)
	{
		vision_valid = get_vision_xy(&vision_x, &vision_y, 1280U, 960U);
		if(Vision_XY_PID_Then_Forward_Backward(vision_x, vision_y, vision_valid,
											   640U, 1280U,
											   480U, 960U,
											   7U, 7U,
											   0.00060f, 0.00000f, 0.00025f,
											   0.00060f, 0.00000f, 0.00025f,
											   0.03f, 0.14f,
											   400.0f, 400.0f,
											   15U,
											   90.0f, 270.0f,
											   0.0f, 180.0f,
											   200U,
											   0.122f, 0.200f,
											   0.20f,
											   0.0f, 180.0f))
		{
			break;
		}
		HAL_Delay(5);
	}


}


void State(void)
{
	//goto aaa1;
	switch(stat)
	{
		case 1:
			Move_StartTranslateForTime(225,0.3, 1350);
			while(1)
			{
				Move_Update();
				if(g_motionActive)
				{
					Move_StartTranslateForTime(180,0.2, 2250);
					while(1)
					{
						Move_Update();
						if(g_motionActive)
						{
							HAL_Delay(300);
							GetTask1SequenceHexArray();
							Move_StartTranslateForTime(0.5,0.4, 3180);
							while(1)
							{
								Move_Update();
								if(g_motionActive)
								{	
									 HAL_Delay(500);
									CHANge(3);
									 Move_RotateAngle(3,0,30,50);
									timstart=HAL_GetTick();
									while(1)
									{
										timnow=HAL_GetTick();
										if((timnow-timstart)>=7200)
										{
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
			Move_StopAll();
			CHANge(2);
			stat=3;
			BuildBall1OrderFromQr();
			HAL_Delay(500);
			break;
		case 2:
			break;
		case 3:
			OLED_ShowChar(1,1,Coulor[1]);
			OLED_ShowChar(2,1,Coulor[2]);
			OLED_ShowChar(3,1,Coulor[3]);
			OLED_ShowChar(4,1,Coulor[4]);
			OLED_ShowChar(5,1,Coulor[5]);

			State_RunTimedTranslate(235.0f, 0.5f, 2550U);
			State_RunTimedTranslate(270.0f, 0.5f, 550U);
			task1_2_step1();
			HAL_Delay(500);
			State_RunTask1VisionPoint();
			
			//set_servo_angle_direction(2U, 36.0f, 1U);
			
			
			
			
			State_RunTimedTranslate(288.0f, 0.35f, 1500U);
			//set_servo_angle_direction(2U, 72.0f, 1U);
			task1_2_step2();
			HAL_Delay(500);
			State_RunTask1VisionPoint();
			

			
			// State_RunTask1VisionPoint1();
			State_RunTimedTranslate(270.0f, 0.35f, 1300U);
			State_RunTimedTranslate(18.0f, 0.4f, 2300U);    
			State_RunTimedTranslate(90.0f, 0.28f, 680U);
			//set_servo_angle_direction(2U, 72.0f, 1U);
			task1_2_step3();
			HAL_Delay(500);
			State_RunTask1VisionPoint();
			

			
			
			State_RunTimedTranslate(245.0f, 0.4f, 1300U);
			HAL_Delay(100);
			//set_servo_angle_direction(2U, 72.0f, 1U);
			task1_2_step4();
			HAL_Delay(500);
			State_RunTask1VisionPoint();
			

			
			State_RunTimedTranslate(270.0f, 0.35f, 1000U);
			State_RunTimedTranslate(0.2f, 0.45f, 1650U);
			//set_servo_angle_direction(2U, 72.0f, 1U);
			task1_2_step5();
			HAL_Delay(500);
			State_RunTask1VisionPoint();
			

			GetTask1SequenceHexArray();
			stat=5;
			
			break;
		case 4:
			break;
		case 5:
		//aaa1:
			Move_StartTranslateForTime(270,0.4,2370);
			while(1)
			{
				Move_Update();
				if(g_motionActive)
				{
					HAL_Delay(500);
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
						else if((timnow-timstart)>=4400)
						{
							if(case5_turn3)
							{
								task2_1_step3();
								case5_turn3=0;
							}
						}
						else if((timnow-timstart)>=3200)
						{
							if(case5_turn2)
							{
								task2_1_step2();
								case5_turn2=0;
							}
						}
						else if((timnow-timstart)>=2050)
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
			Move_RotateAngle(90,0,50,50);
			HAL_Delay(500);
			Move_StopAll();
			HAL_Delay(500);
			Move_StartTranslateForTime(89,0.49,2900);				
			while(1)
			{
				Move_Update();
				if(g_motionActive)
				{
					HAL_Delay(500);
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

					BuildBall2OrderFromQr();
					task2_2_step1();
					State_RunTimedTranslate(0.0f, 0.45f, 1200U);
					HAL_Delay(100);
					State_RunTimedTranslate(180.0f, 0.45f, 1200U);
					
					Move_StartTranslateForTime(90,0.29,1200);
					while(1)
					{
						Move_Update();
						if(g_motionActive)
						{
							task2_2_step2();
							State_RunTimedTranslate(0.0f, 0.45f, 1200U);
							HAL_Delay(100);
							State_RunTimedTranslate(180.0f, 0.45f, 1200U);
							Move_StartTranslateForTime(90,0.29,1200);
							while(1)
							{
								Move_Update();
								if(g_motionActive)
								{
									task2_2_step3();
									State_RunTimedTranslate(0.0f, 0.45f, 1200U);
									HAL_Delay(100);
									State_RunTimedTranslate(180.0f, 0.45f, 1200U);
									HAL_Delay(3000);
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
			Move_StartTranslateForTime(195,0.5,4200); 
			while(1)
			{
				Move_Update();
			}
			break;
		}			
}






