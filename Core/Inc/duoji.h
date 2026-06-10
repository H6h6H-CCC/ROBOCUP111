#ifndef __duoji_H
#define __duoji_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* 舵机参数定义 */
#define duoji_ID_DEFAULT    000     // 出厂默认ID
#define duoji_BROADCAST_ID  255     // 广播ID
#define duoji_PWM_MIN       500     // PWM最小值（补0为0500）
#define duoji_PWM_MID       1500    // PWM中间值
#define duoji_PWM_MAX       2500    // PWM最大值（补0为2500）
#define duoji_TIME_MIN      0       // 时间最小值（补0为0000）
#define duoji_TIME_MAX      9999    // 时间最大值
#define DUOJI_UART_BAUD     115200U

extern uint16_t Sid[];
extern uint16_t Spwm[];
extern uint16_t Stime[];

/* 舵机工作模式定义（对应协议） */
typedef enum
{
    duoji_MODE_270C = 1,  // 270度顺时针
    duoji_MODE_270A,      // 270度逆时针
    duoji_MODE_180C,      // 180度顺时针
    duoji_MODE_180A,      // 180度逆时针
    MOTOR_MODE_360C_CYC,  // 马达360度定圈顺时针
    MOTOR_MODE_360A_CYC,  // 马达360度定圈逆时针
    MOTOR_MODE_360C_TIM,  // 马达360度定时顺时针
    MOTOR_MODE_360A_TIM   // 马达360度定时逆时针
}duoji_Mode_t;

/* 舵机开机模式 */
typedef enum
{
    duoji_PWR_MODE_POS = 1,  // 开机转到启动位置
    duoji_PWR_MODE_HOLD,     // 开机保持当前位置
    duoji_PWR_MODE_NO_FORCE  // 开机无力
}duoji_PwrMode_t;

/* 波特率配置参数（对应协议：1-9600，2-19200...8-1000000） */
typedef enum
{
    duoji_BAUD_9600 = 1,
    duoji_BAUD_19200,
    duoji_BAUD_38400,
    duoji_BAUD_57600,
    duoji_BAUD_115200,
    duoji_BAUD_128000,
    duoji_BAUD_256000,
    duoji_BAUD_1000000
}duoji_Baud_t;

/* 舵机指令缓冲区（最大支持3个舵机同时控制，可扩展） */
#define duoji_CMD_BUF_SIZE  128
extern char duoji_Cmd_Buf[duoji_CMD_BUF_SIZE];

/* 转盘舵机方向 */
typedef enum
{
    duoji_TURNTABLE_CW = 0,
    duoji_TURNTABLE_CCW = 1
} duoji_TurntableDir_t;

/* 舵机基础API */
void duoji_Init(void);  // 舵机初始化（启动串口DMA）
uint8_t duoji_Send_Cmd(char *cmd); // 发送舵机指令（封装DMA发送）

/* 舵机控制API（按协议实现） */
void duoji_Control(uint16_t id, uint16_t pwm, uint16_t time); // 单舵机角度/速度控制
void duoji_Multi_Control(uint8_t num, uint16_t *id, uint16_t *pwm, uint16_t *time); // 多舵机控制
void duoji_Set_ID(uint16_t old_id, uint16_t new_id); // 设置舵机ID
void duoji_Read_ID(uint16_t id); // 读取舵机ID
void duoji_Read_Ver(uint16_t id); // 读取舵机版本
void duoji_Release_Torque(uint16_t id); // 释放扭力
void duoji_Recover_Torque(uint16_t id); // 恢复扭力
void duoji_Set_Mode(uint16_t id, duoji_Mode_t mode); // 设置工作模式
void duoji_Read_Mode(uint16_t id); // 读取工作模式
void duoji_Read_Pos(uint16_t id); // 读取舵机位置
void duoji_Pause(uint16_t id); // 暂停舵机
void duoji_Continue(uint16_t id); // 继续舵机
void duoji_Stop(uint16_t id); // 停止舵机（不可继续）
void duoji_Set_Baud(uint16_t id, duoji_Baud_t baud); // 设置波特率
void duoji_Calib_Mid(uint16_t id); // 矫正中值1500
void duoji_Set_StartPos(uint16_t id, uint16_t pwm); // 设置初始值
void duoji_Set_PwrMode(uint16_t id, duoji_PwrMode_t mode); // 设置开机模式
void duoji_Read_PwrMode(uint16_t id); // 读取开机模式
void duoji_Set_Min(uint16_t id); // 设置当前位置为最小值
void duoji_Set_Max(uint16_t id); // 设置当前位置为最大值
void duoji_Reset_ExceptID(uint16_t id); // 除ID外恢复出厂
void duoji_Reset_All(uint16_t id); // 全恢复出厂
void duoji_Read_Temp_Volt(uint16_t id); // 读取温度和电压

/* 转盘舵机专用API */
uint16_t duoji_Turntable_Get_Current_PWM(void);
void duoji_Turntable_Sync_Current_PWM(uint16_t pwm);
void duoji_Turntable_Reset(void);
void duoji_Turntable_Set_Start_Position(void);
void duoji_Turntable_Sync_Start_Position(void);

/* 舵机1/2角度控制API */
void duoji_Set_ID1_Angle(float angle_deg);
void duoji_Set_ID2_Angle(float angle_deg);
void duoji_Set_ID1_Angle_Time(float angle_deg, uint16_t time);
void duoji_Set_ID2_Angle_Time(float angle_deg, uint16_t time);


/* 参考 duoji.c 的转盘模式函数 */
void task1_1_step1(void);
void task1_1_step2(void);
void task1_1_step3(void);
void task1_1_step4(void);
void task1_1_finish(void);

void task1_2_step1(void);
void task1_2_step2(void);
void task1_2_step3(void);
void task1_2_step4(void);
void task1_2_step5(void);

void task2_1_step1(void);
void task2_1_step2(void);
void task2_1_step3(void);
void task2_2_step1(void);
void task2_2_step2(void);
void task2_2_step3(void);
void duoji_tc();
void yajun_1();
void yajun_2();
void guanjun_1();
void guanjun_2();
#endif /* __duoji_H */

