#ifndef __MOVE_H
#define __MOVE_H

#include <stdbool.h>
#include "main.h"

// 运动学参数结构体（针对X型布局）
typedef struct {
    float wheel_radius;  // 轮子半径 (米)
    float lx;           // X方向轮心到中心距离 (米) - 前后方向
    float ly;           // Y方向轮心到中心距离 (米) - 左右方向
    float layout_scale; // 布局缩放系数，用于调整轮子效应
} KinematicsParam_t;

// 运动类型枚举定义
typedef enum {
    MOVE_FORWARD = 0,    // 前进运动
    MOVE_BACKWARD = 1,   // 后退运动
    MOVE_LEFT = 2,       // 左平移运动
    MOVE_RIGHT = 3,      // 右平移运动
    ROTATE_CW = 4,       // 顺时针旋转
    ROTATE_CCW = 5,      // 逆时针旋转
    MOVE_DIAGONAL = 6,   // 对角线移动
    STOP = 7             // 停止运动
} MoveType_t;

//PID控制器结构体
typedef struct {
    float Kp;         //比例系数
    float Ki;         //积分系数
    float Kd;         //微分系数
    float error;      //当前误差
    float last_error; //上次误差（用于微分项）
    float integral;   //积分项（累计误差）
    float output;     //PID输出值
} PID_t;

// 初始化机器人运动控制系统
void Move_Init(void);

// 使能或禁用所有电机
void Move_EnableMotors(bool enable);

// 立即停止所有电机
void Move_StopAll(void);

void Move_RotateAngle(float angle_deg, uint8_t direction, uint16_t rpm, uint8_t acc);

// 闭环PID控制整车朝向（阻塞式）
void Move_RotateToYaw_PID(float target_yaw_deg, float kp, float ki, float kd, uint16_t max_rpm, uint8_t acc, uint32_t timeout_ms);
void Move_RotateCW_FromInitialYaw(float angle_deg);
void Move_RotateCW90_FromInitialYaw(void);

// 设置单个电机的脉冲数（直接控制）
// motor_id: 电机ID（1=右后, 2=左后, 3=左前, 4=右前）
// direction: 方向（0=CW, 1=CCW）
// pulses: 脉冲数
void Move_SetMotorPulses(uint8_t motor_id, uint8_t direction, uint32_t pulses);
void Move_TranslateForTime(float angle_deg, float velocity_mps, uint32_t duration_ms);
void Move_TranslateContinuous(float angle_deg, float velocity_mps);
void Move_CircleForTime(float radius, float velocity, uint8_t direction, uint32_t duration_ms);
void Move_StartCircleForTime(float radius, float velocity, uint8_t direction, uint32_t duration_ms);

// move.h 中已有的内容后添加

// 非阻塞运动状态
typedef enum {
    MOVE_IDLE,      // 空闲
    MOVE_RUNNING,   // 运动中
    MOVE_STOPPING   // 平滑停止中（可选）
} NonBlockMoveState_t;

// 全局运动状态变量（在 move.c 中定义）
extern NonBlockMoveState_t g_moveState;

/**
  * @brief  启动非阻塞平移运动指定时间
  * @param  angle_deg: 方向角度
  * @param  velocity_mps: 速度（米/秒）
  * @param  duration_ms: 运动时间（毫秒）
  */
void Move_StartTranslateForTime(float angle_deg, float velocity_mps, uint32_t duration_ms);

/**
  * @brief  更新运动状态，需在主循环中周期性调用（如每10ms）
  */
void Move_Update(void);
void Move_Circle(float radius, float velocity, uint8_t direction);
#endif /* __MOVE_H */


