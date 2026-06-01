#ifndef EMM42_DRIVER_H
#define EMM42_DRIVER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* 电机控制参数结构体 */
typedef struct {
    uint8_t addr;      // 驱动器地址 (1-255)
    int32_t target_pos; // 目标位置 (脉冲)
    int32_t current_pos;// 当前位置 (脉冲)
    uint16_t speed_rpm; // 运动速度 (RPM)
    uint8_t accel;      // 加速度档位 (0-255)
    uint8_t dir;        // 方向 (0=CW, 1=CCW)
    bool enabled;       // 使能状态
    bool is_moving;     // 运动状态
} Motor_HandleTypeDef;

/* 串口控制器结构体 (管理一组电机) */
typedef struct {
    UART_HandleTypeDef* huart;          // 指向的串口句柄，如 &huart4
    Motor_HandleTypeDef motor[2];       // 该串口控制的两个电机
    uint8_t motor_count;                // 实际管理的电机数量
    bool sync_ready;                    // 同步准备标志
} Emm42_UART_Controller;

/* 初始化与配置 */
void Emm42_Controller_Init(Emm42_UART_Controller* ctrl, UART_HandleTypeDef* huart, uint8_t id1, uint8_t id2);
void Emm42_SetMotorParams(Emm42_UART_Controller* ctrl, uint8_t motor_idx, uint16_t speed_rpm, uint8_t accel);

/* 基础运动控制 (相对位置) */
bool Emm42_MoveRel(Emm42_UART_Controller* ctrl, uint8_t motor_idx, int32_t pulses, uint8_t dir);
bool Emm42_MoveRel_Sync(Emm42_UART_Controller* ctrl, int32_t* pulses_arr, uint8_t* dir_arr);

/* 角度/圈数控制 (用户友好) */
bool Emm42_MoveAngle(Emm42_UART_Controller* ctrl, uint8_t motor_idx, float angle);
bool Emm42_MoveRevolutions(Emm42_UART_Controller* ctrl, uint8_t motor_idx, float revs);

/* 同步与状态控制 */
bool Emm42_TriggerSync(Emm42_UART_Controller* ctrl);
bool Emm42_StopMotor(Emm42_UART_Controller* ctrl, uint8_t motor_idx);
bool Emm42_EnableMotor(Emm42_UART_Controller* ctrl, uint8_t motor_idx, bool enable);
bool Emm42_SpeedMode(Emm42_UART_Controller* ctrl, uint8_t motor_idx, uint16_t rpm, uint8_t dir, uint8_t accel, bool sync_enable);
#endif /* EMM42_DRIVER_H */
