#include "Emm42_Driver.h"
#include <string.h>

#define EMM42_CMD_POSITION      0xFD
#define EMM42_CMD_ENABLE        0xF3
#define EMM42_CMD_STOP          0xFE
#define EMM42_CMD_SYNC          0xFF
#define EMM42_SUBCODE_ENABLE    0xAB
#define EMM42_SUBCODE_STOP      0x98
#define EMM42_SUBCODE_SYNC      0x66

#define SYNC_DISABLE            0x00
#define SYNC_ENABLE             0x01
#define MODE_RELATIVE           0x00
#define DEFAULT_CHECKSUM        0x6B

/* 私有函数声明 */
static void BuildPositionCommand(uint8_t* buf, uint8_t addr, uint8_t dir, uint16_t speed, uint8_t accel, int32_t pulses, uint8_t sync_flag);
static bool SendCommand(Emm42_UART_Controller* ctrl, uint8_t* cmd, uint16_t len);
static uint32_t AngleToPulses(float angle);
static uint32_t RevolutionsToPulses(float revs);

/* 初始化控制器并设置电机ID */
void Emm42_Controller_Init(Emm42_UART_Controller* ctrl, UART_HandleTypeDef* huart, uint8_t id1, uint8_t id2) {
    ctrl->huart = huart;
    ctrl->motor_count = 0;
    ctrl->sync_ready = false;

    if(id1 > 0 && id1 <= 255) {
        ctrl->motor[0].addr = id1;
        ctrl->motor[0].speed_rpm = 500; // 默认速度
        ctrl->motor[0].accel = 10;      // 默认加速度
        ctrl->motor[0].enabled = false;
        ctrl->motor[0].is_moving = false;
        ctrl->motor_count++;
    }

    if(id2 > 0 && id2 <= 255 && id2 != id1) { // 确保ID不同
        ctrl->motor[1].addr = id2;
        ctrl->motor[1].speed_rpm = 500;
        ctrl->motor[1].accel = 10;
        ctrl->motor[1].enabled = false;
        ctrl->motor[1].is_moving = false;
        ctrl->motor_count++;
    }
}

/* 设置电机运动参数 */
void Emm42_SetMotorParams(Emm42_UART_Controller* ctrl, uint8_t motor_idx, uint16_t speed_rpm, uint8_t accel) {
    if(motor_idx < ctrl->motor_count) {
        if(speed_rpm > 0 && speed_rpm <= 3000) ctrl->motor[motor_idx].speed_rpm = speed_rpm;
        if(accel <= 255) ctrl->motor[motor_idx].accel = accel;
    }
}

/* 构建位置命令 (核心) */
static void BuildPositionCommand(uint8_t* buf, uint8_t addr, uint8_t dir, uint16_t speed, uint8_t accel, int32_t pulses, uint8_t sync_flag) {
    uint32_t abs_pulses = pulses > 0 ? pulses : -pulses;

    buf[0] = addr;
    buf[1] = EMM42_CMD_POSITION;
    buf[2] = dir;
    buf[3] = (uint8_t)(speed >> 8);
    buf[4] = (uint8_t)(speed & 0xFF);
    buf[5] = accel;
    buf[6] = (uint8_t)((abs_pulses >> 24) & 0xFF);
    buf[7] = (uint8_t)((abs_pulses >> 16) & 0xFF);
    buf[8] = (uint8_t)((abs_pulses >> 8) & 0xFF);
    buf[9] = (uint8_t)(abs_pulses & 0xFF);
    buf[10] = MODE_RELATIVE;
    buf[11] = sync_flag;
    buf[12] = DEFAULT_CHECKSUM;
}

/* 发送命令到指定串口 */
static bool SendCommand(Emm42_UART_Controller* ctrl, uint8_t* cmd, uint16_t len) {
    if(ctrl->huart == NULL) return false;
    
    HAL_StatusTypeDef status = HAL_UART_Transmit(ctrl->huart, cmd, len, 100);
    return (status == HAL_OK);
}

/* 单电机相对运动 */
bool Emm42_MoveRel(Emm42_UART_Controller* ctrl, uint8_t motor_idx, int32_t pulses, uint8_t dir) {
    if(motor_idx >= ctrl->motor_count || !ctrl->motor[motor_idx].enabled) return false;

    uint8_t cmd[13];
    uint16_t speed = ctrl->motor[motor_idx].speed_rpm;
    uint8_t accel = ctrl->motor[motor_idx].accel;
    
    BuildPositionCommand(cmd, ctrl->motor[motor_idx].addr, dir, speed, accel, pulses, SYNC_DISABLE);
    ctrl->motor[motor_idx].is_moving = true;
    ctrl->motor[motor_idx].target_pos += (dir == 0 ? pulses : -pulses);
    
    return SendCommand(ctrl, cmd, 13);
}

/* 组内同步运动 (核心功能) */
bool Emm42_MoveRel_Sync(Emm42_UART_Controller* ctrl, int32_t* pulses_arr, uint8_t* dir_arr) {
    if(ctrl->motor_count == 0) return false;

    // 1. 发送所有电机的命令，设置同步标志但不执行
    for(uint8_t i = 0; i < ctrl->motor_count; i++) {
        if(!ctrl->motor[i].enabled) continue;
        
        uint8_t cmd[13];
        uint16_t speed = ctrl->motor[i].speed_rpm;
        uint8_t accel = ctrl->motor[i].accel;
        
        BuildPositionCommand(cmd, ctrl->motor[i].addr, dir_arr[i], speed, accel, pulses_arr[i], SYNC_ENABLE);
        
        if(!SendCommand(ctrl, cmd, 13)) return false;
        HAL_Delay(5); // 命令间微小延时
        
        ctrl->motor[i].is_moving = true;
        ctrl->motor[i].target_pos += (dir_arr[i] == 0 ? pulses_arr[i] : -pulses_arr[i]);
    }
    
    ctrl->sync_ready = true;
    return true;
}

/* 触发同步运动 */
bool Emm42_TriggerSync(Emm42_UART_Controller* ctrl) {
    if(!ctrl->sync_ready) return false;
    
    uint8_t sync_cmd[] = {0x00, EMM42_CMD_SYNC, EMM42_SUBCODE_SYNC, DEFAULT_CHECKSUM};
    bool result = SendCommand(ctrl, sync_cmd, 4);
    
    if(result) {
        ctrl->sync_ready = false;
        // 可以在这里启动一个定时器，用于后续检测运动完成
    }
    
    return result;
}

/* 单位转换辅助函数 */
static uint32_t AngleToPulses(float angle) {
    // 16细分下: 1圈(360度)=3200脉冲
    return (uint32_t)((angle / 360.0f) * 3200.0f);
}

static uint32_t RevolutionsToPulses(float revs) {
    return (uint32_t)(revs * 3200.0f);
}

/* 角度控制接口 */
bool Emm42_MoveAngle(Emm42_UART_Controller* ctrl, uint8_t motor_idx, float angle) {
    uint32_t pulses = AngleToPulses(angle > 0 ? angle : -angle);
    uint8_t dir = (angle >= 0) ? 0 : 1;
    return Emm42_MoveRel(ctrl, motor_idx, pulses, dir);
}

/* 圈数控制接口 */
bool Emm42_MoveRevolutions(Emm42_UART_Controller* ctrl, uint8_t motor_idx, float revs) {
    uint32_t pulses = RevolutionsToPulses(revs > 0 ? revs : -revs);
    uint8_t dir = (revs >= 0) ? 0 : 1;
    return Emm42_MoveRel(ctrl, motor_idx, pulses, dir);
}

/* 使能/失能电机 */
bool Emm42_EnableMotor(Emm42_UART_Controller* ctrl, uint8_t motor_idx, bool enable) {
    if(motor_idx >= ctrl->motor_count) return false;
    
    uint8_t cmd[] = {
        ctrl->motor[motor_idx].addr,
        EMM42_CMD_ENABLE,
        EMM42_SUBCODE_ENABLE,
        enable ? 0x01 : 0x00,
        0x00, // 同步标志
        DEFAULT_CHECKSUM
    };
    
    if(SendCommand(ctrl, cmd, 6)) {
        ctrl->motor[motor_idx].enabled = enable;
        return true;
    }
    return false;
}

/* 紧急停止 */
bool Emm42_StopMotor(Emm42_UART_Controller* ctrl, uint8_t motor_idx) {
    if(motor_idx >= ctrl->motor_count) return false;
    
    uint8_t cmd[] = {
        ctrl->motor[motor_idx].addr,
        EMM42_CMD_STOP,
        EMM42_SUBCODE_STOP,
        0x00, // 同步标志
        DEFAULT_CHECKSUM
    };
    
    if(SendCommand(ctrl, cmd, 5)) {
        ctrl->motor[motor_idx].is_moving = false;
        return true;
    }
    return false;
}

bool Emm42_SpeedMode(Emm42_UART_Controller* ctrl, uint8_t motor_idx, uint16_t rpm, uint8_t dir, uint8_t accel, bool sync_enable) {
    if(motor_idx >= ctrl->motor_count) return false;

    uint8_t cmd[] = {
        ctrl->motor[motor_idx].addr,
        0xF6,
        dir,
        (uint8_t)(rpm >> 8),
        (uint8_t)(rpm & 0xFF),
        accel,
        sync_enable ? 0x01 : 0x00,
        0x6B
    };
    return SendCommand(ctrl, cmd, sizeof(cmd));
}
