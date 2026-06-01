#include "move.h"
#include "Emm_V5.h"
#include "jy61p.h"
#include <math.h>

static KinematicsParam_t g_kinematics_param;
// 全局运动学参数结构体
static KinematicsParam_t g_kinematics;

// 机器人初始化标志
static bool g_robot_initialized = false;

// 电机地址定义
#define MOTOR_RF 4  // 右前电机
#define MOTOR_LF 3  // 左前电机
#define MOTOR_LB 2  // 左后电机
#define MOTOR_RB 1  // 右后电机

#define MOTOR_DIR_FORWARD_14 1U
#define MOTOR_DIR_FORWARD_23 0U

// 机械参数常量
#define WHEEL_RADIUS 0.0385f
#define WHEEL_BASE_LX 0.0950f   // 半轴距（前后）
#define WHEEL_BASE_LY 0.0800f   // 半轴距（左右）
#define LAYOUT_SCALE 1.0f

// 默认运动参数
#define DEFAULT_VELOCITY_RPM 500
#define DEFAULT_ACCELERATION 50
#define PULSE_PER_REV 3200

// 私有函数声明
static void Robot_MoveForward(float velocity, float distance);
static void Robot_MoveBackward(float velocity, float distance);
static void Robot_MoveLeft(float velocity, float distance);
static void Robot_MoveRight(float velocity, float distance);
static void Robot_RotateCW(float angular_velocity, float angle_degrees);
static void Robot_RotateCCW(float angular_velocity, float angle_degrees);
static void Robot_MoveDiagonal(float angle_degrees, float velocity, float distance);
static uint32_t CalculatePulsesFromDistance(float distance);
static void Move_SendAllEnableSplit(bool enable);
static void Move_SendAllPositionSplit(uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses);
static void Move_SendSinglePosition(uint8_t addr, uint8_t direction, uint32_t pulses);
static void Move_SendAllVelocitySplit(uint8_t dir_lf, uint16_t rpm_lf,
                                      uint8_t dir_rf, uint16_t rpm_rf,
                                      uint8_t dir_lb, uint16_t rpm_lb,
                                      uint8_t dir_rb, uint16_t rpm_rb,
                                      uint8_t acc);

static uint8_t Motor_GetForwardDir(uint8_t motor_addr)
{
    return ((motor_addr == MOTOR_RF) || (motor_addr == MOTOR_RB)) ? MOTOR_DIR_FORWARD_14 : MOTOR_DIR_FORWARD_23;
}

static uint8_t Motor_GetDirFromVelocity(uint8_t motor_addr, float wheel_velocity)
{
    uint8_t forward_dir = Motor_GetForwardDir(motor_addr);
    return (wheel_velocity >= 0.0f) ? forward_dir : (uint8_t)(1U - forward_dir);
}

static void Move_SendAllEnableSplit(bool enable)
{
    MMCL_count = 0;
    Emm_V5_MMCL_En_Control(MOTOR_LF, enable, true);
    Emm_V5_MMCL_En_Control(MOTOR_LB, enable, true);
    Emm_V5_Multi_Motor_Cmd_UART4(0);
    HAL_Delay(1);

    MMCL_count = 0;
    Emm_V5_MMCL_En_Control(MOTOR_RF, enable, true);
    Emm_V5_MMCL_En_Control(MOTOR_RB, enable, true);
    Emm_V5_Multi_Motor_Cmd_UART5(0);
}

static void Move_SendAllPositionSplit(uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses)
{
    MMCL_count = 0;
    Emm_V5_MMCL_Pos_Control(MOTOR_LF, dir, rpm, acc, pulses, false, true);
    Emm_V5_MMCL_Pos_Control(MOTOR_LB, dir, rpm, acc, pulses, false, true);
    Emm_V5_Multi_Motor_Cmd_UART4(0);
    HAL_Delay(1);

    MMCL_count = 0;
    Emm_V5_MMCL_Pos_Control(MOTOR_RF, dir, rpm, acc, pulses, false, true);
    Emm_V5_MMCL_Pos_Control(MOTOR_RB, dir, rpm, acc, pulses, false, true);
    Emm_V5_Multi_Motor_Cmd_UART5(0);
}

static void Move_SendSinglePosition(uint8_t addr, uint8_t direction, uint32_t pulses)
{
    MMCL_count = 0;
    Emm_V5_MMCL_Pos_Control(addr, direction, DEFAULT_VELOCITY_RPM, DEFAULT_ACCELERATION, pulses, false, true);

    if ((addr == MOTOR_LF) || (addr == MOTOR_LB)) {
        Emm_V5_Multi_Motor_Cmd_UART4(0);
    } else {
        Emm_V5_Multi_Motor_Cmd_UART5(0);
    }
}

static void Move_SendAllVelocitySplit(uint8_t dir_lf, uint16_t rpm_lf,
                                      uint8_t dir_rf, uint16_t rpm_rf,
                                      uint8_t dir_lb, uint16_t rpm_lb,
                                      uint8_t dir_rb, uint16_t rpm_rb,
                                      uint8_t acc)
{
    MMCL_count = 0;
    Emm_V5_MMCL_Vel_Control(MOTOR_LF, dir_lf, rpm_lf, acc, true);
    Emm_V5_MMCL_Vel_Control(MOTOR_LB, dir_lb, rpm_lb, acc, true);
    Emm_V5_Multi_Motor_Cmd_UART4(0);
    HAL_Delay(1);

    MMCL_count = 0;
    Emm_V5_MMCL_Vel_Control(MOTOR_RF, dir_rf, rpm_rf, acc, true);
    Emm_V5_MMCL_Vel_Control(MOTOR_RB, dir_rb, rpm_rb, acc, true);
    Emm_V5_Multi_Motor_Cmd_UART5(0);
}
void Mecanum_Initq(KinematicsParam_t* param, float radius, float lx, float ly, float layout_scale) 
{
    param->wheel_radius = radius;
    param->lx = lx;
    param->ly = ly;
    param->layout_scale = (layout_scale > 0) ? layout_scale : 1.0f;
    g_kinematics_param = *param;
}

void Move_Init(void)
{
    Mecanum_Initq(&g_kinematics, 
                 WHEEL_RADIUS,
                 WHEEL_BASE_LX,
                 WHEEL_BASE_LY,
                 LAYOUT_SCALE);
}

void Move_EnableMotors(bool enable)
{
    Move_SendAllEnableSplit(enable);
}

void Move_StopAll(void)
{
    Move_SendAllVelocitySplit(0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void Move_RotateAngle(float angle_deg, uint8_t direction, uint16_t rpm, uint8_t acc)
{
     if (angle_deg <= 0.0f) return;
    if (angle_deg > 360.0f) angle_deg = 360.0f;

    // 重新标定的比例：3200脉冲转97度 => 32.99脉冲/度
    #define PULSES_PER_DEGREE (3200.0f / 92.0f)
    uint32_t pulses = (uint32_t)(angle_deg * PULSES_PER_DEGREE + 0.5f);
    if (pulses == 0) return;

    uint8_t dir = (direction == 0) ? 0 : 1;

    Move_SendAllPositionSplit(dir, rpm, acc, pulses);

    uint32_t move_time_ms = (uint32_t)((float)pulses * 60.0f * 1000.0f / ((float)rpm * 3200.0f));
    HAL_Delay(move_time_ms + 200);
}

void Move_SetMotorPulses(uint8_t motor_id, uint8_t direction, uint32_t pulses)
{
    if (!g_robot_initialized) {
        Move_Init();
    }
    
    uint8_t addr = 0;
    switch(motor_id) {
        case 1: addr = MOTOR_RB; break;
        case 2: addr = MOTOR_LB; break;
        case 3: addr = MOTOR_LF; break;
        case 4: addr = MOTOR_RF; break;
        default: return;
    }
    
    Move_SendSinglePosition(addr, direction, pulses);
}

static uint32_t CalculatePulsesFromDistance(float distance)
{
    float wheel_circumference = 2.0f * 3.1415926f * WHEEL_RADIUS;
    float wheel_revolutions = distance / wheel_circumference;
    return (uint32_t)(wheel_revolutions * PULSE_PER_REV);
}

static uint16_t VelocityToRPM(float velocity_mps)
{
    float wheel_circumference = 2.0f * 3.1415926f * WHEEL_RADIUS;
    float rpm_float = (velocity_mps / wheel_circumference) * 60.0f;
    return (uint16_t)fabs(rpm_float);
}

void Move_TranslateContinuous(float angle_deg, float velocity_mps)
{
    float rad = angle_deg * 3.1415926f / 180.0f;
    float vx = velocity_mps * cosf(rad);
    float vy = velocity_mps * sinf(rad);

    float v_lf = vx - vy;
    float v_rf = vx + vy;
    float v_lb = vx + vy;
    float v_rb = vx - vy;

    uint16_t rpm_lf = VelocityToRPM(fabsf(v_lf));
    uint16_t rpm_rf = VelocityToRPM(fabsf(v_rf));
    uint16_t rpm_lb = VelocityToRPM(fabsf(v_lb));
    uint16_t rpm_rb = VelocityToRPM(fabsf(v_rb));

    uint8_t dir_lf = Motor_GetDirFromVelocity(MOTOR_LF, v_lf);
    uint8_t dir_rf = Motor_GetDirFromVelocity(MOTOR_RF, v_rf);
    uint8_t dir_lb = Motor_GetDirFromVelocity(MOTOR_LB, v_lb);
    uint8_t dir_rb = Motor_GetDirFromVelocity(MOTOR_RB, v_rb);

    Move_SendAllVelocitySplit(dir_lf, rpm_lf,
                              dir_rf, rpm_rf,
                              dir_lb, rpm_lb,
                              dir_rb, rpm_rb,
                              DEFAULT_ACCELERATION);
}

void Move_SmoothStop(void)
{
    uint16_t rpm = 0;
    uint8_t acc = DEFAULT_ACCELERATION;

    Move_SendAllVelocitySplit(0, rpm, 0, rpm, 0, rpm, 0, rpm, acc);
}



void Move_Circle(float radius, float velocity, uint8_t direction)
{
    if (radius <= 0.0f || velocity <= 0.0f) return;

    float omega = velocity / radius;
    if (direction == 0) {
        omega = -omega;
    }

    float L = g_kinematics.lx + g_kinematics.ly;

    float v_lf = velocity - 0.0f - L * omega;
    float v_rf = velocity + 0.0f + L * omega;
    float v_lb = velocity + 0.0f - L * omega;
    float v_rb = velocity - 0.0f + L * omega;

    uint16_t rpm_lf = VelocityToRPM(fabsf(v_lf));
    uint16_t rpm_rf = VelocityToRPM(fabsf(v_rf));
    uint16_t rpm_lb = VelocityToRPM(fabsf(v_lb));
    uint16_t rpm_rb = VelocityToRPM(fabsf(v_rb));

    uint8_t dir_lf = Motor_GetDirFromVelocity(MOTOR_LF, v_lf);
    uint8_t dir_rf = Motor_GetDirFromVelocity(MOTOR_RF, v_rf);
    uint8_t dir_lb = Motor_GetDirFromVelocity(MOTOR_LB, v_lb);
    uint8_t dir_rb = Motor_GetDirFromVelocity(MOTOR_RB, v_rb);

    Move_SendAllVelocitySplit(dir_lf, rpm_lf,
                              dir_rf, rpm_rf,
                              dir_lb, rpm_lb,
                              dir_rb, rpm_rb,
                              DEFAULT_ACCELERATION);
}


void Move_TranslateForTime(float angle_deg, float velocity_mps, uint32_t duration_ms)
{
    Move_TranslateContinuous(angle_deg, velocity_mps);
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < duration_ms) {
        HAL_Delay(10);
    }
    Move_StopAll();
}
void Move_CircleForTime(float radius, float velocity, uint8_t direction, uint32_t duration_ms)
{
    Move_Circle(radius, velocity, direction);
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < duration_ms) {
        HAL_Delay(10);
    }
    Move_StopAll();
}


static float NormalizeAngleDeg(float angle_deg)
{
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

void Move_RotateToYaw_PID(float target_yaw_deg, float kp, float ki, float kd, uint16_t max_rpm, uint8_t acc, uint32_t timeout_ms)
{
    float yaw_zero = sensorData.yaw;

    uint32_t start = HAL_GetTick();
    uint32_t last = start;
    float error = 0.0f;
    float last_error = 0.0f;
    float integral = 0.0f;
    const float tol_deg = 0.8f;

    while (1)
    {
        uint32_t now = HAL_GetTick();
        float dt = (now - last) / 1000.0f;
        if (dt <= 0.0f) dt = 0.001f;

        float current_yaw = NormalizeAngleDeg(sensorData.yaw - yaw_zero);
        error = NormalizeAngleDeg(target_yaw_deg - current_yaw);
        if (fabsf(error) <= tol_deg)
        {
            Move_StopAll();
            break;
        }

        integral += error * dt;
        float derivative = (error - last_error) / dt;
        float output = kp * error + ki * integral + kd * derivative;

        float rpm_f = fabsf(output);
        if (rpm_f > (float)max_rpm) rpm_f = (float)max_rpm;
        if (rpm_f < 0.0f) rpm_f = 0.0f;
        uint16_t rpm = (uint16_t)rpm_f;

        uint8_t dir_lf = (output >= 0.0f) ? 1 : 0;
        uint8_t dir_rf = (output >= 0.0f) ? 1 : 0;
        uint8_t dir_lb = (output >= 0.0f) ? 1 : 0;
        uint8_t dir_rb = (output >= 0.0f) ? 1 : 0;
        uint16_t rpm_lf = rpm;
        uint16_t rpm_rf = rpm;
        uint16_t rpm_lb = rpm;
        uint16_t rpm_rb = rpm;

        Move_SendAllVelocitySplit(dir_lf, rpm_lf,
                                  dir_rf, rpm_rf,
                                  dir_lb, rpm_lb,
                                  dir_rb, rpm_rb,
                                  acc);

        last_error = error;
        last = now;

        if (timeout_ms > 0 && (now - start) >= timeout_ms)
        {
            Move_StopAll();
            break;
        }

        HAL_Delay(10);
    }
}
// 运动类型枚举
typedef enum {
    MOTION_NONE,
    MOTION_TRANSLATE,
    MOTION_CIRCLE
} MotionType_t;

// 非阻塞运动状态（合并）
static MotionType_t g_currentMotion = MOTION_NONE;
static float s_moveAngle = 0.0f;
static float s_moveVelocity = 0.0f;
static float s_circleRadius = 0.0f;
static uint8_t s_circleDirection = 0;
static uint32_t s_motionStartTick = 0;
static uint32_t s_motionDuration = 0;

// 全局运动状态标志（0=运动中，1=空闲）
uint8_t g_motionActive = 1;  // 初始为空闲

/**
  * @brief  启动非阻塞平移运动
  */

/**
  * @brief  启动非阻塞绕圈运动
  */
void Move_StartTranslateForTime(float angle_deg, float velocity_mps, uint32_t duration_ms)
{
    if (g_currentMotion != MOTION_NONE) {
        Move_StopAll();
        g_currentMotion = MOTION_NONE;
        g_motionActive = 1;
        HAL_Delay(50);
    }

    Move_TranslateContinuous(angle_deg, velocity_mps);

    g_currentMotion = MOTION_TRANSLATE;
    s_moveAngle = angle_deg;
    s_moveVelocity = velocity_mps;
    s_motionStartTick = HAL_GetTick();
    s_motionDuration = duration_ms;
    g_motionActive = 0;
}

/**
  * @brief  启动非阻塞绕圈运动
  */
void Move_StartCircleForTime(float radius, float velocity, uint8_t direction, uint32_t duration_ms)
{
    if (g_currentMotion != MOTION_NONE) {
        Move_StopAll();
        g_currentMotion = MOTION_NONE;
        g_motionActive = 1;
        HAL_Delay(50);
    }

    Move_Circle(radius, velocity, direction);

    g_currentMotion = MOTION_CIRCLE;
    s_circleRadius = radius;
    s_moveVelocity = velocity;
    s_circleDirection = direction;
    s_motionStartTick = HAL_GetTick();
    s_motionDuration = duration_ms;
    g_motionActive = 0;
}


/**
  * @brief  启动向指定向量位移
  */
void Move_SpecifyVector(uint8_t place[]){
	PID_t PID;
	
	uint8_t x = 100*(place[0] - '0') + 10*(place[1] - '0') + place[2] - '0';  //x轴坐标
	uint8_t y = 100*(place[3] - '0') + 10*(place[4] - '0') + place[5] - '0';  //y轴坐标
	float angle_deg = atan2((float)y,(float)x);    //位移角
	if(x==0&&y==0){
		Move_StopAll();
	}
	PID.last_error = PID.error;
    PID.error = sqrt(pow(x,2) + pow(y,2));
    float velocity_mps = (PID.Kp * PID.error) + (PID.Ki * PID.integral) +  PID.Kd * (PID.error - PID.last_error);
	if(velocity_mps>=0.5) velocity_mps = 0.5;  //上限限幅
	if(velocity_mps<=0.1) velocity_mps = 0.1;  //下限限幅
	uint32_t duration_ms = PID.error/velocity_mps;
	Move_StartTranslateForTime(angle_deg,velocity_mps, duration_ms);
		
//	while(1){
//	uint8_t x = 100*(place[0] - '0') + 10*(place[1] - '0') + place[2] - '0';  //x轴坐标
//	uint8_t y = 100*(place[3] - '0') + 10*(place[4] - '0') + place[5] - '0';  //y轴坐标
//	if(x==0&&y==0){Move_StopAll();	break;}
//	float angle_deg = atan2((float)y,(float)x);      		   //位移角
//	float displacement = sqrt(pow(x,2) + pow(y,2));  		   //位移距离
//	uint32_t duration_ms = displacement/0.2;                   //位移时间
//	Move_StartTranslateForTime(angle_deg, 0.2, duration_ms);  //定速度0.2m/s
//	}
}

/**
  * @brief  更新运动状态，需在主循环中周期性调用（如每10ms）
  */
void Move_Update(void)
{
    if (g_currentMotion == MOTION_NONE) return;

    uint32_t now = HAL_GetTick();
    if (now - s_motionStartTick >= s_motionDuration) {
        // 时间到，平滑停止
        Move_StopAll();
        g_currentMotion = MOTION_NONE;
        g_motionActive = 1;  // 运动结束，变为空闲
    }
}

/**
  * @brief  查询运动是否空闲（空闲返回1，运动中返回0）
  */
uint8_t Move_IsIdle(void)
{
    return g_motionActive;
}









