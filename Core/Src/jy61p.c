#include "JY61P.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "usart.h"
#include "oled.h"

// ==================== 协议相关定义 ====================
#define PACKET_HEADER 0x55
#define PACKET_SIZE 11

// 转换系数
#define ACC_SCALE (16.0f * 9.8f / 32768.0f)    // 加速度量程16g
#define GYRO_SCALE (2000.0f / 32768.0f)       // 角速度量程2000°/s
#define ANGLE_SCALE (180.0f / 32768.0f)       // 角度量程±180°

// 低通滤波器系数 (0 < ALPHA < 1, 越小滤波越强)
// 注意：如果加速过程非常短暂（如<0.2秒），请适当增大ALPHA，例如0.6~0.8，以减少信号衰减
#define LPF_ALPHA 0.6f   // 修改为0.6，平衡噪声抑制和响应速度

// 重力加速度 (m/s2)
#define GRAVITY 9.8f

// ==================== 全局变量 ====================
SensorData_t sensorData = {0};
extern uint8_t accValid;
uint8_t gyroValid = 0;
uint8_t angleValid = 0;
uint32_t lastPacketTime = 0;
extern char displayBuffer[];  // OLED显示缓冲区

// 数据包解析状态机
static ParserState_t parserState = STATE_WAIT_HEADER;

// 积分数据结构（导航坐标系）
typedef struct {
    float vx, vy, vz;          // 速度 (m/s) - 导航坐标系
    float sx, sy, sz;          // 位移 (m)   - 导航坐标系
    uint32_t last_time;        // 上一次积分的时间戳 (ms)
    float last_ax_nav, last_ay_nav, last_az_nav; // 上一次导航加速度，用于梯形积分
    uint8_t first_frame;       // 是否为第一帧数据
} IntegrateData_t;

static IntegrateData_t integ = {0};

// 零偏校准值（载体坐标系）
#define CALIB_SAMPLES   200
#define CALIB_TIMEOUT   5000
float offset_x = 0.0f, offset_y = 0.0f, offset_z = 0.0f;

// 连续静止计数器（用于零速检测）
static uint8_t stationary_count = 0;
#define STATIONARY_THRESHOLD 3      // 连续3帧满足条件才认为静止
#define GYRO_STATIC_THRESH 1.0f     // 角速度静止阈值 (°/s)

// ==================== 静态校准函数 ====================
void JY61P_CalibrateAccel(void)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint32_t count = 0;
    uint32_t start_tick = HAL_GetTick();

    while (count < CALIB_SAMPLES) {
        if (accValid) {
            sum_x += sensorData.ax;
            sum_y += sensorData.ay;
            sum_z += sensorData.az;
            count++;
            accValid = 0;
        }

        if (HAL_GetTick() - start_tick > CALIB_TIMEOUT) {
            if (count > 0) {
                offset_x = sum_x / count;
                offset_y = sum_y / count;
                offset_z = sum_z / count;
            } else {
                offset_x = offset_y = offset_z = 0.0f;
            }
            return;
        }
        HAL_Delay(1);
    }

    offset_x = sum_x / CALIB_SAMPLES;
    offset_y = sum_y / CALIB_SAMPLES;
    offset_z = sum_z / CALIB_SAMPLES;
}

// ==================== 传感器通信函数 ====================
void JY61P_Unlock(void)
{
    uint8_t unlockCmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    HAL_UART_Transmit(&huart3, unlockCmd, sizeof(unlockCmd), 100);
    HAL_Delay(200);
}

void JY61P_Save(void)
{
    uint8_t saveCmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    HAL_UART_Transmit(&huart3, saveCmd, sizeof(saveCmd), 100);
}

void JY61P_WriteRegister(uint8_t addr, int16_t data)
{
    uint8_t cmd[5] = {0xFF, 0xAA, addr, (uint8_t)(data & 0xFF), (uint8_t)(data >> 8)};
    HAL_UART_Transmit(&huart3, cmd, 5, 100);
}

void JY61P_ReadRegister(uint8_t addr) { /* 如有需要可实现 */ }

void JY61P_RequestData(uint8_t type)
{
    uint8_t cmd[] = {0xFF, 0xAA, 0x27, type, 0x00};
    HAL_UART_Transmit(&huart3, cmd, sizeof(cmd), 100);
}

void JY61P_InitConfig(void)
{
    JY61P_Unlock();
    JY61P_WriteRegister(0x03, 0x09);   // 100Hz
    JY61P_Save();
    HAL_Delay(100);
}

// ==================== 数据处理函数 ====================
int16_t BytesToInt16(uint8_t low, uint8_t high)
{
    return (int16_t)((high << 8) | low);
}

void ProcessReceivedData(uint8_t data)
{
    static uint8_t packet[PACKET_SIZE];
    static uint8_t packetIndex = 0;

    switch(parserState)
    {
        case STATE_WAIT_HEADER:
            if(data == PACKET_HEADER) {
                packet[0] = data;
                packetIndex = 1;
                parserState = STATE_WAIT_TYPE;
            }
            break;
        case STATE_WAIT_TYPE:
            packet[1] = data;
            packetIndex = 2;
            parserState = STATE_WAIT_DATA;
            break;
        case STATE_WAIT_DATA:
            packet[packetIndex++] = data;
            if(packetIndex >= PACKET_SIZE) {
                JY61P_ParsePacket(packet);
                parserState = STATE_WAIT_HEADER;
                packetIndex = 0;
                lastPacketTime = HAL_GetTick();
            }
            break;
    }
}

void JY61P_ParsePacket(uint8_t *packet)
{
    if(packet[0] != PACKET_HEADER) return;

    uint8_t checksum = 0;
    for(int i = 0; i < PACKET_SIZE - 1; i++) checksum += packet[i];
    if(checksum != packet[PACKET_SIZE - 1]) return;

    uint8_t type = packet[1];

    switch(type)
    {
        case PACKET_ACCEL:
        {
            int16_t ax = BytesToInt16(packet[2], packet[3]);
            int16_t ay = BytesToInt16(packet[4], packet[5]);
            int16_t az = BytesToInt16(packet[6], packet[7]);
            int16_t temp = BytesToInt16(packet[8], packet[9]);

            sensorData.ax = ax * ACC_SCALE;
            sensorData.ay = ay * ACC_SCALE;
            sensorData.az = az * ACC_SCALE;
            sensorData.temperature = temp / 100.0f;
            accValid = 1;
            break;
        }
        case PACKET_GYRO:
        {
            int16_t wx = BytesToInt16(packet[2], packet[3]);
            int16_t wy = BytesToInt16(packet[4], packet[5]);
            int16_t wz = BytesToInt16(packet[6], packet[7]);

            sensorData.wx = wx * GYRO_SCALE;
            sensorData.wy = wy * GYRO_SCALE;
            sensorData.wz = wz * GYRO_SCALE;
            gyroValid = 1;
            break;
        }
        case PACKET_ANGLE:
        {
            int16_t roll = BytesToInt16(packet[2], packet[3]);
            int16_t pitch = BytesToInt16(packet[4], packet[5]);
            int16_t yaw = BytesToInt16(packet[6], packet[7]);

            sensorData.roll = roll * ANGLE_SCALE;
            sensorData.pitch = pitch * ANGLE_SCALE;
            sensorData.yaw = yaw * ANGLE_SCALE;
            angleValid = 1;
            break;
        }
        default: break;
    }
}



// ==================== OLED显示 ====================
void UpdateDisplay(void)
{
    // 第一行：加速度 X / 角速度 X
    sprintf(displayBuffer, "AX:%.2f", sensorData.ax);
    OLED_ShowString(1, 1, displayBuffer);
    sprintf(displayBuffer, "WX:%.1f", sensorData.wx);
    OLED_ShowString(1, 9, displayBuffer);

    // 第二行：加速度 Y / 角速度 Y
    sprintf(displayBuffer, "AY:%.2f", sensorData.ay);
    OLED_ShowString(2, 1, displayBuffer);
    sprintf(displayBuffer, "WY:%.1f", sensorData.wy);
    OLED_ShowString(2, 9, displayBuffer);

    // 第三行：加速度 Z / 角速度 Z
    sprintf(displayBuffer, "AZ:%.2f", sensorData.az);
    OLED_ShowString(3, 1, displayBuffer);
    sprintf(displayBuffer, "WZ:%.1f", sensorData.wz);
    OLED_ShowString(3, 9, displayBuffer);

    // 第四行：显示 X 轴位移（导航坐标系）
    sprintf(displayBuffer, "SX:%.2f m", integ.sx);
    OLED_ShowString(4, 1, displayBuffer);
    // 可选显示速度
    // sprintf(displayBuffer, "VX:%.2f", integ.vx);
    // OLED_ShowString(4, 9, displayBuffer);
}