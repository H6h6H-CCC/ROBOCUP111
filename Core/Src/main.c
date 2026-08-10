/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "jy61p.h"
#include "EMM_V5.H"
#include "move.H"
#include "gray.H"
#include "string.h"
#include "math.h"
#include "state.h"
#include "duoji.h"
#include "shijue.h"
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PACKET_TIMEOUT 100
#define DUOJI_BOOT_SETTLE_DELAY_MS 50U
#define DUOJI_BOOT_REHOME_DELAY_MS 450U
#define VISION_DEBUG_PRINT_PERIOD_MS 200U
#define VISION_YAJUN_ACTION_DELAY_MS 600
#define JY61P_RX_BUFFER_SIZE 64U
#define JY61P_DEBUG_PRINT_PERIOD_MS 1000U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t count;
uint8_t count1;
uint8_t count2;

uint8_t a;
uint8_t b;
KinematicsParam_t MyChassisParam; // 底盘运动学参�?
extern uint8_t g_motionActive;
extern uint8_t stat;
extern volatile uint32_t g_debug_io_mode;
uint8_t rxBuffer1[256];
uint8_t rxBuffer2[10];
uint8_t txBuffer2[10];
uint8_t rxBuffer3[100];
uint8_t rxBuffer4[256];
uint8_t rxBuffer5[256];
/* 修改：USART6 专用于接收 JY61P 连续数据流。 */
static uint8_t jy61p_rx_buffer[JY61P_RX_BUFFER_SIZE];
static uint8_t jy61p_debug_tx[160];
uint8_t QRPacke[2];
uint8_t QRPacke1[2];
uint8_t Coulor[5];
uint8_t place[7] = {'0', '0', '0', '0', '0', '0', '\0'};
volatile uint8_t vision_place_valid = 0U;
volatile uint8_t vision_current_cmd = 0U;
static volatile uint16_t vision_x_latest = 0U;
static volatile uint16_t vision_y_latest = 0U;
static uint8_t vision_color_count = 0U;
static uint8_t vision_debug_tx[40];
uint8_t distancestate=0;
uint8_t accValid = 0;
char displayBuffer[20];
typedef enum {
    MOTOR_IDLE,
    MOTOR_ENABLING,
    MOTOR_MOVING,
    MOTOR_FINISHED
} MotorState_t;
extern int ball_2[5];
extern uint8_t outLetter[4];
MotorState_t motorState = MOTOR_IDLE;
uint32_t motorTimer = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t get_vision_xy(uint16_t *vision_x, uint16_t *vision_y, uint16_t max_x, uint16_t max_y)
{
    uint16_t x_snapshot = 0U;
    uint16_t y_snapshot = 0U;
    uint8_t valid = 0U;

    __disable_irq();
    x_snapshot = vision_x_latest;
    y_snapshot = vision_y_latest;
    valid = vision_place_valid;
    __enable_irq();

    if (valid == 0U) {
        *vision_x = 0U;
        *vision_y = 0U;
        return 0U;
    }

    if ((x_snapshot > max_x) || (y_snapshot > max_y)) {
        *vision_x = 0U;
        *vision_y = 0U;
        return 0U;
    }

    *vision_x = x_snapshot;
    *vision_y = y_snapshot;

    return 1U;
}

static uint16_t Vision_ReadLe16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t Vision_ReadLe32(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static uint8_t Vision_ColorIdToChar(uint8_t color_id)
{
    switch (color_id) {
    case 1U: return (uint8_t)'w';
    case 2U: return (uint8_t)'r';
    case 3U: return (uint8_t)'b';
    case 4U: return (uint8_t)'g';
    case 5U: return (uint8_t)'k';
    default: return 0U;
    }
}

static uint8_t Vision_ColorDebugChar(uint8_t color)
{
    return (color == 0U) ? (uint8_t)'0' : color;
}

static void Vision_SendParsedDebug(uint8_t type, uint32_t qr_num, uint8_t color_char, int16_t x, int16_t y)
{
    int len = 0;

    if (huart2.gState != HAL_UART_STATE_READY) {
        return;
    }

    if (type == 0x01U) {
        (void)color_char;
        len = snprintf((char *)vision_debug_tx, sizeof(vision_debug_tx), "COLOR,%c%c%c%c%c\r\n",
                       Vision_ColorDebugChar(Coulor[0]),
                       Vision_ColorDebugChar(Coulor[1]),
                       Vision_ColorDebugChar(Coulor[2]),
                       Vision_ColorDebugChar(Coulor[3]),
                       Vision_ColorDebugChar(Coulor[4]));
    } else if (type == 0x02U) {
        len = snprintf((char *)vision_debug_tx, sizeof(vision_debug_tx), "QR,%lu\r\n", (unsigned long)qr_num);
    } else if (type == 0x03U) {
        len = snprintf((char *)vision_debug_tx, sizeof(vision_debug_tx), "LOC,%d,%d\r\n", (int)x, (int)y);
    }

    if ((len > 0) && (len < (int)sizeof(vision_debug_tx))) {
        /* 修改：恢复视觉解析调试输出，发送到 USART2。 */
        HAL_UART_Transmit_DMA(&huart2, vision_debug_tx, (uint16_t)len);
    }
}

static void JY61P_FormatCenti(char *output, uint16_t output_size, float value)
{
    int32_t scaled = (int32_t)(value * 100.0f + ((value >= 0.0f) ? 0.5f : -0.5f));
    uint32_t magnitude = (scaled < 0) ? (uint32_t)(-scaled) : (uint32_t)scaled;

    snprintf(output, output_size, "%s%lu.%02lu",
             (scaled < 0) ? "-" : "",
             (unsigned long)(magnitude / 100U),
             (unsigned long)(magnitude % 100U));
}

/* 修改：陀螺仪调试输出当前停用，保留函数供后续恢复。 */
static void __attribute__((unused)) JY61P_SendDebug(void)
{
    static uint32_t last_print_tick = 0U;
    char ax[16], ay[16], az[16];
    char wx[16], wy[16], wz[16];
    char roll[16], pitch[16], yaw[16], temperature[16];
    uint32_t now = HAL_GetTick();
    int len;

    if (((now - last_print_tick) < JY61P_DEBUG_PRINT_PERIOD_MS) ||
        (lastPacketTime == 0U) ||
        (huart2.gState != HAL_UART_STATE_READY)) {
        return;
    }

    last_print_tick = now;
    JY61P_FormatCenti(ax, sizeof(ax), sensorData.ax);
    JY61P_FormatCenti(ay, sizeof(ay), sensorData.ay);
    JY61P_FormatCenti(az, sizeof(az), sensorData.az);
    JY61P_FormatCenti(wx, sizeof(wx), sensorData.wx);
    JY61P_FormatCenti(wy, sizeof(wy), sensorData.wy);
    JY61P_FormatCenti(wz, sizeof(wz), sensorData.wz);
    JY61P_FormatCenti(roll, sizeof(roll), sensorData.roll);
    JY61P_FormatCenti(pitch, sizeof(pitch), sensorData.pitch);
    JY61P_FormatCenti(yaw, sizeof(yaw), sensorData.yaw);
    JY61P_FormatCenti(temperature, sizeof(temperature), sensorData.temperature);

    len = snprintf((char *)jy61p_debug_tx, sizeof(jy61p_debug_tx),
                   "IMU,ACC=%s,%s,%s,GYRO=%s,%s,%s,ANGLE=%s,%s,%s,TEMP=%s\r\n",
                   ax, ay, az, wx, wy, wz, roll, pitch, yaw, temperature);

    if ((len > 0) && (len < (int)sizeof(jy61p_debug_tx))) {
        HAL_UART_Transmit_DMA(&huart2, jy61p_debug_tx, (uint16_t)len);
    }
}

static void Vision_SaveQr(uint32_t qr_num)
{
    uint8_t tens = (uint8_t)(((qr_num % 100U) / 10U) + (uint32_t)'0');
    uint8_t units = (uint8_t)((qr_num % 10U) + (uint32_t)'0');

    if (vision_current_cmd == 0x01U) {
        QRPacke[0] = tens;
        QRPacke[1] = units;
    } else if (vision_current_cmd == 0x03U) {
        QRPacke1[0] = tens;
        QRPacke1[1] = units;
    }

    Vision_SendParsedDebug(0x02U, qr_num, 0U, 0, 0);
}

static void Vision_SaveColor(uint8_t color_id)
{
    uint8_t color_char = Vision_ColorIdToChar(color_id);
    uint8_t i = 0U;

    if (color_char == 0U) {
        return;
    }

    for (i = 0U; i < vision_color_count; i++) {
        if (Coulor[i] == color_char) {
            Vision_SendParsedDebug(0x01U, 0U, 0U, 0, 0);
            return;
        }
    }

    if (vision_color_count >= 5U) {
        Vision_SendParsedDebug(0x01U, 0U, 0U, 0, 0);
        return;
    }

    Coulor[vision_color_count] = color_char;
    vision_color_count++;

    Vision_SendParsedDebug(0x01U, 0U, 0U, 0, 0);
}

static void Vision_SaveLocation(int16_t x, int16_t y)
{
    uint16_t legacy_x = 0U;
    uint16_t legacy_y = 0U;

    if ((x < 0) || (y < 0)) {
        vision_place_valid = 0U;
        return;
    }

    __disable_irq();
    vision_x_latest = (uint16_t)x;
    vision_y_latest = (uint16_t)y;
    vision_place_valid = 1U;
    __enable_irq();

    legacy_x = ((uint16_t)x > 999U) ? 999U : (uint16_t)x;
    legacy_y = ((uint16_t)y > 999U) ? 999U : (uint16_t)y;
    place[0] = (uint8_t)((legacy_x / 100U) + (uint16_t)'0');
    place[1] = (uint8_t)(((legacy_x / 10U) % 10U) + (uint16_t)'0');
    place[2] = (uint8_t)((legacy_x % 10U) + (uint16_t)'0');
    place[3] = (uint8_t)((legacy_y / 100U) + (uint16_t)'0');
    place[4] = (uint8_t)(((legacy_y / 10U) % 10U) + (uint16_t)'0');
    place[5] = (uint8_t)((legacy_y % 10U) + (uint16_t)'0');

    Vision_SendParsedDebug(0x03U, 0U, 0U, x, y);
}

static void Vision_ParseNewFrame(const uint8_t *buf, uint16_t size)
{
    uint8_t type = 0U;
    uint8_t len = 0U;

    if ((size < 5U) || (buf[0] != 0xAAU)) {
        return;
    }

    type = buf[1];
    len = buf[2];

    if ((len > 4U) || (size < (uint16_t)(len + 4U)) || (buf[len + 3U] != 0x55U)) {
        return;
    }

    if ((type == 0x01U) && (len == 1U)) {
        Vision_SaveColor(buf[3]);
    } else if ((type == 0x02U) && (len == 4U)) {
        Vision_SaveQr(Vision_ReadLe32(&buf[3]));
    } else if ((type == 0x03U) && (len == 4U)) {
        Vision_SaveLocation((int16_t)Vision_ReadLe16(&buf[3]),
                            (int16_t)Vision_ReadLe16(&buf[5]));
    }
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value > max_value) {
        return max_value;
    }

    if (value < min_value) {
        return min_value;
    }

    return value;
}

static void add_vector_by_angle(float angle_deg, float speed, float *vx, float *vy)
{
    float rad = angle_deg * 3.1415926f / 180.0f;

    *vx += speed * cosf(rad);
    *vy += speed * sinf(rad);
}

static void Debug_PrintVisionXY(uint16_t vision_x, uint16_t vision_y, uint8_t valid)
{
    static uint32_t last_print_tick = 0U;
    uint32_t now = HAL_GetTick();

    if (g_debug_io_mode != 1U) {
        return;
    }

    if ((now - last_print_tick) < VISION_DEBUG_PRINT_PERIOD_MS) {
        return;
    }

    last_print_tick = now;
    /* 修改：恢复视觉坐标调试输出。 */
    printf("vision valid=%u x=%u y=%u\r\n",
           (unsigned int)valid,
           (unsigned int)vision_x,
           (unsigned int)vision_y);
}

static uint8_t Vision_XY_PID_Control(uint16_t vision_x,
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
                                     float angle_when_y_high)
{
    static float integral_x = 0.0f;
    static float integral_y = 0.0f;
    static float last_error_x = 0.0f;
    static float last_error_y = 0.0f;
    static uint32_t last_update_tick = 0U;
    static uint8_t pid_initialized = 0U;
    static uint8_t motor_running = 0U;
    float error_x = (float)((int32_t)center_x - (int32_t)vision_x);
    float error_y = (float)((int32_t)center_y - (int32_t)vision_y);
    float abs_error_x = fabsf(error_x);
    float abs_error_y = fabsf(error_y);
    uint32_t now = HAL_GetTick();

    if ((valid == 0U) || (vision_x > max_x) || (vision_y > max_y)) {
        if (motor_running != 0U) {
            Move_StopAll();
        }
        integral_x = 0.0f;
        integral_y = 0.0f;
        last_error_x = 0.0f;
        last_error_y = 0.0f;
        pid_initialized = 0U;
        motor_running = 0U;
        return 0U;
    }

    if ((abs_error_x <= (float)deadband_x_px) && (abs_error_y <= (float)deadband_y_px)) {
        if (motor_running != 0U) {
            Move_StopAll();
        }
        integral_x = 0.0f;
        integral_y = 0.0f;
        last_error_x = error_x;
        last_error_y = error_y;
        pid_initialized = 1U;
        last_update_tick = now;
        motor_running = 0U;
        return 1U;
    }

    if ((update_period_ms > 0U) && ((now - last_update_tick) < update_period_ms)) {
        return 0U;
    }

    float dt = 0.001f;
    if (pid_initialized != 0U) {
        dt = (float)(now - last_update_tick) / 1000.0f;
        if (dt <= 0.0f) {
            dt = 0.001f;
        }
    }

    float output_x = 0.0f;
    float output_y = 0.0f;

    if (abs_error_x > (float)deadband_x_px) {
        integral_x += error_x * dt;
        integral_x = clamp_float(integral_x, -integral_limit_x, integral_limit_x);
        float derivative_x = (pid_initialized != 0U) ? ((error_x - last_error_x) / dt) : 0.0f;
        output_x = (kp_x * error_x) + (ki_x * integral_x) + (kd_x * derivative_x);
    } else {
        integral_x = 0.0f;
    }

    if (abs_error_y > (float)deadband_y_px) {
        integral_y += error_y * dt;
        integral_y = clamp_float(integral_y, -integral_limit_y, integral_limit_y);
        float derivative_y = (pid_initialized != 0U) ? ((error_y - last_error_y) / dt) : 0.0f;
        output_y = (kp_y * error_y) + (ki_y * integral_y) + (kd_y * derivative_y);
    } else {
        integral_y = 0.0f;
    }

    float vector_x = 0.0f;
    float vector_y = 0.0f;

    if (output_x > 0.0f) {
        add_vector_by_angle(angle_when_x_low, fabsf(output_x), &vector_x, &vector_y);
    } else if (output_x < 0.0f) {
        add_vector_by_angle(angle_when_x_high, fabsf(output_x), &vector_x, &vector_y);
    }

    if (output_y > 0.0f) {
        add_vector_by_angle(angle_when_y_low, fabsf(output_y), &vector_x, &vector_y);
    } else if (output_y < 0.0f) {
        add_vector_by_angle(angle_when_y_high, fabsf(output_y), &vector_x, &vector_y);
    }

    float speed = sqrtf((vector_x * vector_x) + (vector_y * vector_y));
    if (speed <= 0.0f) {
        if (motor_running != 0U) {
            Move_StopAll();
        }
        motor_running = 0U;
    } else {
        float angle_deg = atan2f(vector_y, vector_x) * 180.0f / 3.1415926f;
        if (angle_deg < 0.0f) {
            angle_deg += 360.0f;
        }

        speed = clamp_float(speed, min_speed_mps, max_speed_mps);
        Move_TranslateContinuous(angle_deg, speed);
        motor_running = 1U;
    }

    if ((abs_error_x <= (float)deadband_x_px) && (abs_error_y <= (float)deadband_y_px)) {
        Move_StopAll();
        motor_running = 0U;
    }

    last_error_x = error_x;
    last_error_y = error_y;
    last_update_tick = now;
    pid_initialized = 1U;
    return (motor_running == 0U) ? 1U : 0U;
}

static uint8_t s_vision_fb_phase = 0U;
static uint32_t s_vision_fb_stable_start_tick = 0U;
static uint32_t s_vision_fb_wait_start_tick = 0U;

void Vision_XY_PID_Then_Forward_Backward_Reset(void)
{
    s_vision_fb_phase = 0U;
    s_vision_fb_stable_start_tick = 0U;
    s_vision_fb_wait_start_tick = 0U;
}

uint8_t Vision_XY_PID_Then_Forward_Backward(uint16_t vision_x,
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
                                                   float backward_angle_deg)
{
    if (s_vision_fb_phase == 0U) {
        uint8_t aligned = Vision_XY_PID_Control(vision_x, vision_y, valid,
                                                center_x, max_x, center_y, max_y,
                                                deadband_x_px, deadband_y_px,
                                                kp_x, ki_x, kd_x,
                                                kp_y, ki_y, kd_y,
                                                min_speed_mps, max_speed_mps,
                                                integral_limit_x, integral_limit_y,
                                                update_period_ms,
                                                angle_when_x_low, angle_when_x_high,
                                                angle_when_y_low, angle_when_y_high);
        uint32_t now = HAL_GetTick();

        if (aligned == 0U) {
            s_vision_fb_stable_start_tick = 0U;
            return 0U;
        }

        if (s_vision_fb_stable_start_tick == 0U) {
            s_vision_fb_stable_start_tick = now;
            return 0U;
        }

        if ((now - s_vision_fb_stable_start_tick) < stable_required_ms) {
            return 0U;
        }

        if ((forward_distance_m <= 0.0f) || (backward_distance_m <= 0.0f) || (move_speed_mps <= 0.0f)) {
            s_vision_fb_phase = 4U;
            return 1U;
        }

        uint32_t duration_ms = (uint32_t)((forward_distance_m / move_speed_mps) * 1000.0f + 0.5f);
        Move_StartTranslateForTime(forward_angle_deg, move_speed_mps, duration_ms);
        s_vision_fb_phase = 1U;
        return 0U;
    }

    if (s_vision_fb_phase == 1U) {
        Move_Update();
        if (g_motionActive != 0U) {
            s_vision_fb_phase = 2U;
            s_vision_fb_wait_start_tick = HAL_GetTick();
        }
        return 0U;
    }

    if (s_vision_fb_phase == 2U) {
        if ((HAL_GetTick() - s_vision_fb_wait_start_tick) < 100U) {
            return 0U;
        }

        uint32_t duration_ms = (uint32_t)((backward_distance_m / move_speed_mps) * 1000.0f + 0.5f);
        Move_StartTranslateForTime(backward_angle_deg, move_speed_mps, duration_ms);
        s_vision_fb_phase = 3U;
        return 0U;
    }

    if (s_vision_fb_phase == 3U) {
        Move_Update();
        if (g_motionActive != 0U) {
            s_vision_fb_phase = 4U;
            return 1U;
        }
        return 0U;
    }

    return 1U;
}
uint8_t duoooooo=1;
uint8_t Vision_XY_PID_Then_Forward_Backward2(uint16_t vision_x,
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
                                                   float backward_angle_deg)
{
    if (s_vision_fb_phase == 0U) {
        uint8_t aligned = Vision_XY_PID_Control(vision_x, vision_y, valid,
                                                center_x, max_x, center_y, max_y,
                                                deadband_x_px, deadband_y_px,
                                                kp_x, ki_x, kd_x,
                                                kp_y, ki_y, kd_y,
                                                min_speed_mps, max_speed_mps,
                                                integral_limit_x, integral_limit_y,
                                                update_period_ms,
                                                angle_when_x_low, angle_when_x_high,
                                                angle_when_y_low, angle_when_y_high);
        uint32_t now = HAL_GetTick();

        if (aligned == 0U) {
            s_vision_fb_stable_start_tick = 0U;
            return 0U;
        }

        if (s_vision_fb_stable_start_tick == 0U) {
            s_vision_fb_stable_start_tick = now;
            return 0U;
        }

        if ((now - s_vision_fb_stable_start_tick) < stable_required_ms) {
            return 0U;
        }

        if ((forward_distance_m <= 0.0f) || (backward_distance_m <= 0.0f) || (move_speed_mps <= 0.0f)) {
            s_vision_fb_phase = 5U;
            return 1U;
        }

        uint32_t duration_ms = (uint32_t)((forward_distance_m / move_speed_mps) * 1000.0f + 0.5f);
        Move_StartTranslateForTime(forward_angle_deg, move_speed_mps, duration_ms);
        s_vision_fb_phase = 1U;
        return 0U;
    }

    if (s_vision_fb_phase == 1U) {
        Move_Update();
        if (g_motionActive != 0U) {
            if(duoooooo)
            {
                yajun_2();
                duoooooo=0;
            }
            else
            {
                guanjun_2();
            }
            s_vision_fb_phase = 2U;
            s_vision_fb_wait_start_tick = HAL_GetTick();
        }
        return 0U;
    }

    if (s_vision_fb_phase == 2U) {
        if ((HAL_GetTick() - s_vision_fb_wait_start_tick) < VISION_YAJUN_ACTION_DELAY_MS) {
            return 0U;
        }

        uint32_t duration_ms = (uint32_t)((backward_distance_m / move_speed_mps) * 1000.0f + 0.5f);
        Move_StartTranslateForTime(backward_angle_deg, move_speed_mps, duration_ms);
        s_vision_fb_phase = 3U;
        return 0U;
    }

    if (s_vision_fb_phase == 3U) {
        Move_Update();
        if (g_motionActive != 0U) {
           // guanjun_2();
            s_vision_fb_phase = 4U;
            s_vision_fb_wait_start_tick = HAL_GetTick();
        }
        return 0U;
    }

    if (s_vision_fb_phase == 4U) {
        if ((HAL_GetTick() - s_vision_fb_wait_start_tick) >= VISION_YAJUN_ACTION_DELAY_MS) {
            s_vision_fb_phase = 5U;
            return 1U;
        }
        return 0U;
    }

    return 1U;
}

uint8_t Vision_XY_PID_Then_Forward_Backward3(uint16_t vision_x,
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
                                                   float backward_angle_deg)
{
    if (s_vision_fb_phase == 0U) {
        uint8_t aligned = Vision_XY_PID_Control(vision_x, vision_y, valid,
                                                center_x, max_x, center_y, max_y,
                                                deadband_x_px, deadband_y_px,
                                                kp_x, ki_x, kd_x,
                                                kp_y, ki_y, kd_y,
                                                min_speed_mps, max_speed_mps,
                                                integral_limit_x, integral_limit_y,
                                                update_period_ms,
                                                angle_when_x_low, angle_when_x_high,
                                                angle_when_y_low, angle_when_y_high);
        uint32_t now = HAL_GetTick();

        if (aligned == 0U) {
            s_vision_fb_stable_start_tick = 0U;
            return 0U;
        }

        if (s_vision_fb_stable_start_tick == 0U) {
            s_vision_fb_stable_start_tick = now;
            return 0U;
        }

        if ((now - s_vision_fb_stable_start_tick) < stable_required_ms) {
            return 0U;
        }

        if ((forward_distance_m <= 0.0f) || (backward_distance_m <= 0.0f) || (move_speed_mps <= 0.0f)) {
            s_vision_fb_phase = 5U;
            return 1U;
        }

        uint32_t duration_ms = (uint32_t)((forward_distance_m / move_speed_mps) * 1000.0f + 0.5f);
        Move_StartTranslateForTime(forward_angle_deg, move_speed_mps, duration_ms);
        s_vision_fb_phase = 1U;
        return 0U;
    }

    if (s_vision_fb_phase == 1U) {
        Move_Update();
        if (g_motionActive != 0U) {
            guanjun_2();
            s_vision_fb_phase = 2U;
            s_vision_fb_wait_start_tick = HAL_GetTick();
        }
        return 0U;
    }

    if (s_vision_fb_phase == 2U) {
        if ((HAL_GetTick() - s_vision_fb_wait_start_tick) < VISION_YAJUN_ACTION_DELAY_MS) {
            return 0U;
        }

        uint32_t duration_ms = (uint32_t)((backward_distance_m / move_speed_mps) * 1000.0f + 0.5f);
        Move_StartTranslateForTime(backward_angle_deg, move_speed_mps, duration_ms);
        s_vision_fb_phase = 3U;
        return 0U;
    }

    if (s_vision_fb_phase == 3U) {
        Move_Update();
        if (g_motionActive != 0U) {
            yajun_2();
            s_vision_fb_phase = 4U;
            s_vision_fb_wait_start_tick = HAL_GetTick();
        }
        return 0U;
    }

    if (s_vision_fb_phase == 4U) {
        if ((HAL_GetTick() - s_vision_fb_wait_start_tick) >= VISION_YAJUN_ACTION_DELAY_MS) {
            s_vision_fb_phase = 5U;
            return 1U;
        }
        return 0U;
    }

    return 1U;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	(void)Size;
	if (huart == &huart6)
	{
		/* 修改：解析 USART6 收到的全部 JY61P 字节，并立即恢复空闲 DMA 接收。 */
		for (uint16_t i = 0U; i < Size; i++)
		{
			ProcessReceivedData(jy61p_rx_buffer[i]);
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, jy61p_rx_buffer, sizeof(jy61p_rx_buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
		/* 修改：陀螺仪数据输出暂时注释，仅保留 USART6 接收与解析。 */
		// JY61P_SendDebug();
		return;
	}
	if(huart==&huart5)
	{
	//HAL_UART_Transmit_DMA(&huart5,rxBuffer5,Size);
	//HAL_UARTEx_ReceiveToIdle_DMA(&huart5,rxBuffer5,sizeof(rxBuffer5));

	}
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	if(huart==&huart4)
	{
	}
	if(huart == &huart1)
    {
        Vision_ParseNewFrame(rxBuffer2, Size);
        rxBuffer2[0] = 0U;
        if(rxBuffer2[0] == 0xAA && rxBuffer2[3] == 0x55)
		{	 
		  a = rxBuffer2[1];
          b = rxBuffer2[2];
          if (QRPacke1[0]==0 && QRPacke1[1]==0) 
          {
            QRPacke1[0]=a; QRPacke1[1]=b;          // 第一�?
          } 
          else if (QRPacke1[0]==a && QRPacke1[1]==b) 
          {
            // 重复，忽�?
          } 
          else 
          {
            QRPacke[0]=a; QRPacke[1]=b;            // 第二�?最新不同条
          }
		}
		else if(rxBuffer2[0] == 0xAB && rxBuffer2[6] == 0x55)
		{
			Coulor[0]=rxBuffer2[5];
			Coulor[1]=rxBuffer2[4];
			Coulor[2]=rxBuffer2[3];
			Coulor[3]=rxBuffer2[2];
			Coulor[4]=rxBuffer2[1];
		}
		else if(rxBuffer2[0] == 0xAD && rxBuffer2[7] == 0x55)
		{
			place[0]=rxBuffer2[1];
			place[1]=rxBuffer2[2];
			place[2]=rxBuffer2[3];
			place[3]=rxBuffer2[4];
			place[4]=rxBuffer2[5];
			place[5]=rxBuffer2[6];
            vision_place_valid = 1U;
		}
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rxBuffer2, sizeof(rxBuffer2));    
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
    g_debug_io_mode = 1U;
    duoji_Init();
    HAL_Delay(DUOJI_BOOT_SETTLE_DELAY_MS);
    duoji_Turntable_Set_Start_Position();
    HAL_Delay(DUOJI_BOOT_REHOME_DELAY_MS);
    duoji_tc();

    // 启动UART DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rxBuffer2,sizeof(rxBuffer2));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);

    txBuffer2[0] = 0xAA;
    txBuffer2[1] = 0x00;
    txBuffer2[2] = 0x00;
    txBuffer2[3] = 0x55;
    vision_current_cmd = txBuffer2[2];
    HAL_UART_Transmit_DMA(&huart1, txBuffer2, 4);
	    
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4,rxBuffer4,sizeof(rxBuffer4));
	__HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_HT);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,rxBuffer5,sizeof(rxBuffer5));
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT);
	
	// HAL_UARTEx_ReceiveToIdle_DMA(&huart2,rxBuffer2,sizeof(rxBuffer2));
	// __HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);
	
	/* 修改：USART6 通过空闲中断 DMA 连续接收 JY61P 数据。 */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, jy61p_rx_buffer, sizeof(jy61p_rx_buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);

	// 传感器初始化配置
	//HAL_TIM_Base_Start_IT(&htim8);
    HAL_Delay(100);  // 等待100ms
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	OLED_Init();
	//Gray_Trace_GPIO_Init(); 
	Move_Init();
  //Gray_Trace_GPIO_Init(); //确保GPIO已初始化
    //uint8_t a='a';
	//JY61P_CalibrateAccel();
//Move_StartTranslateForTime(0,0.1, 1000);
//Move_CircleForTime(0.8,0.13,0,10000);
//	Emm_V5_MMCL_Pos_Control(3, 1, 500, 30, 6400, false, true);
//	Emm_V5_MMCL_Pos_Control(4, 1, 500, 30, 6400, false, true);

////	// 然后发送多电机同步命令
//	Emm_V5_Multi_Motor_Cmd_UART5(0); HAL_Delay(1); 
//	Emm_V5_MMCL_Pos_Control(1, 1, 500, 30, 6400, false, true);
//	Emm_V5_MMCL_Pos_Control(2, 1, 500, 30, 6400, false, true);
//	Emm_V5_Multi_Motor_Cmd_UART4(0);  
// Move_RotateAngle(90,0,500, 30);
// //	Move_StartCircleForTime(0.8, 0.13,0,10000);
// Emm_V5_MMCL_Vel_Control(1, 1,100, 30,true);
// Emm_V5_MMCL_Vel_Control(2, 1,100, 30,true);
// Emm_V5_MMCL_Vel_Control(4, 1, 100, 30,true);
// Emm_V5_MMCL_Vel_Control(3, 1, 100, 30,true);
// Emm_V5_Multi_Motor_Cmd_UART5(0); HAL_Delay(1); 

//Emm_V5_Multi_Motor_Cmd_UART4(0); HAL_Delay(1);

// stat=1;
// 	//开始之�?
// 	task3(95);
// 	task2(0); 
// 	// HAL_Delay(3000);
// 	//开始之�?
//   task2(-47); //舵机向上为负，向下为�?
// 	task3(-27); //舵机向上为正，向下为�?
//   if (boot_requires_duoji_rehome()) {
//     duoji_set_start_position(2);
//   } else {
//     duoji_sync_start_position(2);
//   }
   //task1_1_step1();
//   //Move_StartTranslateForTime(225,0.3, 1000);
//   Vision_XY_PID_Then_Forward_Backward_Reset();
    // duoji_Set_ID1_Angle(30);
    // HAL_Delay(1);
    // duoji_Set_ID2_Angle(170);
    // HAL_Delay(3000);
    // //舵机恢复转盘转动模式

    //task1_1_step2();
   // duoji_Turntable_MoveToAngle720(720.0f);
   // duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
//   Emm_V5_MMCL_Pos_Control(1, 1, 100, 30, 1600, false, true);
//   Emm_V5_MMCL_Pos_Control(4, 0, 100, 30, 1600, false, true);
//  Emm_V5_Multi_Motor_Cmd_UART5(0); HAL_Delay(1);
//   Emm_V5_MMCL_Pos_Control(2, 1,100, 30,1600, false, true);
//   Emm_V5_MMCL_Pos_Control(3, 0, 100, 30,1600, false, true);
//   Emm_V5_Multi_Motor_Cmd_UART4(0);
stat=1;

HAL_Delay(1);
//Move_StartTranslateForTime(270,0.3, 1000);
// Move_StartTranslateForTime(225,0.3, 1000);
  while (1)
  { 
    State();
//HAL_UART_Transmit_DMA(&huart4, uart4_hello, sizeof(uart4_hello) - 1U);
//HAL_Delay(1000);
    // uint16_t vision_x = 0U;
	// uint16_t vision_y = 0U;
	// uint8_t vision_valid = 0U;

	//  // Vision_XY_PID_Then_Forward_Backward_Reset();
	// 	vision_valid = get_vision_xy(&vision_x, &vision_y, 960U, 720U);
	// 	if(Vision_XY_PID_Then_Forward_Backward(vision_x, vision_y, vision_valid,
	// 										   522U, 960U,
	// 										   360U, 720U,
	// 										   2U, 2U,
	// 										   0.0008f, 0.00000f, 0.00000f,
	// 										   0.0008f, 0.00000f, 0.00000f,
	// 										   0.01f, 0.14f,
	// 										   400.0f, 400.0f,
	// 										   15U,
	// 										   90.0f, 270.0f,
	// 										   0.0f, 180.0f,
	// 										   200U,
	// 										   0.1575f, 0.0f,
	// 										   0.15f,
	// 										   0.0f, 180.0f))
	// 	{
	// 		break;
	// 	}
	// 	HAL_Delay(5);
    //   uint16_t vision_x = 0U;
    //   uint16_t vision_y = 0U;
    //   uint8_t vision_valid = get_vision_xy(&vision_x, &vision_y, 1280U, 960U);

    //   Vision_XY_PID_Then_Forward_Backward(vision_x, vision_y, vision_valid,
    //                                       620U, 1280U,
    //                                       480U, 960U,
    //                                       7U, 7U,
    //                                       0.00040f, 0.00000f, 0.00010f,
    //                                       0.00040f, 0.00000f, 0.00010f,
    //                                       0.03f, 0.14f,
    //                                       400.0f, 400.0f,
    //                                       15U,
    //                                       90.0f, 270.0f,
    //                                       0.0f, 180.0f,
    //                                       200U,
    //                                       0.11f, 0.11f,
    //                                       0.20f,
    //                                       0.0f, 180.0f);
  // State();
//    float gray1 = Gray_Trace_Get_Dir();
    //  XUNji();
     
      //HAL_Delay(5);
    // task1_2_finish();
    //   int QR=2;
	// 	switch(QR)
	// 	{
	// 		case 1:  outLetter[0]='a'; outLetter[1]='b'; outLetter[2]='c';
	// 		break;
	// 		case 2:  outLetter[0]='a'; outLetter[1]='c'; outLetter[2]='b';
	// 		break;
	// 		case 3:  outLetter[0]='b'; outLetter[1]='a'; outLetter[2]='c';
	// 		break;
	// 		case 4:  outLetter[0]='b'; outLetter[1]='c'; outLetter[2]='a';
	// 		break;
	// 		case 5:  outLetter[0]='c'; outLetter[1]='a'; outLetter[2]='b';
	// 		break;
	// 		case 6:  outLetter[0]='c'; outLetter[1]='b'; outLetter[2]='a';
	// 		break;
	// 		default: 
	// 			outLetter[0]=outLetter[1]=outLetter[2]=outLetter[3]=0;
	// 			break;
	// 	}
    //     //task1_2_finish();
    //     HAL_Delay(1000);
    //     BuildBall2OrderFromQr();
    //     output_ball_alternating_second(ball_2[0]);
    //     HAL_Delay(1000);
    //     output_ball_alternating_second(ball_2[1]);
    //     HAL_Delay(1000);
    //     output_ball_alternating_second(ball_2[2]);
    //     HAL_Delay(1000);
    //     while(1)
    //     {

    //     };
    // OLED_ShowString(1, 1, "X:");
	// OLED_ShowNum(1, 3, vision_x, 3);
	// OLED_ShowString(2, 1, "Y:");
	// OLED_ShowNum(2, 3, vision_y, 3);
	// HAL_Delay(30);
	// HAL_Delay(1000);
    //Move_StopAll();
    //State(); 
    //XUNji();
   //Move_Circle(0.8, 0.4, 1);
   //HAL_Delay(8);
//	  OLED_ShowChar(1,1,Coulor[0]);
//	  OLED_ShowChar(2,1,Coulor[1]);
//	  OLED_ShowChar(3,1,Coulor[2]);
//	  OLED_ShowChar(4,1,Coulor[3]);
//	  OLED_ShowChar(4,3,Coulor[4]);
	  
//	  OLED_ShowHexNum(1, 1, place[0], 3);
//	  OLED_ShowHexNum(1, 5, place[1], 3);
//	  OLED_ShowHexNum(2, 1, place[2], 3);
//	  OLED_ShowHexNum(3, 1, place[3], 3);
//	  OLED_ShowHexNum(3, 5, place[4], 3);
//	  OLED_ShowHexNum(4, 1, place[5], 3);

//	  OLED_ShowHexNum(1, 1, QRPacke[0], 2);
//	  OLED_ShowHexNum(2, 1, QRPacke[1], 2);
		
//	  HAL_Delay(2000);
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim==&htim8)
//	{
//		count++;
//		if(count>=1000)
//		{
//			count=0;
//			count1++;
//		}
//	}
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
