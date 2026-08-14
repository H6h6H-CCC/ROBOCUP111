#include "duoji.h"
#include "usart.h"

#define TURNTABLE_duoji_ID            3U
#define TURNTABLE_PWM_CENTER          duoji_PWM_MID
#define TURNTABLE_PWM_START_POSITION  804U
#define TURNTABLE_PWM_STEP_72_DEG     400U
#define TURNTABLE_PWM_STEP_36_DEG     200U
#define TURNTABLE_DEFAULT_MOVE_TIME   450U
#define TURNTABLE_PWM_TRAVEL_RANGE    (duoji_PWM_MAX - duoji_PWM_MIN)
#define TURNTABLE_OUTPUT_BALL_COUNT   5U
#define TURNTABLE_SECOND_BALL_COUNT   3U
#define TURNTABLE_SECOND_BALL1_PWM    1614U
#define TURNTABLE_SECOND_BALL2_PWM    1214U
#define TURNTABLE_SECOND_BALL3_PWM    814U
#define duoji_ID1                     1U
#define duoji_ID2                     2U
#define duoji_ID1_ZERO_PWM            1900U
#define duoji_ID2_ZERO_PWM            2500U
#define AUX_duoji_DEFAULT_MOVE_TIME   2000U

/* 全局指令缓冲区 */
char duoji_Cmd_Buf[duoji_CMD_BUF_SIZE] = {0};
uint16_t Sid[3]={0,1,2};
uint16_t Spwm[3]={2200,2200,2200};
uint16_t Stime[3]={1000,1000,1000};
static uint16_t g_turntable_current_pwm = TURNTABLE_PWM_CENTER;
static duoji_TurntableDir_t g_turntable_output_preferred_dir = duoji_TURNTABLE_CW;

static HAL_StatusTypeDef UART3_DMA_Send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(&huart3, (uint8_t *)data, len, 100U);
}

static uint16_t duoji_Clamp_PWM(int32_t pwm)
{
    if (pwm < duoji_PWM_MIN)
    {
        return duoji_PWM_MIN;
    }

    if (pwm > duoji_PWM_MAX)
    {
        return duoji_PWM_MAX;
    }

    return (uint16_t)pwm;
}

static int32_t duoji_Angle_To_PWM_Delta(float angle_deg)
{
    float pwm_delta = angle_deg * (float)(duoji_PWM_MAX - duoji_PWM_MIN) / 360.0f;

    if (pwm_delta >= 0.0f)
    {
        return (int32_t)(pwm_delta + 0.5f);
    }

    return (int32_t)(pwm_delta - 0.5f);
}

static void duoji_Turntable_Ensure_Position_Mode(void)
{
    /* Servo 3 is treated as a 0..360 positional servo:
       500 -> 0 deg, 1500 -> 180 deg, 2500 -> 360 deg.
       Do not remap it to 180/270 modes here, just make sure torque is enabled. */
    duoji_Stop(TURNTABLE_duoji_ID);
    HAL_Delay(5);
    duoji_Recover_Torque(TURNTABLE_duoji_ID);
    HAL_Delay(20);
}

static void duoji_Turntable_Rotate(duoji_TurntableDir_t dir, uint16_t pwm_delta, uint16_t time)
{
    int32_t target_pwm = g_turntable_current_pwm;

    if (dir == duoji_TURNTABLE_CW)
    {
        target_pwm -= pwm_delta;
    }
    else
    {
        target_pwm += pwm_delta;
    }

    duoji_Control(TURNTABLE_duoji_ID, duoji_Clamp_PWM(target_pwm), time);
}

static uint16_t duoji_Turntable_Get_Output_Target_PWM(uint8_t ball_number)
{
    if (ball_number < 1U)
    {
        ball_number = 1U;
    }
    else if (ball_number > TURNTABLE_OUTPUT_BALL_COUNT)
    {
        ball_number = TURNTABLE_OUTPUT_BALL_COUNT;
    }

    return (uint16_t)(TURNTABLE_PWM_START_POSITION +
                      ((ball_number - 1U) * TURNTABLE_PWM_STEP_72_DEG));
}

static uint16_t duoji_Turntable_Get_Second_Output_Target_PWM(uint8_t ball_number)
{
    switch (ball_number)
    {
    case 1U:
        return TURNTABLE_SECOND_BALL1_PWM;
    case 2U:
        return TURNTABLE_SECOND_BALL2_PWM;
    case 3U:
        return TURNTABLE_SECOND_BALL3_PWM;
    default:
        return TURNTABLE_SECOND_BALL1_PWM;
    }
}

static uint16_t duoji_Turntable_Get_CW_Delta(uint16_t current_pwm, uint16_t target_pwm)
{
    if (current_pwm >= target_pwm)
    {
        return (uint16_t)(current_pwm - target_pwm);
    }

    return (uint16_t)(TURNTABLE_PWM_TRAVEL_RANGE - (target_pwm - current_pwm));
}

static uint16_t duoji_Turntable_Get_CCW_Delta(uint16_t current_pwm, uint16_t target_pwm)
{
    if (target_pwm >= current_pwm)
    {
        return (uint16_t)(target_pwm - current_pwm);
    }

    return (uint16_t)(TURNTABLE_PWM_TRAVEL_RANGE - (current_pwm - target_pwm));
}

static uint8_t duoji_Turntable_Can_Reach(uint16_t current_pwm, duoji_TurntableDir_t dir, uint16_t pwm_delta)
{
    if (dir == duoji_TURNTABLE_CW)
    {
        return (pwm_delta <= (uint16_t)(current_pwm - duoji_PWM_MIN)) ? 1U : 0U;
    }

    return (pwm_delta <= (uint16_t)(duoji_PWM_MAX - current_pwm)) ? 1U : 0U;
}

static duoji_TurntableDir_t duoji_Turntable_Select_Output_Direction(uint16_t target_pwm, uint16_t *pwm_delta)
{
    uint16_t cw_delta = duoji_Turntable_Get_CW_Delta(g_turntable_current_pwm, target_pwm);
    uint16_t ccw_delta = duoji_Turntable_Get_CCW_Delta(g_turntable_current_pwm, target_pwm);
    uint8_t cw_reachable = duoji_Turntable_Can_Reach(g_turntable_current_pwm, duoji_TURNTABLE_CW, cw_delta);
    uint8_t ccw_reachable = duoji_Turntable_Can_Reach(g_turntable_current_pwm, duoji_TURNTABLE_CCW, ccw_delta);

    if ((cw_reachable != 0U) && (ccw_reachable != 0U))
    {
        if (cw_delta < ccw_delta)
        {
            *pwm_delta = cw_delta;
            return duoji_TURNTABLE_CW;
        }

        if (ccw_delta < cw_delta)
        {
            *pwm_delta = ccw_delta;
            return duoji_TURNTABLE_CCW;
        }

        *pwm_delta = cw_delta;
        return g_turntable_output_preferred_dir;
    }

    if (cw_reachable != 0U)
    {
        *pwm_delta = cw_delta;
        return duoji_TURNTABLE_CW;
    }

    *pwm_delta = ccw_delta;
    return duoji_TURNTABLE_CCW;
}

/* 舵机初始化：启动串口3 DMA */
void duoji_Init(void)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    g_turntable_current_pwm = TURNTABLE_PWM_CENTER;
    g_turntable_output_preferred_dir = duoji_TURNTABLE_CW;

    (void)HAL_UART_DMAStop(&huart3);
    huart3.Init.BaudRate = DUOJI_UART_BAUD;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* 发送舵机指令：将字符串指令转为字节流，通过DMA发送 */
uint8_t duoji_Send_Cmd(char *cmd)
{
    uint16_t len = strlen(cmd);
    if(len == 0 || len > duoji_CMD_BUF_SIZE)
    {
        return 1; // 指令长度错误
    }
    /* 调用DMA发送，返回0表示成功，非0失败 */
    return (HAL_OK == UART3_DMA_Send((uint8_t*)cmd, len)) ? 0 : 2;
}

/* 单舵机控制：#XXXPXXXXTXXXX!  XXX=ID, PXXXX=PWM, TXXXX=TIME */
void duoji_Control(uint16_t id, uint16_t pwm, uint16_t time)
{
    uint16_t clamped_pwm = duoji_Clamp_PWM(pwm);

    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    /* 格式化指令，自动补0：ID3位，PWM4位，TIME4位 */
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dP%04dT%04d!", id, clamped_pwm, time);
    duoji_Send_Cmd(duoji_Cmd_Buf);

    if(id == TURNTABLE_duoji_ID)
    {
        g_turntable_current_pwm = clamped_pwm;
    }
}

/* 多舵机控制：{#XXXPXXXXTXXXX!#XXXPXXXXTXXXX!...}  num=舵机数量 */
void duoji_Multi_Control(uint8_t num, uint16_t *id, uint16_t *pwm, uint16_t *time)
{
    if(num < 2 || id == NULL || pwm == NULL || time == NULL)
    {
        return; // 多舵机至少2个，指针非空
    }
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    strcat(duoji_Cmd_Buf, "{"); // 开头加{
    for(uint8_t i=0; i<num; i++)
    {
        char temp[32] = {0};
        uint16_t clamped_pwm = duoji_Clamp_PWM(pwm[i]);
        snprintf(temp, 32, "#%03dP%04dT%04d!", id[i], clamped_pwm, time[i]);
        strcat(duoji_Cmd_Buf, temp);

        if(id[i] == TURNTABLE_duoji_ID)
        {
            g_turntable_current_pwm = clamped_pwm;
        }
    }
    strcat(duoji_Cmd_Buf, "}"); // 结尾加}
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 设置舵机ID：#XXXPIDYYY!  XXX=原ID，YYY=新ID */
void duoji_Set_ID(uint16_t old_id, uint16_t new_id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPID%03d!", old_id, new_id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 读取舵机ID：#XXXPID! */
void duoji_Read_ID(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPID!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 读取舵机版本：#XXXPVER! */
void duoji_Read_Ver(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPVER!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 释放扭力：#XXXPULK! */
void duoji_Release_Torque(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPULK!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 恢复扭力：#XXXPULR! */
void duoji_Recover_Torque(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPULR!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 设置工作模式：#XXXPMODX! */
void duoji_Set_Mode(uint16_t id, duoji_Mode_t mode)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPMOD%d!", id, mode);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 读取工作模式：#XXXPMOD! */
void duoji_Read_Mode(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPMOD!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 读取舵机位置：#XXXPRAD! */
void duoji_Read_Pos(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPRAD!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 暂停舵机：#XXXPDPT! */
void duoji_Pause(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPDPT!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 继续舵机：#XXXPDCT! */
void duoji_Continue(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPDCT!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 停止舵机（不可继续）：#XXXPDST! */
void duoji_Stop(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPDST!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 设置波特率：#XXXPBDX! */
void duoji_Set_Baud(uint16_t id, duoji_Baud_t baud)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPBD%d!", id, baud);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 矫正中值1500：#XXXPSCK! */
void duoji_Calib_Mid(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPSCK!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 设置初始值：#XXXPCSD! */
void duoji_Set_StartPos(uint16_t id, uint16_t pwm)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPCSD!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 设置开机模式：#XXXPCSMX! */
void duoji_Set_PwrMode(uint16_t id, duoji_PwrMode_t mode)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPCSM%d!", id, mode);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 读取开机模式：#XXXPCSM! */
void duoji_Read_PwrMode(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPCSM!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 设置当前位置为最小值：#XXXPSMI! */
void duoji_Set_Min(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPSMI!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 设置当前位置为最大值：#XXXPSMX! */
void duoji_Set_Max(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPSMX!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 除ID外恢复出厂：#XXXPCLE0! */
void duoji_Reset_ExceptID(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPCLE0!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 全恢复出厂：#XXXPCLE! */
void duoji_Reset_All(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPCLE!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

/* 读取温度和电压：#XXXPRTV! */
void duoji_Read_Temp_Volt(uint16_t id)
{
    memset(duoji_Cmd_Buf, 0, duoji_CMD_BUF_SIZE);
    snprintf(duoji_Cmd_Buf, duoji_CMD_BUF_SIZE, "#%03dPRTV!", id);
    duoji_Send_Cmd(duoji_Cmd_Buf);
}

uint16_t duoji_Turntable_Get_Current_PWM(void)
{
    return g_turntable_current_pwm;
}

void duoji_Turntable_Sync_Current_PWM(uint16_t pwm)
{
    g_turntable_current_pwm = duoji_Clamp_PWM(pwm);
}

void duoji_Turntable_Reset(void)
{
    duoji_Turntable_Ensure_Position_Mode();
    duoji_Control(TURNTABLE_duoji_ID, TURNTABLE_PWM_CENTER, TURNTABLE_DEFAULT_MOVE_TIME);
    g_turntable_output_preferred_dir = duoji_TURNTABLE_CW;
}

void duoji_Turntable_Set_Start_Position(void)
{
    duoji_Turntable_Ensure_Position_Mode();
    duoji_Control(TURNTABLE_duoji_ID, TURNTABLE_PWM_START_POSITION, 1000);
    g_turntable_output_preferred_dir = duoji_TURNTABLE_CW;
}

void duoji_Turntable_Sync_Start_Position(void)
{
    /* Software-only sync: this does not send a UART command to the servo. */
    g_turntable_current_pwm = TURNTABLE_PWM_START_POSITION;
    g_turntable_output_preferred_dir = duoji_TURNTABLE_CW;
}

void output_ball_alternating(uint8_t ball_number)
{
    uint16_t target_pwm = 0U;
    uint16_t pwm_delta = 0U;
    duoji_TurntableDir_t dir = duoji_TURNTABLE_CW;

    if ((ball_number < 1U) || (ball_number > TURNTABLE_OUTPUT_BALL_COUNT))
    {
        return;
    }

    target_pwm = duoji_Turntable_Get_Output_Target_PWM(ball_number);

    if (target_pwm == g_turntable_current_pwm)
    {
        return;
    }

    dir = duoji_Turntable_Select_Output_Direction(target_pwm, &pwm_delta);
    duoji_Turntable_Rotate(dir, pwm_delta, TURNTABLE_DEFAULT_MOVE_TIME);
    g_turntable_output_preferred_dir = dir;
}

void output_ball_alternating_second(uint8_t ball_number)
{
    uint16_t target_pwm = 0U;
    uint16_t pwm_delta = 0U;
    duoji_TurntableDir_t dir = duoji_TURNTABLE_CW;

    if ((ball_number < 1U) || (ball_number > TURNTABLE_SECOND_BALL_COUNT))
    {
        return;
    }

    target_pwm = duoji_Turntable_Get_Second_Output_Target_PWM(ball_number);

    if (target_pwm == g_turntable_current_pwm)
    {
        return;
    }

    dir = duoji_Turntable_Select_Output_Direction(target_pwm, &pwm_delta);
    duoji_Turntable_Rotate(dir, pwm_delta, TURNTABLE_DEFAULT_MOVE_TIME);
    g_turntable_output_preferred_dir = dir;
}

void duoji_Set_ID1_Angle(float angle_deg)
{
    duoji_Set_ID1_Angle_Time(angle_deg, AUX_duoji_DEFAULT_MOVE_TIME);
}

void duoji_Set_ID2_Angle(float angle_deg)
{
    duoji_Set_ID2_Angle_Time(angle_deg, AUX_duoji_DEFAULT_MOVE_TIME);
}

void duoji_Set_ID2_Angle_guanjun(float angle_deg)
{
    duoji_Set_ID2_Angle_Time(angle_deg, 900);
}
void duoji_Set_ID2_Angle_yajun(float angle_deg)
{
    duoji_Set_ID2_Angle_Time(angle_deg, 900);
}
void duoji_Set_ID1_Angle_guanjun(float angle_deg)
{
    duoji_Set_ID1_Angle_Time(angle_deg, 900);
}
void duoji_Set_ID1_Angle_yajun(float angle_deg)
{
    duoji_Set_ID1_Angle_Time(angle_deg, 900);
}




void duoji_Set_ID1_Angle_Time(float angle_deg, uint16_t time)
{
    int32_t target_pwm = (int32_t)duoji_ID1_ZERO_PWM + duoji_Angle_To_PWM_Delta(angle_deg);

    duoji_Control(duoji_ID1, duoji_Clamp_PWM(target_pwm), time);
}

void duoji_Set_ID2_Angle_Time(float angle_deg, uint16_t time)
{
    int32_t target_pwm = (int32_t)duoji_ID2_ZERO_PWM - duoji_Angle_To_PWM_Delta(angle_deg);

    duoji_Control(duoji_ID2, duoji_Clamp_PWM(target_pwm), time);
}
void task1_1_step1(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_1_step2(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_1_step3(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_1_step4(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_1_finish(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_36_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_2_step1(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_36_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_2_step2(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_2_step3(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_2_step4(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task1_2_step5(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}

void task2_1_step1(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task2_1_step2(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, 410, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task2_1_step3(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CCW, TURNTABLE_PWM_STEP_36_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task2_2_step1(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_36_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task2_2_step2(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void task2_2_step3(void)
{
    duoji_Turntable_Rotate(duoji_TURNTABLE_CW, TURNTABLE_PWM_STEP_72_DEG, TURNTABLE_DEFAULT_MOVE_TIME);
}
void duoji_tc()
{
    duoji_Set_ID1_Angle(-118);//负值向上抬升 靠近转盘的舵机
    HAL_Delay(1);
    duoji_Set_ID2_Angle(17); //正值向上抬升 
    HAL_Delay(1);
}
void duoji_tc_1()
{
    duoji_Set_ID1_Angle(34);//负值向上抬升 靠近转盘的舵机
    HAL_Delay(1);
    duoji_Set_ID2_Angle(260); //正值向上抬升 
    HAL_Delay(1);
}
void duoji_tc_2()
{
    duoji_Set_ID1_Angle_yajun(-122);//负值向上抬升 靠近转盘的舵机
    HAL_Delay(1);
    duoji_Set_ID2_Angle_yajun(17); //正值向上抬升 
    HAL_Delay(1);
}
void yajun_1()
{
    duoji_Set_ID1_Angle_yajun(-85);//负值向上抬升 靠近转盘的舵机
    HAL_Delay(1);
    duoji_Set_ID2_Angle_yajun(56); //正值向上抬升 
    HAL_Delay(1);
}
void yajun_2()
{
    duoji_Set_ID1_Angle_yajun(-93);
    HAL_Delay(1);
    duoji_Set_ID2_Angle_yajun(25);
    HAL_Delay(1);
}
void guanjun_1()
{
   duoji_Set_ID1_Angle_guanjun(-68);
    HAL_Delay(1);
    duoji_Set_ID2_Angle_guanjun(71); //正值向上抬升 
    HAL_Delay(1);
}
void guanjun_2()
{
    duoji_Set_ID1_Angle_guanjun(-68);//负值向上抬升 靠近转盘的舵机
    HAL_Delay(1);
    duoji_Set_ID2_Angle_guanjun(37); //正值向上抬升 
    HAL_Delay(1);
}
volatile uint32_t timer2_tick_count = 0; 
#define TARGET_20S_COUNT  2000  
//duoji_Init();
// if (boot_requires_duoji_rehome())
// {
//   duoji_Turntable_Set_Start_Position();
//   HAL_Delay(450);
// }
// else
// {
//   duoji_Turntable_Sync_Start_Position();
// }
// HAL_Delay(1);
// duoji_Set_ID1_Angle(30);
// HAL_Delay(1);
// duoji_Set_ID2_Angle(170);
// HAL_Delay(5000);
// //舵机恢复转盘转动模式
// duoji_Set_ID1_Angle(0);
// HAL_Delay(1);
// duoji_Set_ID2_Angle(0);
// HAL_Delay(6000);
// //任务1舵机收物块
// task1_1_step1(); //第1个进
// HAL_Delay(3000);
// task1_1_step2(); //第2个进
// HAL_Delay(3000);
// task1_1_step3(); //第3个进
// HAL_Delay(3000); 
// task1_1_step4(); //第4个进
// HAL_Delay(3000);
// task1_1_finish(); //第5个进
// //舵机放物块
// HAL_Delay(3000);
// task1_2_step1(); //第1个出
// HAL_Delay(3000);
// task1_2_step2(); //第2个出
// HAL_Delay(3000);
// task1_2_step3(); //第3个出
// HAL_Delay(3000);
// task1_2_step4(); //第4个出
// HAL_Delay(3000);
// task1_2_step5(); //第5个出
// HAL_Delay(3000);
// //任务2舵机收物块
// task2_1_step1(); //第1个进
// HAL_Delay(3000);
// task2_1_step2(); //第2个进
// HAL_Delay(3000);
// task2_1_step3(); //第3个进
// HAL_Delay(3000);
// //任务2舵机放物块 
// task2_2_step1(); //第1个出
// HAL_Delay(3000);
// task2_2_step2(); //第2个出
// HAL_Delay(3000);
// task2_2_step3(); //第3个出
// HAL_Delay(3000);

