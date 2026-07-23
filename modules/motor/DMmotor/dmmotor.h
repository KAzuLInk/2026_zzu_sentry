#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-200.0f)
#define DM_V_MAX  200.0f
#define DM_T_MIN  (-10.0f)
#define DM_T_MAX   10.0f
#define DM_KP_MIN  0.0f
#define DM_KP_MAX  5.0f
#define DM_KD_MIN  0.0f
#define DM_KD_MAX  5.0f

typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
}DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
}DMMotor_Send_s;
typedef struct 
{
    DM_Motor_Measure_s measure;
    Motor_Control_Setting_s motor_settings;
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    Motor_Working_Type_e stop_flag;
    CANInstance *motor_can_instace;
    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;
    float p_max;   // 位置映射范围
    float v_max;   // 速度映射范围  
    float t_max;   // 扭矩映射范围
    float kp_max;   // Kp 范围上限
    float kd_max;   // Kd 范围上限
    uint8_t native_mode;        // 目标控制模式
    uint8_t cached_native_mode;  // 已确认切换成功的模式(0=未知)
    // 寄存器读写响应捕获
    uint8_t  reg_pending;        // 1=等待响应中
    uint8_t  reg_addr;           // 请求的寄存器地址
    union {
        float    f;              // float型寄存器 (KP, KI, PMAX...)
        uint32_t u32;            // uint32型寄存器 (CTRL_MODE, MST_ID...)
    } reg_value;                 // 响应数据
}DMMotorInstance;

/* 手册原生控制模式 (寄存器0x0A) */
#define DM_NATIVE_MODE_MIT      1
#define DM_NATIVE_MODE_POSVEL   2  // 位置速度
#define DM_NATIVE_MODE_VEL      3  // 速度
#define DM_NATIVE_MODE_POSCUR   4  // 力位混控

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
void DMMotorSwitchMode(DMMotorInstance *motor, uint8_t native_mode); // 切换原生控制模式(含读→失能→写→存→确认→使能)
#endif // !DMMOTOR