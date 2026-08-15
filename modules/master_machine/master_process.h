#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define VISION_RECV_SIZE 36u // 原来是18.可以正常接收yaw和pitch,现在改成36测试接收第三个数据shoot。
#define Nav_RECV_SIZE 22u
#define VISION_SEND_SIZE 42u
#define CHASSIS_MOTOR_COUNT 4 // 底盘电机数量
#define GIMBAL_MOTOR_COUNT 2  // 云台电机数量(yaw, pitch)

#pragma pack(1)
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;
	// 自瞄相关变量
	float yaw;
	float pitch;
	uint8_t shoot;
	// 导航相关变量
	float vx;
	float vy;
	float wz;
	float spin;
} Vision_Recv_s;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

	// 自瞄相关变量
	float yaw;
	float pitch;
	uint8_t shoot;
	// 导航相关变量
	float vx;
	float vy;
	float wz;
	float spin;
} Nav_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	// Enemy_Color_e enemy_color;
	// Work_Mode_e work_mode;
	// Bullet_Speed_e bullet_speed;

	float yaw;
	float pitch;
	float roll;


} Vision_Send_s;


/* 电池信息回传，msg_id=0x13，payload=5B */
typedef struct
{
	uint16_t voltage;   // 电压 (mV)
	uint16_t current;   // 电流 (mA)
	uint8_t  capacity;  // 电量百分比 (0~100)
} Vision_Battery_Send_s;

/* 机器人状态回传，msg_id=0x14，payload=4B */
typedef struct
{
	uint8_t  mode;   // 机器人模式（当前取裁判系统 game_progress）
	uint16_t hp;     // 血量
	uint8_t  error;  // 错误码 (0=正常)
} Vision_Status_Send_s;

/* ==================== 上行反馈结构（电控 -> 上位机） ==================== */

/* 单个电机反馈：speed int16 + pos int32 + cur int16 = 8B（需 pack(1)） */
typedef struct
{
    int16_t speed;
    int32_t pos;
    int16_t cur;
} Vision_Send_motor;

/* 底盘 4 电机反馈，msg_id=0x10，payload=32B */
typedef struct
{
    Vision_Send_motor motors[CHASSIS_MOTOR_COUNT];
} Vision_Chassis_Motors_s;

/* 云台 2 电机反馈，msg_id=0x11，payload=16B */
typedef struct
{
    Vision_Send_motor motors[GIMBAL_MOTOR_COUNT];
} Vision_Gimbal_Motors_s;

/* IMU 数据，msg_id=0x12，payload=24B */
typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} Vision_IMU_Send_s;

/* ==================== 下行接收结构（上位机 -> 电控） ==================== */

/* 综合控制 fix_control，msg_id=0x05，payload=22B */
typedef struct
{
    uint8_t flags; // bit0=锁敌
    float yaw;
    float pitch;
    uint8_t fire;  // 0/1 开火
    float vx;
    float vy;
    float vz;      // 与视觉组约定：复用为 spin（小陀螺旋转角速度）
} Vision_Fix_Ctrl_s;
#pragma pack()

/**
 * @brief 调用此函数初始化视觉的 USB 虚拟串口通信（VCP）
 */
Vision_Recv_s *VisionInit(void);

/**
 * @brief 设置上行 IMU 姿态数据（电控 -> 上位机）
 */
void VisionSetAltitude(float yaw, float pitch, float roll);

/**
 * @brief 设置电池信息回传数据（电控 -> 上位机）
 */
void VisionSetBattery(uint16_t voltage, uint16_t current, uint8_t capacity);

/**
 * @brief 设置机器人状态回传数据（电控 -> 上位机）
 */
void VisionSetStatus(uint8_t mode, uint16_t hp, uint8_t error);

/* ====================== 上行反馈设置函数（电控 -> 上位机） ====================== */
void VisionSetChassisMotors(int16_t *speed, int32_t *pos, int16_t *cur);
void VisionSetGimbalMotors(int16_t *speed, int32_t *pos, int16_t *cur);
void VisionSetIMU(float yaw, float pitch, float roll, float gx, float gy, float gz);

/* ====================== 上行反馈发送函数 ====================== */
void VisionSendChassis(void);  // msg_id=0x10
void VisionSendGimbal(void);   // msg_id=0x11
void VisionSendIMU(void);      // msg_id=0x12
void VisionSendBattery(void);  // msg_id=0x13
void VisionSendStatus(void);   // msg_id=0x14
void Vision_Send_All(void);    // 状态机轮流发送


#endif // !MASTER_PROCESS_H