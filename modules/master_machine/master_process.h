#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"
#include "seasky_protocol.h"

#define Nav_RECV_SIZE 22u
#define VISION_SEND_SIZE 64u // IMU 帧总长 6+56=62B
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

/* IMU 数据，msg_id=0x12，payload=56B（14×float） */
typedef struct
{
    float yaw;            // 大云台 IMU 偏航角(度)
    float pitch;          // 大云台 IMU 俯仰角(度)
    float roll;           // 大云台 IMU 横滚角(度)
    float gyro_x;         // 大云台 X 轴角速度(rad/s)
    float gyro_y;         // 大云台 Y 轴角速度(rad/s)
    float gyro_z;         // 大云台 Z 轴角速度(rad/s)
    float small_yaw;      // 小云台偏航角(度)
    float small_pitch;    // 小云台俯仰角(度)
    float chassis_yaw;    // 底盘 IMU 偏航角(度)
    float chassis_pitch;  // 底盘 IMU 俯仰角(度)
    float chassis_roll;   // 底盘 IMU 横滚角(度)
    float chassis_gyro_x; // 底盘 IMU X 轴角速度(rad/s)
    float chassis_gyro_y; // 底盘 IMU Y 轴角速度(rad/s)
    float chassis_gyro_z; // 底盘 IMU Z 轴角速度(rad/s)
} Vision_IMU_Send_s;

/* ==================== 下行接收结构（上位机 -> 电控） ==================== */

/* 综合控制 fix_control，msg_id=0x05，payload=22B */
typedef struct
{
    uint8_t flags; // bit0=锁敌 bit1=小陀螺开关
    float yaw;
    float pitch;
    uint8_t fire;  // 0/1 开火
    float vx;
    float vy;
    float vz;      // 底盘旋转角速度，接收后存入 recv_data.wz
} Vision_Fix_Ctrl_s;
#pragma pack()

/**
 * @brief 调用此函数初始化视觉的 USB 虚拟串口通信（VCP）
 */
Vision_Recv_s *VisionInit(void);

/**
 * @brief 获取视觉接收数据指针（不重复初始化 USB/daemon，供云台等其它模块直接读取）
 */
Vision_Recv_s *VisionGetRecv(void);

/**
 * @brief 视觉通信是否在线（VCP daemon 状态）
 */
uint8_t VisionIsOnline(void);

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
/* 只设置三轴角速度（rad/s），欧拉角仍由 VisionSetAltitude 设置 */
void VisionSetGyro(float gx, float gy, float gz);
/* 设置小yaw/pitch（0x12 末尾两字段，单位均为度） */
void VisionSetSmallYawPitch(float small_yaw, float small_pitch);
/* 设置底盘板 IMU（0x12 末尾 6 字段：姿态为度，角速度为 rad/s） */
void VisionSetChassisIMU(float yaw, float pitch, float roll, float gx, float gy, float gz);

/* ====================== 上行反馈发送函数 ====================== */
void VisionSendChassis(void);  // msg_id=0x10
void VisionSendGimbal(void);   // msg_id=0x11
void VisionSendIMU(void);           // msg_id=0x12
void VisionSendBattery(void);       // msg_id=0x13
void VisionSendStatus(void);        // msg_id=0x14
void VisionSendHeartbeatAck(void);  // msg_id=0x1F（空负载，收到 0x0F 心跳后回）
void Vision_Send_All(void);         // 状态机轮流发送


#endif // !MASTER_PROCESS_H