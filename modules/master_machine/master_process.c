/**
 * @file master_process.c
 * @author neozng
 * @brief  module for recv&send vision data (V1 协议，USB 虚拟串口 VCP)
 * @version beta
 * @date 2022-11-03
 * @copyright Copyright (c) 2022
 *
 */
#include "master_process.h"
#include "seasky_protocol.h"
#include "daemon.h"
#include "bsp_log.h"
#include "robot_def.h"

static Vision_Recv_s recv_data;
static Vision_Chassis_Motors_s send_chassis_motors;
static Vision_Gimbal_Motors_s send_gimbal_motors;
static Vision_IMU_Send_s send_imu;
static Vision_Battery_Send_s send_battery;
static Vision_Status_Send_s send_status;

/**
 * @brief 设置上行 IMU 姿态数据（电控 -> 上位机）
 * @attention 与旧实现保持一致：pitch/roll 交换（历史轴系约定）
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_imu.yaw = yaw;
    send_imu.pitch = roll;
    send_imu.roll = pitch;
}

/**
 * @brief 设置电池信息回传数据（电控 -> 上位机）
 */
void VisionSetBattery(uint16_t voltage, uint16_t current, uint8_t capacity)
{
    send_battery.voltage = voltage;
    send_battery.current = current;
    send_battery.capacity = capacity;
}

/**
 * @brief 设置机器人状态回传数据（电控 -> 上位机）
 */
void VisionSetStatus(uint8_t mode, uint16_t hp, uint8_t error)
{
    send_status.mode = mode;
    send_status.hp = hp;
    send_status.error = error;
}

#ifdef VISION_USE_VCP

#include "bsp_usb.h"

static uint8_t *vis_recv_buff;
static DaemonInstance *vision_daemon_instance;

/**
 * @brief 离线回调函数，VCP 模式下仅记录警告，USB 栈自行处理重连
 */
static void VisionOfflineCallback(void *id)
{
    UNUSED(id);
    LOGWARNING("[vision] vision offline via VCP, check USB connection.");
}

/**
 * @brief 接收解包回调，由 USB CDC 接收中断调用（CDC_Receive_HS -> usb_rx_callback）
 *        解析上位机下发的控制帧，写入 recv_data。
 */
static void DecodeVision(uint16_t recv_len)
{
    static uint8_t payload[SEASKY_MAX_PAYLOAD_LEN];
    uint16_t len = 0;
    Vision_Fix_Ctrl_s fix;

    // 综合控制 0x05：flags + yaw + pitch + fire + vx + vy + vz（22B）
    if (seasky_recv(MSG_ID_FIX_CTRL, vis_recv_buff, recv_len, payload, &len) >= 0 &&
        len >= sizeof(Vision_Fix_Ctrl_s))
    {
        memcpy(&fix, payload, sizeof(Vision_Fix_Ctrl_s));
        recv_data.yaw   = fix.yaw;
        recv_data.pitch = fix.pitch;
        recv_data.shoot = fix.fire;
        recv_data.vx    = fix.vx;
        recv_data.vy    = fix.vy;
        recv_data.spin  = fix.vz; // 与视觉组约定：vz 复用为 spin（小陀螺旋转角速度），视觉组需在此填 spin 而非 0
        recv_data.target_state = (fix.flags & 0x01) ? READY_TO_FIRE : NO_TARGET;
        DaemonReload(vision_daemon_instance);
        return;
    }

    // 云台控制 0x02：flags + yaw + pitch（9B）
    if (seasky_recv(MSG_ID_GIMBAL_CTRL, vis_recv_buff, recv_len, payload, &len) >= 0 &&
        len >= 9)
    {
        recv_data.target_state = (payload[0] & 0x01) ? READY_TO_FIRE : NO_TARGET;
        memcpy(&recv_data.yaw,   &payload[1], 4);
        memcpy(&recv_data.pitch, &payload[5], 4);
        DaemonReload(vision_daemon_instance);
        return;
    }
}

/**
 * @brief 初始化视觉 VCP 通信
 */
Vision_Recv_s *VisionInit(void)
{
    USB_Init_Config_s conf = {.rx_cbk = DecodeVision};
    vis_recv_buff = USBInit(conf);

    // 为 master process 注册 daemon，用于判断视觉通信是否离线
    Daemon_Init_Config_s daemon_conf = {
        .callback = VisionOfflineCallback, // 离线时调用的回调函数
        .owner_id = NULL,
        .reload_count = 100, // 离线超过 1s 超时
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/* ====================== 上行反馈设置函数（电控 -> 上位机） ====================== */

void VisionSetChassisMotors(int16_t *speed, int32_t *pos, int16_t *cur)
{
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++)
    {
        send_chassis_motors.motors[i].speed = speed[i];
        send_chassis_motors.motors[i].pos   = pos[i];
        send_chassis_motors.motors[i].cur   = cur[i];
    }
}

void VisionSetGimbalMotors(int16_t *speed, int32_t *pos, int16_t *cur)
{
    for (uint8_t i = 0; i < GIMBAL_MOTOR_COUNT; i++)
    {
        send_gimbal_motors.motors[i].speed = speed[i];
        send_gimbal_motors.motors[i].pos   = pos[i];
        send_gimbal_motors.motors[i].cur   = cur[i];
    }
}

void VisionSetIMU(float yaw, float pitch, float roll, float gx, float gy, float gz)
{
    send_imu.yaw   = yaw;
    send_imu.pitch = pitch;
    send_imu.roll  = roll;
    send_imu.gyro_x = gx;
    send_imu.gyro_y = gy;
    send_imu.gyro_z = gz;
}

/* ====================== 上行反馈发送函数 ====================== */

/**
 * @brief 发送底盘电机反馈 msg_id=0x10，payload=32B
 */
void VisionSendChassis(void)
{
    static uint8_t send_buf[VISION_SEND_SIZE];
    uint16_t tx_len = seasky_send(MSG_ID_CHASSIS_FB, (uint8_t *)&send_chassis_motors,
                                  sizeof(send_chassis_motors), send_buf);
    USBTransmit(send_buf, tx_len);
}

/**
 * @brief 发送云台电机反馈 msg_id=0x11，payload=16B
 */
void VisionSendGimbal(void)
{
    static uint8_t send_buf[VISION_SEND_SIZE];
    uint16_t tx_len = seasky_send(MSG_ID_GIMBAL_FB, (uint8_t *)&send_gimbal_motors,
                                  sizeof(send_gimbal_motors), send_buf);
    USBTransmit(send_buf, tx_len);
}

/**
 * @brief 发送 IMU 数据 msg_id=0x12，payload=24B
 */
void VisionSendIMU(void)
{
    static uint8_t send_buf[VISION_SEND_SIZE];
    uint16_t tx_len = seasky_send(MSG_ID_IMU, (uint8_t *)&send_imu,
                                  sizeof(send_imu), send_buf);
    USBTransmit(send_buf, tx_len);
}

/**
 * @brief 发送电池信息 msg_id=0x13，payload=5B
 */
void VisionSendBattery(void)
{
    static uint8_t send_buf[VISION_SEND_SIZE];
    uint16_t tx_len = seasky_send(MSG_ID_BATTERY, (uint8_t *)&send_battery,
                                  sizeof(send_battery), send_buf);
    USBTransmit(send_buf, tx_len);
}

/**
 * @brief 发送机器人状态 msg_id=0x14，payload=4B
 */
void VisionSendStatus(void)
{
    static uint8_t send_buf[VISION_SEND_SIZE];
    uint16_t tx_len = seasky_send(MSG_ID_ROBOT_STATUS, (uint8_t *)&send_status,
                                  sizeof(send_status), send_buf);
    USBTransmit(send_buf, tx_len);
}

/**
 * @brief 综合发送状态机：轮流发送 IMU / 电池 / 状态数据，避免 DMA 连续发送堵塞。
 *        调用频率 100Hz（RobotCMDTask 周期 10ms），10 个相位 = 100ms 一轮：
 *        电池 @ 10Hz，状态 @ 10Hz，IMU @ 80Hz。
 */
void Vision_Send_All(void)
{
    static uint8_t phase = 0;
    phase++;
    if (phase >= 10) phase = 0;

    if (phase == 0)
    {
        VisionSendBattery(); // 电池信息 @ 10Hz
    }
    else if (phase == 1)
    {
        VisionSendStatus();  // 机器人状态 @ 10Hz
    }
    else
    {
        VisionSendIMU();     // IMU @ 80Hz
    }
}

#endif // VISION_USE_VCP
