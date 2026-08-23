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
 */
void VisionSetAltitude(float yaw, float pitch, float roll)
{
    send_imu.yaw = yaw;
    send_imu.pitch = pitch;
    send_imu.roll = roll;
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

/**
 * @brief 获取视觉接收数据指针（不重复初始化 USB/daemon，供云台等其它模块直接读取）
 */
Vision_Recv_s *VisionGetRecv(void)
{
    return &recv_data;
}

#ifdef VISION_USE_VCP

#include "bsp_usb.h"

#define VISION_RX_BUF_SIZE 1024u /* 跨包累计缓冲，需 ≥ USB 单包(512B) + 最大帧(6+128B) */

static uint8_t *vis_recv_buff;          /* 指向 USB CDC 每包接收缓冲（每包被覆盖） */
static uint8_t vis_rx_buf[VISION_RX_BUF_SIZE]; /* 跨 USB 包累计的接收缓冲 */
static uint16_t vis_rx_len;             /* vis_rx_buf 中有效字节数 */
static DaemonInstance *vision_daemon_instance;
static volatile uint8_t heartbeat_ack_pending; /* 收到 0x0F 心跳后置位，任务上下文回 0x1F */

/**
 * @brief 离线回调函数，VCP 模式下仅记录警告，USB 栈自行处理重连
 */
static void VisionOfflineCallback(void *id)
{
    UNUSED(id);
    /* 视觉断连：清零所有视觉来源的控制量，防止车保持最后一帧速度 / 继续开火 / 小陀螺空转 */
    recv_data.vx    = 0;
    recv_data.vy    = 0;
    recv_data.wz    = 0;
    recv_data.yaw   = 0.0f;
    recv_data.pitch = 0.0f;
    recv_data.shoot = 0;
    recv_data.spin  = 0.0f;
    recv_data.target_state = NO_TARGET;
    LOGWARNING("[vision] vision offline via VCP, check USB connection.");
}

/**
 * @brief 接收解包回调，由 USB CDC 接收中断调用（CDC_Receive_HS -> usb_rx_callback）
 *        先把本包数据追加到跨包累计缓冲，再循环弹出完整帧写入 recv_data，
 *        以正确处理 USB 粘包/半包（单帧拆成多包、多帧并进一包）。
 */
static void DecodeVision(uint16_t recv_len)
{
    static uint8_t payload[SEASKY_MAX_PAYLOAD_LEN];
    uint16_t len = 0;
    int16_t msg_id;

    /* 追加新收到的数据到累计缓冲，并做溢出保护 */
    if (recv_len > 0 && vis_recv_buff != NULL)
    {
        if (recv_len >= VISION_RX_BUF_SIZE) /* 单包异常过大（理论 ≤512B）：整缓冲丢弃 */
        {
            vis_rx_len = 0;
        }
        else if (vis_rx_len + recv_len > VISION_RX_BUF_SIZE)
        {
            /* 溢出：丢弃最旧一半，保留尾部（半帧优先保留） */
            uint16_t keep = vis_rx_len / 2;
            memmove(vis_rx_buf, vis_rx_buf + keep, vis_rx_len - keep);
            vis_rx_len -= keep;
        }
        memcpy(&vis_rx_buf[vis_rx_len], vis_recv_buff, recv_len);
        vis_rx_len += recv_len;
    }

    /* 循环解出所有完整帧，半帧留在缓冲里等下次补数 */
    while ((msg_id = seasky_pop_frame(vis_rx_buf, &vis_rx_len, payload, &len)) > 0)
    {
        Vision_Fix_Ctrl_s fix;

        // 综合控制 0x05：flags + yaw + pitch + fire + vx + vy + vz（22B）
        if (msg_id == MSG_ID_FIX_CTRL && len >= sizeof(Vision_Fix_Ctrl_s))
        {
            memcpy(&fix, payload, sizeof(Vision_Fix_Ctrl_s));
            recv_data.yaw   = fix.yaw;
            recv_data.pitch = fix.pitch;
            recv_data.shoot = fix.fire;
            recv_data.vx    = fix.vx;
            recv_data.vy    = fix.vy;
            recv_data.wz    = fix.vz * 57.295779513f; // 协议 vz(rad/s) -> 底盘转速 wz(°/s)
            recv_data.spin  = (fix.flags & 0x02) ? 1.0f : 0.0f; // bit1 = 小陀螺开关
            recv_data.target_state = (fix.flags & 0x01) ? READY_TO_FIRE : NO_TARGET;
            DaemonReload(vision_daemon_instance);
        }
        // 底盘控制 0x01：flags + vx + vy + vw（13B），vw 同为 rad/s
        else if (msg_id == MSG_ID_CHASSIS_CTRL && len >= 13)
        {
            memcpy(&recv_data.vx, &payload[1], 4);
            memcpy(&recv_data.vy, &payload[5], 4);
            memcpy(&recv_data.wz, &payload[9], 4);
            recv_data.wz  *= 57.295779513f; // rad/s -> °/s
            recv_data.spin = (payload[0] & 0x02) ? 1.0f : 0.0f; // bit1 = 小陀螺开关
            DaemonReload(vision_daemon_instance);
        }
        // 发射控制 0x03：fire_mode + fire_speed + bullet_type（6B）
        else if (msg_id == MSG_ID_SHOOT_CTRL && len >= 6)
        {
            recv_data.shoot = (payload[0] >= 1) ? 1 : 0; // fire_mode: 0=停,1=单发,2=连发 → 非 0 即开火
            DaemonReload(vision_daemon_instance);
        }
        // 云台控制 0x02：flags + yaw + pitch（9B）
        else if (msg_id == MSG_ID_GIMBAL_CTRL && len >= 9)
        {
            recv_data.target_state = (payload[0] & 0x01) ? READY_TO_FIRE : NO_TARGET;
            memcpy(&recv_data.yaw,   &payload[1], 4);
            memcpy(&recv_data.pitch, &payload[5], 4);
            DaemonReload(vision_daemon_instance);
        }
        // 心跳包 0x0F：空负载，刷新在线状态并置位回包标志（回包在任务上下文发送）
        else if (msg_id == MSG_ID_HEARTBEAT)
        {
            heartbeat_ack_pending = 1;
            DaemonReload(vision_daemon_instance);
        }
        /* 其余未用 msg_id 已由 pop 消费，直接丢弃 */
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
        .alarm_silent = 1,   // 视觉掉线暂不触发蜂鸣器报警
        .name = "vision",
    };
    vision_daemon_instance = DaemonRegister(&daemon_conf);

    return &recv_data;
}

/**
 * @brief 视觉通信是否在线（VCP daemon 状态）
 */
uint8_t VisionIsOnline(void)
{
    return DaemonIsOnline(vision_daemon_instance);
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

void VisionSetGyro(float gx, float gy, float gz)
{
    send_imu.gyro_x = gx;
    send_imu.gyro_y = gy;
    send_imu.gyro_z = gz;
}

void VisionSetSmallYawPitch(float small_yaw, float small_pitch)
{
    send_imu.small_yaw = small_yaw;
    send_imu.small_pitch = small_pitch;
}

void VisionSetChassisIMU(float yaw, float pitch, float roll, float gx, float gy, float gz)
{
    send_imu.chassis_yaw    = yaw;
    send_imu.chassis_pitch  = pitch;
    send_imu.chassis_roll   = roll;
    send_imu.chassis_gyro_x = gx;
    send_imu.chassis_gyro_y = gy;
    send_imu.chassis_gyro_z = gz;
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
 * @brief 发送 IMU 数据 msg_id=0x12，payload=56B
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
 * @brief 发送心跳响应 msg_id=0x1F，payload=0B（空负载）
 */
void VisionSendHeartbeatAck(void)
{
    static uint8_t send_buf[VISION_SEND_SIZE];
    uint16_t tx_len = seasky_send(MSG_ID_HEARTBEAT_ACK, NULL, 0, send_buf);
    USBTransmit(send_buf, tx_len);
}

/**
 * @brief 综合发送状态机：轮流发送 IMU / 电池 / 状态 / 底盘 / 云台数据，避免 DMA 连续发送堵塞。
 *        调用频率 100Hz（RobotCMDTask 周期 10ms），10 个相位 = 100ms 一轮：
 *        电池 @ 10Hz，状态 @ 10Hz，底盘 @ 10Hz，云台 @ 10Hz，IMU @ 60Hz。
 *        收到 0x0F 心跳后在本函数顶部立即回 0x1F（不占相位）。
 */
void Vision_Send_All(void)
{
    static uint8_t phase = 0;

    /* 收到上位机心跳 0x0F → 立即回 0x1F，证明电控在线（不走相位机，保证及时） */
    if (heartbeat_ack_pending)
    {
        heartbeat_ack_pending = 0;
        VisionSendHeartbeatAck();
    }

    phase++;
    if (phase >= 10) phase = 0;

    switch (phase)
    {
    case 0:
        VisionSendBattery(); // 电池信息 @ 10Hz
        break;
    case 1:
        VisionSendStatus();  // 机器人状态 @ 10Hz
        break;
    case 2:
        VisionSendChassis(); // 底盘电机反馈 @ 10Hz
        break;
    case 3:
        VisionSendGimbal();  // 云台电机反馈 @ 10Hz
        break;
    default:
        VisionSendIMU();     // IMU @ 60Hz
        break;
    }
}

#endif // VISION_USE_VCP
