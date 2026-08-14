#ifndef __SEASKY_PROTOCOL_H
#define __SEASKY_PROTOCOL_H

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define PROTOCOL_CMD_ID 0XA5
#define OFFSET_BYTE 8 // 出数据段外，其他部分所占字节数

/* ==================== 旧协议（保留，供旧 UART 视觉链路使用） ==================== */

typedef struct
{
	struct
	{
		uint8_t sof;
		uint16_t data_length;
		uint8_t crc_check; // 帧头CRC校验
	} header;			   // 数据帧头
	uint16_t cmd_id;	   // 数据ID
	uint16_t frame_tail;   // 帧尾CRC校验
} protocol_rm_struct;

/*更新发送数据帧，并计算发送数据帧长度*/
void get_protocol_send_data(uint16_t send_id,		 // 信号id
							uint16_t flags_register, // 16位寄存器
							float *tx_data,			 // 待发送的float数据
							uint8_t float_length,	 // float的数据长度
							uint8_t *tx_buf,		 // 待发送的数据帧
							uint16_t *tx_buf_len);	 // 待发送的数据帧长度

/*接收数据处理*/
uint16_t get_protocol_info(uint8_t *rx_buf,			 // 接收到的原始数据
						   uint16_t *flags_register, // 接收数据的16位寄存器地址
						   uint8_t *rx_data);		 // 接收的float数据存储地址

/* ==================== V1 协议（USB 虚拟串口，见 USB协议与类使用说明_V1.md） ==================== */

/*
 * Seasky V1 帧结构：
 * ┌───────┬─────────┬────────┬───────────┬──────────┐
 * │ 0xA5  │  Len    │ Msg_ID │  Payload  │  CRC-16  │
 * │ 1B    │ 2B LE   │ 1B     │  0~N B    │  2B LE   │
 * └───────┴─────────┴────────┴───────────┴──────────┘
 * 帧总长 = 6 + Len；CRC-16/Modbus 覆盖前 4+Len 字节（不含 CRC 本身）。
 */

/* 消息 ID：主控(上位机) -> 电控(本机) */
#define MSG_ID_CHASSIS_CTRL 0x01 /* 底盘控制 vx,vy,vw (3×float) */
#define MSG_ID_GIMBAL_CTRL  0x02 /* 云台控制 flags+yaw+pitch */
#define MSG_ID_SHOOT_CTRL   0x03 /* 发射控制 fire_mode+fire_speed+bullet_type */
#define MSG_ID_FIX_CTRL     0x05 /* 综合控制 flags+yaw+pitch+fire+vx+vy+vz */
#define MSG_ID_HEARTBEAT    0x0F /* 心跳包（空负载） */

/* 消息 ID：电控(本机) -> 主控(上位机) */
#define MSG_ID_CHASSIS_FB    0x10 /* 底盘电机反馈 4×(int16+int32+int16) */
#define MSG_ID_GIMBAL_FB     0x11 /* 云台电机反馈 2×(int16+int32+int16) */
#define MSG_ID_IMU           0x12 /* IMU 数据 6×float */
#define MSG_ID_BATTERY       0x13 /* 电池 voltage+current+capacity */
#define MSG_ID_ROBOT_STATUS  0x14 /* 状态 mode+hp+error */
#define MSG_ID_HEARTBEAT_ACK 0x1F /* 心跳响应（空负载） */

/* V1 组帧：payload 打包写入 tx_buf，返回帧总长（6+payload_len） */
uint16_t seasky_send(uint8_t msg_id, const uint8_t *payload, uint16_t payload_len,
					 uint8_t *tx_buf);

/* V1 解帧：在 rx_buf(长度 rx_len) 中搜索并校验一帧，
 * msg_id 匹配时拷贝 payload 并返回 payload 长度（>=0）；
 * 未收到完整/匹配帧返回 -1，CRC 错误的假帧头被跳过。 */
int16_t seasky_recv(uint8_t msg_id, const uint8_t *rx_buf, uint16_t rx_len,
					uint8_t *payload, uint16_t *payload_len);

#endif
