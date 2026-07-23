/**
 * @file referee_protocol.h
 * @author kidneygood (you@domain.com)
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */

#ifndef referee_protocol_H
#define referee_protocol_H

#include "stdint.h"

/****************************宏定义部分****************************/

#define REFEREE_SOF 0xA5 // 起始字节,协议固定为0xA5
#define Robot_Red 0
#define Robot_Blue 1
#define Communicate_Data_LEN 5 // 自定义交互数据长度，该长度决定了我方发送和他方接收，自定义交互数据协议更改时只需要更改此宏定义即可

#pragma pack(1)

/****************************通信协议格式****************************/

/* 通信协议格式偏移，枚举类型,代替#define声明 */
typedef enum
{
	FRAME_HEADER_Offset = 0,
	CMD_ID_Offset = 5,
	DATA_Offset = 7,
} JudgeFrameOffset_e;

/* 通信协议长度 */
typedef enum
{
	LEN_HEADER = 5, // 帧头长
	LEN_CMDID = 2,	// 命令码长度
	LEN_TAIL = 2,	// 帧尾CRC16

	LEN_CRC8 = 4, // 帧头CRC8校验长度=帧头+数据长+包序号
} JudgeFrameLength_e;

/****************************帧头****************************/
/****************************帧头****************************/

/* 帧头偏移 */
typedef enum
{
	SOF = 0,		 // 起始位
	DATA_LENGTH = 1, // 帧内数据长度,根据这个来获取数据长度
	SEQ = 3,		 // 包序号
	CRC8 = 4		 // CRC8
} FrameHeaderOffset_e;

/* 帧头定义 */
typedef struct
{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t Seq;
	uint8_t CRC8;
} xFrameHeader;

/****************************cmd_id命令码说明****************************/
/****************************cmd_id命令码说明****************************/

/* 命令码ID,用来判断接收的是什么数据 */
typedef enum
{
	ID_game_state = 0x0001,				   // 比赛状态数据
	ID_game_result = 0x0002,			   // 比赛结果数据
	ID_game_robot_survivors = 0x0003,	   // 比赛机器人血量数据
	ID_event_data = 0x0101,				   // 场地事件数据
	//ID_supply_projectile_action = 0x0102,  // 场地补给站动作标识数据
	ID_supply_projectile_booking = 0x0103, // 场地补给站预约子弹数据
	ID_referee_warning = 0x0104,           // 裁判警告数据（新增）
	ID_dart_info = 0x0105,                 // 飞镖发射相关数据（新增）
	ID_game_robot_state = 0x0201,		   // 机器人状态数据
	ID_power_heat_data = 0x0202,		   // 实时功率热量数据
	ID_game_robot_pos = 0x0203,			   // 机器人位置数据
	ID_buff_musk = 0x0204,				   // 机器人增益数据
	ID_aerial_robot_energy = 0x0205,	   // 空中机器人能量状态数据
	ID_robot_hurt = 0x0206,				   // 伤害状态数据
	ID_shoot_data = 0x0207,				   // 实时射击数据
	ID_projectile_allowance = 0x0208,      // 允许发弹量（新增）
	ID_rfid_status = 0x0209,               // RFID模块状态（新增）
	ID_dart_client_cmd = 0x020A,          // 飞镖选手端指令数据（新增）
	ID_ground_robot_position = 0x020B,     // 地面机器人位置数据（新增）
	ID_radar_mark_progress = 0x020C,       // 雷达标记进度数据（新增）
	ID_sentry_info = 0x020D,               // 哨兵自主决策信息同步（新增）
	ID_radar_info = 0x020E,                // 雷达自主决策信息同步（新增）
	ID_student_interactive = 0x0301,	   // 机器人间交互数据
	ID_map_command = 0x0303,               // 可选：小地图交互数据
    ID_custom_controller_data = 0x0306,    // 可选：自定义控制器数据
} CmdID_e;

// /* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
// typedef enum
// {
// 	LEN_game_state = 3,							 // 0x0001
// 	LEN_game_result = 1,						 // 0x0002
// 	LEN_game_robot_HP = 2,						 // 0x0003
// 	LEN_event_data = 4,							 // 0x0101
// 	LEN_supply_projectile_action = 4,			 // 0x0102
// 	LEN_game_robot_state = 13,					 // 0x0201
// 	LEN_power_heat_data = 16,					 // 0x0202
// 	LEN_game_robot_pos = 16,					 // 0x0203
// 	LEN_buff_musk = 1,							 // 0x0204
// 	LEN_aerial_robot_energy = 1,				 // 0x0205
// 	LEN_robot_hurt = 1,							 // 0x0206
// 	LEN_shoot_data = 7,							 // 0x0207
// 	LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301

// } JudgeDataLength_e;

/* 命令码数据段长,根据官方协议来定义长度，还有自定义数据长度 */
 typedef enum
 {
 	LEN_game_state = 11,						 // 0x0001
 	LEN_game_result = 1,						 // 0x0002
 	LEN_game_robot_HP = 32,						 // 0x0003
 	LEN_event_data = 4,							 // 0x0101
 	LEN_supply_projectile_action = 4,			 // 0x0102
	LEN_referee_warning = 3,              		 // 0x0104
	LEN_dart_info = 3,                    		 // 0x0105
	LEN_game_robot_state = 13,					 // 0x0201
 	LEN_power_heat_data = 16,					 // 0x0202
	LEN_game_robot_pos = 12,					 // 0x0203
 	LEN_buff_musk = 8,							 // 0x0204
 	LEN_aerial_robot_energy = 1,				 // 0x0205
 	LEN_robot_hurt = 1,							 // 0x0206
 	LEN_shoot_data = 7,							 // 0x0207
	LEN_projectile_allowance = 8,         		 // 0x0208
	LEN_rfid_status = 5,                  		 // 0x0209
	LEN_dart_client_cmd = 6,              		 // 0x020A
	LEN_ground_robot_position = 36,              // 0x020B
	LEN_radar_mark_progress = 2,          		 // 0x020C
	LEN_sentry_info = 6,                         // 0x020D
	LEN_radar_info = 1,                          // 0x020E
 	LEN_receive_data = 6 + Communicate_Data_LEN, // 0x0301

 } JudgeDataLength_e;

/****************************接收数据的详细说明****************************/
/****************************接收数据的详细说明****************************/

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} ext_game_state_t;

/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef struct
{
	uint8_t winner;
} ext_game_result_t;

/* ID: 0x0003  Byte:  32    比赛机器人血量数据 */
typedef struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t red_5_robot_HP;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t blue_5_robot_HP;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} ext_game_robot_HP_t;

/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef struct
{
	uint32_t event_type;
} ext_event_data_t;

/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef struct
{
	uint8_t supply_projectile_id;
	uint8_t supply_robot_id;
	uint8_t supply_projectile_step;
	uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

/* 新增0x0104数据结构 */
typedef struct
{
    uint8_t level;               // 判罚等级
    uint8_t offending_robot_id;  // 违规机器人ID
    uint8_t count;               // 违规次数
} ext_referee_warning_t;

/* ID: 0x0105  Byte: 3  飞镖发射相关数据 */
typedef struct
{
    uint8_t dart_remaining_time; // 己方飞镖发射剩余时间，单位：秒
    uint16_t dart_info;          // 飞镖信息位掩码
    /* dart_info位定义:
   	 * bit 0-2: 最近一次己方飞镖击中的目标
     * bit 3-5: 对方最近被击中的目标累计被击中计次数
     * bit 6-8: 飞镖此时选定的击打目标
     * bit 9-15: 保留
     */
} ext_dart_info_t;

/* ID: 0X0201  Byte: 13    机器人状态数据 */
typedef struct
{
	uint8_t robot_id; //本机器人ID
  	uint8_t robot_level; //机器人等级
  	uint16_t current_HP;  //机器人当前血量 
  	uint16_t maximum_HP; //机器人血量上限
  	uint16_t shooter_barrel_cooling_value; //机器人射击热量每秒冷却值 
  	uint16_t shooter_barrel_heat_limit; //机器人射击热量上限
  	uint16_t chassis_power_limit;  //机器人底盘功率上限
  	uint8_t power_management_gimbal_output : 1; //bit 0：gimbal口输出，0为无输出，1为 24V输出
  	uint8_t power_management_chassis_output : 1;  //bit 1：chassis口输出，0为无输出，1为24V输出
  	uint8_t power_management_shooter_output : 1; //bit 2：shooter口输出，0为无输出，1为24V输出
} ext_game_robot_state_t;

/* ID: 0X0202  Byte: 16    实时功率热量数据 */
typedef struct
{
	uint16_t chassis_voltage; //保留位// 底盘输出电压，单位：mV
	uint16_t chassis_current;//保留位// 底盘输出电流，单位：mA
  	float chassis_power; //保留位// 底盘功率，单位：W
  	uint16_t buffer_energy; //缓冲能量（单位：J）
  	uint16_t shooter_17mm_1_barrel_heat; //第1个17mm发射机构的射击热量
  	uint16_t shooter_17mm_2_barrel_heat; //第2个17mm发射机构的射击热量
  	uint16_t shooter_42mm_barrel_heat; //42mm发射机构的射击热量
} ext_power_heat_data_t;

/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef struct
{
	float x;
	float y;
	float angle;  // 本机器人测速模块的朝向，单位：度。正北为0度
} ext_game_robot_pos_t;

/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef struct
{
	uint8_t heal_gain;           // 回血增益 (百分比，值为10表示每秒恢复血量上限的10%)
    uint16_t heat_cooling_gain;  // 射击热量冷却增益具体值 (直接值，值为x表示热量冷却增加x/s)
    uint8_t defense_gain;        // 防御增益 (百分比，值为50表示50%防御增益)
    uint8_t negative_defense_gain; // 负防御增益 (百分比，值为30表示-30%防御增益)
    uint16_t attack_gain;        // 攻击增益 (百分比，值为50表示50%攻击增益)
	uint8_t power_rune_buff;    // 剩余能量值比例（十六进制），仅当<50%时有效，否则为0x80
} ext_buff_t;

/* ID: 0x0205  Byte:  1    空中机器人能量状态数据 */
typedef struct
{
	uint8_t attack_time;
} aerial_robot_energy_t;

/* ID: 0x0206  Byte:  1    伤害状态数据 */
typedef struct
{
	uint8_t armor_id : 4;   // 装甲模块ID编号
	uint8_t hurt_type : 4;	// 血量变化类型 (0=弹丸攻击,1=模块离线,5=撞击)
} ext_robot_hurt_t;

/* ID: 0x0207  Byte:  7    实时射击数据 */
typedef struct
{
	uint8_t bullet_type;	// 弹丸类型 (bit定义)
	uint8_t shooter_id;		// 发射机构ID (1=17mm, 3=42mm)
	uint8_t bullet_freq;	// 弹丸射速 (单位：Hz)
	float bullet_speed;		// 弹丸初速度 (单位：m/s)
} ext_shoot_data_t;

/* 新增0x0208数据结构 */
typedef struct
{
    uint16_t projectile_allowance_17mm;		// 17mm弹丸允许发弹量
    uint16_t projectile_allowance_42mm;		// 42mm弹丸允许发弹量
    uint16_t remaining_gold_coin;			// 剩余金币数量
    uint16_t reserved_17mm_projectile_allowance;// 堡垒增益点提供的储备17mm弹丸允许发弹量
} ext_projectile_allowance_t;

/* ID: 0x0209  Byte: 5  RFID模块状态 (新增) */
typedef struct
{
    uint32_t rfid_status;     // RFID卡检测状态位掩码 (bit 0-31)
    uint8_t rfid_status_2;     // 额外的RFID卡检测 (bit 0-1)
    /* rfid_status位定义 (表1-18、19、20):
     * bit 0: 己方基地增益点
     * bit 1: 己方中央高地增益点
     * ... (共32 bits)
     * rfid_status_2位定义:
     * bit 0: 对方地形跨越增益点(隧道)(靠近对方公路一侧下方)
     * bit 1: 对方地形跨越增益点(隧道)(靠近对方公路一侧中间)
     * ... (共6 bits)
     */
} ext_rfid_status_t;

/* ID: 0x020A  Byte: 6  飞镖选手端指令数据 (新增) */
typedef struct
{
    uint8_t dart_launch_opening_status; // 飞镖发射站状态: 1=关闭, 2=开启/关闭中, 0=已开启
    uint8_t reserved;                   // 保留位
    uint16_t target_change_time;        // 切换击打目标时的比赛剩余时间 (单位：秒)
    uint16_t latest_launch_cmd_time;    // 最后一次操作手确定发射指令时的比赛剩余时间
} ext_dart_client_cmd_t;

/* ID: 0x020B  Byte: 36  地面机器人位置数据 (新增) */
typedef struct
{
    float hero_x;       // 己方英雄机器人位置x坐标 (单位：m)
    float hero_y;       // 己方英雄机器人位置y坐标 (单位：m)
    float engineer_x;   // 己方工程机器人位置x坐标
    float engineer_y;   // 己方工程机器人位置y坐标
    float standard_3_x; // 己方3号步兵机器人位置x坐标
    float standard_3_y; // 己方3号步兵机器人位置y坐标
    float standard_4_x; // 己方4号步兵机器人位置x坐标
    float standard_4_y; // 己方4号步兵机器人位置y坐标
    float reserved1;    // 保留位
    float reserved2;    // 保留位
} ext_ground_robot_position_t;

/* ID: 0x020C  Byte: 2  雷达标记进度数据 (新增) */
typedef struct
{
    uint16_t mark_progress; // 标记进度位掩码
    /* bit 0: 对方1号英雄机器人易伤情况
     * bit 1: 对方2号工程机器人易伤情况
     * bit 2: 对方3号步兵机器人易伤情况
     * bit 3: 对方4号步兵机器人易伤情况
     * bit 4: 对方空中机器人特殊标识情况 ✅
     * bit 5: 对方哨兵机器人易伤情况 ✅
     * bit 6: 己方1号英雄机器人特殊标识情况
     * bit 7: 己方2号工程机器人特殊标识情况
     * bit 8: 己方3号步兵机器人特殊标识情况
     * bit 9: 己方4号步兵机器人特殊标识情况
     * bit 10: 己方空中机器人特殊标识情况
     * bit 11: 己方哨兵机器人特殊标识情况
     * bit 12-15: 保留位
     */
} ext_radar_mark_progress_t;

/* ID: 0x020D  Byte: 6  哨兵自主决策信息同步 (新增) */
typedef struct
{
    uint32_t sentry_info;   // 哨兵信息位掩码 (详见协议bit 0-31定义)
    uint16_t sentry_info_2; // 哨兵额外信息
    /* sentry_info位定义:
     * bit 0-10: 除远程兑换外，哨兵机器人成功兑换的允许发弹量
     * bit 11-14: 哨兵机器人成功远程兑换允许发弹量的次数
     * bit 15-18: 哨兵机器人成功远程兑换血量的次数
     * bit 19: 哨兵机器人当前是否可以确认免费复活
     * bit 20: 哨兵机器人当前是否可以兑换立即复活
     * bit 21-30: 哨兵机器人当前若兑换立即复活需要花费的金币数
     * bit 31: 保留位
     * sentry_info_2位定义 (表1-23):
     * bit 0: 哨兵当前是否处于脱战状态
     * bit 1-11: 队伍17mm允许发弹量的剩余可兑换数
     * bit 12-13: 哨兵当前姿态 (1=进攻,2=防御,3=移动)
     * bit 14: 己方能量机关是否能够进入正在激活状态
     * bit 15: 保留位
     */
} ext_sentry_info_t;

/* ID: 0x020E  Byte: 1  雷达自主决策信息同步 (新增) */
typedef struct
{
    uint8_t radar_info; // 雷达信息位掩码
    /* bit 0-1: 雷达是否拥有触发双倍易伤的机会
     * bit 2: 对方是否正在被触发双倍易伤
     * bit 3-4: 己方加密等级 (对方干扰波难度等级)
     * bit 5: 当前是否可以修改密钥
     * bit 6-7: 保留位
     */
} ext_radar_info_t;

/****************************机器人交互数据****************************/
/****************************机器人交互数据****************************/
/* 发送的内容数据段最大为 113 检测是否超出大小限制?实际上图形段不会超，数据段最多30个，也不会超*/
/* 交互数据头结构 */
typedef struct
{
	uint16_t data_cmd_id; // 由于存在多个内容 ID，但整个cmd_id 上行频率最大为 10Hz，请合理安排带宽。注意交互部分的上行频率
	uint16_t sender_ID;
	uint16_t receiver_ID;
} ext_student_interactive_header_data_t;

/* 机器人id */
typedef enum
{
	// 红方机器人ID
	RobotID_RHero = 1,
	RobotID_REngineer = 2,
	RobotID_RStandard1 = 3,
	RobotID_RStandard2 = 4,
	RobotID_RStandard3 = 5,
	RobotID_RAerial = 6,
	RobotID_RSentry = 7,
	RobotID_RRadar = 9,
	// 蓝方机器人ID
	RobotID_BHero = 101,
	RobotID_BEngineer = 102,
	RobotID_BStandard1 = 103,
	RobotID_BStandard2 = 104,
	RobotID_BStandard3 = 105,
	RobotID_BAerial = 106,
	RobotID_BSentry = 107,
	RobotID_BRadar = 109,
} Robot_ID_e;

/* 交互数据ID */
typedef enum
{
	UI_Data_ID_Del = 0x100,
	UI_Data_ID_Draw1 = 0x101,
	UI_Data_ID_Draw2 = 0x102,
	UI_Data_ID_Draw5 = 0x103,
	UI_Data_ID_Draw7 = 0x104,
	UI_Data_ID_DrawChar = 0x110,

	/* 自定义交互数据部分 */
	Communicate_Data_ID = 0x0200,

} Interactive_Data_ID_e;
/* 交互数据长度 */
typedef enum
{
	Interactive_Data_LEN_Head = 6,
	UI_Operate_LEN_Del = 2,
	UI_Operate_LEN_PerDraw = 15,
	UI_Operate_LEN_DrawChar = 15 + 30,

	/* 自定义交互数据部分 */
	// Communicate_Data_LEN = 5,

} Interactive_Data_Length_e;

/****************************自定义交互数据****************************/
/*
	学生机器人间通信 cmd_id 0x0301，内容 ID:0x0200~0x02FF
	自定义交互数据 机器人间通信：0x0301。
	发送频率：上限 10Hz
*/
// 自定义交互数据协议，可更改，更改后需要修改最上方宏定义数据长度的值
typedef struct
{
	uint8_t data[Communicate_Data_LEN]; // 数据段,n需要小于113
} robot_interactive_data_t;

// 机器人交互信息_发送
typedef struct
{
	xFrameHeader FrameHeader;
	uint16_t CmdID;
	ext_student_interactive_header_data_t datahead;
	robot_interactive_data_t Data; // 数据段
	uint16_t frametail;
} Communicate_SendData_t;
// 机器人交互信息_接收
typedef struct
{
	ext_student_interactive_header_data_t datahead;
	robot_interactive_data_t Data; // 数据段
} Communicate_ReceiveData_t;

/****************************UI交互数据****************************/

/* 图形数据 */
typedef struct
{
	uint8_t graphic_name[3];
	uint32_t operate_tpye : 3;
	uint32_t graphic_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;
	uint32_t start_angle : 9;
	uint32_t end_angle : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t radius : 10;
	uint32_t end_x : 11;
	uint32_t end_y : 11;
} Graph_Data_t;

typedef struct
{
	Graph_Data_t Graph_Control;
	uint8_t show_Data[30];
} String_Data_t; // 打印字符串数据

/* 删除操作 */
typedef enum
{
	UI_Data_Del_NoOperate = 0,
	UI_Data_Del_Layer = 1,
	UI_Data_Del_ALL = 2, // 删除全部图层，后面的参数已经不重要了。
} UI_Delete_Operate_e;

/* 图形配置参数__图形操作 */
typedef enum
{
	UI_Graph_ADD = 1,
	UI_Graph_Change = 2,
	UI_Graph_Del = 3,
} UI_Graph_Operate_e;

/* 图形配置参数__图形类型 */
typedef enum
{
	UI_Graph_Line = 0,		// 直线
	UI_Graph_Rectangle = 1, // 矩形
	UI_Graph_Circle = 2,	// 整圆
	UI_Graph_Ellipse = 3,	// 椭圆
	UI_Graph_Arc = 4,		// 圆弧
	UI_Graph_Float = 5,		// 浮点型
	UI_Graph_Int = 6,		// 整形
	UI_Graph_Char = 7,		// 字符型

} UI_Graph_Type_e;

/* 图形配置参数__图形颜色 */
typedef enum
{
	UI_Color_Main = 0, // 红蓝主色
	UI_Color_Yellow = 1,
	UI_Color_Green = 2,
	UI_Color_Orange = 3,
	UI_Color_Purplish_red = 4, // 紫红色
	UI_Color_Pink = 5,
	UI_Color_Cyan = 6, // 青色
	UI_Color_Black = 7,
	UI_Color_White = 8,

} UI_Graph_Color_e;

#pragma pack()

#endif
