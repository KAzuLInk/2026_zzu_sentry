#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "dmmotor.h"

#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"

#include "remote_control.h"

#define TORQUE_THRESHOLD  6.0f  // 卡弹扭矩阈值 N·m，实测调（太低会误判正常顶弹为卡弹）
#define REVERSE_SPEED     -5.0f  // 回转速度 rad/s（负=反转）
#define REVERSE_TIME     500.0f  // 回转持续时间 ms

// 拨盘发弹参数(Ozone可调, 需按机械实测标定)
volatile float loader_fire_speed = 20.0f;   // 拨盘转速 rad/s
volatile float single_shot_ms =80.0f;     // 单发一发所需拨盘转动时间 ms（20rad/s实测≈33ms/发, 40ms≈1发留余量）
volatile float friction_spinup_ms = 150.0f; // 摩擦轮起转到全速时间 ms(两段式延时)

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r; // 拨盘电机
static DMMotorInstance *loader;//拨弹盘电机注册

// static servo_instance *lid; 需要增加弹舱盖
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static ServoInstance *servo;

static enum {
    LOADER_NORMAL,
    LOADER_REVERSING
}loader_state = LOADER_NORMAL;// 拨盘状态机 


static float reverse_start_time = 0;

// 单发状态机: 每次shoot上升沿只拨一发弹丸
static uint8_t  single_shot_pending = 0;          // 已登记一发, 等摩擦轮起转到位
static uint8_t  single_shot_active = 0;           // 单发进行中
static float    single_shot_start = 0;            // 单发起始时间(ms)

// 两段式: 先开摩擦轮, 起转到位后再拨弹
static uint8_t friction_was_on = 0;               // 上一拍摩擦轮状态(上升沿检测)
static float   friction_on_start = 0;             // 摩擦轮开启时间(ms)



extern RC_ctrl_t *rc_data;//遥控器，主要是左边滚轮

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;


void ShootInit()
{
 


    Servo_Init_Config_s servo_conf = {
        .pwm_init_config = {.htim = &htim1,
            .channel = TIM_CHANNEL_1,
            .dutyratio = 0,
            .period = 20000-1
        },
        .servo_id = 1,
        .servo_type = PWM_Servo
    };
    servo = ServoInit(&servo_conf);
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, // 20
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 30000,
                .MaxOut_ = -30000,
                .DeadBand = 0
            },
            .current_PID = { 
                .Kp = 1.1, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 30000,
                .MaxOut_ = -30000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
            friction_config.debug_flag=0;

    friction_config.can_init_config.tx_id = 2,
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.tx_id = 1; // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // // 拨盘电机
    // Motor_Init_Config_s loader_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan2,
    //         .tx_id = 3,
    //     },
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
    //             .Kp = 1000, // 10
    //             .Ki = 0,
    //             .Kd = 0,
    //             .MaxOut = 10000,
    //             .MaxOut_ = -10000
    //         },
    //         .speed_PID = {
    //             .Kp = 10, // 10
    //             .Ki = 0, // 1
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit | PID_ErrorHandle,
    //             .IntegralLimit = 5000,
    //             .MaxOut = 7000,
    //             .MaxOut_ = -7000
    //         },
    //         .current_PID = {
    //             .Kp = 0.7, // 0.7
    //             .Ki = 0, // 0.1
    //             .Kd = 0,
    //             .Improve = PID_Integral_Limit,
    //             .IntegralLimit = 5000,
    //             .MaxOut = 7000,
    //             .MaxOut_ = -7000
    //         },
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
    //         .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
    //         .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
    //         .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
    //     },
    //     .motor_type = M2006 // 英雄使用m3508
    // };

    Motor_Init_Config_s dm_config = {
        .can_init_config = {.can_handle = &hcan2,.tx_id = 0x01,.rx_id = 0x11},
    };
    loader = DMMotorInit(&dm_config);
    DMMotorSetRef(loader,0);


    // shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据(robot_cmd已综合视觉shoot + 遥控器拨轮)
    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    // LOAD_1_BULLET 是单周期事件，先锁存，避免摩擦轮未就绪或卡弹反转时丢失。
    if (shoot_cmd_recv.load_mode == LOAD_1_BULLET)
        single_shot_pending = 1;

    // ====== 卡弹回转保护 ======
    if (loader_state == LOADER_REVERSING)
    {
        DMMotorSetRef(loader, REVERSE_SPEED);
        if (DWT_GetTimeline_ms() - reverse_start_time > REVERSE_TIME)
            loader_state = LOADER_NORMAL;
        return;
    }
    if (loader->measure.torque > TORQUE_THRESHOLD)
    {
        loader_state = LOADER_REVERSING;
        reverse_start_time = DWT_GetTimeline_ms();
        DMMotorSetRef(loader, REVERSE_SPEED);
        return;
    }

    // ====== 摩擦轮: 跟随cmd的friction_mode, 记录起转时间(两段式用) ======
    uint8_t friction_on = (shoot_cmd_recv.friction_mode == FRICTION_ON);
    if (friction_on && !friction_was_on) // 上升沿: 记录起转时间
        friction_on_start = DWT_GetTimeline_ms();
    friction_was_on = friction_on;

    if (friction_on)
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorSetRef(friction_l, 80000);
        DJIMotorSetRef(friction_r, 80000);
    }
    else
    {
        // 速度闭环给 0：靠速度环反向力矩主动刹车，比 Stop(失能滑行) 停得快
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 摩擦轮是否已起转到位(两段式延时)
    uint8_t friction_ready = friction_on &&
        (DWT_GetTimeline_ms() - friction_on_start >= friction_spinup_ms);

    // ====== 拨盘: 两段式 — 摩擦轮起转到位后才拨弹 ======
    // LOAD_1_BULLET  : 单发 — shoot上升沿登记一发, 摩擦轮到位后拨一发
    // LOAD_BURSTFIRE : 连发 — 持续拨弹(遥控器拨轮手动模式), 同样等摩擦轮到位
    // 其余(LOAD_STOP): 停止
    uint8_t loader_run = 0;
  
    if (friction_on)
    {
        if (shoot_cmd_recv.load_mode == LOAD_BURSTFIRE)
        {
            loader_run = friction_ready; // 连发持续转(等摩擦轮到位)
            single_shot_pending = 0;
            single_shot_active = 0;
        }
        else
        {
            // 单发请求已经锁存，即使当前命令恢复LOAD_STOP也继续等待摩擦轮。
            if (single_shot_pending && !single_shot_active && friction_ready)
            {
                single_shot_pending = 0;
                single_shot_active = 1;
                single_shot_start = DWT_GetTimeline_ms();
            }

            if (single_shot_active)
            {
                if (DWT_GetTimeline_ms() - single_shot_start < single_shot_ms)
                    loader_run = 1;         // 单发窗口内转动
                else
                    single_shot_active = 0; // 时间到, 停止
            }
        }
    }
    else
    {
        single_shot_pending = 0;
        single_shot_active = 0;
    }
    DMMotorSetRef(loader, loader_run ? loader_fire_speed : 0);


    
 }
