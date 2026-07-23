#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "dmmotor.h"

#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "servo_motor.h"

#include "remote_control.h"

#define TORQUE_THRESHOLD  3.0f  // 卡弹扭矩阈值 N·m，实测调
#define REVERSE_SPEED     -5.0f  // 回转速度 rad/s（负=反转）
#define REVERSE_TIME     500.0f  // 回转持续时间 ms

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
}loader_state = LOADER_NORMAL;

static float reverse_start_time = 0;



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
    // shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    // chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
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

    // ====== 摩擦轮: 拨轮<-400切换开关(边沿触发) ======
    static uint8_t friction_on = 0;
    static uint8_t friction_toggle_armed = 1; // 拨轮回弹到>-200后才允许下次切换
    float dial_raw = rc_data[TEMP].rc.dial;

    if (dial_raw > -200)
        friction_toggle_armed = 1; // 回弹, 允许下次触发
    if (dial_raw < -400 && friction_toggle_armed)
    {
        friction_on = !friction_on;
        friction_toggle_armed = 0;
    }

    if (friction_on)
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorSetRef(friction_l, 80000);
        DJIMotorSetRef(friction_r, 80000);
    }
    else
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
    }

    // ====== 拨盘: 拨轮正值控制速度 ======
    float dead_zone = 100.0f;
    float max_speed = 20.0f;
    float dial_out = 0;

    if (dial_raw > dead_zone)
        dial_out = (dial_raw - dead_zone) / (660.0f - dead_zone) * max_speed;

    DMMotorSetRef(loader, dial_out);


    //双板通信不用sub-pub协议
    // // 从cmd获取控制数据 双板停用
    // SubGetMessage(shoot_sub, &shoot_cmd_recv);
    // SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    // ServoSetAngle(servo, 6 * (shoot_cmd_recv.lid_mode));

    //后续加上了摩擦轮恢复下面的代码

    // // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    // if (shoot_cmd_recv.shoot_mode == SHOOT_OFF||(DaemonIsOnline(friction_l->daemon)==0)&&(DaemonIsOnline(friction_r->daemon)==0))
    // {
    //     DJIMotorStop(friction_l);
    //     DJIMotorStop(friction_r);
    //     DMMotorStop(loader);
    // }
    // else // 恢复运行
    // {
    //     DJIMotorEnable(friction_l);
    //     DJIMotorEnable(friction_r);
    //     DMMotorEnable(loader);
    // }

    // // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    // // if (hibernate_time + dead_time > DWT_GetTimeline_ms())
    // //     return;

    // // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    // if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    // {
    //     switch (shoot_cmd_recv.load_mode)
    //     {
    //         // 停止拨盘
    //     case LOAD_STOP:
    //         DMMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
    //         break;
    //     // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    //     // case LOAD_1_BULLET: // 激活能量机关/干扰对方用,英雄用
    //     //     if(dead_time++>300)
    //     //     {
    //     //         DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到角度环
    //     //         DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
    //     //         dead_time = 0;
    //     //     }
    //     //     // hibernate_time = DWT_GetTimeline_ms();                                        // 记录触发指令的时间
    //     //     // dead_time = 150;                                                              // 完成1发弹丸发射的时间
    //     //     break;
    //     // 三连发,如果不需要后续可能删除
    //     case LOAD_3_BULLET:
    //         DMMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 1.6);
    //         // DJIMotorOuterLoop(loader, ANGLE_LOOP); // 切换到速度环
    //         // DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
    //         // hibernate_time = DWT_GetTimeline_ms();                                            // 记录触发指令的时间
    //         // dead_time = 300;                                                                  // 完成3发弹丸发射的时间
    //         break;
    //     // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    //     case LOAD_BURSTFIRE:
    //         DMMotorSetRef(loader, (shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 2));
    //         // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
    //         break;
    //     // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    //     // 也有可能需要从switch-case中独立出来
    //     case LOAD_REVERSE:
            
    //         // ...
    //         break;
    //     default:
    //         while (1)
    //             ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    //     }
    // }else
    // {
    //         DMMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
    // }
    // // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    // if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    // {
    //     // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入
    //     switch (shoot_cmd_recv.bullet_speed)
    //     {
    //     case SMALL_AMU_15:
    //         DJIMotorSetRef(friction_l, 30000);
    //         DJIMotorSetRef(friction_r, 30000);
    //         break;
    //     case SMALL_AMU_18:
    //         DJIMotorSetRef(friction_l, 36000);
    //         DJIMotorSetRef(friction_r, 36000);
    //         break;
    //     case SMALL_AMU_30:
    //         DJIMotorSetRef(friction_l, 60000);
    //         DJIMotorSetRef(friction_r, 60000);
    //         break;
    //     default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
    //         DJIMotorSetRef(friction_l, 40000);
    //         DJIMotorSetRef(friction_r, 40000);
    //         break;
    //     }
    // }
    // else // 关闭摩擦轮
    // {
    //     DJIMotorSetRef(friction_l, 0);
    //     DJIMotorSetRef(friction_r, 0);
    // }

    // // 开关弹舱盖
    // if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    // {
    //     //...
    // }
    // else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    // {
    //     //...
    // }


    //双板通信不用sub-pub协议
//     // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
//     shoot_feedback_data.shoot_mode = shoot_cmd_recv.shoot_mode;
//     shoot_feedback_data.load_mode = shoot_cmd_recv.load_mode;
//     PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
 }