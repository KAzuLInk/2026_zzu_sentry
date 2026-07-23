#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "dmmotor.h"
#include "remote_control.h"
extern RC_ctrl_t *rc_data;

static attitude_t *gimba_IMU_data; // 云台IMU数据
static DMMotorInstance *pitch_motor;//Pitch用的达妙4310
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static DMMotorInstance *yaw_motor; //达妙4310大yaw CAN1
static DJIMotorInstance *small_yaw_motor; //GM6020小yaw CAN2
static float speed_forward;
static float rotate_compensator;

// 双环PID: 角度环(外环) + 速度环(内环)
static PIDInstance yaw_angle_pid; // 角度环: YawTotalAngle → 目标角速度
static PIDInstance yaw_speed_pid; // 速度环: Gyro[2] → 电机速度指令
static float yaw_angle_ref;       // 目标角度(惯性空间, 上电时锁定当前位置)
static uint8_t yaw_angle_ref_locked; // 是否已锁定初始角度
static uint8_t small_yaw_locked;      // 小yaw是否已锁定

// Pitch轴双环PID
static PIDInstance pitch_angle_pid;
static PIDInstance pitch_speed_pid;
static float pitch_angle_ref;
static uint8_t pitch_angle_ref_locked;


void GimbalInit()
{

    //姿态还没写，写了解除注释
    // gimba_IMU_data = INS_Init(); // IMU初始化在StartINSTASK中完成,此处不需要
    
    //小YAW电机 GM6020 — CAN2(编码器闭环, 锁住当前位置)
    Motor_Init_Config_s small_yaw_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 40,
                .Ki = 0,
                .Kd = 8.2f,
                .DeadBand = 0.5f,
                .Derivative_LPF_RC = 0.0135f,
                .Output_LPF_RC = 0.02f,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 100,
                .MaxOut = 1000,
                .MaxOut_ = -1000
            },
            .speed_PID = {
                .Kp = 5,
                .Ki = 0,
                .Kd = 0.012f,
                .MaxOut = 10000,
                .MaxOut_ = -10000,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    small_yaw_motor = DJIMotorInit(&small_yaw_config);

    // PITCH 哨兵用的4310
    
    // Motor_Init_Config_s pitch_config = {
    //     .can_init_config = {
    //         .can_handle = &hcan1,
    //         .tx_id = 1,
    //     },   
    //     .controller_param_init_config = {
    //         .angle_PID = {
    //             .Kp = 40, // 10
    //             .Ki = 0,
    //             .Kd = 0.6,
    //             .Derivative_LPF_RC = 0.2,
    //             .Output_LPF_RC = 0.13,
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 100,
    //             .MaxOut = 500,
    //             .MaxOut_ = -500
    //         },
    //         .speed_PID = {
    //             .Kp = 80,  // 50
    //             .Ki = 350, // 350
    //             .Kd = 0,   // 0
    //             .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    //             .IntegralLimit = 2500,
    //             .MaxOut = 25000,
    //             .MaxOut_ = -25000
    //         },
    //         .other_angle_feedback_ptr = &gimba_IMU_data->Roll,
    //         // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
    //         .other_speed_feedback_ptr = (&gimba_IMU_data->Gyro[1]),
    //         .speed_feedforward_ptr = &speed_forward,
    //     },
    //     .controller_setting_init_config = {
    //         .angle_feedback_source = OTHER_FEED,
    //         .speed_feedback_source = OTHER_FEED,
    //         .outer_loop_type = ANGLE_LOOP,
    //         .close_loop_type = SPEED_LOOP | ANGLE_LOOP,
    //         .feedforward_flag = SPEED_FEEDFORWARD,
    //         .motor_reverse_flag = MOTOR_DIRECTION_REVERSE
    //     },
    //     .motor_type = GM6020,
    // };

    //大yaw和pitch用的都是4310
    Motor_Init_Config_s yaw_dm_config = {
    .can_init_config = {.can_handle = &hcan1, .tx_id = 0x01, .rx_id = 0x11},
    };
    yaw_motor = DMMotorInit(&yaw_dm_config);
    yaw_motor->v_max = 30.0f;
    yaw_motor->kp_max = 500.0f;
    yaw_motor ->p_max = 3.141593f;
    yaw_motor ->v_max = 30.0f;
    yaw_motor ->t_max = 18.0f;
    yaw_motor->kd_max = 5.0f;
    yaw_motor->kp_max = 5.0f;
    // yaw_motor->native_mode = DM_NATIVE_MODE_MIT; // 已切回MIT, 注释
    DMMotorSetRef(yaw_motor, 0);  // 初始停住

    // ====== 初始化角度环PID(外环) ======
    // 输入: YawTotalAngle(°)  输出: 目标角速度(°/s)  喂给速度环
    // 调参顺序: 先只给Kp, 手推云台能回弹不抖; 再加Ki克服静摩擦; 最后加Kd
    PID_Init_Config_s yaw_angle_config = {
        .Kp = 2.0f,                             // 角度误差比例增益
        .Ki = 0.03f,                            // 积分增益 — 消除稳态误差, 克服静摩擦
        .Kd = 0.25f,                            // 微分增益 — 抑制超调震荡
        .MaxOut = 60.0f,                        // 输出上限 = 目标角速度 °/s
        .MaxOut_ = -60.0f,
        .DeadBand = 0.05f,                      // 死区 0.05°
        .IntegralLimit = 3.0f,                   // 积分限幅 — I最多贡献±3°/s
        .Improve = PID_Integral_Limit,
        .Derivative_LPF_RC = 0.0f,
        .Output_LPF_RC = 0.0f,
    };
    PIDInit(&yaw_angle_pid, &yaw_angle_config);

    // ====== 初始化速度环PID(内环) ======
    // 输入: Gyro[2](°/s)  输出: 电机速度指令
    // 先把速度环当成一个简单的阻尼环节, 角度环的输出几乎直接传下去
    PID_Init_Config_s yaw_speed_config = {
        .Kp = 1.0f,                             // 角速度误差比例增益
        .Ki = 0.0f,                             // 积分 — 先关掉
        .Kd = 0.0f,                             // 微分 — 先关掉
        .MaxOut = 15.0f,                         // 输出上限
        .MaxOut_ = -15.0f,
        .DeadBand = 0.2f,                       // 死区
        .IntegralLimit = 10.0f,                  // 积分限幅
        .Improve = PID_Integral_Limit,
        .Derivative_LPF_RC = 0.0f,
        .Output_LPF_RC = 0.0f,
    };
    PIDInit(&yaw_speed_pid, &yaw_speed_config);

    yaw_angle_ref_locked = 0; // 等待Task中首次读取IMU后锁定

    // ====== Pitch电机 DM4310 — CAN1 ======
    Motor_Init_Config_s pitch_dm_config = {
        .can_init_config = {.can_handle = &hcan1, .tx_id = 0x02, .rx_id = 0x12},
    };
    pitch_motor = DMMotorInit(&pitch_dm_config);
    pitch_motor->v_max = 30.0f;
    pitch_motor->kp_max = 5.0f;
    pitch_motor->p_max = 3.141593f;
    pitch_motor->t_max = 18.0f;
    pitch_motor->kd_max = 5.0f;
    pitch_motor->native_mode = DM_NATIVE_MODE_POSVEL; // 原生位置速度模式
    DMMotorSetRef(pitch_motor, 0);

    // ====== Pitch角度环PID (裸电机: 关积分防饱和震荡, 上车后加回) ======
    PID_Init_Config_s pitch_angle_config = {
        .Kp = 0.8f,        // 裸电机: 0.8 | 上车: 2.0+
        .Ki = 0.0f,        // 裸电机关积分
        .Kd = 0.01f,       // 微分先行, Kd/dt≈1.0
        .MaxOut = 1.0f,    // 裸电机: 1.0 | 上车: 60
        .MaxOut_ = -1.0f,
        .DeadBand = 0.03f, // 裸电机: 0.03| 上车: 0.05
        .IntegralLimit = 0.5f,
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
    };
    PIDInit(&pitch_angle_pid, &pitch_angle_config);

    // ====== Pitch速度环PID (裸电机: 单速度环, 上车后加角度环) ======
    PID_Init_Config_s pitch_speed_config = {
        .Kp = 0.15f,       // 裸电机: 0.15| 上车: 1.0+
        .Ki = 0.0f,
        .Kd = 0.0f,
        .MaxOut = 3.0f,
        .MaxOut_ = -3.0f,
        .DeadBand = 0.2f,
        .IntegralLimit = 1.5f,
        .Improve = PID_Integral_Limit,
    };
    PIDInit(&pitch_speed_pid, &pitch_speed_config);

    pitch_angle_ref_locked = 0;

    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    // yaw_motor = DJIMotorInit(&yaw_config);
    // pitch_motor = DJIMotorInit(&pitch_config);

    //双板不用sub-pub
    // chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    // gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    // gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

//也是先注释，调云台再改
// static void change_spd_ford()
// {
//     if(pitch_motor->motor_controller.angle_PID.Err > 0)
//         speed_forward = pitch_motor->motor_controller.angle_PID.Err > 0.5 ? (5 * cos(gimba_IMU_data->Roll*PI/360) + 10) : 0;
//     else
//         speed_forward = pitch_motor->motor_controller.angle_PID.Err < -0.5 ? (5 * cos(gimba_IMU_data->Roll*PI/360) - 10) : 0;
// }
/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */

/* 云台双环PID控制 — 替代纯速度环补偿
 *  外环(角度): YawTotalAngle 反馈, 保持云台在惯性空间指向不变
 *             积分项累积小角度偏差, 自动克服静摩擦, 低速也有效
 *  内环(速度): Gyro[2] 反馈, 跟踪角度环输出的目标角速度, 提供阻尼
 *  级联输出 → DMMotorSetRef → MIT velocity_des
 */
float debugtest; // 诊断用: yaw速度环输出 → DMMotorSetRef
volatile uint8_t rc_online;        // 遥控器在线状态
volatile float pitch_debug_rockr;  // 摇杆
volatile float pitch_debug_ref;    // 目标角度 rad
volatile float pitch_debug_err;    // 位置误差 rad
volatile float pitch_debug_vel;    // 电机速度 rad/s
volatile float pitch_debug_gyro;   // (未使用)
volatile float pitch_debug_out;    // 位置指令 rad
volatile float pitch_debug_torque; // 电机实际扭矩 Nm (顶限位时飙升)
volatile float small_yaw_debug;    // 小yaw发送电流值
volatile uint16_t small_yaw_ecd;   // 小yaw编码器值

void GimbalTask()
{
    // 小Yaw: 位置模式 — 摇杆→ecd→total_angle, 绝对值映射不累积
    if (small_yaw_motor != NULL && small_yaw_motor->feed_cnt > 0)
    {
        static uint8_t yaw_init = 0;
        if (!yaw_init)
        {
            small_yaw_motor->motor_settings.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
            small_yaw_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
            yaw_init = 1;
        }

        // 摇杆线性映射: +660→405ecd, 0→中位, -660→2250ecd (ecd增大为正方向)
        float stick = -rc_data[TEMP].rc.rocker_r_;
        float ratio = (stick + 660.0f) / 1320.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < 0.0f) ratio = 0.0f;
        float ecd_target = 405.0f + ratio * (2250.0f - 405.0f);

        // 限位
        uint16_t ecd = small_yaw_motor->measure.ecd;
        if (ecd > 2250 && ecd_target > ecd) ecd_target = ecd;
        if (ecd < 405  && ecd_target < ecd) ecd_target = ecd;
        if (ecd_target > 2250) ecd_target = 2250;
        if (ecd_target < 405)  ecd_target = 405;

        // 绝对值: total_angle = 当前total + (目标ecd - 当前ecd)
        float total_ref = small_yaw_motor->measure.total_angle + (ecd_target - (float)ecd);
        DJIMotorSetRef(small_yaw_motor, total_ref);

        small_yaw_debug = total_ref;
        small_yaw_ecd   = ecd;

        // 大小yaw联动: 小yaw超范围时大yaw追踪, 带回差防边界抖动
        static uint8_t follow = 0;
        if (ecd > 1850) follow = 1;          // 触发追踪
        if (ecd < 1550 && follow == 1) follow = 0; // 回差退出
        if (ecd < 750)  follow = 2;
        if (ecd > 1050 && follow == 2) follow = 0;

        if (follow == 1 && yaw_angle_ref_locked)
            yaw_angle_ref += ((float)ecd - 1500.0f) * 0.005f;
        else if (follow == 2 && yaw_angle_ref_locked)
            yaw_angle_ref += ((float)ecd - 1000.0f) * 0.005f;
    }

    rc_online = RemoteControlIsOnline(); // 看Ozone: 1=在线 0=离线

    // ====== Pitch轴: 原生位置速度模式, 摇杆直接映射到角度 ======
    // 摇杆 -660~+660 → 角度 0.0~0.48 rad, 中位=0.24 rad
    if (pitch_motor != NULL)
    {
        static uint16_t startup_cnt = 0;
        static float    pos_ref    = 0;

        if (DaemonIsOnline(pitch_motor->motor_daemon) == 0)
        {
            startup_cnt = 0;
            pos_ref     = pitch_motor->measure.position;
            DMMotorSetRef(pitch_motor, pos_ref);
            goto pitch_done;
        }
        startup_cnt++;
        if (startup_cnt < 50)
        {
            pos_ref = pitch_motor->measure.position;
            DMMotorSetRef(pitch_motor, pos_ref);
            goto pitch_done;
        }

        // 摇杆线性映射到机械范围
        float hi = 0.48f, lo = 0.0f;
        float stick = rc_data[TEMP].rc.rocker_r1;           // -660 ~ +660
        float ratio = (stick + 660.0f) / 1320.0f;           // 0.0 ~ 1.0
        if      (ratio > 1.0f) ratio = 1.0f;
        else if (ratio < 0.0f) ratio = 0.0f;
        pos_ref = lo + ratio * (hi - lo);                   // lo ~ hi

        DMMotorSetRef(pitch_motor, pos_ref);
        pitch_debug_torque = pitch_motor->measure.torque; // 顶限位时飙升
        pitch_done:;
    }

    // 懒加载IMU数据指针
    if (gimba_IMU_data == NULL)
        gimba_IMU_data = INS_Init();

    if (gimba_IMU_data != NULL)
    {
        if (!yaw_angle_ref_locked)
        {
            yaw_angle_ref = gimba_IMU_data->YawTotalAngle;
            yaw_angle_ref_locked = 1;
        }

        // 摇杆控制: 左水平摇杆调整yaw目标角度 (度/周期)
        float stick = -rc_data[TEMP].rc.rocker_l_;
        if (stick > 30 || stick < -30)
            yaw_angle_ref += stick / 660.0f * 0.5f;

        float current_angle = gimba_IMU_data->YawTotalAngle;
        float target_vel = PIDCalculate(&yaw_angle_pid, current_angle, yaw_angle_ref);
        float current_gyro_z = gimba_IMU_data->Gyro[2] * GYRO2GIMBAL_DIR_YAW;
        float motor_ref = PIDCalculate(&yaw_speed_pid, current_gyro_z, target_vel);

        debugtest = motor_ref;
        DMMotorSetRef(yaw_motor, motor_ref);
    }

    //调云台时候解除注释再改

    // // 获取云台控制数据
    // // 后续增加未收到数据的处理
    // SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    // change_spd_ford();
    // // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref

    //  if(DaemonIsOnline(pitch_motor->daemon)==0)
    // {
    //     DJIMotorStop(yaw_motor);
    // }
    // else
    // {
    //     DJIMotorEnable(yaw_motor);
    // }
    
    // last_total_angle = pitch_motor->measure.total_angle;


    // switch (gimbal_cmd_recv.gimbal_mode)
    // {
    // // 停止
    // case GIMBAL_ZERO_FORCE:
    //     DJIMotorStop(yaw_motor);
    //     DJIMotorStop(pitch_motor);
    //     break;
    // // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    // case GIMBAL_GYRO_MODE: // 后续只保留此模式
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
        
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);

    //     DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);

    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
    //     break;
    // // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    // case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
    //     DJIMotorEnable(yaw_motor);
    //     DJIMotorEnable(pitch_motor);
    //     DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
    //     DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
    //     DJIMotorOuterLoop(pitch_motor, SPEED_LOOP);
    //     DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, MOTOR_FEED);
    //     DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
    //     DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch_v);
    //     break;
    // case GIMBAL_DEBUG_MODE:
    //     DJIMotorStop(yaw_motor);
    //     //yaw_motor->stop_flag = MOTOR_DEBUG;
    //     break;
    // case GIMBAL_NAV_MODE:
        
    //     break;
    // default:
    //     break;
    // }

    // // 在合适的地方添加pitch重力补偿前馈力矩
    // // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // // ...
    // // DJIMotorStop(yaw_motor);
    // // DJIMotorStop(pitch_motor);
    // // 设置反馈数据,主要是imu和yaw的ecd
    // gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
    // gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // // 推送消息
    // PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}