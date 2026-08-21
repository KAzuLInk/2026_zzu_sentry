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
static Vision_Recv_s *vision_data; // 视觉接收数据(右中自瞄用, 懒加载)
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
                .Kp = 25,
                .Ki = 0,
                .Kd = 0.0f,
                .DeadBand = 0.1f,
                .Derivative_LPF_RC = 0.01f,
                .Output_LPF_RC = 0.02f,
                .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut = 2000,
                .MaxOut_ = -2000
            },
            .speed_PID = {
                .Kp = 55.0f,
                .Ki = 10.0f,
                .Kd = 0.0f,
                .MaxOut = 30000,
                .MaxOut_ = -30000,
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
        .Kp = 3.0f,                             // 角度误差比例增益
        .Ki = 0.03f,                            // 积分增益 — 消除稳态误差, 克服静摩擦
        .Kd = 0.2f,                            // 微分增益 — 抑制超调震荡
        .MaxOut = 150.0f,                       // 输出上限 = 目标角速度 °/s (回中模式需150, 原60)
        .MaxOut_ = -150.0f,
        .DeadBand = 0.0f,                      // 死区 0.05°
        .IntegralLimit = 3.0f,                   // 积分限幅 — I最多贡献±3°/s
        .Improve = PID_Integral_Limit | PID_Derivative_On_Measurement,
        .Derivative_LPF_RC = 0.02f,             // 微分低通滤波, 抑制IMU噪声放大
        .Output_LPF_RC = 0.0f,
    };
    PIDInit(&yaw_angle_pid, &yaw_angle_config);

    // ====== 初始化速度环PID(内环) ======
    // 输入: target_vel_rad(rad/s) 与 Gyro[2](rad/s), 两路统一为 rad/s
    // 输出: MIT velocity_des (rad/s), 电机内部Kd=2.0叠加工作
    // 先内环后外环
    PID_Init_Config_s yaw_speed_config = {
        .Kp = 1.0f,                             // 速度误差比例增益 (rad/s→rad/s), 前馈兜底后P只做阻尼
        .Ki = 0.0f,                             // I=0, 前馈已保证稳态速度, 无需积分
        .Kd = 0.0f,                             // 微分 — 先关掉
        .MaxOut = 2.0f,                        // 输出上限 = 修正项 ±2 rad/s(前馈目标在外部叠加)
        .MaxOut_ = -2.0f,
        .DeadBand = 0.0f,                      // 死区 0.01 rad/s ≈ 0.57°/s
        .IntegralLimit = 1.0f,                  // 积分限幅 ±1 rad/s
        .Improve = PID_Integral_Limit,
        .Derivative_LPF_RC = 0.0f,
        .Output_LPF_RC = 0.01f,                 // 输出低通滤波, 抑制毛刺
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
volatile float yaw_motor_ref_debug; // 诊断用: 大yaw速度指令 → DMMotorSetRef
float gyro_lpf_val;          // 陀螺仪滤波后的值 (rad/s)
float gyro_lpf_rc = 0.03f;    // LPF的RC常数, Ozone可改: 越大滤波越强
uint32_t gyro_lpf_dwt;       // LPF时间戳
static float yaw_chassis_rate_rad_s;            // 底盘IMU yaw角速度(rad/s)
static const float yaw_chassis_ff_gain = 1.0f;  // 电机相对底盘速度补偿比例
volatile uint8_t rc_online;        // 遥控器在线状态
volatile float pitch_debug_rockr;  // 摇杆
volatile float pitch_debug_ref;    // 目标角度 rad
volatile float pitch_debug_err;    // 位置误差 rad
volatile float pitch_debug_vel;    // 电机速度 rad/s
volatile float pitch_debug_gyro;   // (未使用)
volatile float pitch_debug_out;    // 位置指令 rad
volatile float pitch_debug_torque; // 电机实际扭矩 Nm (顶限位时飙升)
volatile float small_yaw_debug;    // 小yaw的total_ref
volatile uint16_t small_yaw_ecd;   // 小yaw编码器值
volatile float yaw_speed_ref_deg = 150.0f;  // 速控满偏角速度 °/s, Ozone可调
static uint16_t small_yaw_home_ecd = 0;     // 小yaw上电锁定位(速控模式用)
static uint8_t  small_yaw_ctl_init = 0;     // 小yaw闭环初始化标志
static uint8_t  last_speed_control = 0;     // 上一拍是否速控(用于切回时重锁定)
static uint8_t  last_vision_track = 0;      // 上一拍是否视觉跟踪(锁敌), 用于切入时重锁定

// 小yaw主控 — 大yaw回中曲线 (Ozone可调)
volatile float recenter_max_vel  = 150.0f;  // 边缘处大yaw回中角速度上限 °/s (= 60×2.5)
volatile float recenter_deadband = 300.0f;  // 中心死区(ecd计数, ±300 → 1000~1600不动)
volatile float recenter_sat      = 800.0f;  // 速度饱和偏移量(ecd, 离中心800 → 2100/500, 留余量防小yaw超调)
volatile float recenter_exp      = 3.0f;    // 曲线指数: 2=平方, 3=三次方
volatile float recenter_debug    = 0.0f;    // 诊断: 当前回中角速度 °/s
static uint32_t recenter_dwt     = 0;       // 回中积分时间戳

// 视觉俯仰映射
volatile float pitch_vision_horizon = 0.020f;  // 视觉pitch=0°(水平)对应的电机位置(rad)
volatile float pitch_vision_min     = -0.78f;  // 俯仰下极限(rad)
volatile float pitch_vision_max     = 0.022f;  // 俯仰上极限(rad)
volatile float pitch_vision_dir     = 1.0f;    // 方向: 1=视觉向上=位置增大, -1=反向

// 视觉射击到位诊断（Ozone直接观察）
volatile float vision_align_tolerance_deg;
volatile float vision_align_yaw_target_deg;
volatile float vision_align_yaw_actual_deg;
volatile float vision_align_yaw_error_deg;
volatile float vision_align_pitch_target_rad;
volatile float vision_align_pitch_actual_rad;
volatile float vision_align_pitch_error_deg;
volatile uint8_t vision_align_yaw_ready;
volatile uint8_t vision_align_pitch_ready;
volatile uint8_t vision_align_ready;

/* 角度环绕: 把 target 折到 current 最近邻表示(±180内), 返回绝对角 */
static float WrapAngleDeg(float target, float current)
{
    float error = target - current;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return current + error;
}

void GimbalSetChassisYawRate(float yaw_rate_rad_s)
{
    /* 拒绝NaN和明显异常值，双板通信离线时调用方传0。 */
    if (yaw_rate_rad_s != yaw_rate_rad_s ||
        yaw_rate_rad_s > 20.0f || yaw_rate_rad_s < -20.0f)
        yaw_rate_rad_s = 0.0f;
    yaw_chassis_rate_rad_s = yaw_rate_rad_s;
}

/* 小yaw位置闭环到指定ecd(绝对值映射, 不累积), 带机械限位 */
static void SmallYawSetEcd(uint16_t ecd_target)
{
    if (small_yaw_motor == NULL || small_yaw_motor->feed_cnt <= 0)
        return;
    uint16_t ecd = small_yaw_motor->measure.ecd;
    // 限位: 顶到限位时不反向拉, 且目标不超机械范围
    if (ecd > 2250 && ecd_target > ecd) ecd_target = ecd;
    if (ecd < 405  && ecd_target < ecd) ecd_target = ecd;
    if (ecd_target > 2250) ecd_target = 2250;
    if (ecd_target < 405)  ecd_target = 405;
    float total_ref = small_yaw_motor->measure.total_angle
                    + ((float)ecd_target - (float)ecd) * ECD_ANGLE_COEF_DJI;
    DJIMotorSetRef(small_yaw_motor, total_ref);
    small_yaw_debug = total_ref;
    small_yaw_ecd   = ecd;
}

/* 小yaw偏离中心(1300)→大yaw回中角速度(°/s), 三次方曲线 */
static float RecenterVel(void)
{
    float recenter_vel = 0.0f;
    if (small_yaw_motor != NULL && small_yaw_motor->feed_cnt > 0)
    {
        float off = (float)small_yaw_motor->measure.ecd - 1300.0f; // 右正左负
        float mag = off < 0.0f ? -off : off;
        if (mag > recenter_deadband)
        {
            float x = (mag - recenter_deadband) / (recenter_sat - recenter_deadband);
            if (x > 1.0f) x = 1.0f;
            float sign = off > 0.0f ? 1.0f : -1.0f;
            recenter_vel = sign * recenter_max_vel * powf(x, recenter_exp);
        }
    }
    recenter_debug = recenter_vel;
    return recenter_vel;
}

/* 大yaw速度环: 世界系目标角速度 → 电机相对底盘速度指令。 */
static void BigYawSpeedControl(float target_vel_rad)
{
    if (gimba_IMU_data == NULL)
        return;
    float current_gyro_z = gimba_IMU_data->Gyro[2] * GYRO2GIMBAL_DIR_YAW;      // rad/s
    // 陀螺仪输入低通滤波 — 切断机械振动→速度环的正反馈路径
    float dt_lpf = DWT_GetDeltaT(&gyro_lpf_dwt);
    if (dt_lpf > 0.01f) dt_lpf = 0.001f;
    gyro_lpf_val = current_gyro_z * dt_lpf / (gyro_lpf_rc + dt_lpf)
                 + gyro_lpf_val * gyro_lpf_rc / (gyro_lpf_rc + dt_lpf);
    /* MIT的velocity_des是电机相对底盘的转速，而target_vel_rad是世界系云台角速度。
     * 因此提前减去底盘角速度，避免必须先产生角度误差才能抵消小陀螺旋转。 */
    float chassis_velocity_ff = -yaw_chassis_rate_rad_s * yaw_chassis_ff_gain;
    float motor_ref = target_vel_rad + chassis_velocity_ff
                    + PIDCalculate(&yaw_speed_pid, gyro_lpf_val, target_vel_rad);
    yaw_motor_ref_debug = motor_ref;
    DMMotorSetRef(yaw_motor, motor_ref);
}

/* 大yaw角度环 + 速度环 → DMMotorSetRef, ref为惯性空间绝对角(°) */
static void BigYawAngleControl(float angle_ref)
{
    if (gimba_IMU_data == NULL)
        return;
    float current_angle = gimba_IMU_data->YawTotalAngle;
    float target_vel = PIDCalculate(&yaw_angle_pid, current_angle, angle_ref); // °/s
    BigYawSpeedControl(target_vel * DEGREE_2_RAD);                              // °/s → rad/s
}

void GimbalTask()
{
    uint8_t sw_right    = rc_data[TEMP].rc.switch_right;
    uint8_t sw_left     = rc_data[TEMP].rc.switch_left;
    uint8_t remote_mode = switch_is_down(sw_right); // 右下 = 遥控器模式
    uint8_t vision_mode = switch_is_mid(sw_right);  // 右中 = 视觉自瞄模式
    uint8_t vision_online = 0;
    uint8_t vision_locked = 0;

    // 小yaw初始化(上电一次): 闭环类型 + 锁定初始位
    if (small_yaw_motor != NULL && small_yaw_motor->feed_cnt > 0 && !small_yaw_ctl_init)
    {
        small_yaw_motor->motor_settings.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
        small_yaw_motor->motor_settings.outer_loop_type = ANGLE_LOOP;
        small_yaw_home_ecd = small_yaw_motor->measure.ecd; // 上电初始位
        small_yaw_ctl_init = 1;
    }

    // 子模式: (右下或右中) + 左上 = 失能 → 停所有电机
    if ((remote_mode || vision_mode) && switch_is_up(sw_left))
    {
        // 大yaw(MIT): vel_des=0 配合 Kd=2.0 主动阻尼刹车
        //   (DMMotorStop发全0帧, Kd=0被固件拒收→电机疯转, 不能用)
        DMMotorSetRef(yaw_motor, 0);
        // Pitch(POSVEL): 失能回到安全位 0.001 rad (在行程范围 -0.78~0.022 内)
        DMMotorSetRef(pitch_motor, 0.001f);
        // 小yaw(GM6020): 0电流失能
        if (small_yaw_motor != NULL) DJIMotorStop(small_yaw_motor);
        yaw_angle_ref_locked = 0; // 重新使能后重锁定当前角, 防猛转回旧ref
        rc_online = RemoteControlIsOnline();
        return;
    }

    // 非失能: 小yaw恢复使能 (失能分支用DJIMotorStop置停)
    if (small_yaw_motor != NULL) DJIMotorEnable(small_yaw_motor);

    // 子模式: 右下 + 左中 = 速控 (大yaw摇杆速控 + 小yaw锁死)
    uint8_t speed_control = remote_mode && switch_is_mid(sw_left);

    // 刚从速控切回跟随: 重锁定当前角, 防大yaw猛转回旧ref
    if (!speed_control && last_speed_control)
        yaw_angle_ref_locked = 0;
    last_speed_control = speed_control;

    // 懒加载IMU数据指针 + 视觉数据指针
    if (gimba_IMU_data == NULL)
        gimba_IMU_data = INS_Init();
    if (vision_data == NULL)
        vision_data = VisionGetRecv();

    if (vision_mode)
    {
        // ====== 视觉自瞄模式 (右中切换) ======
        // 未锁敌: 大yaw带动小yaw寻敌 (大yaw指向视觉yaw, 小yaw回中)
        // 锁敌:   小yaw带动大yaw跟踪 (小yaw精细跟踪残差, 大yaw回中曲线跟随)
        vision_online = VisionIsOnline();
        vision_locked = vision_online && vision_data != NULL
                        && vision_data->target_state == READY_TO_FIRE;

        // 进入/退出跟踪子模式: 重锁大yaw目标角, 防角度跳变
        if (vision_locked != last_vision_track)
            yaw_angle_ref_locked = 0;
        last_vision_track = vision_locked;

        if (vision_locked)
        {
            // ---- 锁敌: 小yaw带动大yaw ----
            // 小yaw主动跟踪: 视觉yaw是小yaw相对中位的目标偏角(度)，大yaw只负责从动回中
            if (small_yaw_motor != NULL && small_yaw_motor->feed_cnt > 0)
            {
                float ecd_target = 1300.0f
                                 + vision_data->yaw / ECD_ANGLE_COEF_DJI;
                if (ecd_target > 2250.0f) ecd_target = 2250.0f;
                else if (ecd_target < 405.0f) ecd_target = 405.0f;
                SmallYawSetEcd((uint16_t)ecd_target);
            }
            // 大yaw: 回中曲线(小yaw偏离中心越多, 大yaw回中越快) + 角度环
            if (gimba_IMU_data != NULL)
            {
                if (!yaw_angle_ref_locked)
                {
                    yaw_angle_ref = gimba_IMU_data->YawTotalAngle;
                    yaw_angle_ref_locked = 1;
                }
                float recenter_vel = RecenterVel();
                float dt = DWT_GetDeltaT(&recenter_dwt);
                if (dt > 0.02f) dt = 0.01f;
                yaw_angle_ref += recenter_vel * dt;
                BigYawAngleControl(yaw_angle_ref);
            }
        }
        else
        {
            // ---- 未锁敌/离线: 大yaw带动小yaw寻敌 (离线时大yaw锁当前角保持) ----
            SmallYawSetEcd(1300); // 小yaw回中
            if (gimba_IMU_data != NULL)
            {
                if (vision_online && vision_data != NULL)
                    BigYawAngleControl(WrapAngleDeg(vision_data->yaw, gimba_IMU_data->YawTotalAngle)); // 大yaw寻敌扫描
                else
                {
                    if (!yaw_angle_ref_locked) // 视觉离线: 锁当前角保持
                    {
                        yaw_angle_ref = gimba_IMU_data->YawTotalAngle;
                        yaw_angle_ref_locked = 1;
                    }
                    BigYawAngleControl(yaw_angle_ref);
                }
            }
        }
    }
    else if (speed_control)
    {
        // ====== 速控模式 ======
        // 小yaw: 锁死在上电初始位
        if (small_yaw_motor != NULL && small_yaw_motor->feed_cnt > 0)
        {
            float total_ref = small_yaw_motor->measure.total_angle
                            + ((float)small_yaw_home_ecd - (float)small_yaw_motor->measure.ecd) * ECD_ANGLE_COEF_DJI;
            DJIMotorSetRef(small_yaw_motor, total_ref);
            small_yaw_debug = total_ref;
            small_yaw_ecd   = small_yaw_motor->measure.ecd;
        }

        // 大yaw: 摇杆速控(跳过角度环), 满偏 = yaw_speed_ref_deg
        if (gimba_IMU_data != NULL)
        {
            float stick = rc_data[TEMP].rc.rocker_r_;           // -660 ~ +660
            if (stick > -20.0f && stick < 20.0f) stick = 0.0f;  // 中心死区, 防漂移
            float target_vel_deg = stick / 660.0f * yaw_speed_ref_deg;
            float target_vel_rad = target_vel_deg * DEGREE_2_RAD; // °/s → rad/s

            BigYawSpeedControl(target_vel_rad);
        }
    }
    else
    {
        // ====== 跟随模式 ======
        // 小Yaw: 位置模式 — 摇杆→ecd→total_angle, 绝对值映射不累积
        // 摇杆线性映射: +660→405ecd, 0→中位, -660→2250ecd (ecd增大为正方向)
        float stick = -rc_data[TEMP].rc.rocker_r_;
        float ratio = (stick + 660.0f) / 1320.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        if (ratio < 0.0f) ratio = 0.0f;
        float ecd_target = 405.0f + ratio * (2250.0f - 405.0f);
        SmallYawSetEcd((uint16_t)ecd_target);

        // 大yaw: IMU自稳(锁绝对角) + 小yaw回中(偏离中心越大角度偏移越大)
        if (gimba_IMU_data != NULL)
        {
            if (!yaw_angle_ref_locked)
            {
                yaw_angle_ref = gimba_IMU_data->YawTotalAngle;
                yaw_angle_ref_locked = 1;
            }

            // 回中曲线: 小yaw偏离中心越多, 回中角速度越大, 积分进角度ref
            float recenter_vel = RecenterVel();
            float dt = DWT_GetDeltaT(&recenter_dwt);
            if (dt > 0.02f) dt = 0.01f; // 首次调用/机械延迟防护, 100Hz下约10ms
            yaw_angle_ref += recenter_vel * dt; // 积分进角度ref

            BigYawAngleControl(yaw_angle_ref);
        }
    }

    rc_online = RemoteControlIsOnline(); // 看Ozone: 1=在线 0=离线

    // ====== Pitch轴: 原生位置速度模式, 摇杆直接映射到角度 ======
    // 摇杆 -660~+660 → 角度 -0.78~0.022 rad
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

        // 目标位置: 视觉模式用视觉pitch(度→rad), 否则摇杆映射
        if (vision_mode)
        {
            // 右中默认保持水平，只有视觉明确报告锁敌成功才使用视觉 pitch。
            pos_ref = pitch_vision_horizon;
            if (vision_locked)
            {
                // 锁敌: 跟随视觉 pitch
                float vp = vision_data->pitch;
                if (vp > -90.0f && vp < 90.0f) // 异常数据过滤
                {
                    float pitch_ref = pitch_vision_horizon + vp * DEGREE_2_RAD * pitch_vision_dir;
                    if (pitch_ref > pitch_vision_max) pitch_ref = pitch_vision_max;
                    if (pitch_ref < pitch_vision_min) pitch_ref = pitch_vision_min;
                    pos_ref = pitch_ref;
                }
                else
                    pos_ref = pitch_motor->measure.position; // 视觉数据异常: 保持
            }
        }
        else
        {
            // 摇杆线性映射到机械范围
            float hi = 0.022f, lo = -0.78f;
            float stick = rc_data[TEMP].rc.rocker_r1;           // -660 ~ +660
            float ratio = (stick + 660.0f) / 1320.0f;           // 0.0 ~ 1.0
            if      (ratio > 1.0f) ratio = 1.0f;
            else if (ratio < 0.0f) ratio = 0.0f;
            pos_ref = lo + ratio * (hi - lo);                   // lo ~ hi
        }

        pitch_debug_ref = pos_ref;
        DMMotorSetRef(pitch_motor, pos_ref);
        pitch_debug_torque = pitch_motor->measure.torque; // 顶限位时飙升
    pitch_done:;
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

/*
 * 视觉开火到位判定：
 * yaw 比较视觉给定的小yaw目标偏角与小yaw编码器实际偏角；
 * pitch 直接比较电机实际位置和视觉pitch对应的机械位置。
 */
uint8_t GimbalVisionTargetAligned(float tolerance_deg)
{
    if (tolerance_deg < 0.0f)
        tolerance_deg = -tolerance_deg;

    vision_align_tolerance_deg = tolerance_deg;
    vision_align_yaw_ready = 0;
    vision_align_pitch_ready = 0;
    vision_align_ready = 0;

    if (vision_data == NULL)
        vision_data = VisionGetRecv();

    if (!VisionIsOnline() || vision_data == NULL ||
        vision_data->target_state != READY_TO_FIRE ||
        pitch_motor == NULL ||
        pitch_motor->motor_daemon == NULL ||
        pitch_motor->motor_daemon->temp_count == 0)
        return 0;

    /*
     * 射击时视觉yaw表示小yaw相对中位的目标偏角，
     * 大yaw是从动回中轴，不参与小yaw到位误差计算。
     */
    float small_yaw_deg = 0.0f;
    if (small_yaw_motor != NULL && small_yaw_motor->feed_cnt > 0)
    {
        small_yaw_deg = ((float)small_yaw_motor->measure.ecd - 1300.0f)
                      * ECD_ANGLE_COEF_DJI;
    }
    float yaw_error = vision_data->yaw - small_yaw_deg;
    if (yaw_error < 0.0f)
        yaw_error = -yaw_error;

    vision_align_yaw_target_deg = vision_data->yaw;
    vision_align_yaw_actual_deg = small_yaw_deg;
    vision_align_yaw_error_deg = yaw_error;
    vision_align_yaw_ready = (yaw_error <= tolerance_deg) ? 1 : 0;

    /* 使用与GimbalTask相同的视觉pitch→电机位置映射，并复用机械限位 */
    float vp = vision_data->pitch;
    if (!(vp > -90.0f && vp < 90.0f))
        return 0;
    float pitch_target = pitch_vision_horizon
                       + vp * DEGREE_2_RAD * pitch_vision_dir;
    if (pitch_target > pitch_vision_max) pitch_target = pitch_vision_max;
    if (pitch_target < pitch_vision_min) pitch_target = pitch_vision_min;

    float pitch_error = (pitch_motor->measure.position - pitch_target) * RAD_2_DEGREE;
    if (pitch_error < 0.0f)
        pitch_error = -pitch_error;

    vision_align_pitch_target_rad = pitch_target;
    vision_align_pitch_actual_rad = pitch_motor->measure.position;
    vision_align_pitch_error_deg = pitch_error;
    vision_align_pitch_ready = (pitch_error <= tolerance_deg) ? 1 : 0;
    vision_align_ready = (vision_align_yaw_ready && vision_align_pitch_ready) ? 1 : 0;

    return vision_align_ready;
}

/* 云台电机反馈：DM4310 量纲为 rad / rad/s / Nm，放大为整数回传（视觉侧按约定换算） */
void GimbalGetMotorFeedback(int16_t speed[2], int32_t pos[2], int16_t cur[2])
{
    DMMotorInstance *motors[2] = {yaw_motor, pitch_motor};
    for (uint8_t i = 0; i < 2; i++)
    {
        if (motors[i] == NULL)
        {
            speed[i] = 0;
            pos[i]   = 0;
            cur[i]   = 0;
            continue;
        }
        speed[i] = (int16_t)(motors[i]->measure.velocity * 100.0f); // rad/s × 100
        pos[i]   = (int32_t)(motors[i]->measure.position * 1000.0f); // rad × 1000 (mrad)
        cur[i]   = (int16_t)(motors[i]->measure.torque   * 1000.0f); // Nm × 1000 (mNm)
    }
}

/* 云台 pitch 轴电机位置（rad），0x12 回传的 small_pitch 字段使用 */
float GimbalGetPitchPosition(void)
{
    if (pitch_motor == NULL)
        return 0.0f;
    return pitch_motor->measure.position;
}

/* 大yaw单圈角度（度），用于把导航速度从云台坐标系转换到底盘坐标系 */
float GimbalGetYawSingleRoundAngle(void)
{
    if (yaw_motor == NULL)
        return 0.0f;

    // 达妙位置反馈为[-pi, pi] rad，这里转换为旧接口使用的[0, 360) deg。
    float angle_deg = yaw_motor->measure.position * RAD_2_DEGREE;
    if (angle_deg < 0.0f)
        angle_deg += 360.0f;
    return angle_deg;
}

/* 小yaw(GM6020)相对中心(1300ecd)的偏转角(度)，右正左负，0x12 回传的 small_yaw 字段 */
float GimbalGetSmallYawPosition(void)
{
    if (small_yaw_motor == NULL)
        return 0.0f;
    return ((float)small_yaw_motor->measure.ecd - 1300.0f) * ECD_ANGLE_COEF_DJI;
}
