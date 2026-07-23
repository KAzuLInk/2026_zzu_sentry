#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "SEGGER_RTT.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

/*
 * 写DM电机寄存器 (手册 "写入参数" 章节)
 *
 * 帧格式: CAN ID = 0x7FF (广播)
 *   D[0] = CANID_L     电机CAN ID低8位
 *   D[1] = CANID_H     电机CAN ID高8位
 *   D[2] = 0x55        写命令标志
 *   D[3] = RID         寄存器地址 (如 0x0A=控制模式, 0x23=波特率)
 *   D[4:7] = value     4字节数据 (float或uint32, 小端)
 *
 * 写成功后电机会以相同格式回复一帧(MST_ID代替0x7FF)
 * 寄存器立即生效但掉电丢失, 需发0xAA命令存入flash
 */
// 写uint32寄存器 (如CTRL_MODE, MST_ID等)
static void DMMotorWriteRegU32(DMMotorInstance *motor, uint8_t reg, uint32_t value)
{
    uint8_t tx[8];
    uint16_t can_id = motor->motor_can_instace->tx_id;
    tx[0] = (uint8_t)(can_id & 0xFF);
    tx[1] = (uint8_t)((can_id >> 8) & 0xFF);
    tx[2] = 0x55;
    tx[3] = reg;
    tx[4] = (uint8_t)(value & 0xFF);
    tx[5] = (uint8_t)((value >> 8) & 0xFF);
    tx[6] = (uint8_t)((value >> 16) & 0xFF);
    tx[7] = (uint8_t)((value >> 24) & 0xFF);

    uint8_t  save_buff[8];
    uint32_t save_id = motor->motor_can_instace->txconf.Identifier;
    memcpy(save_buff, motor->motor_can_instace->tx_buff, 8);
    memcpy(motor->motor_can_instace->tx_buff, tx, 8);

#ifdef FDCAN
    motor->motor_can_instace->txconf.Identifier = 0x7FF;
#else
    motor->motor_can_instace->txconf.StdId = 0x7FF;
#endif
    CANTransmit(motor->motor_can_instace, 1);
    memcpy(motor->motor_can_instace->tx_buff, save_buff, 8);
#ifdef FDCAN
    motor->motor_can_instace->txconf.Identifier = save_id;
#else
    motor->motor_can_instace->txconf.StdId = save_id;
#endif
}

static void DMMotorWriteReg(DMMotorInstance *motor, uint8_t reg, float value)
{
    uint8_t tx[8];
    uint16_t can_id = motor->motor_can_instace->tx_id;
    tx[0] = (uint8_t)(can_id & 0xFF);
    tx[1] = (uint8_t)((can_id >> 8) & 0xFF);
    tx[2] = 0x55;
    tx[3] = reg;
    memcpy(&tx[4], &value, 4);

    // 暂存原tx_buff和Identifier, 用0x7FF发送
    uint8_t  save_buff[8];
    uint32_t save_id = motor->motor_can_instace->txconf.Identifier;
    memcpy(save_buff, motor->motor_can_instace->tx_buff, 8);
    memcpy(motor->motor_can_instace->tx_buff, tx, 8);

#ifdef FDCAN
    motor->motor_can_instace->txconf.Identifier = 0x7FF;
#else
    motor->motor_can_instace->txconf.StdId = 0x7FF;
#endif
    CANTransmit(motor->motor_can_instace, 1);

    // 恢复
    memcpy(motor->motor_can_instace->tx_buff, save_buff, 8);
#ifdef FDCAN
    motor->motor_can_instace->txconf.Identifier = save_id;
#else
    motor->motor_can_instace->txconf.StdId = save_id;
#endif
}

/*
 * 读电机寄存器 (手册 "读取参数" 章节)
 *
 * 请求帧: CAN ID=0x7FF
 *   D[0:1] = 电机CAN ID
 *   D[2]   = 0x33 (读命令)
 *   D[3]   = RID (寄存器地址)
 *
 * 响应帧: CAN ID = MST_ID
 *   D[0] = ID|ERR<<4  D[1]=CANID_H  D[2]=0x33  D[3]=RID
 *   D[4:7] = 数据 (float或uint32, 小端)
 *
 * 注意: 响应帧格式与常规反馈帧不同, 当前DMMotorDecode不会处理它
 *       这里只发请求, 响应需要单独处理或通过上位机观察
 */
static void DMMotorReadReg(DMMotorInstance *motor, uint8_t reg)
{
    motor->reg_pending = 1;  // 标记等待响应
    motor->reg_addr    = reg;

    uint8_t tx[8];
    uint16_t can_id = motor->motor_can_instace->tx_id;
    tx[0] = (uint8_t)(can_id & 0xFF);
    tx[1] = (uint8_t)((can_id >> 8) & 0xFF);
    tx[2] = 0x33;
    tx[3] = reg;
    memset(&tx[4], 0, 4);

    uint8_t  save_buff[8];
    uint32_t save_id = motor->motor_can_instace->txconf.Identifier;
    memcpy(save_buff, motor->motor_can_instace->tx_buff, 8);
    memcpy(motor->motor_can_instace->tx_buff, tx, 8);

#ifdef FDCAN
    motor->motor_can_instace->txconf.Identifier = 0x7FF;
#else
    motor->motor_can_instace->txconf.StdId = 0x7FF;
#endif
    CANTransmit(motor->motor_can_instace, 1);

    memcpy(motor->motor_can_instace->tx_buff, save_buff, 8);
#ifdef FDCAN
    motor->motor_can_instace->txconf.Identifier = save_id;
#else
    motor->motor_can_instace->txconf.StdId = save_id;
#endif
}

/*
 * 切换电机原生控制模式 (手册 "模式切换" + "存储参数")
 *
 * 流程:  检查缓存 → 相同则跳过
 *        失能 → 写寄存器0x0A → 存flash → 读回确认 → 失败重试(最多3次)
 *        使能 → 更新缓存
 *
 * @param motor       DM电机实例
 * @param native_mode 目标模式: DM_NATIVE_MODE_POSVEL(1)/POSVEL(2)/VEL(3)/POSCUR(4)
 *
 * 注意: flash擦写寿命~1万次, cached_native_mode确保只在真正需要时才写入
 */
/*
 * 切换电机原生控制模式 (手册 "模式切换" + "存储参数")
 *
 * 流程:  读当前模式 → 已是目标则跳过
 *        失能 → 写0x0A → 存flash → 读回确认 → 失败重试
 *        使能
 */
void DMMotorSwitchMode(DMMotorInstance *motor, uint8_t native_mode)
{
    uint32_t target = (uint32_t)native_mode; // CTRL_MODE是uint32寄存器!

    // 1. 读当前模式寄存器, 已是目标模式则跳过 (省一次flash写入)
    DMMotorReadReg(motor, 0x0A);
    for (uint16_t wait = 0; wait < 500; wait++)
    {
        if (motor->reg_pending == 0) break;
        osDelay(1);
    }
    if (motor->reg_pending == 0 && motor->reg_value.u32 == native_mode)
    {
        motor->cached_native_mode = native_mode;
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 模式已正确, 但需要使能
        osDelay(1);
        return;
    }

    // 2. 失能
    DMMotorSetMode(DM_CMD_RESET_MODE, motor); // 0xFD
    osDelay(5);

    // 3. 写寄存器 + 存flash + 读回确认, 失败重试3次
    for (uint8_t retry = 0; retry < 3; retry++)
    {
        DMMotorWriteRegU32(motor, 0x0A, target); // CTRL_MODE是uint32寄存器!
        osDelay(10);

        // 存储到flash (0x7FF D[2]=0xAA D[3]=0x01)
        {
            uint8_t tx[8];
            uint16_t can_id = motor->motor_can_instace->tx_id;
            tx[0] = (uint8_t)(can_id & 0xFF);
            tx[1] = (uint8_t)((can_id >> 8) & 0xFF);
            tx[2] = 0xAA; tx[3] = 0x01;
            memset(&tx[4], 0, 4);

            uint8_t  save_buff[8];
            uint32_t save_id = motor->motor_can_instace->txconf.Identifier;
            memcpy(save_buff, motor->motor_can_instace->tx_buff, 8);
            memcpy(motor->motor_can_instace->tx_buff, tx, 8);
#ifdef FDCAN
            motor->motor_can_instace->txconf.Identifier = 0x7FF;
#else
            motor->motor_can_instace->txconf.StdId = 0x7FF;
#endif
            CANTransmit(motor->motor_can_instace, 1);
            memcpy(motor->motor_can_instace->tx_buff, save_buff, 8);
#ifdef FDCAN
            motor->motor_can_instace->txconf.Identifier = save_id;
#else
            motor->motor_can_instace->txconf.StdId = save_id;
#endif
        }
        osDelay(50); // flash写入最大30ms, 留余量

        // 读回确认
        DMMotorReadReg(motor, 0x0A);
        for (uint16_t wait = 0; wait < 100; wait++)
        {
            if (motor->reg_pending == 0) break;
            osDelay(1);
        }
        if (motor->reg_pending == 0 && motor->reg_value.u32 == native_mode)
            break; // 确认成功, 退出重试
    }

    // 4. 使能
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor); // 0xFC
    osDelay(5);

    motor->cached_native_mode = native_mode;
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;

    // 寄存器响应帧拦截 (D[2]=0x33读/0x55写/0xAA存, 与反馈帧格式不同)
    // 反馈帧D[2]是POS[7:0], 概率上极少恰好等于这些命令字, 用pending标记区分
    if (motor->reg_pending && (rxbuff[2] == 0x33 || rxbuff[2] == 0x55 || rxbuff[2] == 0xAA))
    {
        memcpy(&motor->reg_value.u32, &rxbuff[4], 4); // D[4:7] = 数据
        motor->reg_pending = 0;
        return; // 不更新measure, 避免污染电机状态
    }

    uint16_t tmp;
    DM_Motor_Measure_s *measure = &(motor->measure);

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    motor ->p_max = 12.5f;
    motor ->v_max = 200.0f;
    motor ->t_max =10.0f;
    motor->kd_max = 5.0f;
    motor->kp_max = 5.0f;

    // 不在 Init 阶段发使能命令——会使电机进入 MIT 模式，但此时还没有任务发控制帧，
    // 电机看门狗超时（~20ms）后会进入 FAULT 状态，导致后续 Task 里的 0xFC 被拒绝。
    // 使能和归零统一在 DMMotorTask 开头发送，紧跟 MIT 控制帧，消除空窗期。
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}


// 诊断计数器 — Ozone Watch 窗口查看
volatile uint32_t dm_diag_task_loop   = 0;  // 自增=Task 在运行
volatile uint32_t dm_diag_tx_ok       = 0;  // CANTransmit 返回 1 的次数
volatile uint32_t dm_diag_tx_fail     = 0;  // CANTransmit 返回 0 的次数
volatile uint32_t dm_diag_rx_cnt      = 0;  // 收到电机反馈的次数

// CAN帧诊断 — 每次发送前更新, Ozone直接看
volatile uint32_t diag_can_id    = 0;  // 发送用的CAN ID
volatile uint32_t diag_tx_buf32[2];    // tx_buff[0..7] 拆成两个32位
volatile float    diag_pid_ref   = 0;  // 发给电机的值
volatile uint8_t  diag_native_mode = 0; // 当前native_mode

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void DMMotorTask(void const *argument)
{
    float  pid_ref, set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    DMMotor_Send_s motor_send_mailbox;
    uint8_t tx_ret;

    dm_diag_task_loop = 0;
    dm_diag_tx_ok = 0;
    dm_diag_tx_fail = 0;
    dm_diag_rx_cnt = 0;

    /*
     * 先设CAN ID: 原生模式所有命令(使能/归零/清错误)都需用模式对应的CAN ID
     *   电机靠ID低8位匹配, 高3位区分模式
     *   POSVEL→0x102, VEL→0x202, POSCUR→0x302
     */
    if (motor->native_mode >= DM_NATIVE_MODE_POSVEL)
    {
#ifdef FDCAN
        motor->motor_can_instace->txconf.Identifier = 0x100 * (motor->native_mode - 1) + motor->motor_can_instace->tx_id;
#else
        motor->motor_can_instace->txconf.StdId = 0x100 * (motor->native_mode - 1) + motor->motor_can_instace->tx_id;
#endif
    }

    // 清错误
    DMMotorSetMode(DM_CMD_CLEAR_ERROR, motor);
    osDelay(1);

    if (motor->native_mode >= DM_NATIVE_MODE_POSVEL)
    {
        DMMotorSwitchMode(motor, motor->native_mode);
    }
    else
    {
        DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
        osDelay(1);
    }

    // 归零 (仅MIT模式需要, 原生模式使用绝对值编码器不需要)
    if (motor->native_mode == 0)
    {
        DMMotorCaliEncoder(motor);
        osDelay(1);
    }

   
   
       if (motor->native_mode >= DM_NATIVE_MODE_POSVEL)
    {
        for (uint16_t w = 0; w < 500 && motor->motor_daemon->temp_count == 0; w++)
            osDelay(1);
        motor->pid_ref = motor->measure.position;
    }
   
   // 原生模式: 等第一帧反馈后, 把pid_ref设成当前真实位置, 避免上电猛转


    while (1)
    {
        dm_diag_task_loop++;

        // 检查是否有电机反馈
        if (motor->motor_daemon->temp_count > 0)
            dm_diag_rx_cnt++;

        // 1Hz 诊断打印
        if (dm_diag_task_loop % 500 == 0)
            SEGGER_RTT_printf(0, "DM loop:%u rx:%u tx_ok:%u tx_fail:%u\r\n",
                              dm_diag_task_loop, dm_diag_rx_cnt, dm_diag_tx_ok, dm_diag_tx_fail);

        pid_ref = motor->pid_ref;

        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;

        LIMIT_MIN_MAX(set, -motor->t_max, motor->t_max);

        /*
         * 原生位置速度模式 (手册CTRL_MODE=2, DM_NATIVE_MODE_POSVEL)
         *
         * 与MIT完全不同:
         *   - CAN ID = 0x100*(mode-1) + 电机ID
         *   - 帧格式是直存float, 不是位打包:
         *     D[0:3] = p_des  目标位置 (float LE, rad)
         *     D[4:7] = v_des  速度限制 (float LE, rad/s), 梯形速度上限
         *   - 电机内部用自己的PID (寄存器KP_APR/KI_APR/KP_ASR/KI_ASR)
         *   - MCU只发目标, 不用Kp/Kd
         *
         * MIT模式(旧):  扭矩=Kp*(pos_des-pos)+Kd*(vel_des-vel)+t_ff
         *                位打包8字节, Kp=0被固件拒绝
         * 原生模式(新):  固件自己跑位置速度双环, 干净稳定
         */
        if (motor->native_mode == DM_NATIVE_MODE_POSVEL)
        {
            float p_cmd = set;
            float v_cmd = 2.0f;
            memcpy(&motor->motor_can_instace->tx_buff[0], &p_cmd, 4);
            memcpy(&motor->motor_can_instace->tx_buff[4], &v_cmd, 4);
        }
        else
        {
            // MIT速度模式(原逻辑, yaw和loader用)
            motor_send_mailbox.position_des = float_to_uint(0, -motor->p_max, motor->p_max, 16);
            motor_send_mailbox.velocity_des = float_to_uint(pid_ref, -motor->v_max, motor->v_max, 12);
            motor_send_mailbox.Kp = float_to_uint(0, 0, motor->kp_max, 12);
            motor_send_mailbox.Kd = float_to_uint(2.0f, 0, motor->kd_max, 12);
            motor_send_mailbox.torque_des = float_to_uint(0, -motor->t_max, motor->t_max, 12);

            if(motor->stop_flag == MOTOR_STOP)
                motor_send_mailbox.torque_des = float_to_uint(0, -motor->t_max, motor->t_max, 12);

            motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
            motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
            motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
            motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
            motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
            motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
            motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
            motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);
        }

        if(motor->stop_flag == MOTOR_STOP)
            memset(motor->motor_can_instace->tx_buff, 0, 8);
        // // ★ 硬编码测试帧：vel=10, Kd=0.5, 和上位机完全一致
        // motor->motor_can_instace->tx_buff[0] = 0x7F;
        // motor->motor_can_instace->tx_buff[1] = 0xFF;
        // motor->motor_can_instace->tx_buff[2] = 0x86;
        // motor->motor_can_instace->tx_buff[3] = 0x50;
        // motor->motor_can_instace->tx_buff[4] = 0x00;
        // motor->motor_can_instace->tx_buff[5] = 0x19;
        // motor->motor_can_instace->tx_buff[6] = 0x97;
        // motor->motor_can_instace->tx_buff[7] = 0xFF;
        // 诊断抓取 — Ozone直接看
        diag_pid_ref    = pid_ref;
        diag_native_mode = motor->native_mode;
#ifdef FDCAN
        diag_can_id     = motor->motor_can_instace->txconf.Identifier;
#else
        diag_can_id     = motor->motor_can_instace->txconf.StdId;
#endif
        memcpy((void*)diag_tx_buf32, motor->motor_can_instace->tx_buff, 8);

        tx_ret = CANTransmit(motor->motor_can_instace, 1);
        if (tx_ret)
            dm_diag_tx_ok++;
        else
            dm_diag_tx_fail++;

        osDelay(2);
    }
}
void DMMotorControlInit()
{
    if (!idx)
        return;
    if (idx >= 1) {
        osThreadDef(dm0, DMMotorTask, osPriorityNormal, 0, 256);
        dm_task_handle[0] = osThreadCreate(osThread(dm0), dm_motor_instance[0]);
    }
    if (idx >= 2) {
        osThreadDef(dm1, DMMotorTask, osPriorityNormal, 0, 256);
        dm_task_handle[1] = osThreadCreate(osThread(dm1), dm_motor_instance[1]);
    }
    if (idx >= 3) {
        osThreadDef(dm2, DMMotorTask, osPriorityNormal, 0, 256);
        dm_task_handle[2] = osThreadCreate(osThread(dm2), dm_motor_instance[2]);
    }
    if (idx >= 4) {
        osThreadDef(dm3, DMMotorTask, osPriorityNormal, 0, 256);
        dm_task_handle[3] = osThreadCreate(osThread(dm3), dm_motor_instance[3]);
    }
}