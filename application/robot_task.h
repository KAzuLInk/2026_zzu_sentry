/* 注意该文件应只用于任务初始化,只能被robot.c包含*/
#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"
#include "go_motor.h"
#include "dmmotor.h"
#include "bsp_log.h"
#include "bsp_can.h"

osThreadId insTaskHandle;
osThreadId robotTaskHandle;
osThreadId motorTaskHandle;
osThreadId daemonTaskHandle;
osThreadId uiTaskHandle;
osThreadId bootTaskHandle;

void StartINSTASK(void const *argument);
void StartMOTORTASK(void const *argument);
void StartDAEMONTASK(void const *argument);
void StartROBOTTASK(void const *argument);
void StartUITASK(void const *argument);
void StartBOOTTASK(void const *argument);

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OSTaskInit()
{
    // 开机音效任务: 与 INS 同级, 保证 INS_Init 忙等期间能穿插播放; 播完自杀
    osThreadDef(boottask, StartBOOTTASK, osPriorityAboveNormal, 0, 256);
    bootTaskHandle = osThreadCreate(osThread(boottask), NULL);

    osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 由于是阻塞读取传感器,为姿态解算设置较高优先级,确保以1khz的频率执行
    // 后续修改为读取传感器数据准备好的中断处理,

    // osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 256);
    osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(daemontask, StartDAEMONTASK, osPriorityNormal, 0, 128);
    daemonTaskHandle = osThreadCreate(osThread(daemontask), NULL);

    osThreadDef(robottask, StartROBOTTASK, osPriorityNormal, 0, 1024);
    robotTaskHandle = osThreadCreate(osThread(robottask), NULL);
    
#if defined(ONE_BOARD) || defined(CHASSIS_BOARD)
    // osThreadDef(uitask, StartUITASK, osPriorityNormal, 0, 512);
    // uiTaskHandle = osThreadCreate(osThread(uitask), NULL);
#endif

    // GOMotorControlInit();
    //HTMotorControlInit(); // 没有注册HT电机则不会执行
    DMMotorControlInit();
}

__attribute__((noreturn)) void StartINSTASK(void const *argument)
{
    static float ins_start;
    static float ins_dt;
    INS_Init(); // 确保BMI088被正确初始化.
    LOGINFO("[freeRTOS] INS Task Start");
    for (;;)
    {
        // 1kHz
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            LOGERROR("[freeRTOS] INS Task is being DELAY! dt = [%f]", &ins_dt);
        // VisionSend(); // 解算完成后发送视觉数据,但是当前的实现不太优雅,后续若添加硬件触发需要重新考虑结构的组织
        osDelay(1);
    }
}

__attribute__((noreturn)) void StartMOTORTASK(void const *argument)
{
    static float motor_dt;
    static float motor_start;
    LOGINFO("[freeRTOS] MOTOR Task Start");
    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            LOGERROR("[freeRTOS] MOTOR Task is being DELAY! dt = [%f]", &motor_dt);
        osDelay(1);
    }
}

/* 开机音效任务: 上电后循环播前奏直到 IMU 初始化完成, 再滴滴两声, 然后自杀 */
void StartBOOTTASK(void const *argument)
{
    BuzzerInit(); // 幂等, 与 daemon 的调用不冲突

#ifdef GIMBAL_BOARD
    {
        Buzzer_config_s boot_cfg = {
            .alarm_level = ALARM_LEVEL_LOW,   // 最低优先级, 不干扰报警
            .loudness = 0.5f,
            .octave = OCTAVE_1,
        };
        BuzzzerInstance *boot_buzzer = BuzzerRegister(&boot_cfg);

        // 前奏音: 0=休止, 1~7=中音 do~si, 8/9/10=高音 do/re/mi
        static const uint8_t boot_note[] = {
            5, 9, 8, 5, 5, 8, 9, 10, 9, 8, 9,
            5, 9, 8, 5, 5, 8, 9, 10, 9, 8, 9,
            5, 9, 8, 5, 5, 8, 9, 10, 9, 8, 9,
            5, 9, 8, 5,
        };
        // 时值编码: 2=八分, 4=四分, 8=二分(末拍 4-4 连音)
        static const uint8_t boot_dur[] = {
            4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2,
            4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2,
            4, 4, 4, 4, 4, 2, 2, 2, 2, 2, 2,
            4, 4, 4, 8,
        };
        const uint32_t boot_beat_ms = 172; // 一拍 172ms, 前奏(29拍) ≈ 5s

        while (!INS_init_done)
        {
            const uint32_t note_count = sizeof(boot_note) / sizeof(boot_note[0]);
            for (uint32_t i = 0; i < note_count; ++i)
            {
                uint32_t ms;
                switch (boot_dur[i])
                {
                case 2: ms = boot_beat_ms / 2; break; // 八分 86ms
                case 4: ms = boot_beat_ms;     break; // 四分 172ms
                case 8: ms = boot_beat_ms * 2; break; // 二分 344ms
                default: ms = boot_beat_ms / 2; break;
                }
                if (boot_note[i] == 0)
                    AlarmSetStatus(boot_buzzer, ALARM_OFF); // 休止
                else
                {
                    boot_buzzer->octave = (octave_e)(boot_note[i] - 1); // 1..10 -> OCTAVE_1..OCTAVE_10
                    AlarmSetStatus(boot_buzzer, ALARM_ON);
                }
                BuzzerTask(); // 应用 PWM 设置
                osDelay(ms);
            }
        }

        // 初始化完成: 滴滴两声 (高音 do)
        for (uint8_t i = 0; i < 2; ++i)
        {
            boot_buzzer->octave = OCTAVE_8;
            AlarmSetStatus(boot_buzzer, ALARM_ON);
            BuzzerTask();
            osDelay(150);
            AlarmSetStatus(boot_buzzer, ALARM_OFF);
            BuzzerTask();
            osDelay(100);
        }
    }
#endif

    vTaskDelete(NULL); // 开机音效结束, 删除自己
    for (;;) {}        // 保险, 不可达
}

__attribute__((noreturn)) void StartDAEMONTASK(void const *argument)
{
    static float daemon_dt;
    static float daemon_start;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");
    for (;;)
    {
        // 100Hz
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        CANBusOffRecovery(); // FDCAN bus-off 检测恢复, 支持电机热插拔
        BuzzerTask();
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            LOGERROR("[freeRTOS] Daemon Task is being DELAY! dt = [%f]", &daemon_dt);
        osDelay(10);
    }
}

__attribute__((noreturn)) void StartROBOTTASK(void const *argument)
{
    static float robot_dt;
    static float robot_start;
    LOGINFO("[freeRTOS] ROBOT core Task Start");
    // 200Hz-500Hz,若有额外的控制任务如平衡步兵可能需要提升至1kHz
    for (;;)
    {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 5)
            LOGERROR("[freeRTOS] ROBOT core Task is being DELAY! dt = [%f]", &robot_dt);
        osDelay(10);
    }
}

// __attribute__((noreturn)) void StartUITASK(void const *argument)
// {
//     LOGINFO("[freeRTOS] UI Task Start");
//     MyUIInit();
//     LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
//     for (;;)
//     {
        
//         // 每给裁判系统发送一包数据会挂起一次,详见UITask函数的refereeSend()
//         UITask();
//         osDelay(5); // 即使没有任何UI需要刷新,也挂起一次,防止卡在UITask中无法切换
//     }
// }
