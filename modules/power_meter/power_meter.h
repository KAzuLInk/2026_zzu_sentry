#ifndef POWER_METER_H
#define POWER_METER_H

#include "bsp_can.h"

// 选择功率计型号（只可定义一个）
// #define My_try1
#define My_try2

#ifdef My_try1
// ----------------------------- My_try1 定义 -----------------------------
typedef enum {
    PM_TYPE_A,  // A型号功率计
    PM_TYPE_B   // B型号功率计
} PowerMeterType_e;

#pragma pack(1)
typedef struct {
    float voltage;      // 电压(V)
    float current;      // 电流(A)
    float power;        // 功率(W)
} PowerMeterMeasure_s;
#pragma pack()

typedef struct {
    PowerMeterType_e meter_type;       // 功率计类型
    CANInstance* can_instance;         // CAN实例指针
    PowerMeterMeasure_s measure;       // 测量数据
    void* daemon;                      // 守护进程(用于检测通信丢失)
    uint32_t feed_cnt;                 // 饲喂计数器
    float dt;                          // 时间间隔
} PowerMeterInstance;

typedef struct {
    CAN_Init_Config_s can_init_config; // CAN初始化配置
    PowerMeterType_e meter_type;       // 功率计类型
} PowerMeter_Init_Config_s;

// 函数声明
PowerMeterInstance* PowerMeterInit(PowerMeter_Init_Config_s* config);
void PowerMeterDecodeCallback(CANInstance* _instance);
void PowerMeterLostCallback(void* meter_ptr);
float PowerMeterGetVoltage(PowerMeterInstance* meter);
float PowerMeterGetCurrent(PowerMeterInstance* meter);
float PowerMeterGetPower(PowerMeterInstance* meter);
#endif // My_try1

#ifdef My_try2
// ----------------------------- My_try2 定义 -----------------------------
#pragma pack(1)
typedef struct {
    float vol;      // 电压 (V)
    float current;  // 电流 (A)
    float power;    // 总功率 (W)
} PowerMeter_Msg_s;
#pragma pack()

/* 功率计实例 */
typedef struct {
    CANInstance *can_ins;           // CAN实例
    PowerMeter_Msg_s cap_msg;       // 功率计数据
} PowerMeterInstance;

/* 功率计初始化配置 */
typedef struct {
    CAN_Init_Config_s can_config;
} PowerMeter_Init_Config_s;

/**
 * @brief 初始化功率计
 * @param powermeter_config 功率计初始化配置
 * @return PowerMeterInstance* 功率计实例指针
 */
PowerMeterInstance *PowerMeterInit(PowerMeter_Init_Config_s *powermeter_config);

/**
 * @brief 发送控制命令到功率计
 * @param instance 功率计实例
 * @param data 要发送的8字节数据
 */
void PowerMeterSend(PowerMeterInstance *instance, uint8_t *data);

/**
 * @brief 获取功率计实测总功率（单位：W）
 * @return 总功率值，若无效返回0
 */
float PowerMeterGetTotalPower(void);

#endif // My_try2

#endif // POWER_METER_H