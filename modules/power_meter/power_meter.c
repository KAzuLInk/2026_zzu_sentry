#include "power_meter.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include <stdlib.h>
#include <string.h>

#ifdef My_try1
// ----------------------------- My_try1 实现 -----------------------------
static PowerMeterInstance *power_meter_instances[2] = {NULL};
static uint8_t pm_idx = 0;

void PowerMeterDecodeCallback(CANInstance *_instance) {
    PowerMeterInstance *meter = (PowerMeterInstance *)_instance->id;
    uint8_t *rxbuff = (uint8_t *)_instance->rx_buff;

    // DaemonReload(meter->daemon); // 若需守护进程可开启
    meter->dt = DWT_GetDeltaT(&meter->feed_cnt);

    switch (meter->meter_type) {
        case PM_TYPE_A:
            // 示例解析（需根据实际协议修改）
            meter->measure.voltage = (int16_t)((rxbuff[0] << 8) | rxbuff[1]) / 100.0f;
            // meter->measure.current = (int16_t)((rxbuff[2] << 8) | rxbuff[3]) / 1000.0f;
            // meter->measure.power   = (int16_t)((rxbuff[4] << 8) | rxbuff[5]) / 10.0f;
            break;
        case PM_TYPE_B:
            meter->measure.voltage = ((rxbuff[0] << 24) | (rxbuff[1] << 16) | (rxbuff[2] << 8) | rxbuff[3]) / 1000.0f;
            break;
    }
}

void PowerMeterLostCallback(void *meter_ptr) {
    PowerMeterInstance *meter = (PowerMeterInstance *)meter_ptr;
    LOGWARNING("[power_meter] Power meter lost, id [%d]", meter->can_instance->tx_id);
}

PowerMeterInstance *PowerMeterInit(PowerMeter_Init_Config_s *config) {
    PowerMeterInstance *instance = (PowerMeterInstance *)malloc(sizeof(PowerMeterInstance));
    memset(instance, 0, sizeof(PowerMeterInstance));
    instance->meter_type = config->meter_type;

    config->can_init_config.can_module_callback = PowerMeterDecodeCallback;
    config->can_init_config.id = instance;
    instance->can_instance = CANRegister(&config->can_init_config);

    if (pm_idx < 2) {
        power_meter_instances[pm_idx++] = instance;
    } else {
        LOGERROR("[power_meter] Too many power meters registered");
    }
    return instance;
}

float PowerMeterGetVoltage(PowerMeterInstance *meter) { return meter->measure.voltage; }
float PowerMeterGetCurrent(PowerMeterInstance *meter) { return meter->measure.current; }
float PowerMeterGetPower(PowerMeterInstance *meter)   { return meter->measure.power; }
#endif // My_try1

#ifdef My_try2
// ----------------------------- My_try2 实现 -----------------------------
static PowerMeterInstance *PowerMeter_cap_instance = NULL; // 由调用者保存的实例指针

// CAN接收回调
static void PowerMeterRxCallback(CANInstance *_instance) {
    uint8_t *rx_buff = _instance->rx_buff;
    PowerMeter_Msg_s *Msg = &PowerMeter_cap_instance->cap_msg;

    // 解析16位有符号数据（大端）
    int16_t vol_raw     = (int16_t)((uint16_t)rx_buff[0] << 8 | (uint16_t)rx_buff[1]);
    int16_t current_raw = (int16_t)((uint16_t)rx_buff[2] << 8 | (uint16_t)rx_buff[3]);
    int16_t power_raw   = (int16_t)((uint16_t)rx_buff[4] << 8 | (uint16_t)rx_buff[5]);

    // 根据实际量程转换（假设电压0.01V，电流0.1A，功率0.01W）
    Msg->vol     = vol_raw     / 100.0f;
    Msg->current = current_raw / 1000.0f;
    Msg->power   = power_raw   / 100.0f;
}

PowerMeterInstance *PowerMeterInit(PowerMeter_Init_Config_s *powermeter_config) {
    PowerMeter_cap_instance = (PowerMeterInstance *)malloc(sizeof(PowerMeterInstance));
    memset(PowerMeter_cap_instance, 0, sizeof(PowerMeterInstance));

    powermeter_config->can_config.can_module_callback = PowerMeterRxCallback;
    PowerMeter_cap_instance->can_ins = CANRegister(&powermeter_config->can_config);
    return PowerMeter_cap_instance;
}

void PowerMeterSend(PowerMeterInstance *instance, uint8_t *data) {
    memcpy(instance->can_ins->tx_buff, data, 8);
    CANTransmit(instance->can_ins, 1);
}

float PowerMeterGetTotalPower(void) {
    if (PowerMeter_cap_instance != NULL) {
        return PowerMeter_cap_instance->cap_msg.power;
    }
    return 0.0f;
}
#endif // My_try2