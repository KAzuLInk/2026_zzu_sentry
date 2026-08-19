#ifndef GIMBAL_H
#define GIMBAL_H

#include <stdint.h>

/**
 * @brief 初始化云台,会被RobotInit()调用
 *
 */
void GimbalInit();

/**
 * @brief 云台任务
 *
 */
void GimbalTask();

/**
 * @brief 判断视觉目标是否已经进入允许开火的角度窗口
 * @param tolerance_deg yaw、pitch 允许误差（度）
 * @return 1=视觉锁敌且两轴均到位，0=未锁敌或未到位
 */
uint8_t GimbalVisionTargetAligned(float tolerance_deg);

/**
 * @brief 获取云台 yaw/pitch 电机反馈（供视觉回传 0x11 使用）
 * @param speed [out] 2 电机转速 (rad/s × 100)
 * @param pos   [out] 2 电机位置 (rad × 1000, 即 mrad)
 * @param cur   [out] 2 电机力矩 (Nm × 1000, 即 mNm)
 */
void GimbalGetMotorFeedback(int16_t speed[2], int32_t pos[2], int16_t cur[2]);

/**
 * @brief 获取云台 pitch 轴电机位置（供视觉回传 0x12 的 small_pitch 字段使用）
 * @return pitch 电机位置 (rad)
 */
float GimbalGetPitchPosition(void);

/**
 * @brief 获取小yaw(GM6020)相对中心(1300ecd)的偏转角（供视觉回传 0x12 的 small_yaw 字段使用）
 * @return 偏转角 (度)，右正左负
 */
float GimbalGetSmallYawPosition(void);

#endif // GIMBAL_H
