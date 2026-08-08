# gimbal

## 当前状态 (2026-08-08)

大Yaw: 双环PID，实机已调稳定（不再震）。
Pitch: 4310原生POSVEL模式，摇杆直接映射，待调。
小Yaw: GM6020编码器闭环锁住，PID待调。

### 实机调参记录 (2026-08-08)

**问题**: 大Yaw原地震荡。
**根因**: 陀螺仪Z轴采集到机械高频振动 → 速度环Kp放大 → 电机输出抖动 → 正反馈共振。
速度环Ki=0.5积分项在零附近漂移，间歇性触发振荡。

**修复**:
- 速度环 Ki: 0.5 → 0 (间歇震荡源，静差由外环角度I兜底)
- 陀螺仪输入端新增一阶LPF (`gyro_lpf_rc=0.1`)，噪声在进PID前就滤掉
- 速度环 Output_LPF_RC: 0.01 → 0.05

**关键经验**: 滤波要放在增益之前（输入侧），不是之后（输出侧）。
Output_LPF_RC滤的是Kp放大后的噪声，晚了；gyro_lpf_rc从源头切断。
D项(速度环Kd)一加就震，因为对gyro微分放大高频噪声。

### 大Yaw PID参数

| 环 | 参数 | 值 | 说明 |
|---|---|---|---|
| 角度环 | Kp | 2.0 | 角度误差比例增益 (°→°/s) |
| 角度环 | Ki | 0.03 | 克服静摩擦，消除稳态误差 |
| 角度环 | Kd | 0.25 | 抑制超调和震荡 |
| 角度环 | DeadBand | 0.05° | 死区 |
| 角度环 | MaxOut | ±60°/s | 输出上限 |
| 角度环 | IntegralLimit | ±3°/s | 积分限幅 |
| 速度环 | Kp | 7.0 | 速度误差比例增益 (rad/s→rad/s) |
| 速度环 | Ki | 0.0 | **关闭** — 间歇震荡源 |
| 速度环 | Kd | 0.0 | **关闭** — 对gyro微分放大噪声 |
| 速度环 | DeadBand | 0.01 rad/s | 死区 ≈0.57°/s |
| 速度环 | MaxOut | ±12 rad/s | 输出上限 |
| 速度环 | IntegralLimit | ±3 rad/s | 积分限幅(Ki=0时无作用) |
| 速度环 | Output_LPF_RC | 0.05 | 输出低通滤波 |

### 陀螺仪输入LPF

| 参数 | 值 | 说明 |
|---|---|---|
| gyro_lpf_rc | 0.1 | 一阶低通RC常数，Ozone可调，越大滤波越强 |

### 数据流 (当前)

```
YawTotalAngle(°) → [角度PID] → target_vel(°/s) → ×DEGREE_2_RAD → target_vel_rad(rad/s)
                                                                       ↓
Gyro[2](rad/s) → [gyro LPF] → gyro_lpf_val → [速度PID] → motor_ref(rad/s)
                                                      ↓
                                               DMMotorSetRef → MIT velocity_des
```

---

## 工作流程

初始化pitch和yaw电机以及imu。订阅gimbal_cmd消息（来自robot_cmd）并发布gimbal_feed话题。

1. 从消息中心获取gimbal_cmd话题的消息
2. 根据消息中的控制模式进行模式切换，如果急停则关闭所有电机
3. 由设定的模式，进行电机反馈数据来源的切换，修改反馈数据指针，设置前馈控制数据指针等。
4. 设置反馈数据，包括yaw电机的绝对角度和imu数据
5. 推送反馈数据到gimbal_feed话题下
