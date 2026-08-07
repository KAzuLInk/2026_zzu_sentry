# gimbal

## 当前状态 (2026-08-07)

大Yaw: 双环PID，**单位已修复**（Gyro[2] rad/s 与 target_vel_rad rad/s 统一），待实机按"先内环后外环"重调。
Pitch: 4310原生POSVEL模式，摇杆直接映射，待调。
小Yaw: GM6020编码器闭环锁住，PID待调。

### 关键修复 (2026-08-07)

**根因**: 速度环 `target_vel(°/s) - Gyro[2](rad/s)` 混合单位，反馈被低估 57 倍 → 震荡+响应异常。

**修复**: 
- 速度环统一为 rad/s: `target_vel * DEGREE_2_RAD` 转换后进入速度环
- 速度环参数重标定: Kp=5.0, Ki=0.5, MaxOut=±12, DeadBand=0.01, Output_LPF_RC=0.01

### 大Yaw PID参数

| 环 | 参数 | 值 | 说明 |
|---|---|---|---|
| 角度环 | Kp | 2.0 | 角度误差比例增益 (°→°/s) |
| 角度环 | Ki | 0.03 | 克服静摩擦，消除稳态误差 |
| 角度环 | Kd | 0.25 | 抑制超调和震荡 |
| 角度环 | DeadBand | 0.05° | 死区 |
| 角度环 | MaxOut | ±60°/s | 输出上限 |
| 角度环 | IntegralLimit | ±3°/s | 积分限幅 |
| 速度环 | Kp | 5.0 | 速度误差比例增益 (rad/s→rad/s) |
| 速度环 | Ki | 0.5 | 消除稳态速度误差 |
| 速度环 | Kd | 0.0 | 微分先关 |
| 速度环 | DeadBand | 0.01 rad/s | 死区 ≈0.57°/s |
| 速度环 | MaxOut | ±12 rad/s | 输出上限 |
| 速度环 | IntegralLimit | ±3 rad/s | 积分限幅 |
| 速度环 | Output_LPF_RC | 0.01 | 输出低通滤波 |

### 数据流 (修复后)

```
YawTotalAngle(°) → [角度PID] → target_vel(°/s) → ×DEGREE_2_RAD → target_vel_rad(rad/s)
                                                                        ↓
              Gyro[2](rad/s) ─────────────────────────────────→ [速度PID] → motor_ref(rad/s)
                                                                        ↓
                                                                 DMMotorSetRef → MIT velocity_des
```

### 4310的MIT模式自带Kd=2.0速度阻尼，叠加软件速度环工作。

### 当前行为

- 上电自动锁定当前角度，保持云台在惯性空间指向不变
- 底盘慢转时积分累积克服静摩擦，云台可跟随（解决了纯速度环低速不转的问题）
- 小陀螺待实测
- 剧烈来回甩动时电机会抖（实战不会遇到）

### Pitch轴

待调试，之前被注释掉了。

---

## 后续优化方向

1. **大Yaw速度环PID细调** — 目前速度环只用了Kp=1.0，Ki/Kd均为0
2. **Pitch PID调参** — 参数直接复用了大Yaw的，需根据实际负载单独调
3. **Output_LPF_RC** — 角度环输出加低通滤波，消除突变毛刺
4. **速度前馈** — 用Gyro直接算补偿量绕过PID，加快响应
5. **MaxOut阶梯** — 误差小时大输出，误差大时限幅，防猛甩时电机疯跑
6. **遥控器指令接入** — 将gimbal_cmd_recv.yaw叠加到yaw_angle_ref上
7. **积分抗饱和** — 机械限位触发时需清掉积分项，避免回弹

---

## 工作流程

初始化pitch和yaw电机以及imu。订阅gimbal_cmd消息（来自robot_cmd）并发布gimbal_feed话题。

1. 从消息中心获取gimbal_cmd话题的消息
2. 根据消息中的控制模式进行模式切换，如果急停则关闭所有电机
3. 由设定的模式，进行电机反馈数据来源的切换，修改反馈数据指针，设置前馈控制数据指针等。
4. 设置反馈数据，包括yaw电机的绝对角度和imu数据
5. 推送反馈数据到gimbal_feed话题下
