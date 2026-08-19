# USB 串口通信协议 & serial_bridge 使用说明

> 版本 V2（2026-08-17）
>
> 相对 V1(4) 的变化：补充了 [serial_bridge](serial_bridge/) 包的 **ROS2 话题 ↔ 协议帧映射**章节（第五章），
> 反映反馈数据**原生直传**的最新设计（自定义消息直接对应协议字段、不做任何换算），并精简了发布话题。
> 传输层协议（帧格式 / CRC / Payload 结构 / USB 类 API）未变，与 V1(4) 一致。

---

## 一、概述

本项目通过 USB 虚拟串口（`/dev/ttyACM0`）与机器人电控（下位机）通信。通信基于**自定义二进制帧协议**（Seasky 协议），包含帧头、长度、消息 ID、负载数据、CRC-16 校验。上层通过 `USB` 类封装了发送和接收操作。

代码分两层：

- **[serial_bridge/tools/usb.cpp](serial_bridge/tools/usb.cpp) / [usb.h](serial_bridge/tools/usb.h)** —— 传输层。负责串口读写、组帧/拆帧、CRC、断线重连。仓库根目录 [usb.cpp](usb.cpp) / [usb.h](usb.h) 与此为同一文件。
- **[serial_bridge](serial_bridge/)** —— ROS2 节点（`serial_bridge_split` / `serial_bridge_merge`）。负责串口 ↔ ROS2 话题的桥接。**反馈类话题全部原生直传**：收到哪一帧，就按协议数据接口格式发布对应话题，字段与协议原始数据一一对应，不做单位换算 / 坐标系包装 / 派生计算（详见第五章）。

### 通信参数

| 参数 | 值 |
|------|-----|
| 波特率 | 115200（默认，构造时可指定） |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | 无 |
| 流控 | 无 |
| 读取模式 | 非阻塞（VMIN=0, VTIME=0） |

---

## 二、帧协议（Seasky 协议）

### 2.1 帧结构

```
┌───────┬───────┬─────────┬───────────┬───────────┐
│ 0xA5 │  Len  │ Msg_ID  │ Payload  │  CRC-16   │
│ 1B    │ 2B    │  1B     │  0~N B    │   2B      │
└───────┴───────┴─────────┴───────────┴───────────┘
```

| 字段 | 偏移 | 大小 | 说明 |
|------|------|------|------|
| Header | 0 | 1 字节 | 固定值 `0xA5`，用于帧同步 |
| Len | 1 | 2 字节 | 负载（Payload）长度，单位字节，**小端序** |
| Msg_ID | 3 | 1 字节 | 消息类型 ID |
| Payload | 4 | Len 字节 | 实际数据负载（发送帧的第一个字节 8 位全部为标志位，目前只用到最低两位，高位留作扩展先全部置 0。bit0=锁敌，0 表示没有锁敌，1 为锁敌；bit1=小陀螺） |
| CRC-16 | 4+Len | 2 字节 | CRC-16/Modbus 校验值，**小端序**（低字节在前） |

**帧总长** = 1 + 2 + 1 + Len + 2 = **6 + Len 字节**

### 2.2 CRC 校验

- **算法**：CRC-16/Modbus（多项式 `0xA001`，初始值 `0xFFFF`）
- **计算范围**：Header + Len + Msg_ID + Payload（共 4 + Len 字节），**不包含 CRC 字段本身**
- **字节序**：小端序写入，低字节在前

代码位置：[serial_bridge/tools/usb.cpp:197-206](serial_bridge/tools/usb.cpp#L197-L206)（发送侧拼接）、[serial_bridge/tools/usb.cpp:309-316](serial_bridge/tools/usb.cpp#L309-L316)（接收侧提取）

### 2.3 帧边界识别（处理粘包/半包）

接收缓冲区是线性缓冲区，工作流程：

1. **`rx_read()`** 从串口非阻塞读取，追加到 `rx_buf` 尾部
2. **`recv_frame(expected_id, payload, &len)`** 在 `rx_buf` 中搜索帧头 `0xA5`：
   - 找到 `0xA5` → 读取 Len 字段：
     - Len 异常（整帧超过缓冲区 1024B 上限）→ 视为假帧头跳过，继续搜索（防脏长度卡死缓冲/越界读）
     - 数据不够一帧 → 返回 -1，等待下次 `rx_read()` 补充
   - 数据够一帧 → 计算 CRC-16 → 比对
   - CRC 匹配 → 检查 msg_id 是否等于 `expected_id`
     - 匹配 → 提取 Payload，**只从缓冲区删除这一帧**（它前后的数据原样保留），返回 0
     - 不匹配 → **整帧保留并跳过**，继续往后搜索（留给其他 `recv_*` 处理）
   - CRC 不匹配 → 跳过当前 `0xA5`（假帧头），继续搜索
3. 返回前清除搜索过的无效字节（防止缓冲区无限增长），但**保留**不匹配的合法帧和尾部可能的半帧

### 2.4 缓冲区溢出保护

缓冲区大小 `RX_BUF_SIZE = 1024` 字节。当剩余空间不足 256 字节时，丢弃最旧的 512 字节数据腾出空间（`memmove` 向前搬运）。适用于实时流式数据场景——宁可丢旧数据也要保证新数据能写入。

> 注意：如果某类消息电控持续上报但程序从不调用对应的 `recv_*` 去取，这些帧会留在缓冲区中越积越多，最终触发此保护被丢弃。正常轮询所有在用的消息类型即可避免。

代码位置：[serial_bridge/tools/usb.cpp:249-251](serial_bridge/tools/usb.cpp#L249-L251)

### 2.5 发送可靠性

`seasky_send` 对发送侧做了三层防护：

1. **长度边界检查**：负载超过 258 字节（帧缓冲上限 264B）直接丢弃并报错，防止溢出 `frame[]`
2. **短写重试循环**：`write()` 可能只写入部分数据（USB CDC 特性），循环写直到整帧写完；`write()` 返回 `<=0` 判定设备断开，关闭 fd
3. **空负载保护**：负载长度为 0（如心跳包）时跳过 `memcpy`，避免空指针操作

代码位置：[serial_bridge/tools/usb.cpp:188-242](serial_bridge/tools/usb.cpp#L188-L242)

---

## 三、消息 ID 定义

### 3.1 主控 → 电控（发送侧）

| Msg_ID | 消息名 | Payload 大小 | 说明 |
|--------|--------|-------------|------|
| `0x01` | 底盘控制 | 13B | 标志位（1B：bit0=锁敌 bit1=小陀螺）+ vx + vy + vw（各 float 4B） |
| `0x02` | 云台控制 | 9B | 标志位（1B），yaw, pitch（各 float 4B） |
| `0x03` | 发射控制 | 6B | fire_mode(1B) + fire_speed(float 4B) + bullet_type(1B) |
| `0x05` | 综合控制 | 22B | 标志位（1B：bit0=锁敌 bit1=小陀螺）+ yaw + pitch + fire + vx + vy + vz（5×float + 1B） |
| `0x0F` | 心跳包 | 0B | 空负载，仅用于保活检测 |

### 3.2 电控 → 主控（接收侧）

| Msg_ID | 消息名 | Payload 大小 | 结构 | serial_bridge 对应话题 |
|--------|--------|-------------|------|----------------------|
| `0x10` | 底盘电机反馈 | 32B | 4 电机 × (speed int16 + pos int32 + cur int16) = 4×8B | `/wheel_states` |
| `0x11` | 云台电机反馈 | 16B | 2 电机(yaw/pitch) × (speed int16 + pos int32 + cur int16) = 2×8B | `/gimbal_feedback` |
| `0x12` | IMU 数据 | 56B | yaw + pitch + roll + gyro_x + gyro_y + gyro_z + small_yaw + small_pitch + chassis_yaw + chassis_pitch + chassis_roll + chassis_gyro_x + chassis_gyro_y + chassis_gyro_z（各 float 4B，后 6 个为底盘板 IMU） | `/imu/data` |
| `0x13` | 电池信息 | 5B | voltage uint16 + current uint16 + capacity uint8 | `/battery` |
| `0x14` | 机器人状态 | 4B | mode uint8 + hp uint16 + error uint8 | `/robot_status` |
| `0x1F` | 心跳响应 | 0B | 空负载，收到表示电控在线 | （不发布话题，只刷新在线状态） |

---

## 四、Payload 详细结构

> flags 字节定义（发送侧 0x01/0x02/0x05 共用）：bit0=`FLAG_LOCK`(0x01)=锁敌，bit1=`FLAG_SPIN`(0x02)=小陀螺，其余位保留置 0，可组合（如 `0x03` = 锁敌+小陀螺）。

### 4.1 底盘控制 `0x01` — 13 字节

```
┌───────────┬──────────┬──────────┬──────────┐
│ flags     │ vx       │ vy       │ vw       │
│ uint8 1B  │ float 4B │ float 4B │ float 4B │
└───────────┴──────────┴──────────┴──────────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| flags | uint8_t | 标志位：bit0=锁敌，bit1=小陀螺（1=开启，0=关闭），其余位保留置 0 |
| vx | float | 前进速度（m/s），正值向前 |
| vy | float | 横移速度（m/s），正值向左 |
| vw | float | 旋转角速度（rad/s），正值逆时针 |

### 4.2 云台控制 `0x02` — 9 字节

```
┌───────────┬──────────────┬─────────────┐
│  flags    │    yaw       │  pitch      │
│ uint8 1B  │ float 4B     │ float 4B    │
└───────────┴──────────────┴─────────────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| flags | uint8_t | 标志位：bit0=锁敌，bit1=小陀螺（1=开启，0=关闭），其余位保留置 0 |
| yaw | float | 云台偏航角（度） |
| pitch | float | 云台俯仰角（度） |

### 4.3 发射控制 `0x03` — 6 字节

```
┌───────────┬──────────────┬─────────────┐
│ fire_mode │ fire_speed   │ bullet_type │
│ uint8 1B  │ float 4B     │ uint8 1B    │
└───────────┴──────────────┴─────────────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| fire_mode | uint8_t | 发射模式（0=停，1=单发，2=连发等） |
| fire_speed | float | 发射速度（m/s） |
| bullet_type | uint8_t | 弹丸类型 |

### 4.4 综合控制 `0x05` — 22 字节

```
┌───────┬──────┬───────┬──────┬──────┬──────┬──────┐
│ flags │ yaw  │ pitch │ fire │ vx   │ vy   │ vz   │
│ 1B    │ 4B   │ 4B    │ 1B   │ 4B   │ 4B   │ 4B   │
└───────┴──────┴───────┴──────┴──────┴──────┴──────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| flags | uint8_t | 标志位：bit0=锁敌，bit1=小陀螺（1=开启，0=关闭），其余位保留置 0 |
| yaw | float | 云台偏航角（度） |
| pitch | float | 云台俯仰角（度） |
| fire | uint8_t | 开火模式（0=停，1=单发，2=连发） |
| vx | float | 前进速度（m/s） |
| vy | float | 横移速度（m/s） |
| vz | float | 底盘旋转角速度（rad/s） |

> 注：ROS 消息 [FixCmd.msg](serial_bridge/msg/FixCmd.msg) 中对应字段名写作 `vw`，与协议字节含义相同（vz = vw = 旋转角速度）。

### 4.5 电机反馈 `0x10` / `0x11` — 每个电机 8 字节

底盘 4 个电机共 32B，云台 2 个电机（yaw, pitch）共 16B。每个电机结构相同：

```
┌───────────┬───────────┬──────────┐
│ speed     │ pos       │ cur      │
│ int16 2B  │ int32 4B  │ int16 2B │
└───────────┴───────────┴──────────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| speed | int16_t | 电机转速（编码器计数/s 或原始值，取决于电控固件） |
| pos | int32_t | 编码器累计位置（脉冲数），上电从 0 开始，int32 范围 ±21 亿 |
| cur | int16_t | 电机电流（mA 或原始值） |

> serial_bridge **不做单位换算**：`/wheel_states`、`/gimbal_feedback` 发布的就是上面这三个原生数组（见 5.4）。

### 4.6 IMU 数据 `0x12` — 56 字节

```
字段顺序（小端序，各 4B）：
yaw, pitch, roll, gyro_x, gyro_y, gyro_z, small_yaw, small_pitch,
chassis_yaw, chassis_pitch, chassis_roll, chassis_gyro_x, chassis_gyro_y, chassis_gyro_z
```

| 字段 | 类型 | 说明 |
|------|------|------|
| yaw | float | 大云台偏航角（度），含磁力计修正，相对大地 |
| pitch | float | 大云台俯仰角（度） |
| roll | float | 大云台横滚角（度） |
| gyro_x | float | 大云台 X 轴角速度（度/秒） |
| gyro_y | float | 大云台 Y 轴角速度（度/秒） |
| gyro_z | float | 大云台 Z 轴角速度（度/秒） |
| small_yaw | float | 小云台偏航角（度） |
| small_pitch | float | 小云台俯仰角（度） |
| chassis_yaw | float | 底盘 IMU 偏航角（度），协议末尾追加 |
| chassis_pitch | float | 底盘 IMU 俯仰角（度） |
| chassis_roll | float | 底盘 IMU 横滚角（度） |
| chassis_gyro_x | float | 底盘 IMU X 轴角速度（度/秒） |
| chassis_gyro_y | float | 底盘 IMU Y 轴角速度（度/秒） |
| chassis_gyro_z | float | 底盘 IMU Z 轴角速度（度/秒） |

> serial_bridge 不把角度转弧度、也不转四元数，`/imu/data` 发布的就是上面 14 个 float 原值。
> 兼容旧帧：电控仍发旧帧（无 small / chassis 字段）时，缺的字段按 0 填充，不丢已有 IMU 数据。

### 4.7 电池信息 `0x13` — 5 字节

```
┌──────────┬──────────┬──────────┐
│ voltage  │ current  │ capacity │
│ uint16 2B│ uint16 2B│ uint8 1B │
└──────────┴──────────┴──────────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| voltage | uint16_t | 电压（mV） |
| current | uint16_t | 电流（mA） |
| capacity | uint8_t | 电量百分比（0~100） |

> serial_bridge 不把 mV 换算成 V，`/battery` 发布 `voltage_mv / current_ma / capacity_percent` 原值。

### 4.8 机器人状态 `0x14` — 4 字节

```
┌───────┬───────┬───────┐
│ mode  │ hp    │ error │
│ 1B    │ 2B    │ 1B    │
└───────┴───────┴───────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| mode | uint8_t | 机器人当前模式 |
| hp | uint16_t | 血量值 |
| error | uint8_t | 错误码（0=正常） |

---

## 五、serial_bridge：ROS2 话题 ↔ 协议帧映射

serial_bridge 包提供两个互为替代的节点，传输层（USB 类）与反馈接收逻辑共用基类 [serial_bridge_base](serial_bridge/src/serial_bridge_base.cpp)：

| 节点 | 适用场景 | 发送方式 |
|------|---------|---------|
| `serial_bridge_split` | 上游分别发布云台/底盘指令 | 事件驱动：收到话题立即发对应帧（延迟最低） |
| `serial_bridge_merge` | 上游统一发布合包指令 | 固定频率（`control_hz`，默认 50Hz）合并发 `0x05`+`0x03` |

### 5.1 发送侧：订阅话题 → 协议帧

两个节点共同订阅的 Bool 标志话题（[serial_bridge_base.cpp:86-98](serial_bridge/src/serial_bridge_base.cpp#L86-L98)）：

| 订阅话题 | 消息类型 | 作用 | 如何进入协议帧 |
|----------|---------|------|---------------|
| `/cmd_spin` | `std_msgs/Bool` | 小陀螺开关 | 置 flags bit1（`FLAG_SPIN`） |
| `/gimbal_lock` | `std_msgs/Bool` | 云台锁定（锁敌） | 置 flags bit0（`FLAG_LOCK`） |
| `/fire_cmd` | `std_msgs/Bool` | 开火（电平：持续 true = 持续单发） | 0x03 fire_mode=1 / 0x05 fire=1 |

> Bool 命令 QoS 为 best_effort + depth=1：只认最新，旧命令作废，绝不把过时指令发到串口。
> 标志位由节点内部电平保持（`spin_on_`/`lock_on_`/`fire_on_`），话题上只带开关数值。

**split 节点**（[serial_bridge_split.cpp](serial_bridge/src/serial_bridge_split.cpp)）额外订阅：

| 订阅话题 | 消息类型 | 发出帧 | 说明 |
|----------|---------|--------|------|
| `/gimbal_cmd` | `serial_bridge/msg/GimbleCmd`（yaw/pitch，度） | `0x02` | 收到立即发，flags 带当前锁敌位 |
| `/chassis_cmd` | `geometry_msgs/Twist`（linear.x/y, angular.z） | `0x01` | 收到立即发，带小陀螺位 |

小陀螺/云台锁定状态变化时，split 会用最近一次底盘/云台值**立即重发**对应帧，让新标志立刻生效（[serial_bridge_split.cpp:49-58](serial_bridge/src/serial_bridge_split.cpp#L49-L58)）。上游断流 = 对应帧停发。

**merge 节点**（[serial_bridge_merge.cpp](serial_bridge_merge.cpp)）额外订阅：

| 订阅话题 | 消息类型 | 发出帧 | 说明 |
|----------|---------|--------|------|
| `/fix_cmd` | `serial_bridge/msg/FixCmd`（yaw/pitch/vx/vy/vw/fire） | `0x05` + `0x03` | 锁存到定时器，`control_hz` 固定频率合并发送 |

merge 的活性清零规则（[serial_bridge_merge.cpp:39-73](serial_bridge/src/serial_bridge_merge.cpp#L39-L73)）：

- 各命令源超过 `cmd_timeout_ms`（默认 500ms）未更新 → 对应字段清零、标志位复位
- **从未收到过任何指令** → 静默不发（不给 MCU 发无意义的清零帧）
- 全部超时 → 停止发送（MCU 侧超时保护兜底）
- fire_mode 优先取 `/fix_cmd.fire`；为 0 时若 `/fire_cmd` 在线且为 true，退回 fire_mode=1

### 5.2 接收侧：协议帧 → 发布话题（原生直传）

接收线程为事件驱动（[serial_bridge_base.cpp:173-197](serial_bridge/src/serial_bridge_base.cpp#L173-L197)）：`wait_readable()` 挂起等待，收到数据后把缓冲里的完整帧全部取出并分发。**读到哪帧立即发布对应话题，无发布定时器。**

| 协议帧 | 发布话题 | 消息类型 | QoS |
|--------|---------|---------|-----|
| `0x10` 底盘电机反馈 | `/wheel_states` | `serial_bridge/msg/ChassisFeedback` | best_effort, depth=10 |
| `0x11` 云台电机反馈 | `/gimbal_feedback` | `serial_bridge/msg/GimbalFeedback` | best_effort, depth=10 |
| `0x12` IMU 数据 | `/imu/data` | `serial_bridge/msg/ImuData` | best_effort, depth=10 |
| `0x13` 电池信息 | `/battery` | `serial_bridge/msg/BatteryInfo` | reliable, depth=10 |
| `0x14` 机器人状态 | `/robot_status` | `serial_bridge/msg/RobotStatus` | reliable, depth=10 |
| `0x1F` 心跳响应 | （不发布） | —— | 只刷新在线状态 |
| 看门狗 | `/serial_online` | `std_msgs/Bool` | reliable, depth=10 |

> 数据类话题（IMU / 电机反馈）用 best_effort 容忍丢帧；低频状态类（电池 / 状态 / 在线）用 reliable 保证不丢。

### 5.3 发布侧总览

```
                           ┌─────────────────────────────┐
   0x10 底盘电机反馈 ──────▶ │ /wheel_states              │
   0x11 云台电机反馈 ──────▶ │ /gimbal_feedback           │
   0x12 IMU         ──────▶ │ /imu/data                  │
   0x13 电池信息     ──────▶ │ /battery                   │
   0x14 机器人状态   ──────▶ │ /robot_status              │
   0x1F 心跳响应     ──────▶ │ （仅刷新在线）               │
   看门狗活性        ──────▶ │ /serial_online (Bool)      │
                           └─────────────────────────────┘
```

话题名均可通过参数改名（`topic_imu` / `topic_wheel_states` …，见 [config/params.yaml](serial_bridge/config/params.yaml)）。

### 5.4 反馈消息定义（自定义消息，字段与协议原生一致）

每个反馈帧对应一个自定义消息，字段与 4.x 的 payload 字节**一一对应、无任何换算**：

**`ChassisFeedback`**（0x10，[msg/ChassisFeedback.msg](serial_bridge/msg/ChassisFeedback.msg)）
| 字段 | 类型 | 对应协议字段 |
|------|------|-------------|
| `wheel_speed` | int16[4] | 各轮 speed（编码器计数/s，原生单位） |
| `wheel_pos` | int32[4] | 各轮 pos（编码器计数，原生单位） |
| `wheel_cur` | int16[4] | 各轮 cur（原生单位） |

**`GimbalFeedback`**（0x11，[msg/GimbalFeedback.msg](serial_bridge/msg/GimbalFeedback.msg)）
| 字段 | 类型 | 对应协议字段 |
|------|------|-------------|
| `motor_speed` | int16[2] | yaw/pitch 两电机 speed |
| `motor_pos` | int32[2] | yaw/pitch 两电机 pos |
| `motor_cur` | int16[2] | yaw/pitch 两电机 cur |

**`ImuData`**（0x12，[msg/ImuData.msg](serial_bridge/msg/ImuData.msg)）
| 字段 | 类型 | 对应协议字段 |
|------|------|-------------|
| `yaw` / `pitch` / `roll` | float32 | 姿态角原值（不转弧度，大云台） |
| `gyro_x` / `gyro_y` / `gyro_z` | float32 | 角速度原值（不转四元数） |
| `small_yaw` / `small_pitch` | float32 | 小云台姿态角原值（协议末尾追加） |
| `chassis_yaw` / `chassis_pitch` / `chassis_roll` | float32 | 底盘 IMU 姿态角原值 |
| `chassis_gyro_x` / `chassis_gyro_y` / `chassis_gyro_z` | float32 | 底盘 IMU 角速度原值 |

**`BatteryInfo`**（0x13，[msg/BatteryInfo.msg](serial_bridge/msg/BatteryInfo.msg)）
| 字段 | 类型 | 对应协议字段 |
|------|------|-------------|
| `voltage_mv` | uint16 | voltage（mV，不换算成 V） |
| `current_ma` | uint16 | current（mA） |
| `capacity_percent` | uint8 | capacity（%） |

**`RobotStatus`**（0x14，[msg/RobotStatus.msg](serial_bridge/msg/RobotStatus.msg)）
| 字段 | 类型 | 对应协议字段 |
|------|------|-------------|
| `mode` | uint8 | mode |
| `hp` | uint16 | hp |
| `error` | uint8 | error |

### 5.5 原生直传约定

通信节点只负责**串口 ↔ 话题的桥接**，遵循两条原则：

1. **反馈 = 协议原样**：收到什么数据就按协议数据接口格式发布相应话题，字段与协议原生一致，不做单位换算（度→弧度、mV→V）、不做坐标系/四元数包装、不做派生计算（里程计积分、麦轮逆解等）。
2. **不加附加话题**：监控类、派生计算类话题（如里程计、tf、diagnostics）由上层自行实现，不在桥接节点内。

这样设计的好处：桥接节点是"透明的管道"，上层拿到的是电控定义的原生语义，任何换算/派生由真正需要的上层决定，避免重复和错误换算。

### 5.6 心跳 / 看门狗 / 在线状态

- **心跳发送**：定时器按 `heartbeat_hz`（默认 2Hz）发 `0x0F`（[serial_bridge_base.cpp:101-104](serial_bridge/src/serial_bridge_base.cpp#L101-L104)）
- **接收侧 `0x1F`**：仅刷新 `USB::last_recv_ms()` 活性，不发布话题
- **看门狗**（每秒一次，[serial_bridge_base.cpp:151-170](serial_bridge/src/serial_bridge_base.cpp#L151-L170)）：
  - 距上次收到合法帧超过 `link_timeout_ms`（默认 2000ms）→ 日志告警 + `/serial_online=false` + 强制重连（USB 内部 500ms 限频）
  - 正常 → `/serial_online=true`

### 5.7 参数

完整参数见 [config/params.yaml](serial_bridge/config/params.yaml)。关键项：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `serial_port` | `/dev/ttyACM0` | 串口设备路径 |
| `baudrate` | 115200 | 波特率 |
| `control_hz` | 50 | merge 发送频率 |
| `cmd_timeout_ms` | 500 | 命令源活性阈值 |
| `link_timeout_ms` | 2000 | 链接活性阈值 |
| `heartbeat_hz` | 2.0 | 心跳频率 |
| `fire_speed` / `bullet_type` | 0.0 / 1 | 0x03 发射参数 |
| `topic_*` | 见 params.yaml | 各话题名，可改名 |

---

## 六、USB 类 API 参考

### 6.1 构造与析构

```cpp
USB usb("/dev/ttyACM0");            // 打开串口并初始化（默认波特率 115200）
USB usb2("/dev/ttyACM0", 460800);   // 指定波特率
// 析构时自动 close(fd)
```

构造函数自动调用 `serial_init()` 配置 8N1、非阻塞读取。默认波特率 115200，可通过第二个参数指定（支持 9600/19200/38400/57600/115200/230400/460800/921600，未匹配时回退 115200）。USB CDC 虚拟串口通常忽略波特率，但保持配置一致。

### 6.2 发送方法

所有发送方法内部调用 `seasky_send(msg_id, payload, len)` 完成组帧 → CRC 计算 → 写入串口。

**返回值**：所有发送方法统一返回 `int`——`0` = 帧已完整写入串口；`-1` = 未写入（未连接 / 负载超长 / 写入失败断开）。调用方可据此判断指令是否真正发出。

#### `send_chassis_control(vx, vy, vw, spin=false)`

```cpp
int send_chassis_control(float vx, float vy, float vw, bool spin = false);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| vx | float | 前进速度 m/s |
| vy | float | 横移速度 m/s |
| vw | float | 旋转角速度 rad/s |
| spin | bool | 小陀螺开关：true=开启（置 flags bit1=0x02），false=关闭 |

发送 msg_id=`0x01`，Payload=13 字节。

#### `send_gimbal_control(yaw, pitch, flags=0)`

```cpp
int send_gimbal_control(float yaw, float pitch, uint8_t flags = 0);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| yaw | float | 云台偏航角（度） |
| pitch | float | 云台俯仰角（度） |
| flags | uint8_t | 标志位：bit0=锁敌，bit1=小陀螺，其余位保留置 0（与 0x01/0x05 共用同一定义） |

发送 msg_id=`0x02`，Payload=9 字节。

#### `send_shoot_control(fire_mode, fire_speed, bullet_type)`

```cpp
int send_shoot_control(uint8_t fire_mode, float fire_speed, uint8_t bullet_type);
```

发送 msg_id=`0x03`，Payload=6 字节。

#### `send_fix_control(yaw, pitch, fire, vx, vy, vz, flags=0, spin=false)`

```cpp
int send_fix_control(float yaw, float pitch, uint8_t fire,
                     float vx, float vy, float vz, uint8_t flags = 0, bool spin = false);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| yaw | float | 云台偏航角（度） |
| pitch | float | 云台俯仰角（度） |
| fire | uint8_t | 是否开火（0/1） |
| vx | float | 前进速度（m/s） |
| vy | float | 横移速度（m/s） |
| vz | float | 底盘旋转角速度（rad/s） |
| flags | uint8_t | 标志位：bit0=锁敌（1=锁敌，0=未锁敌），其余位保留置 0 |
| spin | bool | 小陀螺开关：true=开启（置 flags bit1=0x02），false=关闭 |

综合发送，一个调用同时控制云台+底盘+发射。发送 msg_id=`0x05`，Payload=22 字节。**这是最常用的发送接口。** flags 默认 0（未锁敌），自瞄锁定时传 1；小陀螺开启时传 spin=true。三个控制接口共用同一 flags 字节定义、位可组合——如 `send_fix_control(..., FLAG_LOCK, true)` 得到 flags=0x03（锁敌+小陀螺）。

#### `send_heartbeat()`

```cpp
int send_heartbeat();
```

发送空负载心跳包，msg_id=`0x0F`。用于检测电控是否在线。

### 6.3 接收方法

所有接收方法内部调用 `recv_frame(expected_id, payload, &len)`，传入期望的 msg_id。不匹配的帧不消费、跳过继续搜索，留给对应的 `recv_*` 处理。非阻塞，没收到帧时返回负值。

返回值约定：

| 返回值 | 含义 |
|--------|------|
| 0 | 成功收到一帧 |
| -1 | 暂无匹配的完整帧（需下次再调） |
| -3 | Payload 长度不足 |

#### `recv_chassis_feedback(speed, pos, cur)`

```cpp
int recv_chassis_feedback(int16_t *speed, int32_t *pos, int16_t *cur);
```

接收 msg_id=`0x10`。4 个电机的反馈数据，调用者需提供长度为 4 的数组：

```cpp
int16_t speed[4];
int32_t pos[4];
int16_t cur[4];
if (usb.recv_chassis_feedback(speed, pos, cur) == 0) {
    // speed[0..3], pos[0..3], cur[0..3] 已填充
}
```

#### `recv_gimbal_feedback(speed, pos, cur)`

```cpp
int recv_gimbal_feedback(int16_t *speed, int32_t *pos, int16_t *cur);
```

接收 msg_id=`0x11`。2 个云台电机（yaw=索引 0, pitch=索引 1），调用者需提供长度为 2 的数组。

#### `recv_imu_data(...)`

```cpp
int recv_imu_data(float *yaw, float *pitch, float *roll,
                  float *gyro_x, float *gyro_y, float *gyro_z,
                  float *small_yaw, float *small_pitch,
                  float *chassis_yaw, float *chassis_pitch, float *chassis_roll,
                  float *chassis_gyro_x, float *chassis_gyro_y, float *chassis_gyro_z);
```

接收 msg_id=`0x12`。14 个参数全部是输出指针，调用者传入单个变量地址；后 8 个依次是**小云台 yaw/pitch** 和**底盘 IMU 姿态/角速度**（电控仍发旧帧时按 0 返回）：

```cpp
float yaw, pitch, roll, gx, gy, gz, sy, sp;
float cy, cp, cr, cgx, cgy, cgz;
if (usb.recv_imu_data(&yaw, &pitch, &roll, &gx, &gy, &gz, &sy, &sp,
                      &cy, &cp, &cr, &cgx, &cgy, &cgz) == 0) {
    printf("yaw=%.1f° small_yaw=%.1f° chassis_yaw=%.1f°\n", yaw, sy, cy);
}
```

#### `recv_battery_info(voltage, current, capacity)`

```cpp
int recv_battery_info(uint16_t *voltage, uint16_t *current, uint8_t *capacity);
```

接收 msg_id=`0x13`。

#### `recv_robot_status(mode, hp, error)`

```cpp
int recv_robot_status(uint8_t *mode, uint16_t *hp, uint8_t *error);
```

接收 msg_id=`0x14`。

#### `recv_heartbeat_response()`

```cpp
int recv_heartbeat_response();
```

接收 msg_id=`0x1F`。返回 0 表示电控在线。

### 6.4 连接管理

#### `is_connected()`

```cpp
bool is_connected();
```

检查串口是否打开。`fd >= 0` 即为已连接。

#### `reconnect()`

```cpp
int reconnect();
```

尝试重新打开串口。**调用时机**：在主循环中周期性调用（建议每 5~10ms 一次），检测到断开后自动尝试恢复。

| 返回值 | 含义 |
|--------|------|
| 0 | 已连接（本就正常，或本次重连成功） |
| -1 | 距上次尝试不足 `RECONNECT_INTERVAL_MS`（500ms），跳过 |
| -2 | 尝试了但打开失败（设备还没就绪，下次再试） |

**设计要点**：

- **频率限制**：内置 500ms 最小重试间隔，避免疯狂 `open()` 占用 CPU
- **自动检测**：`seasky_send()` 检测到 `write()` 失败、`recv_frame()` 检测到 `read()` 硬错误时，会通过 `mark_disconnected(dead_fd)` **同时持有收发锁**安全 `close(fd)` 并置 `fd=-1`（避免 send/recv 两线程并发关闭同一 fd；只关闭仍指向同一设备的 fd，不误关重连后的新 fd），下次 `reconnect()` 就会尝试恢复
- **双锁安全**：`reconnect()` 同时持有 `send_mtx_` + `recv_mtx_`（用 `std::lock` 避免死锁），安全替换 `fd`
- **清缓冲**：重连成功后 `rx_len=0`，丢弃断开期间残留的半帧数据

#### `last_recv_ms()`

```cpp
uint32_t last_recv_ms();
```

返回距上次**成功收到一帧**（CRC 校验通过，任意 msg_id 都算）的毫秒数，用于电控活性监控。未收到过任何帧时返回自启动以来的时长（大值 = 失联）。

```cpp
while (true) {
    usb.recv_chassis_feedback(speed, pos, cur);   // 正常轮询，内部会刷新活性
    if (usb.last_recv_ms() > 1000) {
        printf("[WARN] 电控超过 1s 未上报任何数据\n");
    }
    usleep(5000);
}
```

> **注意**：活性由收到**任意合法帧**刷新，不要求是调用方期望的 msg_id。只要电控在持续发任何帧，即使当前没轮询到那类帧，活性也不会误报。

---

## 七、典型使用示例

### 7.1 发送底盘控制

```cpp
#include "usb.h"

int main() {
    USB usb("/dev/ttyACM0");

    // 前进 0.5 m/s，持续 3 秒，50Hz
    for (int i = 0; i < 150; i++) {
        usb.send_chassis_control(0.5f, 0.0f, 0.0f);
        usleep(20000);  // 20ms
    }

    // 停止
    usb.send_chassis_control(0.0f, 0.0f, 0.0f);
    return 0;
}
```

### 7.2 综合控制（最常用）

```cpp
USB usb("/dev/ttyACM0");

// 云台看前方+前进 0.3 m/s+不开火+未锁敌
usb.send_fix_control(0.0f,    // yaw   = 0°
                     0.0f,    // pitch = 0°
                     0,       // 不开火
                     0.3f,    // vx = 0.3 m/s
                     0.0f,    // vy = 0
                     0.0f);   // vz = 0

// 自瞄锁定目标：锁定标志位置1
usb.send_fix_control(aim_yaw, aim_pitch, fire, vx, vy, 0.0f, 1);
```

### 7.3 读取 IMU 数据（轮询方式）

```cpp
USB usb("/dev/ttyACM0");

while (true) {
    float yaw, pitch, roll, gx, gy, gz, sy, sp;
    int ret = usb.recv_imu_data(&yaw, &pitch, &roll, &gx, &gy, &gz, &sy, &sp);
    if (ret == 0) {
        printf("yaw=%.1f pitch=%.1f roll=%.1f small_yaw=%.1f small_pitch=%.1f\n",
               yaw, pitch, roll, sy, sp);
    }
    usleep(5000);  // 5ms，200Hz 轮询
}
```

### 7.4 同时读取多种反馈

```cpp
USB usb("/dev/ttyACM0");

int16_t chassis_speed[4], gimbal_speed[2];
int32_t chassis_pos[4], gimbal_pos[2];
int16_t chassis_cur[4], gimbal_cur[2];
float yaw, pitch, roll, gx, gy, gz;

while (true) {
    // 非阻塞轮询所有类型的帧
    usb.recv_chassis_feedback(chassis_speed, chassis_pos, chassis_cur);
    usb.recv_gimbal_feedback(gimbal_speed, gimbal_pos, gimbal_cur);
    usb.recv_imu_data(&yaw, &pitch, &roll, &gx, &gy, &gz);

    // 每个 recv_* 内部调 rx_read() 从串口取数，
    // 收到对应 msg_id 就解包返回 0，否则返回负值继续下一个

    usleep(5000);
}
```

> **注意**：每个 `recv_*` 调用内部都会触发 `rx_read()`。不匹配 expected_id 的帧不会被消费，留在缓冲区等待对应的 `recv_*` 来取。因此调用顺序不影响接收结果——无论缓冲区里帧怎么排列，每个 `recv_*` 都能精准找到自己的帧。

### 7.5 ROS 2 集成示例（直接使用 USB 类）

参考 [ros.cpp](ros.cpp)，`USB` 对象作为节点成员变量，订阅 `/turtle1/cmd_vel`（geometry_msgs/Twist）转换为底盘控制：

```cpp
class TurtleController : public rclcpp::Node {
public:
    TurtleController() : Node("turtle_controller"), usb_("/dev/ttyACM0") {...}
private:
    void pose_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        float vx = msg->linear.x  * 0.2f;
        float vy = msg->linear.y  * 0.2f;
        float wz = msg->angular.z * 0.2f;
        usb_.send_fix_control(0.0f, 0.0f, 0, vx, vy, wz);
    }
    USB usb_;   // 构造时打开串口，析构时自动关闭
    ...
};
```

### 7.6 带断线重连的主循环（推荐）

```cpp
#include "usb.h"
#include <thread>

USB usb("/dev/ttyACM0");

// 发送线程：视觉处理 + 控制指令
void vision_thread_func() {
    while (true) {
        // 断开时跳过发送，等重连
        if (!usb.is_connected()) {
            usleep(10000);
            continue;
        }
        // ... 图像处理、目标检测 ...
        usb.send_fix_control(aim_yaw, aim_pitch, fire, vx, vy, 0);
        usleep(10000);  // 100Hz
    }
}

// 接收线程：读取电控反馈 + 断线检测
void comm_thread_func() {
    while (true) {
        // 断线了就尝试重连
        if (!usb.is_connected()) {
            int ret = usb.reconnect();
            if (ret == 0) {
                printf("[Main] 重连成功，恢复通信\n");
            }
            usleep(10000);              // 断开时慢速轮询
            continue;
        }

        // 正常轮询所有反馈
        float yaw, pitch, roll, gx, gy, gz;
        usb.recv_imu_data(&yaw, &pitch, &roll, &gx, &gy, &gz);
        usb.recv_chassis_feedback(chassis_speed, chassis_pos, chassis_cur);
        usb.recv_battery_info(&voltage, &current, &capacity);

        usleep(5000);                   // 200Hz 轮询
    }
}

int main() {
    std::thread vt(vision_thread_func);
    std::thread ct(comm_thread_func);
    vt.join();
    ct.join();
    return 0;
}
```

> **工作原理**：断开后 `seasky_send()` / `rx_read()` 自动把 `fd` 置为 -1 → `is_connected()` 返回 false → 主循环调用 `reconnect()` 尝试恢复 → 成功后清零 `rx_len`，通信恢复正常。整个过程对上层透明，发送侧自动跳过断开期间的无用 `write()`。

---

## 八、注意事项

| 事项 | 说明 |
|------|------|
| **非阻塞** | 读取是非阻塞的（VMIN=0, VTIME=0），没数据立即返回，不会卡住主循环 |
| **单帧消费** | `recv_frame()` 每次最多消费一帧，缓冲区有多个帧时需要反复调用 |
| **msg_id 过滤** | `recv_frame(expected_id, ...)` 传入期望的 msg_id，不匹配的帧**不消费、跳过继续搜**，不会抢别人的帧 |
| **调用顺序无关** | 每个 `recv_*` 只取自己类型的帧，调用顺序与缓冲区中帧排列顺序无关 |
| **字节序** | 所有多字节字段均为**小端序**（LSB 在前） |
| **浮点类型** | 所有浮点参数统一用 `float`（32 位 IEEE 754），不用 `double` |
| **断线重连** | `seasky_send()` 检测 `write()` 失败自动 `close(fd)`，`rx_read()` 检测 `read()` 异常自动 `close(fd)`。主循环调用 `reconnect()` 恢复通信，内置 500ms 重试间隔 |
| **波特率** | 默认 115200，构造时可通过第二个参数指定（见 6.1） |
| **活性监控** | `last_recv_ms()` 返回距上次收帧毫秒数，超过阈值未收到即可判定电控失联（见 6.4） |
| **返回值检查** | 接收侧必须检查返回值判断是否拿到有效帧（0=成功, -1=无帧, -3=长度不足） |
| **线程安全** | `seasky_send()` 和 `recv_frame()` 各自有独立锁（`send_mtx_` / `recv_mtx_`），发送和接收互不阻塞，多线程可安全并发调用；断开时错误路径经 `mark_disconnected()` **双锁**关闭 fd，不会并发 close 同一 fd |
| **编码器溢出** | `int32_t` 编码器位置全速连续转约 9 小时才会溢出，比赛场景无需担心 |
| **软件流控（IXON）** | `serial_init()` 必须清除 `IXON\|IXOFF\|IXANY`。`0x11`(XON) / `0x13`(XOFF) 恰好是 ASCII 软件流控字符，若 tty 输入流控未关，这两个字节会被内核当流控符吞掉——**表现为 0x11 云台反馈、0x13 电池反馈帧头丢失、永远收不到，而 0x10/0x12 非流控字符则正常**。这是"IMU 正常、云台/电池收不到"的典型根因。代码见 [serial_bridge/tools/usb.cpp:158](serial_bridge/tools/usb.cpp#L158) |

### 多线程使用

`seasky_send()` 和 `recv_frame()` 各自使用独立的 `std::mutex`（`send_mtx_` / `recv_mtx_`），发送和接收互不阻塞。所有公共方法最终都调用这两个之一，因此多线程可安全使用：

```cpp
USB usb("/dev/ttyACM0");

// 线程A：图像处理 + 发送指令
std::thread vision_thread([&]() {
    while (true) {
        auto target = detect_armor(frame);
        usb.send_fix_control(aim_yaw, aim_pitch, fire, vx, vy, 0);  // 自动加锁
    }
});

// 线程B：持续接收电控反馈
std::thread comm_thread([&]() {
    while (true) {
        usb.recv_imu_data(&yaw, &pitch, &roll, &gx, &gy, &gz);     // 自动加锁
        usb.recv_chassis_feedback(speed, pos, cur);
        usb.recv_battery_info(&voltage, &current, &capacity);
    }
});

vision_thread.join();
comm_thread.join();
```

两个线程可以**同时**分别调用发送和接收方法——`send_mtx_` 和 `recv_mtx_` 是两把独立的锁，互不阻塞。同时有多个发送（或多个接收）调用时，同类之间排队。
