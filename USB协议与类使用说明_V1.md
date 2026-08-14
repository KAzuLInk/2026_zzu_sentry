# USB 串口通信协议 & USB 类使用说明

---

## 一、概述

本项目通过 USB 虚拟串口（`/dev/ttyACM0`）与机器人电控（下位机）通信。通信基于**自定义二进制帧协议**，包含帧头、长度、消息 ID、负载数据、CRC-16 校验。上层通过 `USB` 类封装了发送和接收操作。

### 通信参数

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 |
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
| Payload | 4 | Len 字节 | 实际数据负载（第一个字节8位全部为标志位，目前只用到最低位，高位留作扩展先全部置0，为锁敌标志位，0表示没有锁敌，1为锁敌） |
| CRC-16 | 4+Len | 2 字节 | CRC-16/Modbus 校验值，**小端序**（低字节在前） |

**帧总长** = 1 + 2 + 1 + Len + 2 = **6 + Len 字节**

### 2.2 CRC 校验

- **算法**：CRC-16/Modbus（多项式 `0xA001`，初始值 `0xFFFF`）
- **计算范围**：Header + Len + Msg_ID + Payload（共 4 + Len 字节），**不包含 CRC 字段本身**
- **字节序**：小端序写入，低字节在前

代码位置：[usb.cpp:111-113](usb.cpp#L111-L113)（发送侧拼接）、[usb.cpp:171-174](usb.cpp#L171-L174)（接收侧提取）

### 2.3 帧边界识别（处理粘包/半包）

接收缓冲区是线性缓冲区，工作流程：

1. **`rx_read()`** 从串口非阻塞读取，追加到 `rx_buf` 尾部
2. **`recv_frame(expected_id, payload, &len)`** 在 `rx_buf` 中搜索帧头 `0xA5`：
   - 找到 `0xA5` → 读取 Len 字段 → 检查剩余数据是否够一帧 → 计算 CRC-16 → 比对
   - CRC 匹配 → 检查 msg_id 是否等于 `expected_id`
     - 匹配 → 提取 Payload，**只从缓冲区删除这一帧**（它前后的数据原样保留），返回 0
     - 不匹配 → **整帧保留并跳过**，继续往后搜索（留给其他 `recv_*` 处理）
   - CRC 不匹配 → 跳过当前 `0xA5`（假帧头），继续搜索
   - 数据不够一帧 → 返回 -1，等待下次 `rx_read()` 补充
3. 返回前清除搜索过的无效字节（防止缓冲区无限增长），但**保留**不匹配的合法帧和尾部可能的半帧

### 2.4 缓冲区溢出保护

缓冲区大小 `RX_BUF_SIZE = 1024` 字节。当剩余空间不足 256 字节时，丢弃最旧的 512 字节数据腾出空间（`memmove` 向前搬运）。适用于实时流式数据场景——宁可丢旧数据也要保证新数据能写入。

> 注意：如果某类消息电控持续上报但程序从不调用对应的 `recv_*` 去取，这些帧会留在缓冲区中越积越多，最终触发此保护被丢弃。正常轮询所有在用的消息类型即可避免。

代码位置：[usb.cpp:125-137](usb.cpp#L125-L137)

---

## 三、消息 ID 定义

### 3.1 主控 → 电控（发送侧）

| Msg_ID | 消息名 | Payload 大小 | 说明 |
|--------|--------|-------------|------|
| `0x01` | 底盘控制 | 12B | vx, vy, vw（各 float 4B） |
| `0x02` | 云台控制 | 9B | 标志位（1B），yaw, pitch（各 float 4B） |
| `0x03` | 发射控制 | 6B | fire_mode(1B) + fire_speed(float 4B) + bullet_type(1B) |
| `0x05` | 综合控制 | 22B | 标志位（1B）+ yaw + pitch + fire + vx + vy + vz（5×float + 1B） |
| `0x0F` | 心跳包 | 0B | 空负载，仅用于保活检测 |

### 3.2 电控 → 主控（接收侧）

| Msg_ID | 消息名 | Payload 大小 | 结构 |
|--------|--------|-------------|------|
| `0x10` | 底盘电机反馈 | 32B | 4 电机 × (speed int16 + pos int32 + cur int16) = 4×8B |
| `0x11` | 云台电机反馈 | 16B | 2 电机(yaw/pitch) × (speed int16 + pos int32 + cur int16) = 2×8B |
| `0x12` | IMU 数据 | 24B | yaw + pitch + roll + gyro_x + gyro_y + gyro_z（各 float 4B） |
| `0x13` | 电池信息 | 5B | voltage uint16 + current uint16 + capacity uint8 |
| `0x14` | 机器人状态 | 4B | mode uint8 + hp uint16 + error uint8 |
| `0x1F` | 心跳响应 | 0B | 空负载，收到表示电控在线 |

---

## 四、Payload 详细结构

### 4.1 底盘控制 `0x01` — 12 字节

```
┌──────────┬──────────┬──────────┐
│ vx       │ vy       │ vw       │
│ float 4B │ float 4B │ float 4B │
└──────────┴──────────┴──────────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
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
| flags | uint8_t | 标志位：bit0=锁敌（1=锁敌，0=未锁敌），bit1~7 保留置 0 |
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
| flags | uint8_t | 标志位：bit0=锁敌（1=锁敌，0=未锁敌），bit1~7 保留置 0 |
| yaw | float | 云台偏航角 |
| pitch | float | 云台俯仰角 |
| fire | uint8_t | 是否开火（0/1） |
| vx | float | 前进速度（m/s） |
| vy | float | 横移速度（m/s） |
| vz | float | 预留，填 0 |

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
| speed | int16_t | 电机转速（rpm 或原始值，取决于电控固件） |
| pos | int32_t | 编码器累计位置（脉冲数），上电从 0 开始，int32 范围 ±21 亿 |
| cur | int16_t | 电机电流（mA 或原始值） |

### 4.6 IMU 数据 `0x12` — 24 字节

```
┌───────┬────────┬───────┬────────┬────────┬────────┐
│ yaw   │ pitch  │ roll  │ gyro_x │ gyro_y │ gyro_z │
│ 4B    │ 4B     │ 4B    │ 4B     │ 4B     │ 4B     │
└───────┴────────┴───────┴────────┴────────┴────────┘
```

| 字段 | 类型 | 说明 |
|------|------|------|
| yaw | float | 偏航角（度），含磁力计修正，相对大地 |
| pitch | float | 俯仰角（度） |
| roll | float | 横滚角（度） |
| gyro_x | float | X 轴角速度（度/秒） |
| gyro_y | float | Y 轴角速度（度/秒） |
| gyro_z | float | Z 轴角速度（度/秒） |

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

## 五、USB 类 API 参考

### 5.1 构造与析构

```cpp
USB usb("/dev/ttyACM0");  // 打开串口并初始化
// 析构时自动 close(fd)
```

构造函数自动调用 `serial_init()` 配置波特率 115200、8N1、非阻塞读取。

### 5.2 发送方法

所有发送方法内部调用 `seasky_send(msg_id, payload, len)` 完成组帧 → CRC 计算 → 写入串口。

#### `send_chassis_control(vx, vy, vw)`

```cpp
void send_chassis_control(float vx, float vy, float vw);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| vx | float | 前进速度 m/s |
| vy | float | 横移速度 m/s |
| vw | float | 旋转角速度 rad/s |

发送 msg_id=`0x01`，Payload=12 字节。

#### `send_gimbal_control(yaw, pitch)`

```cpp
void send_gimbal_control(float yaw, float pitch);
```

| 参数 | 类型 | 说明 |
|------|------|------|
| yaw | float | 云台偏航角（度） |
| pitch | float | 云台俯仰角（度） |

发送 msg_id=`0x02`，Payload=8 字节。

#### `send_shoot_control(fire_mode, fire_speed, bullet_type)`

```cpp
void send_shoot_control(uint8_t fire_mode, float fire_speed, uint8_t bullet_type);
```

发送 msg_id=`0x03`，Payload=6 字节。

#### `send_fix_control(yaw, pitch, fire, vx, vy, vz)`

```cpp
void send_fix_control(float yaw, float pitch, uint8_t fire,
                      float vx, float vy, float vz);
```

综合发送，一个调用同时控制云台+底盘+发射。发送 msg_id=`0x05`，Payload=22 字节。**这是最常用的发送接口。**

#### `send_heartbeat()`

```cpp
void send_heartbeat();
```

发送空负载心跳包，msg_id=`0x0F`。用于检测电控是否在线。

### 5.3 接收方法

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

#### `recv_imu_data(yaw, pitch, roll, gyro_x, gyro_y, gyro_z)`

```cpp
int recv_imu_data(float *yaw, float *pitch, float *roll,
                  float *gyro_x, float *gyro_y, float *gyro_z);
```

接收 msg_id=`0x12`。6 个参数全部是输出指针，调用者传入单个变量地址：

```cpp
float yaw, pitch, roll, gx, gy, gz;
if (usb.recv_imu_data(&yaw, &pitch, &roll, &gx, &gy, &gz) == 0) {
    printf("yaw=%.1f°\n", yaw);
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

### 5.4 连接管理

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
- **自动检测**：`seasky_send()` 检测到 `write()` 失败、`rx_read()` 检测到 `read()` 异常时，会自动 `close(fd)` 并置 `fd=-1`，下次 `reconnect()` 就会尝试恢复
- **双锁安全**：`reconnect()` 同时持有 `send_mtx_` + `recv_mtx_`（用 `std::lock` 避免死锁），安全替换 `fd`
- **清缓冲**：重连成功后 `rx_len=0`，丢弃断开期间残留的半帧数据

---

## 六、典型使用示例

### 6.1 发送底盘控制

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

### 6.2 综合控制（最常用）

```cpp
USB usb("/dev/ttyACM0");

// 云台看前方+前进 0.3 m/s+不开火
usb.send_fix_control(0.0f,    // yaw  = 0°
                     0.0f,    // pitch = 0°
                     0,       // 不开火
                     0.3f,    // vx = 0.3 m/s
                     0.0f,    // vy = 0
                     0.0f);   // vz = 0
```

### 6.3 读取 IMU 数据（轮询方式）

```cpp
USB usb("/dev/ttyACM0");

while (true) {
    float yaw, pitch, roll, gx, gy, gz;
    int ret = usb.recv_imu_data(&yaw, &pitch, &roll, &gx, &gy, &gz);
    if (ret == 0) {
        printf("yaw=%.1f pitch=%.1f roll=%.1f\n", yaw, pitch, roll);
    }
    usleep(5000);  // 5ms，200Hz 轮询
}
```

### 6.4 同时读取多种反馈

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

### 6.5 ROS 2 集成示例

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

---

## 七、注意事项

| 事项 | 说明 |
|------|------|
| **非阻塞** | 读取是非阻塞的（VMIN=0, VTIME=0），没数据立即返回，不会卡住主循环 |
| **单帧消费** | `recv_frame()` 每次最多消费一帧，缓冲区有多个帧时需要反复调用 |
| **msg_id 过滤** | `recv_frame(expected_id, ...)` 传入期望的 msg_id，不匹配的帧**不消费、跳过继续搜**，不会抢别人的帧 |
| **调用顺序无关** | 每个 `recv_*` 只取自己类型的帧，调用顺序与缓冲区中帧排列顺序无关 |
| **字节序** | 所有多字节字段均为**小端序**（LSB 在前） |
| **浮点类型** | 所有浮点参数统一用 `float`（32 位 IEEE 754），不用 `double` |
| **断线重连** | `seasky_send()` 检测 `write()` 失败自动 `close(fd)`，`rx_read()` 检测 `read()` 异常自动 `close(fd)`。主循环调用 `reconnect()` 恢复通信，内置 500ms 重试间隔 |
| **返回值检查** | 接收侧必须检查返回值判断是否拿到有效帧（0=成功, -1=无帧, -3=长度不足） |
| **线程安全** | `seasky_send()` 和 `recv_frame()` 各自有独立锁（`send_mtx_` / `recv_mtx_`），发送和接收互不阻塞，多线程可安全并发调用 |
| **编码器溢出** | `int32_t` 编码器位置全速连续转约 9 小时才会溢出，比赛场景无需担心 |

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

### 6.6 带断线重连的主循环（推荐）

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
