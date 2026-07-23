# 哨兵电控 —— Basic Framework MC02

> **ZZU 中州烛龙 · 哨兵机器人嵌入式固件**

本项目基于[南昌航空大学洪鹰战队 Basic_Framework_MC02](https://gitee.com/LitzJ/basic_framework_mc02) 改造，以[湖南大学岳麓战队 basic_framework](https://gitee.com/hnuyuelurm/basic_framework) 为应用层框架进行适配。在此对上游开源项目表示感谢。

## 硬件平台

| 项目 | 型号 |
| ---- | ---- |
| **主控** | STM32H723VGTx（Cortex-M7, 480MHz） |
| **IMU** | BMI088（SPI） |
| **底盘电机** | M3508 ×4（FDCAN1） |
| **云台电机** | GM6020 ×2（FDCAN1） |
| **摩擦轮** | M3508 ×2（FDCAN2） |
| **拨弹轮** | M2006 ×1（FDCAN2） |
| **遥控器** | DJI DT7（DBUS） |
| **调试输出** | SEGGER RTT / USB CDC 虚拟串口 |

## 开发环境

| 工具 | 用途 |
| ---- | ---- |
| **STM32CubeMX** | 外设初始化代码生成 |
| **VS Code** | 代码编辑与开发 |
| **Ozone** | 调试（J-Link / DAP-Link） |
| **arm-none-eabi-gcc** | 交叉编译 |
| **MinGW-w64** | Windows 下的 make 工具 |

```bash
make                 # 编译
make download_dap    # DAP-Link 烧录
make download_jlink  # J-Link 烧录
```

## 项目结构

```text
├── Core/                  # STM32CubeMX 生成的 HAL 层初始化代码
├── bsp/                   # 板级支持包（硬件抽象层）
│   ├── adc/               # ADC 封装
│   ├── can/               # FDCAN 驱动（实例化注册模式）
│   ├── dwt/               # DWT 周期计数器（微秒级精确延时）
│   ├── gpio/              # GPIO 封装（支持 EXTI）
│   ├── iic/               # I2C 驱动
│   ├── log/               # SEGGER RTT 日志系统
│   ├── pwm/               # PWM 驱动
│   ├── spi/               # SPI 驱动（独立 CS 控制）
│   ├── usart/             # UART 驱动（阻塞/中断/DMA）
│   └── usb/               # USB CDC 虚拟串口
├── modules/               # 功能模块库
│   ├── algorithm/         # PID 控制器、EKF 姿态解算、卡尔曼滤波、CRC
│   ├── alarm/             # 蜂鸣器报警
│   ├── BMI088/            # BMI088 IMU 底层驱动
│   ├── imu/               # INS 姿态解算任务（四元数 EKF, 1kHz）
│   ├── bluetooth/         # HC05 蓝牙通信
│   ├── buffer/            # 循环缓冲队列（视觉-电控相位对齐）
│   ├── can_comm/          # 板间 CAN 通信协议
│   ├── daemon/            # 模块看门狗（离线检测）
│   ├── master_machine/    # 视觉上位机通信（Seasky 协议）
│   ├── message_center/    # 发布-订阅消息总线
│   ├── motor/             # 电机驱动框架
│   │   ├── DJImotor/      # M3508 / M2006 / GM6020
│   │   ├── DMmotor/       # DM 电机
│   │   ├── GOmotor/       # GO 电机
│   │   ├── HTmotor/       # HT04 电机
│   │   ├── LKmotor/       # LK9025 电机
│   │   ├── servo_motor/   # 舵机
│   │   └── step_motor/    # 步进电机
│   ├── oled/              # OLED 显示屏（I2C）
│   ├── power_meter/       # 功率计（CAN）
│   ├── referee/           # RM 裁判系统协议解析 + 操作手 UI
│   ├── remote/            # 遥控器解析（DBUS）
│   ├── super_cap/         # 超级电容控制
│   └── TFminiPlus/        # TFmini Plus 激光测距
├── application/           # 机器人应用层
│   ├── robot.c/h          # 机器人初始化与任务调度
│   ├── robot_def.h        # 整机参数与数据结构定义
│   ├── robot_task.h       # RTOS 任务配置
│   ├── cmd/               # 指令分发（遥控/视觉 → 底盘/云台/射击）
│   ├── chassis/           # 麦轮运动学 + 功率限制
│   ├── gimbal/            # 云台双轴控制（IMU 反馈）
│   └── shoot/             # 射击控制（摩擦轮 + 拨弹 + 弹舱盖）
├── USB_DEVICE/            # STM32 USB CDC 设备
├── Drivers/               # CMSIS + HAL 库
├── Middlewares/           # FreeRTOS / SEGGER RTT / ARM DSP
└── Makefile               # GCC Makefile 构建系统
```

## 软件架构

### 初始化流程

```text
main() → HAL_Init() → MX_外设_Init() → RobotInit()
  ├── BSPInit()          # DWT + RTT 日志
  ├── 各模块 Init()       # 遥控/裁判系统/视觉/电机/IMU...
  └── OSTaskInit()       # 创建 FreeRTOS 任务
→ osKernelStart()        # 启动调度器
```

### FreeRTOS 任务

| 任务 | 频率 | 说明 |
| ---- | ---- | ---- |
| **INS Task** | 1kHz | BMI088 数据读取 + 四元数 EKF 姿态解算 |
| **Motor Task** | 1kHz | 所有电机 PID 级联控制 |
| **Robot Task** | 100Hz | 主控制循环：指令分发 → 云台 → 射击 → 底盘 |
| **UI Task** | 200Hz | 裁判系统 UI 更新 + 数据转发 |
| **Daemon Task** | 100Hz | 模块在线检测 + 蜂鸣器报警 |

### 应用间通信

四个应用模块（`cmd`、`chassis`、`gimbal`、`shoot`）通过 **发布-订阅消息总线** 解耦通信，模块间无直接调用：

```text
遥控器/视觉 → robot_cmd（指令分发）
                ├── → Chassis_Ctrl_Cmd   → chassis（底盘控制）
                ├── → Gimbal_Ctrl_Cmd    → gimbal（云台控制）
                └── → Shoot_Ctrl_Cmd     → shoot（射击控制）

chassis  → Chassis_Upload_Data → robot_cmd
gimbal   → Gimbal_Upload_Data  → robot_cmd
shoot    → Shoot_Upload_Data   → robot_cmd
referee  → Referee_Upload_Data → robot_cmd
```

### 控制模式

| 子系统 | 模式 | 说明 |
| ---- | ---- | ---- |
| **底盘** | ZERO_FORCE / ROTATE / NO_FOLLOW / FOLLOW_GIMBAL_YAW | 零力、小陀螺、自由平移、跟随云台 |
| **云台** | ZERO_FORCE / FREE_MODE / GYRO_MODE / NAV_MODE | 零力、编码器反馈、IMU 反馈、导航模式 |
| **射击** | OFF/ON + 摩擦轮 + LOAD_STOP/REVERSE/1_BULLET/3_BULLET/BURSTFIRE | 单发、三连发、连发、反转 |

## 关键特性

- **麦轮运动学解算**：四轮全向底盘，支持跟随云台、小陀螺、自由平移模式
- **IMU 姿态融合**：1kHz 四元数 EKF，云台使用陀螺仪角速度反馈实现稳定控制
- **级联 PID 控制**：角度环 → 速度环 → 电流环，支持前馈 + 多模式改进 PID
- **视觉相位对齐**：周期 Buffer 缓存 0~20 个周期的数据，消除电控-视觉通信因缺少时间戳导致的相位差
- **超级电容功率管理**：结合裁判系统数据与电容状态，动态限制底盘输出功率
- **枪口热量控制**：根据裁判系统热量数据自动限制射速，冷却后恢复
- **模块看门狗**：各关键模块注册守护进程，离线时触发报警与保护逻辑
- **板间 CAN 通信**：支持双板分离部署（底盘板 + 云台板），可通过宏切换单板/双板模式
- **裁判系统 UI**：操作手界面实时显示机器人状态（血量、热量、弹速、功率等）
- **多种电机支持**：DJI M3508 / M2006 / GM6020、DM、GO、HT04、LK9025、舵机、步进电机

## 配置与移植

修改 [application/robot_def.h](application/robot_def.h) 中的宏定义即可适配不同机器人：

- `ONE_BOARD` / `CHASSIS_BOARD` / `GIMBAL_BOARD` — 单板/双板模式
- `WHEEL_BASE`、`TRACK_WIDTH`、`RADIUS_WHEEL` — 底盘机械参数
- `YAW_CHASSIS_ALIGN_ECD`、`PITCH_HORIZON_ECD` — 云台校准值
- `PITCH_MAX_ANGLE`、`PITCH_MIN_ANGLE` — 俯仰角限位
- `VISION_USE_UART` / `VISION_USE_VCP` — 视觉通信接口

## 传承

ZZU 哨兵 中州烛龙

| 年份 | 成员 | QQ |
| ---- | ---- | ---- |
| 2023 | zhangchi | 2193878796 |
| 2024 | wuyunhang | 1594681838 |
| 2024 | wangjiexun | 2962529135 |
| 2024 | 电控组长 yehontao | 743009492 |
| 2024 | 视觉组长 yangchenzhi | 2796554066 |
| 2024 | 哨兵视觉 heboyui | 1900420991 |

## 致谢

- [南昌航空大学洪鹰战队 Basic_Framework_MC02](https://gitee.com/LitzJ/basic_framework_mc02)
- [湖南大学岳麓战队 basic_framework](https://gitee.com/hnuyuelurm/basic_framework)
