# amorph_chassis_controller — AI Agent Guide

全向三舵轮底盘 + 二轴自稳云台 + 双六轴机械臂的 STM32 主控固件。

- **MCU**: STM32H723XG (Cortex-M7, 280MHz)
- **构建**: CMake + Ninja, GCC ARM / starm-clang
- **语言标准**: C++20（无 RTTI、无异常、无线程安全静态变量）
- **执行模型**: 裸机，定时器中断驱动任务调度（无 RTOS）
- **依赖库**: [librm](libs/librm/)（跨平台机器人中间件，详见其 [AGENTS.md](libs/librm/AGENTS.md)）

---

## 快速定位

| 需要了解的内容                              | 文件                                                                                         |
| ------------------------------------------- | -------------------------------------------------------------------------------------------- |
| 入口 & 任务调度                              | [app/src/main.cc](app/src/main.cc)                                                           |
| 全局共享资源（单例，所有外设 & 设备）        | [app/include/shared_resources.hpp](app/include/shared_resources.hpp)                         |
| 全局对象声明（底盘/云台/机械臂指针）         | [app/include/global_objects.hpp](app/include/global_objects.hpp)                             |
| 全局编译期常量（裁判系统协议版本等）         | [app/include/global_config.hpp](app/include/global_config.hpp)                               |
| 底盘子系统 — 三舵轮运动学 + PID              | [app/include/chassis/tri_steer_chassis_controller.hpp](app/include/chassis/tri_steer_chassis_controller.hpp) |
| 底盘子系统 — 状态机                          | [app/include/chassis/fsm.hpp](app/include/chassis/fsm.hpp)                                   |
| 底盘子系统 — 参数配置                        | [app/include/chassis/config.hpp](app/include/chassis/config.hpp)                             |
| 机械臂子系统 — 6-DOF 轨迹控制器              | [app/include/arm/arm_controller.hpp](app/include/arm/arm_controller.hpp)                     |
| 机械臂子系统 — 正/逆运动学                  | [app/include/arm/kinematics.hpp](app/include/arm/kinematics.hpp)                             |
| 机械臂子系统 — 9 状态 FSM                    | [app/include/arm/fsm.hpp](app/include/arm/fsm.hpp)                                           |
| 机械臂子系统 — 参数配置                      | [app/include/arm/config.hpp](app/include/arm/config.hpp)                                     |
| 机械臂 — 夹爪控制器                          | [app/include/arm/gripper_controller.hpp](app/include/arm/gripper_controller.hpp)             |
| 云台子系统 — 5 状态 FSM                      | [app/include/gimbal/fsm.hpp](app/include/gimbal/fsm.hpp)                                     |
| 定时器任务调度器（多频任务分发 + 性能统计）  | [app/include/bsp/timer_task_scheduler.hpp](app/include/bsp/timer_task_scheduler.hpp)         |
| 上位机通信协议（CAN FD，≤64B 压缩帧）       | [app/include/host_comm.hpp](app/include/host_comm.hpp)                                       |
| 遥控器驱动（Wfly ET16s SBUS）               | [app/include/et16s.hpp](app/include/et16s.hpp)                                               |
| CubeMX HAL 层（外设句柄定义）               | [Core/Inc/main.h](Core/Inc/main.h)                                                           |
| 链接脚本（含 .sram4 自定义段）               | [STM32H723XG_FLASH.ld](STM32H723XG_FLASH.ld)                                                 |
| librm 快速参考                               | [libs/librm/AGENTS.md](libs/librm/AGENTS.md)                                                 |

---

## 硬件拓扑

```
              ┌──────────────────────────────────────────────────────┐
              │                 STM32H723XG                          │
              │                                                      │
  ┌───────────┤ FDCAN1 (left_can)    ──── 左路 CAN 总线              │
  │           │ FDCAN2 (right_can)   ──── 右路 CAN 总线（机械臂电机）│
  │           │ FDCAN3 (comm_can)    ──── 通信 CAN                   │
  │  CAN      │ SPI1 → MCP2515       ──── 外部 CAN FD（底盘+云台电机）│
  │  总线     │                                                      │
  ├───────────┤ UART1 (servo_serial) ──── 舵机 485 半双工            │
  │           │ UART2 (motor_485)    ──── 电机 485 通信              │
  │  串口     │ UART5 (sbus_serial)  ──── Wfly ET16s 遥控接收机      │
  │           │ UART7 + UART10       ──── 图传链路（两路互备冗余）    │
  │           │ USB CDC              ──── 调试日志输出                │
  ├───────────┤ SPI2                 ──── BMI088 板载 IMU            │
  │  传感器   │ ADC1                 ──── 24V 总线电压检测            │
  ├───────────┤ SPI6                 ──── WS2812 LED                 │
  │  执行器   │ TIM12 CH2            ──── 蜂鸣器 PWM                 │
  ├───────────┤ TIM13                ──── 任务调度器时基              │
  │  定时器   │                                                      │
  └───────────┴──────────────────────────────────────────────────────┘
```

**机器人物理参数**：
- 底盘：3 个转向舵 + 3 个行走轮，三角布局（base_len=0.39m, height=0.4m），全向移动
- 右臂：6-DOF 关节（J1 腰回转 → J2/J3 肩俯仰 → J4 肘回转 → J5 腕俯仰 → J6 腕回转）+ 独立夹爪，l1≈0.41m, l2≈0.49m
- 云台：Yaw 连续回转（GM6020）+ Pitch 舵机离散位姿（HiWonder），AHRS 自稳

---

## 架构与设计模式

### 1. 执行模型：定时器驱动的多频任务调度

**无 RTOS，纯中断驱动。** TIM13 硬件定时器产生周期性中断，`TimerTaskScheduler` 按频率分发任务：

| 任务               | 频率   | 职责                                                       |
| ------------------ | ------ | ---------------------------------------------------------- |
| `AhrsLoop1khz`     | 1000Hz | IMU 读取 + Mahony AHRS 姿态解算                            |
| `ChassisLoop500hz` | 500Hz  | 底盘电机反馈读取 → FSM 状态机 → 运动学 + PID → CAN 输出   |
| `RightArmLoop300hz`| 300Hz  | 机械臂轨迹规划 + 动力学补偿 + 关节 PD → CAN 输出           |
| `GimbalLoop250hz`  | 250Hz  | 云台 FSM → IMU/编码器 PID → CAN/串口输出                   |
| `FastIoLoop125hz`  | 125Hz  | 遥控器更新、设备管理器更新、总线电压                       |
| `SlowIoLoop50hz`   | 50Hz   | LED 图案控制、蜂鸣器旋律                                   |
| `LoggingLoop1hz`   | 1Hz    | 所有设备在线状态通过 USB CDC 输出                          |

**主循环**（`for(;;)`）：持续处理 4 路 CAN 的定频发送 + MCP2515 Bus-Off 恢复。

**中断安全**：TIM13 和串口 DMA 中断优先级必须保持一致，防止同时操作 FSM 状态机。MCP2515 的 EXTI 中断在 SPI 写操作期间临时关闭。

### 2. 单例全局共享资源

`SharedResources::GetInstance()` 懒加载单例，一次 `new`（后续不再动态分配）。持有所有外设对象（CAN、串口、IMU、AHRS、遥控器、LED、蜂鸣器、设备管理器）。

**关键约定**：需要被 DMA 访问的对象（串口、ADC）放入 `SharedResourcesNoDtcm`，置于 `.sram4` 段（RAM_D1），因为 DTCM 不能被 DMA 访问。

**通过 `SharedResources` 访问一切**：任何子系统都能通过 `SharedResources::GetInstance()` 获取遥控器数据、IMU 姿态、总线电压等。不要在子系统中创建独立的全局变量。

### 3. 子系统封装模式

每个子系统（底盘/机械臂/云台）遵循相同的三层结构：

```
Subsystem (e.g., Arm)
  ├── Config    (ArmConfig::MakeRight()) — 编译期+运行时可配置参数
  ├── Resources (ArmResources)           — 聚合电机、控制器、设备管理器
  ├── FSM       (fsm_arm::Arm + 9 state structs) — ETL 状态机
  └── Top-level (Arm struct)             — 持有以上三者，提供 Init() / Tick()
```

**修改子系统时的标准流程**：
1. 在 `config.hpp` 中添加/修改参数
2. 在 `resources.hpp` 中注册新设备或控制器
3. 在 `fsm.hpp` 中添加新状态/事件/转换
4. 在顶层 `.hpp` 的 `Init()` 注册遥控器开关回调、`Tick()` 中调用 FSM

### 4. FSM 状态机模式（ETL based）

所有子系统使用 ETL（Embedded Template Library）的有限状态机框架：

- **状态定义**：每个状态是一个 `struct`，包含 `on_enter()` / `on_exit()` / `handle_event()` 方法
- **事件**：以 `struct` 表示（如 `event::ControlLoop{...}`, `event::ForceModeSwitch{...}`）
- **消息路由**：`fsm.receive(event)` 驱动状态转换
- **状态列表**：必须在 FSM 所在 struct 中保持 `etl::ifsm_state *state_list_[]` 与 FSM 同生命周期

**添加新状态**：
1. 在 `fsm.hpp` 中定义状态 struct（继承所需事件处理的模板）
2. 在顶层 struct 中声明状态实例
3. 将新状态指针加入 `state_list_[]`
4. 在 `Init()` 中注册触发转换的回调（通常是遥控器开关）

### 5. 设备管理器 & 在线状态

`rm::device::DeviceManager<N>` 集中追踪所有设备（电机、传感器、遥控器）的在线/离线状态。每个设备通过 `SetName()` 注册，通过 `device_manager.Update()` 定期检查心跳。

**设备在线状态变化回调**：在 `SharedResources::Init()` 和各子系统的 `Init()` 中注册，用于自动切换模式（如遥控器离线 → 自动进入无力状态）。

### 6. PID 控制 & 轨迹限制器

所有控制算法来自 librm：
- `rm::modules::PID` — 位置/速度 PID，支持积分分离和输出限幅
- `rm::modules::TrajectoryLimiter` — 指数斜坡加速度限制
- `rm::modules::MotorPowerModel` — 底盘功率预算分配（限 120W 总功率）
- `rm::modules::MotorCalibrator` — 电机零点偏移 + 方向校准
- `rm::modules::MahonyAhrs` — 姿态解算

---

## 通信协议

### 上位机 ↔ STM32（CAN FD，≤64B 帧）

通过 `host_comm.hpp` 定义：
- **`Stm32FeedbackCompressed`**：STM32 → 上位机，包含模式枚举（9 种）、Float16 压缩的 6 关节位置/速度/力矩、夹爪状态
- **`HostCommand`**：上位机 → STM32，take_control 标志 + 6 个目标关节角 + 夹爪开合命令
- 控制模式切换：上位机发送 `take_control=true` → FSM 转入 `HostControl` 状态

### 遥控器 → STM32（SBUS 协议）

Wfly ET16s 16 通道接收机，通过 `WflyET16s` 封装：
- LEFT_X/Y, RIGHT_X/Y：摇杆归一化到 `[-1.0, +1.0]`
- SA～SH：拨杆开关，通过 `SparseValueWatcher` 检测边沿变化
- 开关值范围：上(0-500)、中(500-1100)、下(1300-1700)

### 图传链路（串口 7/10）

裁判系统协议附带的 `custom_robot_data` 字段传输自定义指令：
- `CustomControllerCommand`（≤30B）：4 按钮位 + 6 关节目标位置
- `CartesianCustomCtrlCommand`（≤30B）：按钮 + 平移(x,y,z) + 旋转四元数(w,x,y,z)

---

## 关键约定 & 注意事项

### 内存与性能
1. **禁止动态内存分配** — 所有对象必须在编译期或初始化阶段一次性分配（`new` 仅用于 `SharedResources` 单例和子系统顶层对象）
2. **优先使用 ETL 而非 STL** — `etl::string`, `etl::vector`, `etl::map`, `etl::delegate` 等，避免堆分配
3. **DTCM vs SRAM4** — 需要 DMA 访问的对象必须放在 `.sram4` 段（`__attribute__((section(".sram4")))`）
4. **中断优先级一致性** — TIM13 与所有会访问 FSM 的 DMA 中断必须保持相同优先级，互斥执行

### 代码风格
5. **命名空间**：子系统 FSM 用独立命名空间（`fsm_chassis::`, `fsm_arm::`, `fsm_gimbal::`），librm 用 `rm::`，全局不污染
6. **Google C++ Style** — 约束通过 clang-format 保证
7. **角度单位** — 统一使用弧度（rad），librm 提供 `using namespace angle_literals` 支持 `10_deg` 字面量

### 调试与日志
8. **USB CDC 日志** — `CDC_Transmit_HS()` 输出 `etl::string<2048>` 格式的设备状态
9. **DWT 性能统计** — `TimerTaskScheduler` 自动用 DWT 周期计数器测量每个任务的执行时间（EWMA + 最大最小 + 超限检测）
10. **电机校准** — 所有电机启动前必须通过 `MotorCalibrator` 设定零点和方向，校准数据持久化在 `config.hpp` 中

### 安全
11. **遥控器离线 → 自动无力** — 所有子系统在 `Init()` 中注册设备状态回调
12. **MCP2515 Bus-Off 自动恢复** — 主循环检测 TXBO/TXEP 标志并重初始化
13. **图传暂停键 → MCU 复位** — `FastIoLoop125hz` 中检测并执行 `NVIC_SystemReset()`

---

## 常见任务指引

### 添加新电机
1. 在子系统的 `config.hpp` 中添加电机 ID 和 CAN 总线分配
2. 在 `resources.hpp` 中声明电机对象 + 校准器，注册到 `device_manager`
3. 在 `Tick()` / FSM 状态中添加控制和反馈读取

### 添加新的机械臂控制模式
1. 在 `arm/fsm.hpp` 中：添加新 `StateId` 枚举值、新状态 struct、新事件 struct、新状态转换
2. 在 `arm/arm.hpp` 中：声明状态实例、加入 `state_list_[]`、在 `Init()` 注册触发回调
3. 在 `host_comm.hpp` 中：在 `Mode` 枚举中添加新模式
4. 在 `arm/arm_controller.hpp` 或新建控制器文件中实现控制逻辑

### 修改底盘运动学参数
1. 修改 `chassis/config.hpp` 中的 `base_len`, `height` 和电机校准值
2. `tri_steer_chassis_controller.hpp` 中的三舵轮运动学由 librm 的 `rm::modules::TriSteeringChassis` 模板提供，配置通过 `ChassisConfig` 传入

### 调整控制频率
修改 `main.cc` 中 `sched.AddTask()` 的参数（频率 Hz），注意：
- 调整频率后可能需要同步调整 PID 增益（因为 dt 改变）
- 确保不超调（用 DWT 统计确认最大执行时间 < 周期）

### 添加新外设
1. 在 CubeMX `.ioc` 中配置并重新生成 `Core/` 代码
2. 在 `shared_resources.hpp` 中添加外设对象（DMA 相关 → `SharedResourcesNoDtcm`）
3. 在 `SharedResources::Init()` 中初始化
4. 如需要周期性更新，在对应频率的任务循环中添加

---

## 构建与烧录

```bash
# 配置（Debug）
cmake --preset Debug

# 构建
cmake --build build/Debug

# 烧录（OpenOCD + ST-Link/CMSIS-DAP）
# 配置文件位于 openocd/ 目录
```

编译产物：`build/Debug/app/app_main`（ELF，可执行文件）

`compile_commands.json` 已导出到 `build/Debug/`，可供 clangd 等 LSP 使用。
