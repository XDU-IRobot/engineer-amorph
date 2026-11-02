# Amorph robot AI agent dev guide

amorph(codename) ，是一台三舵轮构型移动底盘、双串联六轴机械臂机器人，用途是参加 robomaster 比赛，完成从补给区取能量单元，运送到中央高地的科技核心前完成能量单元装配的任务。

本仓库是amorph的开发仓库，包含4个部分：

- 用于机械臂运动学、动力学分析的离线计算代码(src/offline)

- 负责机器人基本运动控制的STM32H7固件代码(src/stm32/chassis)

- 操作手使用的操纵杆的STM32F4固件代码(src/stm32/amorph_custom_controller)

- 负责视觉辅助功能的ROS2代码，运行在一台Jetson Orin NX上(src/jetson)

除此之外其他部分大部分是已弃用的方案，遗留在仓库里备用，不要参考、不用关心。

# Guidelines

## overall

代码风格严格以Google C++ Style Guide为基础。在此之上：

- 80列限制改为120列

- 对定义在.cc源文件里的类，成员函数直接内联定义在类定义里；若有必要把类定义放在.h头文件里，成员函数的实现仍然放在.cc源文件里。

- 对类、函数、命名空间、成员等的注释采用doxygen规范，对行内的简单解释可直接//后接文本，自行灵活决定

已配置了clang-format规则，在仓库根目录下。

- 由于项目性质为个人竞赛作品，所以更看重开发效率、代码可读性而非交付结果的流程上的规范性。严格遵循最小功能原则，仅实现要求实现的功能，不要擅自添加各种过度的防御性判断、正常调试时基本不会动的可调参数，久而久之这些垃圾代码会积重难返。

- 让注释保持简洁可读。不要写大段的注释，非必要不要用等号拼分割线、不要用箭头、emoji等等非ASCII字符，在有必要说明的地方简明扼要地注一两行解释，或在行内写一句话，过多的废话严重影响代码的可读性。

- 若需要用到某些数学上很复杂的算法，首先考虑有没有第三方库有现成的实现，非必要不要手搓。

以上两条的反面例子：

```cmake
# 第三方依赖：OpenMP + TensorRT + TensorRT-YOLO + 自定义插件。
#
# 本文件对外只暴露一个接口目标 `${PROJECT_NAME}_libs`，外层直接链接它即可：
#
#     add_subdirectory(libs)
#     target_link_libraries(<target> ${PROJECT_NAME}_libs)
#
# 不使用 libs/TensorRT-YOLO 自带的顶层 CMakeLists，原因：
#   - 它强制要求 -DTRT_PATH，且拼死 ${TRT_PATH}/include 与 ${TRT_PATH}/lib，
#     不接受 Debian multiarch / JetPack 的散装布局；
#   - 带自己的 install 规则，会把 trtyolo.so 装进 ament 前缀；
#   - 会 FORCE 覆盖 CMAKE_CUDA_ARCHITECTURES。
# 因此这里直接编译它的源码。

find_package(CUDAToolkit REQUIRED)

# ------------------------------------------------------------------------------
# TensorRT 探测
#
# 支持两种安装形态，路径全部来自缓存变量或本文件，不改动系统目录：
#   1) 标准布局：-DTRT_PATH=/opt/tensorrt，其下有 include/ 与 lib/
#   2) Debian/Ubuntu multiarch 或 JetPack 布局：头在 /usr/include/<triplet>，
#      库在 /usr/lib/<triplet>（本机即 /usr/include/x86_64-linux-gnu）
# ------------------------------------------------------------------------------
set(TRT_PATH "" CACHE PATH "TensorRT 根目录（含 include/ 与 lib/）；留空则自动探测系统安装")
```

- agentic coding时，必须使用IDE或MCP Server提供的tool，以保持修改可追溯，严禁直接操作终端修改文件，若tool call反复出现问题，导致无法推进，告知问题并停止操作，不要擅自决定fallback到操作终端。

## librm

仓库包含一个团队私有的电控软件中间件 `librm`，其中实现了大量有用的通信、HAL和算法模块，可用时应首先考虑采用librm的实现，而不是自己重复造轮子。详情参见 [src/stm32/librm/AGENTS.md](src/stm32/librm/AGENTS.md)

## 第三方库

- ROS2生态内的package，声明在package.xml里，然后在CMakeLists.txt里用ament_auto处理。

- ROS2生态外的第三方库（如OpenCV、Eigen3），按一般情况在CMakeLists.txt里用find_package处理。

- 在允许的情况下，尽量用面向对象的方式继承rclcpp::Node编写节点，而非像过程式代码一样直接实例化rclcpp::Node。

- 若某个库需要用git把源码下载下来，然后用CMake `add_subdirectory`，则把这个库放到ROS2包的`libs`目录下，用git submodule管理。

# 使用的硬件

## 动力系统

底盘：等腰三角形三舵轮构型，即底盘后方两个舵轮组，前方中点一个舵轮组，从上向下看构成一个等腰三角形。舵由一个GM6020电机驱动，轮由一套C620电调+M3508电机驱动。

机械臂：双串联六轴机械臂，位于机体左右两侧，两侧完全对称。

j1: 达妙 DM-J4340P

j2: 达妙 DM-J8009P

j3: 达妙 DM-J4340P

j4/j5: 达妙 DM-J4310

j6: GM6020

夹爪为二指构型，由一套丝杆+钢丝绳传动带动，一个达妙 DM-J3507 电机驱动。

## 其他电装

一台Jetson Orin NX 8GB，用于视觉和机械臂规划任务。

一台Orbbec Gemini 336L 双目红外结构光深度相机。

一块DM-MC02开发板，STM32H723VG，三路自带FDCAN，以及一块MCP2515 SPI-FDCAN 转换板，总计四路FDCAN。板载三路分别用于控制两条机械臂以及和Jetson通信，MCP2515一路用于控制底盘。

RM赛事要求安装的裁判系统。需要关心图传模块和电源管理模块上的串口，可接收操作手传来的键盘鼠标和操纵杆指令，接到STM32的两路串口上。

整车电源由两块TB48S电池提供（24V）。

# STM32固件部分

TODO...

# Jetson ROS2部分

## How to build

不要直接用`colcon build`命令，而是使用`scripts/build.py`脚本用于构建 Jetson 上的 ROS2 代码。原因是在venv中直接使用`colcon build`可能还是会用系统全局的python，`build.py` 显式地指明了用当前venv环境下的colcon模块。

脚本比较简短，具体的实现细节可自行阅读。

## Project structure

Jetson上的代码由几个ROS2包组成：

- `amorph_bringup`: 启动各个组件的launch文件。

- `amorph_vision_cpp`: 定位、跟踪能量单元，并把结果发布到tf上。C++，TensorRT-YOLO 推理 + PCL 几何计算。现役实现。

- `amorph_vision`: 同上功能的 Python 版（ultralytics + open3d），保留作回退与算法对拍，不再维护。

- `amorph_transport`: 处理和下位机的FDCAN通信。C++。

- `amorph_master`: 组织应用层逻辑。接收操作手输入以及视觉模块的输出，操作moveit_cpp完成路径规划，再通过transport节点操作机械臂运动。C++。

- `*_urdf`: 机器人的urdf文件，目前在用的版本为v5。

- `*_moveit_config`: moveit配置文件，目前在用的版本为v5。

## 视觉模型资产

`amorph_vision_cpp` 的推理引擎与 TensorRT 版本、GPU 架构强绑定，**不进仓库**，必须在运行它的机器上生成：

1. 任意有 ultralytics 的机器：`scripts/export_eu_engine.py` → 产出 `resource/best-trtyolo.onnx`（走 LFS 提交）
2. 目标机（Jetson Orin NX，JetPack 6.x / TensorRT 10.3）：`scripts/build_eu_engine.sh` → 产出 `resource/best.engine`

Segment 模型的自定义 mask 插件必须序列化进 engine（脚本已传 `--staticPlugins` 与 `--setPluginsToSerialize`），否则反序列化会失败。
