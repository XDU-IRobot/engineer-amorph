# 2026 赛季工程 ｜ CODENAME `amorph`

## CI status

[![build_host](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml)
[![build_custom_ctrller](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml)
[![build_arm_ctrller](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_arm_ctrller.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_arm_ctrller.yaml)

## 操作指南

> [!CAUTION]  
> **如果疯车，立刻把遥控器正面的全部拨杆（`SA`、`SB`、`SC`、`SD`）拨到“下”位置，如果无效，想办法断电**

### 遥控器

遥控器拨杆主要作为具有最高优先级的调试开关使用：

> [!NOTE]  
> 务必保证车上电时 ABCD 四个拨杆全部在”下“位置，传感器数据在上电后一段时间内可能不稳定，上电之后，等 1 秒钟再把拨杆拨到“上”位置启动。

|     | 上/外                                                                                           | 中                     | 下/内              |
| --- | ----------------------------------------------------------------------------------------------- | ---------------------- | ------------------ |
| SA  | 杆：有力                                                                                        | 杆：调试模式           | 杆：无力           |
| SB  | 左机械臂：有力                                                                                  | 左机械臂：调试模式     | 左机械臂：无力     |
| SC  | 右机械臂：有力                                                                                  | 右机械臂：调试模式     | 右机械臂：无力     |
| SD  | 底盘：有力，纯速度环                                                                            | 底盘：有力，imu 位置环 | 底盘：无力         |
| SF  | 杆（调试模式）：升                                                                              | /                      | 杆（调试模式）：降 |
| SE  | /                                                                                               | / /                    |
| SG  | /                                                                                               | /                      | /                  |
| SH  | （机械臂调试模式）切换关节，按一下切换一次，对应机械臂上的蜂鸣器会响 N 声，表示当前选中了关节 N | /                      | /                  |

模拟通道：

| axis     | 说明                                             |
| -------- | ------------------------------------------------ |
| LD       | /                                                |
| RD       | /                                                |
| 左杆     | 控制底盘前后左右移动                             |
| 右杆左右 | 控制底盘旋转                                     |
| 右杆上下 | （机械臂调试模式）控制选中关节的角度             |
| LS       | （机械臂调试模式）控制左臂夹爪开关，向下关向上开 |
| RS       | （机械臂调试模式）控制右臂夹爪开关，向下关向上开 |

### 键鼠

比赛时把遥控器正面 ABCD 四个拨杆全部拨到上，然后就不用管遥控器了，用键鼠和自定义控制器控车：

| 键位     | 说明             | 备注 |
| -------- | ---------------- | ---- |
| WASD     | 前后左右移动     |      |
| 鼠标     | 控制视角旋转     |      |
| 鼠标滚轮 | /                |      |
| QE       | 控制视角左右旋转 |      |
| G        | /                |      |
| F        | 杆升             |      |
| C        | 杆降             |      |
| V        | /                |      |
| B        | /                |      |
| Ctrl     | 刷新客户端 UI    |      |

组合键用于控制一些复杂行为，所有组合键遵循“左臂 Ctrl，右臂 Shift”的原则：

| 键位             | 说明                                                                                                                                                                          |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Ctrl + R         | 左机械臂收回                                                                                                                                                                  |
| Shift + R        | 右机械臂收回                                                                                                                                                                  |
| Ctrl + Shift + R | 两个机械臂收回                                                                                                                                                                |
| Ctrl/Shift + F   | 执行规划成功的动作                                                                                                                                                            |
| Ctrl/Shift + X   | 规划用某条机械臂取能量单元                                                                                                                                                    |
|                  | 准星对准要取的能量单元，UI 上出现一个绿框框住这个能量单元时表示识别到并且已选中，按 Ctrl + Z 运行定位和规划算法，若 UI 显示“PLANNING SUCCESS”，按 Ctrl/Shift + F 执行取矿动作 |
| Shift + V        | /                                                                                                                                                                             |
| Ctrl + V         | /                                                                                                                                                                             |
| Ctrl + X         | /                                                                                                                                                                             |
| Shift + X        | /                                                                                                                                                                             |
| Ctrl/Shift + B   | 打断正在执行的动作                                                                                                                                                            |
|                  | 机械臂正在执行动作时，UI 上会在准星左侧或右侧显示“EXEC”，此时可以使用这个组合键来强制打断动作                                                                                 |

### 自定义控制器

自定义控制器上电时重力补偿默认关闭，按主控 C 板上的"KEY"键一次，开启重力补偿，再按一次关闭重力补偿。

自定义控制器的优先级高于上述组合键控制的“复杂行为”，自定义控制器启用时上述组合键无效。

使用两档开关 A 切换自定义控制器控制的机械臂，UI 上会通过准星周围的左半圆和右半圆高亮来显示当前控制的是左机械臂还是右机械臂。

按住末端的微动开关 A，自定义控制器启用，此时可以操作机械臂运动。

微动开关 A 未按下的情况下，按住微动开关 B，自定义控制器会产生朝向机械臂目前位置的力反馈，此时可以借助力反馈把自定义控制器移动到机械臂的实际位置附近。

两档开关 B 控制夹爪开合。

## 环境配置（Jetson 系统内）

下面是在 Jetpack 5.0.2 (Ubuntu 20.04) 的 Xavier NX 上进行的环境配置步骤

### ROS2

安装 rolling 版本

```bash
wget http://fishros.com/install -O fishros && . fishros
```

### ultralytics

建一个虚拟环境，然后安装 pytorch 和 ultralytics

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
# ...安装jetson对应的pytorch版本
pip install ultralytics
```

## 环境配置（Docker 容器内）

Moveit Jazzy 的开发完成度比较高，moveit 这个项目能用新的就不要用旧的，但是 xavier nx 的系统是 20，升级到 24 基本是不可能的事情，即使有可能也非常麻烦，所以只能跑一个 ROS2 Jazzy 的容器来运行 moveit

### ROS2 + moveit

moveit/moveit2 这个镜像没有 jazzy tag，所以拉一个 ROS 的镜像，然后在里面配置环境

拉镜像，然后运行容器：

```bash
sudo docker pull ros:jazzy
sudo docker run --privileged --net=host --name moveit_test -it ros:jazzy
```

给容器里的系统换源，换完之后

```bash
sudo apt update
sudo apt install -y ros-jazzy-moveit ros-jazzy-xacro ros-jazzy-backward-ros ros-jazzy-trac-ik ros-jazzy-generate-parameter-library ros-jazzy-librealsense2 ros-jazzy-realsense2* ros-jazzy-py-binding-tools ros-jazzy-ros2-control ros-jazzy-moveit-ros-perception ros-jazzy-apriltag-ros
unminimize
```
