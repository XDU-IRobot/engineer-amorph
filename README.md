# 2026 赛季工程 ｜ CODENAME `amorph`

## CI status

[![build_host](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml)
[![build_stm32](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_stm32.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_stm32.yaml)
[![build_custom_ctrller](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml)

## 操作指南

> [!CAUTION]  
> **如果疯车，立刻把遥控器正面的全部拨杆（`SA`、`SB`、`SC`、`SD`）拨到“下”位置，如果无效，想办法断电**

### 遥控器

遥控器拨杆主要作为具有最高优先级的调试开关使用：

> [!NOTE]  
> 务必保证车上电时 ABCD 四个拨杆全部在”下“位置，传感器数据在上电后一段时间内可能不稳定，上电之后，等 1 秒钟再把拨杆拨到“上”位置启动。

|     | 上/外                                                                                           | 中                 | 下/内              |
| --- | ----------------------------------------------------------------------------------------------- | ------------------ | ------------------ |
| SA  | 杆：有力                                                                                        | 杆：调试模式       | 杆：无力           |
| SB  | 左机械臂：有力                                                                                  | 左机械臂：调试模式 | 左机械臂：无力     |
| SC  | 右机械臂：有力                                                                                  | 右机械臂：调试模式 | 右机械臂：无力     |
| SD  | 底盘：有力                                                                                      | 底盘：无力         | 底盘：无力         |
| SF  | 杆（调试模式）：升                                                                              | /                  | 杆（调试模式）：降 |
| SE  | /                                                                                               | /                  | /                  |
| SG  | /                                                                                               | /                  | /                  |
| SH  | （机械臂调试模式）切换关节，按一下切换一次，对应机械臂上的蜂鸣器会响 N 声，表示当前选中了关节 N | /                  | /                  |

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

| 键位     | 说明                       | 备注 |
| -------- | -------------------------- | ---- |
| WASD     | 前后左右移动               |      |
| 鼠标     | 控制视角旋转               |      |
| 鼠标右键 | 视角垂直位置复位到平视前方 |      |
| QE       | 控制视角左右旋转           |      |
| G        | /                          |      |
| F        | 杆升                       |      |
| C        | 杆降                       |      |
| V        | /                          |      |
| B        | /                          |      |
| Ctrl     | 刷新客户端 UI              |      |

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

### T265自定义控制器

![](https://private-user-images.githubusercontent.com/41246950/574174627-2b7ecb7d-9c2c-4f71-a60e-0ab76e72e2dc.jpg?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NzU0ODM5NzgsIm5iZiI6MTc3NTQ4MzY3OCwicGF0aCI6Ii80MTI0Njk1MC81NzQxNzQ2MjctMmI3ZWNiN2QtOWMyYy00ZjcxLWE2MGUtMGFiNzZlNzJlMmRjLmpwZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNjA0MDYlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjYwNDA2VDEzNTQzOFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTRiZGJjOTMyMDZiY2I3Y2E0YmU4ZDEwMTU2N2YyMDFjZmI3ZjM5NjBmYWRjNzA1MjkzZDZjZTcwN2RkZWEwMDYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.Gr_4KALLbL9RZSvmD4s7NZmBQ3HzBT_i4gRGL3zcktU)

- 按住按钮A，使能自定义控制器，可以控制机械臂运动，松开按钮A，自定义控制器失能，机械臂停止运动。

- 按住按钮A，控制器使能的情况下，按钮B控制夹爪开合。

- 不按按钮A的情况下，按一下按钮B，机械臂归位。

- 自定义控制器上有一个小屏幕，显示一个人工地平线表示控制器当前的姿态、一个小箭头表示哪个方向代表机械臂的正前方。

- 若需要重置正前方方向/T265的位姿飞掉，按按钮C重置位姿。

### 同构自定义控制器

自定义控制器上电时重力补偿默认关闭，按主控 C 板上的"KEY"键一次，开启重力补偿，再按一次关闭重力补偿。

自定义控制器的优先级高于上述组合键控制的“复杂行为”，自定义控制器启用时上述组合键无效。

使用两档开关 A 切换自定义控制器控制的机械臂，UI 上会通过准星周围的左半圆和右半圆高亮来显示当前控制的是左机械臂还是右机械臂。

按住末端的微动开关 A，自定义控制器启用，此时可以操作机械臂运动。

微动开关 A 未按下的情况下，按住微动开关 B，自定义控制器会产生朝向机械臂目前位置的力反馈，此时可以借助力反馈把自定义控制器移动到机械臂的实际位置附近。

两档开关 B 控制夹爪开合。

## 环境配置

计算平台为一款AMD的minipc：[铭凡EM680](https://minisforum.hk/pages/em680-%E8%A6%8F%E6%A0%BC%E5%8F%83%E6%95%B8)，系统使用Ubuntu 24.04，最好通过Ubuntu Pro替换上RT内核

### ultralytics

建一个虚拟环境，然后安装ultralytics

```bash
sudo apt update && sudo apt install -y python3-pip python3-venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install --upgrade pip
pip install ultralytics
pip install "numpy<2"
```

重新安装rocm版本的torch和torchvision

https://pytorch.org/get-started/locally/

在 `.bashrc` 里加一行：

```bash
export HSA_OVERRIDE_GFX_VERSION=10.3.0
```

```bash
sudo usermod -a -G render,video $USER
```

然后重启，如果配置成功，执行下面的python代码应该能看到GPU信息：

```python
import torch
print(torch.cuda.is_available())
print(torch.cuda.get_device_name(0))
```

### ROS2 + moveit

ROS2使用jazzy版本

```bash
sudo apt update
# ros-${ROS_DISTRO}-librealsense2 \
# ros-${ROS_DISTRO}-realsense2* \
sudo apt install -y \
  ros-${ROS_DISTRO}-moveit \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-backward-ros \
  ros-${ROS_DISTRO}-trac-ik \
  ros-${ROS_DISTRO}-generate-parameter-library \
  ros-${ROS_DISTRO}-py-binding-tools \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-moveit-ros-perception \
  ros-${ROS_DISTRO}-apriltag-ros \
  python3-transforms3d \
  libgflags-dev \
  nlohmann-json3-dev \
  ros-${ROS_DISTRO}-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-${ROS_DISTRO}-image-publisher \
  ros-${ROS_DISTRO}-camera-info-manager \
  ros-${ROS_DISTRO}-diagnostic-updater \
  ros-${ROS_DISTRO}-diagnostic-msgs \
  ros-${ROS_DISTRO}-statistics-msgs \
  libdw-dev \
  ros-${ROS_DISTRO}-foxglove-bridge
```

## 相机udev规则

```bash
sudo src/jetson/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

# 编译运行

```bash
git clone ...
cd engineer-amorph
git submodule update --init --recursive --depth=1

# unshallow libInterpolate，否则cmake读不到版本号会报错
cd src/jetson/amorph_transport/libs/libInterpolate
git fetch --unshallow

git lfs pull
```
