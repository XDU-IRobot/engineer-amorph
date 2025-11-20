# 2026 赛季工程 ｜ CODENAME `amorph`

## CI status

[![build_host](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml)
[![build_custom_ctrller](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml)
[![build_arm_ctrller](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_arm_ctrller.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_arm_ctrller.yaml)

## 环境配置（Jetson系统内）

下面是在 Jetpack 5.0.2 (Ubuntu 20.04) 的 Xavier NX 上进行的环境配置步骤

### ROS2

安装 galactic 版本

```bash
wget http://fishros.com/install -O fishros && . fishros
```

## 环境配置（Docker容器内）

Moveit Jazzy的开发完成度比较高，moveit这个项目能用新的就不要用旧的，但是xavier nx的系统是20，升级到24基本是不可能的事情，即使有可能也非常麻烦，所以只能跑一个ROS2 Jazzy的容器来运行moveit

### ROS2 + moveit

moveit/moveit2这个镜像没有jazzy tag，所以拉一个ROS的镜像，然后在里面配置环境

拉镜像，然后运行容器：

```bash
sudo docker pull ros:jazzy
sudo docker run --privileged --net=host --name moveit_test -it ros:jazzy
```

给容器里的系统换源，换完之后

```bash
sudo apt update
sudo apt install -y ros-jazzy-moveit ros-jazzy-xacro ros-jazzy-realsense2-* ros-jazzy-backward-ros ros-jazzy-trac-ik ros-jazzy-generate-parameter-library
```

### librealsense

- librealsense `== 2.56.5`
- realsense-ros `== 4.56.4`

编译安装librealsense，注意和realsense-ros的版本一一对应

这一步会缺一堆东西，缺什么用apt装什么即可

```bash
git clone --depth 1 https://github.com/IntelRealSense/librealsense.git
cd librealsense
cmake -DFORCE_LIBUVC=true -DCMAKE_BUILD_TYPE=release -Bbuild
cd build
make -j6
sudo make install
```

### ultralytics

建一个虚拟环境，然后安装ultralytics

```bash
python3 -m venv ~/ultralytics_venv
source ~/ultralytics_venv/bin/activate
pip install ultralytics
```
