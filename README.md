# 2026 赛季工程 ｜ CODENAME `amorph`

## CI status

[![build_host](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_host.yaml)
[![build_stm32](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_stm32.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_stm32.yaml)
[![build_custom_ctrller](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml/badge.svg)](https://github.com/XDU-IRobot/engineer-amorph/actions/workflows/build_custom_ctrller.yaml)

## 操作指南

https://w0ybodqyg7f.feishu.cn/wiki/Q9LIw2H5Viq3B2klIKdcK1jnnRc?from=from_copylink

## 环境配置

```bash
wget http://fishros.com/install -O fishros && . fishros
```

```bash
git clone ...
cd engineer-amorph
git submodule update --init --recursive --depth=1

# unshallow libInterpolate，否则cmake读不到版本号会报错
cd src/jetson/amorph_transport/libs/libInterpolate
git fetch --unshallow

git lfs pull
```

整个系统分为在车上的Jetson和自定义客户端两部分，需要单独配置环境

### Jetson

1. 安装ROS2 rolling

2. 安装ultralytics

    ```bash
    python3 -m venv --system-site-packages .venv
    source .venv/bin/activate
    # ...安装jetson对应的pytorch版本
    pip install ultralytics
    ```

### 自定义客户端

1. 安装ROS2 jazzy

2. 安装相机udev规则

   ```bash
   sudo src/jetson/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

3. ↓

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
   ros-${ROS_DISTRO}-tf-transformations \
   libdw-dev \
   ros-${ROS_DISTRO}-foxglove-bridge \
   libgstreamer1.0-dev \
   libgstreamer-plugins-base1.0-dev \
   libgstreamer-plugins-bad1.0-dev \
   gstreamer1.0-plugins-base-apps \
   gstreamer1.0-plugins-bad \
   gstreamer1.0-plugins-good \
   gstreamer1.0-libav
   ```

4. 安装 Protobuf + MQTT（自定义客户端通信）

   ```bash
   sudo apt install -y \
     protobuf-compiler \
     libprotobuf-dev \
     libpaho-mqtt-dev \
     libpaho-mqttpp-dev
   ```
