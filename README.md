# 2025 赛季工程 ｜ CODENAME `fluorite`

## CI status

<table>
<tr>
<td>foxy</td>
<td rowspan=2>
    <a href="https://github.com/XDU-IRobot/engineer-2025/actions/workflows/build_eol.yaml">
        <img src="https://github.com/XDU-IRobot/engineer-2025/actions/workflows/build_eol.yaml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>galactic</td>
</tr>
<tr>
<td>humble</td>
<td rowspan=4>
    <a href="https://github.com/XDU-IRobot/engineer-2025/actions/workflows/build.yaml">
        <img src="https://github.com/XDU-IRobot/engineer-2025/actions/workflows/build.yaml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>iron</td>
</tr>
<tr>
<td>jazzy</td>
</tr>
<tr>
<td>rolling</td>
</tr>
</table>

工程车所有功能的开发工作在本仓库进行（控制、决策 AI、机械臂运动规划、导航、自动兑换视觉和自定义控制器）。

这篇 README 提供了一些这辆工程最基础的使用和复现指引，如果需要更详细的信息，请去阅读它的技术报告(wip)。

## 操作方法

> [!CAUTION]  
> **如果车失控，把遥控器正面的全部拨杆（`SA`、`SB`、`SC`、`SD`）置于“下”位置。**

![](assets/remote_layout.jpg)

### 遥控器拨杆

使用遥控器拨杆可以切换不同模式：

|     | 上/外                                     | 中                            | 下/内                         |
| --- | ----------------------------------------- | ----------------------------- | ----------------------------- |
| SA  | 底盘：小陀螺                              | 底盘：跟随底盘                | 底盘：无力                    |
| SB  | 大 yaw 轴：有力                           | 大 yaw 轴：无力               | 大 yaw 轴：无力               |
| SC  | 机械臂：手动模式（键盘/自定义控制器控制） | 机械臂：键盘控制模式          | 机械臂：保持收回              |
| SD  | 机械臂：有力                              | 机械臂：无力                  | 机械臂：无力                  |
| SF  | 机械臂：单关节调试模式                    | /                             | 机械臂正常                    |
| SE  | 单关节调试模式：选择小 yaw                | 单关节调试模式：选择 pitch1   | 单关节调试模式：SE 无效       |
| SG  | 单关节调试模式：选择 pitch2               | 单关节调试模式：选择末端 yaw1 | 单关节调试模式：SG 无效       |
| SH  | 单关节调试模式：选择 末端 pitch           | /                             | 单关节调试模式：选择末端 yaw2 |

优先级：`SD` > `SF` > `SE > SG > SH` > `SC`

人话：SD 优先级最高，只有 SD 置于有力状态时，机械臂才会动；SF 优先级次之，一旦 SF 置于单关节调试模式，机械臂就会进入单关节调试模式，SC 会被忽略；SE、SG、SH 用于选择单关节调试模式下的关节，优先级递减，SC 优先级最低。

### 遥控器摇杆、旋钮

| axis     | 说明                                                                                 |
| -------- | ------------------------------------------------------------------------------------ |
| LD       | 在机械臂有力且处于单关节调试模式时，控制吸盘的真空泵功率，中间为 0，向左拧吸向右拧吹 |
| RD       | 在底盘处于小陀螺模式时，控制小陀螺转速                                               |
| 左杆左右 | 平移                                                                                 |
| 左杆上下 | 前进后退                                                                             |
| 右杆左右 | 旋转                                                                                 |
| 右杆上下 | 机械臂单关节调试模式下，控制选中关节的角度                                           |
| LS       | 控制左边的存矿机构，拨到上面吸，拨到下面关                                           |
| RS       | 控制右边的存矿机构，拨到上面吸，拨到下面关                                           |

### 键鼠

比赛时把档位拨到：`SA 中` `SB 上` `SC 上` `SD 上` `SF 外`

即可正常在客户端里控制底盘和机械臂：

| 键位     | 说明                   | 备注                                                                         |
| -------- | ---------------------- | ---------------------------------------------------------------------------- |
| WSAD     | 控制车辆移动           |                                                                              |
| 鼠标左右 | 控制视角左右旋转       |                                                                              |
| 鼠标滚轮 | 控制 pitch1 关节的角度 | 用来把图传对着地面，在比赛时观察银矿，机械臂在收回状态时是看不到地上的银矿的 |
| QE       | 控制视角左右旋转       |                                                                              |
| G        | 开关自定义控制器       |                                                                              |
| C        | 开关左边存矿机构的吸盘 |                                                                              |
| V        | 开关机械臂末端的吸盘   |                                                                              |
| B        | 开关右边存矿机构的吸盘 |                                                                              |
| Ctrl     | 刷新客户端 UI          |                                                                              |

机械臂有一系列行为树控制的动作组，当机械臂的模式设置处于`SC 上` `SD 上`，且没有启用自定义控制器时，就可以用组合键触发这些动作组：

| 键位      | 说明                       | 备注                                                                   |
| --------- | -------------------------- | ---------------------------------------------------------------------- |
| Ctrl + R  | 机械臂收回到初始位置       |                                                                        |
| Ctrl + F  | 取银矿                     |                                                                        |
| Ctrl + C  | 把取到的矿存到右边         |                                                                        |
| Ctrl + V  | 把存到右边的银矿拿起来     |                                                                        |
| Shift + V | 把存到右边的金矿拿起来     |                                                                        |
| Ctrl + Z  | 把取到的矿存到左边         |                                                                        |
| Ctrl + X  | 把存到左边的银矿拿起来     |                                                                        |
| Shift + X | 把存到左边的金矿拿起来     |                                                                        |
| Ctrl + B  | 强制跳过当前正在跟踪的轨迹 | 如果机械臂卡在某个轨迹中间，无法完成执行，用这个组合键强制完成当前轨迹 |

> [!NOTE]  
> 当动作组正在执行时，操作手的客户端 UI 上会显示“EXECUTING”，这个时候机械臂无法操作；如果需要打断当前正在执行的动作，用遥控器上的 SC 或 SD 来回切换一次机械臂的模式即可。

当机械臂处于键盘控制模式（SC 拨到中）时，可以使用下面的键位控制末端执行器在空间中移动：

| 键位      | 说明                             |
| --------- | -------------------------------- |
| F         | 机械臂末端向前运动               |
| V         | 机械臂末端向后运动               |
| C         | 机械臂末端向左运动               |
| B         | 机械臂末端向右运动               |
| Shift + F | 机械臂末端向上运动               |
| Shift + V | 机械臂末端向下运动               |
| Z         | 机械臂末端逆时针旋转（图传视角） |
| X         | 机械臂末端顺时针旋转（图传视角） |
| Ctrl + F  | 机械臂末端 pitch 向上            |
| Ctrl + V  | 机械臂末端 pitch 向下            |
| Ctrl + C  | 机械臂末端 yaw 向左              |
| Ctrl + B  | 机械臂末端 yaw 向右              |

## 主要硬件

- 电机

  - 底盘：6020 x4, 3508 x4

  - 机械臂：本末 p1010a_111 x1, DM-J8009-2EC x3, DM-J4310-2EC x2, 6020 x1

- IMU `维特智能 HWT606` `->` `uart4@orange pi`。

- 遥控器/接收机 `天地飞ET16S/RF209S` `->` `uart0@orange pi`。

- 计算设备

  - 视觉 `Jetson xavier nx 8gb` `ip = 192.168.1.202`

  - 控制/机械臂规划/导航 `orangepi 5 8gb` `ip = 192.168.1.201`

- 主相机 `Realsense D435i` `->` `Jetson`

- 自定义控制器

  - 主控 `自制带IIC复用器和RS232收发电路的ESP32-S3开发板`

  - 磁编码器 x6 `AS5600`

- 真空泵 x2 `VN-C4`

### PCB

整车用到了 4 块 自行设计的 PCB，PCB 文件使用 git lfs 管理，在仓库里执行`git lfs pull`即可下载：

- orange pi 5 的扩展板 [`pcb/opi5.eprj`](pcb/opi5.eprj)

- 位于底盘的电源继电器板 [`pcb/engineer_2025.eprj`](pcb/engineer_2025.eprj)

- 底盘分电板(x2) [`pcb/engineer_2025.eprj`](pcb/engineer_2025.eprj)

- 位于云台的电磁阀驱动器/分电板 [`pcb/engineer_2025.eprj`](pcb/engineer_2025.eprj)

## 软件架构

### 手动模式

![](assets/manual_flowchart.png)

### 自动模式

![](assets/auto_flowchart.png)

## 软件开发/部署指引

### 环境配置

#### 开发本仓库需要在电脑上安装：

- ROS2 `>= humble`

- moveit `apt install ros-humble-moveit*`

- SDL 1.2 `apt install libsdl1.2-dev`

- behaviortree_cpp_v3 `apt install ros-humble-behaviortree-cpp-v3`

- NLopt `apt install libnlopt-dev libnlopt-cxx-dev`

- camera_info_manager `apt install ros-humble-camera-info-manager`

- cv_bridge `apt install ros-humble-cv-bridge`

- [大恒相机驱动 `Galaxy Linux-x86-Gige-U3 SDK_CN-EN`](https://www.daheng-imaging.com/index.php?m=content&c=index&a=lists&catid=59&czxt=9&sylx=23&syxj=#mmd)

#### 在香橙派(`ubuntu 22.04, RT Kernel`)上：

1. 安装以下软件

   - ROS2 `>= humble`

   - moveit `apt install ros-humble-moveit*`

   - behaviortree_cpp_v3 `apt install ros-humble-behaviortree-cpp-v3`

   - NLopt `apt install libnlopt-dev libnlopt-cxx-dev`

   - [wiringOP](https://github.com/orangepi-xunlong/wiringOP)

2. 执行初始化脚本：

   ```bash
   sudo python3 ./scripts/setup_env.py -o
   ```

#### 在 Jetson(`ubuntu 20.04`) 上：

1. 安装以下软件

   - Jetpack `apt install nvidia-jetpack`

     > [!TIP]  
     > 如果 apt 提示找不到这个包，参考[这篇文章](https://blog.csdn.net/pleasescanf/article/details/133189245)添加一下 l4t 的 apt repo。

   - OpenCV `apt install libopencv-dev libopencv-contrib-dev`

   - ROS2 `>= galactic`

   - cv_bridge `apt install ros-galactic-cv-bridge`

   - camera_info_manager `apt install ros-galactic-camera-info-manager`

   - [大恒相机驱动 `Galaxy Linux-armhf-Gige-U3 SDK`](https://www.daheng-imaging.com/index.php?m=content&c=index&a=lists&catid=59&czxt=30&sylx=23&syxj=#mmd)

2. 按指引配好 [mrc-net](https://github.com/amzn/mrc-net-6d-pose) 的运行环境，被折磨一个周（祝你好运:)))))）

3. 执行初始化脚本：

   ```bash
   sudo python3 ./scripts/setup_env.py -j
   ```

### 编译

```bash
# 按情况
python3 scripts/build.py -j     # 编译jetson上运行的包
python3 scripts/build.py -o     # 编译香橙派上运行的包
python3 scripts/build.py -c     # 编译自定义控制器上运行的程序

# 也可以在你的电脑上编译
python3 scripts/build.py -d     # 编译所有东西，一般只有开发的时候才需要
```

### 手动运行

```bash
source install/setup.bash
# 同上
chrt -f 99 ros2 launch fluorite_bringup opi.launch.py
# or
# ros2 launch fluorite_bringup jetson.launch.py
```

### [可选] 开机自启

通过向 systemd 注册服务的方法实现程序开机自启：

```bash
sudo ./scripts/install_service.py -j  # jetson
# or
sudo ./scripts/install_service.py -o  # orange pi
sudo ./scripts/install_service.py -c  # 自定义控制器
```

如果不想开机自启，卸载服务即可：

```bash
sudo ./scripts/install_service.py --uninstall
```

## CAN 总线 设备列表

| 设备        | CAN 总线 | master id                                | slave id  | 备注                  |
| ----------- | -------- | ---------------------------------------- | --------- | --------------------- |
| 左前轮      | can0     | 0x202                                    | 0x200     | 2 号 3508，反馈 500hz |
| 左后轮      | can0     | 0x203                                    | 0x200     | 3 号 3508，反馈 500hz |
| 右前轮      | can0     | 0x201                                    | 0x200     | 1 号 3508，反馈 500hz |
| 右后轮      | can0     | 0x204                                    | 0x200     | 4 号 3508，反馈 500hz |
| 左前舵      | can0     | 0x206                                    | 0x1ff     | 2 号 6020，反馈 500hz |
| 左后舵      | can0     | 0x207                                    | 0x1ff     | 3 号 6020，反馈 500hz |
| 右前舵      | can0     | 0x205                                    | 0x1ff     | 1 号 6020，反馈 500hz |
| 右后舵      | can0     | 0x208                                    | 0x1ff     | 4 号 6020，反馈 500hz |
| 大 yaw 轴   | can1     | 0x51, 0x61, 0x71, 0x81, 0x91, 0xa1, 0xb1 | 0x32~0x3a | 1 号 本末 p1010a_111  |
| 小 yaw 轴   | can1     | 0xc9                                     | 0xca      | DM-J8009-2EC          |
| pitch #1    | can1     | 0xc1                                     | 0xc2      | DM-J8009-2EC          |
| pitch #2    | can1     | 0xc3                                     | 0xc4      | DM-J8009-2EC          |
| 末端 yaw #1 | can1     | 0xc5                                     | 0xc6      | DM-J4310-2EC          |
| 末端 pitch  | can1     | 0xc7                                     | 0xc8      | DM-J4310-2EC          |
| 末端 yaw #2 | can1     | 0x205                                    | 0x1ff     | 1 号 6020，反馈 500hz |

## 上位机/硬件配置

### 遥控器通道设置

<img src="./assets/rc1.jpg" align="center" width=350/>
<img src="./assets/rc2.jpg" align="center" width=350/>

<img src="./assets/rc3.jpg" align="center" width=350/>
<img src="./assets/rc4.jpg" align="center" width=350/>

<img src="./assets/rc5.jpg" align="center" width=350/>
<img src="./assets/rc6.jpg" align="center" width=350/>

### 小 yaw 轴 达妙 8009

![](assets/yaw.jpg)

### pitch #1 达妙 8009

![](assets/pitch1.jpg)

### pitch #2 达妙 8009

![](assets/pitch2.jpg)

### 末端 yaw #1 达妙 4310

![](assets/endyaw1.jpg)

### 末端 pitch 达妙 4310

![](assets/pitch3.jpg)
