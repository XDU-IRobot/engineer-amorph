# 2025 赛季工程 ｜ `fluorite`

> 为了解决给包和模型起名字的问题，给这辆车起个代号名吧：
>
> CODENAME **fluorite**

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

工程车所有功能的开发工作在本仓库进行，包括控制、机械臂运动规划、导航和自动兑换视觉。

## 操作方法

> [!CAUTION]  
> **如果车失控，把遥控器正面的全部拨杆（`SA`、`SB`、`SC`、`SD`）置于“下”位置。**

![](assets/remote_layout.jpg)

### 拨杆

使用遥控器拨杆可以切换不同模式：

|     | 上/外                | 中                   | 下/内          |
| --- | -------------------- | -------------------- | -------------- |
| SA  | 底盘遥控器/键鼠控制  | 底盘接收导航指令     | 底盘无力       |
| SB  | 大 yaw 轴跟随底盘    | 大 yaw 轴跟随机械臂  | 大 yaw 轴无力  |
| SC  | 机械臂键盘控制       | 机械臂接收决策树指令 | 机械臂保持收回 |
| SD  | 机械臂有力           | 机械臂无力           | 机械臂无力     |
| SF  | 机械臂单关节调试模式 | /                    | 机械臂正常     |
| SE  | 选择小 yaw           | 选择 pitch1          | 无效           |
| SG  | 选择 pitch2          | 选择末端 yaw1        | 无效           |
| SH  | 选择 末端 pitch      | /                    | 选择末端 yaw2  |

优先级：`SD` > `SF` > `SE > SG > SH` > `SC`

人话：SD 优先级最高，只有 SD 置于有力状态时，机械臂才会动；SF 优先级次之，一旦 SF 置于单关节调试模式，机械臂就会进入单关节调试模式，SC 会被忽略；SE、SG、SH 用于选择单关节调试模式下的关节，优先级递减，且只有在 SF 置于单关节调试模式时才有效。SC 优先级最低。

### 轴

| axis     | 说明                                                                                                 |
| -------- | ---------------------------------------------------------------------------------------------------- |
| LD       | 在机械臂有力且不处于接收决策树指令模式的情况下，控制存矿机构的真空泵功率，中间为 0，向左拧吸向右拧吹 |
| RD       | 在机械臂有力且不处于接收决策树指令模式的情况下，控制吸盘的真空泵功率，中间为 0，向左拧吸向右拧吹     |
| 右杆左右 | 控制底盘平移                                                                                         |
| 右杆上下 | 控制底盘前进后退                                                                                     |
| 左杆左右 | 控制底盘旋转                                                                                         |
| 左杆上下 | 机械臂单关节调试模式下，控制选中关节的角度                                                           |
| LS       | 未使用                                                                                               |
| RS       | 未使用                                                                                               |

### 键盘

当底盘或机械臂被设为键盘控制模式时，可以使用键盘控制底盘或机械臂。

| 键位      | 说明                             |
| --------- | -------------------------------- |
| W         | 底盘前进                         |
| S         | 底盘后退                         |
| A         | 底盘左移                         |
| D         | 底盘右移                         |
| Q         | 底盘左旋/打断锁定                |
| Shift + Q | 底盘锁定左旋                     |
| E         | 底盘右旋/打断锁定                |
| Shift + E | 底盘锁定右旋                     |
| I         | 机械臂末端向前运动               |
| K         | 机械臂末端向后运动               |
| J         | 机械臂末端向左运动               |
| L         | 机械臂末端向右运动               |
| R         | 机械臂末端向上运动               |
| F         | 机械臂末端向下运动               |
| U         | 机械臂末端逆时针旋转（图传视角） |
| O         | 机械臂末端顺时针旋转（图传视角） |
| Shift + I | 机械臂末端 pitch 向上            |
| Shift + K | 机械臂末端 pitch 向下            |
| Shift + J | 机械臂末端 yaw 向左              |
| Shift + L | 机械臂末端 yaw 向右              |

## 硬件

- IMU `维特智能 HWT606` `->` `uart4@orange pi`。

- 遥控器/接收机 `天地飞ET16S/RF209S` `->` `uart0@orange pi`。

- 主控

  - 视觉 `Jetson xavier nx 8gb` `ip = 10.0.3.101`

  - 控制/机械臂规划/导航 `orangepi 5 8gb` `ip = 10.0.3.102`

- 相机 `大恒水星 MER2-230` `->` `Jetson`。

### PCB

整车用到了 3 块 自行设计的 PCB，PCB 文件使用 git lfs 管理，在仓库里执行`git lfs pull`即可下载：

- orange pi 5 的扩展板 [`pcb/opi5.eprj`](pcb/opi5.eprj)

- 位于底盘的电源继电器板 [`pcb/engineer_2025.eprj`](pcb/engineer_2025.eprj)

- 位于云台设备舱的电磁阀驱动器/分电板 [`pcb/engineer_2025.eprj`](pcb/engineer_2025.eprj)

## 程序使用方法

### 环境配置

#### 在香橙派(`ubuntu 22.04, RT Kernel`)上：

1. 安装以下软件

   - ROS2 `>= humble`

   - moveit `apt install ros-humble-moveit*`

   - Eigen `apt install libeigen3-dev`

   - behaviortree_cpp_v3 `apt install ros-humble-behaviortree-cpp-v3`

   - [qpOASES](https://github.com/coin-or/qpOASES)

   - [wiringOP](https://github.com/orangepi-xunlong/wiringOP)

2. 执行初始化脚本：

   ```bash
   sudo python3 ./scripts/setup_env.py -o
   ```

#### 在 Jetson(`ubuntu 20.04`) 上：

1. 安装以下软件

   - Jetpack `apt install nvidia-jetpack`

   - ROS2 `>= galactic`

   - [大恒相机驱动 `Galaxy Linux-armhf-Gige-U3 SDK`](https://www.daheng-imaging.com/index.php?m=content&c=index&a=lists&catid=59&czxt=30&sylx=23&syxj=#mmd)

2. 执行初始化脚本：

   ```bash
   sudo python3 ./scripts/setup_env.py -j
   ```

### 编译

```bash
# 按情况，在哪台电脑上编译就选哪个
python3 scripts/build.py -j     # 编译jetson上运行的包
python3 scripts/build.py -o     # 编译香橙派上运行的包
```

### 运行

```bash
source install/setup.bash
# 同上
chrt -f 99 ros2 launch fluorite_bringup opi.launch.py
ros2 launch fluorite_bringup jetson.launch.py
```

### [可选] 注册 systemd 服务（开机自启）

车目前的状态应该已经注册过，如果板子重新装过系统需要重新配置，执行：

```bash
./scripts/install_service.sh
```

如果不想开机自启，取消注册服务或者暂时 disable 服务均可：

```bash
./scripts/uninstall_service.sh
# or
sudo systemctl disable fluorite_opi.service
sudo systemctl disable fluorite_jetson.service
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
