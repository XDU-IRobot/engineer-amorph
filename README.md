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

除导航外，这辆车所有功能的开发工作在本仓库进行，包括控制、机械臂运动规划和自动兑换视觉。

## 硬件

- 主控 `Jetson orin nx 8gb`

- 相机 `大恒水星 MER2-230`。

### CAN 总线 设备列表

![](assets/p1.jpg)

底盘电机布局如上图

| 设备       | CAN 总线 | master id | slave id | 备注      |
| ---------- | -------- | --------- | -------- | --------- |
| 左前轮     | can0     | 0x201     | 0x200    | 1 号 3508 |
| 左后轮     | can0     | 0x202     | 0x200    | 2 号 3508 |
| 右前轮     | can0     | 0x205     | 0x1ff    | 5 号 3508 |
| 右后轮     | can0     | 0x206     | 0x1ff    | 6 号 3508 |
| 左中轮     | can0     | 0x203     | 0x200    | 3 号 3508 |
| 右中轮     | can0     | 0x207     | 0x1ff    | 7 号 3508 |
| 左抬升机构 | can0     | 0x204     | 0x200    | 4 号 3508 |
| 右抬升机构 | can0     | 0x208     | 0x1ff    | 8 号 3508 |

## dependencies

- ROS2 `>= humble`

- moveit `apt install ros-humble-moveit*`

- ros2_control `apt install ros-humble-ros2-control*`

- [qpOASES](https://github.com/coin-or/qpOASES)

- [大恒相机驱动 `Galaxy Linux-armhf-Gige-U3 SDK`](https://www.daheng-imaging.com/index.php?m=content&c=index&a=lists&catid=59&czxt=30&sylx=23&syxj=#mmd)
