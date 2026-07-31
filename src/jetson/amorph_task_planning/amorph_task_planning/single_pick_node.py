#!/usr/bin/env python3
"""
single_pick_node: 监听两个能量单元位姿 → 计算预抓取位 + IK → 可视化。

步骤1: 利用两个圆柱主轴张成的平面法向量计算 pre-grasp 位置。
步骤2: 对 pre-grasp 位置调用 moveit RobotState.set_from_ik 求逆解。
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation

# moveit 直接 API (不走 move_group service)
from moveit import MoveItPy
from moveit.core.robot_state import RobotState


class SinglePickNode(Node):
    """单次抓取规划节点。

    订阅 eu_locator_v3_node 的 pose_l / pose_r，
    用两根主轴法向量计算接近方向 → pre-grasp → IK。
    """

    def __init__(self):
        super().__init__('single_pick_node')

        # ---- 参数 ----
        self.declare_parameter('pose_l_topic', '/eu_locator_v3_node/pose_l')
        self.declare_parameter('pose_r_topic', '/eu_locator_v3_node/pose_r')
        self.declare_parameter('offset_distance', 0.15)
        self.declare_parameter('sphere_radius', 0.02)
        self.declare_parameter('sphere_color', [0.0, 1.0, 0.0, 0.8])
        self.declare_parameter('pick_target', 'l')            # 'l' / 'r'
        self.declare_parameter('timestamp_tolerance', 0.1)
        self.declare_parameter('ik_timeout', 0.01)            # IK 超时 (s)

        self.offset_distance = self.get_parameter('offset_distance').value
        self.sphere_radius = self.get_parameter('sphere_radius').value
        self.sphere_color = self.get_parameter('sphere_color').value
        self.pick_target = self.get_parameter('pick_target').value
        self.ts_tol = self.get_parameter('timestamp_tolerance').value
        self.ik_timeout = self.get_parameter('ik_timeout').value

        # ---- 根据 pick_target 确定 arm group / tip link ----
        if self.pick_target == 'l':
            self.arm_group = 'arm_l'
            self.tip_link = 'l6_l'
        else:
            self.arm_group = 'arm_r'
            self.tip_link = 'l6_r'

        # ---- 初始化 MoveItPy (一次性, 加载 IK 求解器) ----
        self.get_logger().info('Initializing MoveItPy...')
        self._moveit = MoveItPy(
            node_name='single_pick_moveit',
            config_dict=self._build_moveit_config(),
            provide_planning_service=False,
        )
        self._robot_model = self._moveit.get_robot_model()
        self.get_logger().info(
            f'MoveItPy ready, arm_group={self.arm_group}, tip={self.tip_link}')

        # ---- 缓存 ----
        self.pose_l: PoseStamped | None = None
        self.pose_r: PoseStamped | None = None

        # ---- 订阅 ----
        self.sub_l = self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_l_topic').value,
            lambda msg: self._on_pose(msg, 'l'), 10)
        self.sub_r = self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_r_topic').value,
            lambda msg: self._on_pose(msg, 'r'), 10)

        # ---- 发布 ----
        self.marker_pub = self.create_publisher(Marker, '~/marker', 10)

        self.get_logger().info(
            f'single_pick_node ready, pick_target={self.pick_target}, '
            f'offset={self.offset_distance:.2f}m')

    # ------------------------------------------------------------------
    def _build_moveit_config(self) -> dict:
        """用 MoveItConfigsBuilder 生成 MoveItPy 需要的配置字典。

        MoveItCpp 内部通过 PlanningPipelineOptions::load(node) 读取:
          planning_pipelines.pipeline_names (字符串数组)
        MoveItConfigsBuilder.to_dict() 输出的是 planning_pipelines: ["ompl"]
        需要转换为 MoveItCpp 期望的嵌套格式。
        """
        from moveit_configs_utils import MoveItConfigsBuilder

        mc = (
            MoveItConfigsBuilder('amorph', package_name='amorph_v5_moveit_config')
            .planning_pipelines(pipelines=['ompl'])
            .robot_description(file_path='config/amorph.urdf.xacro')
            .robot_description_semantic(file_path='config/amorph.srdf')
            .joint_limits(file_path='config/joint_limits.yaml')
            .robot_description_kinematics(file_path='config/kinematics.yaml')
            .to_moveit_configs()
        )
        config = mc.to_dict()

        # planning_pipelines 在 to_dict() 中是 ["ompl"], 但 MoveItCpp 期望
        # planning_pipelines.pipeline_names: ["ompl"]
        pipeline_names = config.pop('planning_pipelines', [])
        config['planning_pipelines'] = {'pipeline_names': pipeline_names}
        # default_planning_pipeline 是 move_group 专用的, 删掉避免干扰
        config.pop('default_planning_pipeline', None)

        return config

    # ------------------------------------------------------------------
    def _on_pose(self, ps: PoseStamped, side: str):
        if side == 'l':
            self.pose_l = ps
        else:
            self.pose_r = ps
        self._try_compute()

    def _try_compute(self):
        if self.pose_l is None or self.pose_r is None:
            return

        t_l = self._ts(self.pose_l)
        t_r = self._ts(self.pose_r)
        if abs(t_l - t_r) > self.ts_tol:
            return

        # ---- 法向量 (外推方向) ----
        axis_l = self._axis(self.pose_l)
        axis_r = self._axis(self.pose_r)
        normal = np.cross(axis_l, axis_r)
        n = np.linalg.norm(normal)
        if n < 1e-6:
            self.get_logger().warn('parallel axes, cannot compute normal')
            return
        normal /= n
        if np.dot(normal, np.array([1.0, 0.0, 0.0])) < 0:
            normal = -normal

        # ---- 目标物体 ----
        target_ps = self.pose_l if self.pick_target == 'l' else self.pose_r
        center = np.array([target_ps.pose.position.x,
                           target_ps.pose.position.y,
                           target_ps.pose.position.z])
        target_axis = self._axis(target_ps)

        # ---- pre-grasp 位置 ----
        pre_grasp_pos = center + normal * self.offset_distance

        # ---- IK ----
        joint_positions = self._solve_ik(pre_grasp_pos, normal, target_axis)

        self.get_logger().info(
            f'axis_l=({axis_l[0]:.3f},{axis_l[1]:.3f},{axis_l[2]:.3f}) '
            f'axis_r=({axis_r[0]:.3f},{axis_r[1]:.3f},{axis_r[2]:.3f}) '
            f'normal=({normal[0]:.3f},{normal[1]:.3f},{normal[2]:.3f}) '
            f'pre_grasp=({pre_grasp_pos[0]:.4f},{pre_grasp_pos[1]:.4f},{pre_grasp_pos[2]:.4f}) '
            f'IK={joint_positions}'
        )

        # ---- 可视化 ----
        hdr = target_ps.header
        self._sphere(hdr, pre_grasp_pos, ns='pre_grasp', mid=0,
                     color=self.sphere_color, scale=self.sphere_radius * 2.0)
        self._arrow(hdr, pre_grasp_pos, center, ns='pre_grasp', mid=1,
                    color=[1.0, 1.0, 0.0, 0.8])
        self._arrow(hdr, center, center + target_axis * 0.10, ns='debug', mid=0,
                    color=[0.0, 0.0, 1.0, 0.8], shaft_d=0.003, head_d=0.006)
        self._arrow(hdr, center, center + normal * 0.10, ns='debug', mid=1,
                    color=[1.0, 0.0, 0.0, 0.8], shaft_d=0.003, head_d=0.006)
        other_ps = self.pose_r if self.pick_target == 'l' else self.pose_l
        self._sphere(hdr, np.array([other_ps.pose.position.x,
                                    other_ps.pose.position.y,
                                    other_ps.pose.position.z]),
                     ns='debug', mid=2, color=[0.3, 0.3, 1.0, 0.5], scale=0.015)

    # ------------------------------------------------------------------
    def _solve_ik(self, pos: np.ndarray, normal: np.ndarray,
                  axis_dir: np.ndarray) -> list | None:
        """对 pre-grasp 位置求 IK。

        末端执行器朝向: Z 指向物体中心 (抓取方向), Y 沿圆柱主轴方向。
        返回关节角度列表，失败返回 None。
        """
        # 构建右手坐标系
        ez = -normal  # 末端 Z 轴: 指向物体中心 (从 pre-grasp → object)
        ey = self._normalize(axis_dir - np.dot(axis_dir, ez) * ez)  # Y: 主轴投影到 Z 垂直面
        ex = np.cross(ey, ez)

        quat = Rotation.from_matrix(np.column_stack([ex, ey, ez])).as_quat()

        pose = Pose()
        pose.position.x = float(pos[0])
        pose.position.y = float(pos[1])
        pose.position.z = float(pos[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        state = RobotState(self._robot_model)
        ok = state.set_from_ik(self.arm_group, pose, self.tip_link, self.ik_timeout)
        if ok:
            return state.get_joint_group_positions(self.arm_group).tolist()
        else:
            self.get_logger().warn('IK failed for pre-grasp pose')
            return None

    # ------------------------------------------------------------------
    @staticmethod
    def _normalize(v: np.ndarray) -> np.ndarray:
        n = np.linalg.norm(v)
        return v / n if n > 1e-10 else v

    @staticmethod
    def _ts(ps: PoseStamped) -> float:
        return ps.header.stamp.sec + ps.header.stamp.nanosec * 1e-9

    @staticmethod
    def _axis(ps: PoseStamped) -> np.ndarray:
        q = [ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w]
        return Rotation.from_quat(q).apply(np.array([0.0, 0.0, 1.0]))

    # ---- Marker helpers -------------------------------------------------
    def _sphere(self, header, pos, *, ns, mid, color, scale):
        m = Marker(header=header, ns=ns, id=mid, type=Marker.SPHERE,
                   action=Marker.ADD)
        m.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = color
        self.marker_pub.publish(m)

    def _arrow(self, header, frm, to, *, ns, mid, color,
               shaft_d=0.005, head_d=0.01):
        m = Marker(header=header, ns=ns, id=mid, type=Marker.ARROW,
                   action=Marker.ADD)
        m.points = [Point(x=frm[0], y=frm[1], z=frm[2]),
                    Point(x=to[0], y=to[1], z=to[2])]
        m.scale.x, m.scale.y, m.scale.z = shaft_d, head_d, 0.0
        m.color.r, m.color.g, m.color.b, m.color.a = color
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SinglePickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
