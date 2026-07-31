"""
Fake Energy Unit Interactive Marker Node

在 RViz 中创建一个可拖拽的 6-DOF 交互标记，用户拖动后自动广播
world -> energy_unit_selected 的 TF 变换，供 master node 抓取任务使用。
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer

import os

from ament_index_python.packages import get_package_share_directory

import tf2_ros


class FakeEUMarkerNode(Node):
    def __init__(self):
        super().__init__("fake_eu_marker")

        self.declare_parameter("tf_rate", 10.0)
        self.declare_parameter("child_frame_id", "energy_unit_selected")

        self.parent_frame = "world"
        self.child_frame = self.get_parameter("child_frame_id").value

        # 初始位姿全零
        self.current_pose = Pose()
        self.current_pose.orientation.w = 1.0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        tf_rate = self.get_parameter("tf_rate").value
        self.tf_timer = self.create_timer(1.0 / tf_rate, self.publish_tf)

        self.server = InteractiveMarkerServer(self, "fake_eu_marker")
        self._make_interactive_marker()
        self.server.applyChanges()

        self.get_logger().info(
            f"Fake EU marker ready, broadcasting {self.parent_frame} -> {self.child_frame}"
        )

    def _make_interactive_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.parent_frame
        int_marker.name = "energy_unit_marker"
        int_marker.description = "Energy Unit (drag me)"
        int_marker.pose = self.current_pose
        int_marker.scale = 0.3

        # 逐轴平移控件 (MOVE_AXIS)
        for axis_orient, name in [
            ((1.0, 0.0, 0.0, 1.0), "move_x"),
            ((0.0, 1.0, 0.0, 1.0), "move_y"),
            ((0.0, 0.0, 1.0, 1.0), "move_z"),
        ]:
            ctrl = InteractiveMarkerControl()
            ctrl.name = name
            ctrl.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            ctrl.orientation.x = axis_orient[0]
            ctrl.orientation.y = axis_orient[1]
            ctrl.orientation.z = axis_orient[2]
            ctrl.orientation.w = axis_orient[3]
            int_marker.controls.append(ctrl)

        # 逐轴旋转控件 (ROTATE_AXIS)
        for axis_orient, name in [
            ((1.0, 0.0, 0.0, 1.0), "rotate_x"),
            ((0.0, 1.0, 0.0, 1.0), "rotate_y"),
            ((0.0, 0.0, 1.0, 1.0), "rotate_z"),
        ]:
            ctrl = InteractiveMarkerControl()
            ctrl.name = name
            ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            ctrl.orientation.x = axis_orient[0]
            ctrl.orientation.y = axis_orient[1]
            ctrl.orientation.z = axis_orient[2]
            ctrl.orientation.w = axis_orient[3]
            int_marker.controls.append(ctrl)

        # 可视化：使用 energy_unit.stl 模型
        mesh = Marker()
        mesh.type = Marker.MESH_RESOURCE
        try:
            stl_path = os.path.join(
                get_package_share_directory("amorph_assets"), "assets", "energy_unit.stl"
            )
            mesh.mesh_resource = "file://" + stl_path
            mesh.mesh_use_embedded_materials = False
        except Exception:
            self.get_logger().warn("Cannot find energy_unit.stl, fallback to cube")
            mesh.type = Marker.CUBE
        mesh.scale.x = 1.0
        mesh.scale.y = 1.0
        mesh.scale.z = 1.0
        mesh.color.r = 0.0
        mesh.color.g = 1.0
        mesh.color.b = 0.0
        mesh.color.a = 0.7

        mesh_ctrl = InteractiveMarkerControl()
        mesh_ctrl.always_visible = True
        mesh_ctrl.markers.append(mesh)
        int_marker.controls.append(mesh_ctrl)

        self.server.insert(int_marker, feedback_callback=self._feedback_callback)

    def _feedback_callback(self, feedback: InteractiveMarkerFeedback):
        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            self.current_pose = feedback.pose
            self.publish_tf()

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = self.current_pose.position.x
        t.transform.translation.y = self.current_pose.position.y
        t.transform.translation.z = self.current_pose.position.z
        t.transform.rotation = self.current_pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeEUMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
