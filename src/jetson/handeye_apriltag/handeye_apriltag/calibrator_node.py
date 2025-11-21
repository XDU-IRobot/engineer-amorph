import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
import cv2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
from scipy.spatial.transform import Rotation as R
import os
import yaml
import sys
import select
import termios
import tty
import threading
from ament_index_python.packages import get_package_share_directory

class HandEyeCalibrator(Node):
    def __init__(self):
        super().__init__('handeye_calibrator')

        # Parameters
        self.declare_parameter('aruco_dictionary', 'DICT_APRILTAG_36h11')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_size', 0.055) # meters
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('eef_frame', 'link6') # End-effector frame
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        
        self.aruco_dict_name = self.get_parameter('aruco_dictionary').get_parameter_value().string_value
        self.tag_id = self.get_parameter('tag_id').get_parameter_value().integer_value
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.eef_frame = self.get_parameter('eef_frame').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('camera_optical_frame').get_parameter_value().string_value

        # Aruco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.aruco_dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters()

        # TF Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # CV Bridge
        self.bridge = CvBridge()

        # Data storage
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []
        
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)

        # Services (Optional now)
        self.create_service(Trigger, 'handeye/capture', self.capture_callback)
        self.create_service(Trigger, 'handeye/compute', self.compute_callback)
        self.create_service(Trigger, 'handeye/reset', self.reset_callback)
        self.create_service(Trigger, 'handeye/save', self.save_callback)

        self.get_logger().info('HandEye Calibrator Node Started')
        
        # Keyboard control setup
        self.settings = termios.tcgetattr(sys.stdin)
        self.print_usage()
        self.key_thread = threading.Thread(target=self.keyboard_loop)
        self.key_thread.daemon = True
        self.key_thread.start()

    def print_usage(self):
        print("\n" + "="*40)
        print("HandEye Calibrator Keyboard Control")
        print("  c: Capture sample")
        print("  e: Compute (Execute) calibration")
        print("  r: Reset samples")
        print("  s: Save result")
        print("  q: Quit")
        print("="*40 + "\n")

    def getKey(self):
        # Check if data is available
        dr,dw,de = select.select([sys.stdin], [], [], 0.1)
        if not dr:
            return ''
        
        # Data available, read it in raw mode
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def keyboard_loop(self):
        while rclpy.ok():
            key = self.getKey()
            if key == 'c':
                self.capture_action()
            elif key == 'e':
                self.compute_action()
            elif key == 'r':
                self.reset_action()
            elif key == 's':
                self.save_action()
            elif key == 'q':
                print("Quitting...")
                os._exit(0)

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')

    def image_callback(self, msg):
        self.latest_image = msg

    def get_transform(self, target_frame, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
            return t
        except Exception as ex:
            self.get_logger().warn(f'Could not transform {source_frame} to {target_frame}: {ex}')
            return None

    def capture_action(self):
        if self.latest_image is None:
            self.get_logger().warn("No image received")
            return False, "No image received"
        
        if self.camera_matrix is None:
            self.get_logger().warn("No camera info received")
            return False, "No camera info received"

        # 1. Detect Tag
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None or self.tag_id not in ids:
            self.get_logger().warn(f"Tag ID {self.tag_id} not detected")
            return False, f"Tag ID {self.tag_id} not detected"

        index = np.where(ids == self.tag_id)[0][0]
        
        obj_points = np.array([
            [-self.tag_size/2, self.tag_size/2, 0],
            [self.tag_size/2, self.tag_size/2, 0],
            [self.tag_size/2, -self.tag_size/2, 0],
            [-self.tag_size/2, -self.tag_size/2, 0]
        ], dtype=np.float32)
        
        success, rvec, tvec = cv2.solvePnP(obj_points, corners[index], self.camera_matrix, self.dist_coeffs)
        
        if not success:
            self.get_logger().warn("Failed to solve PnP")
            return False, "Failed to solve PnP"

        R_tag2cam, _ = cv2.Rodrigues(rvec)
        t_tag2cam = tvec

        # 2. Get Robot Pose
        tf_msg = self.get_transform(self.base_frame, self.eef_frame)
        if tf_msg is None:
            return False, f"Could not get transform from {self.eef_frame} to {self.base_frame}"

        trans = tf_msg.transform.translation
        rot = tf_msg.transform.rotation
        
        t_gripper2base = np.array([trans.x, trans.y, trans.z])
        quat = [rot.x, rot.y, rot.z, rot.w]
        R_gripper2base = R.from_quat(quat).as_matrix()

        self.R_gripper2base.append(R_gripper2base)
        self.t_gripper2base.append(t_gripper2base)
        self.R_target2cam.append(R_tag2cam)
        self.t_target2cam.append(t_tag2cam)

        count = len(self.R_gripper2base)
        msg = f"Sample {count} captured successfully"
        self.get_logger().info(msg)
        return True, msg

    def capture_callback(self, request, response):
        success, msg = self.capture_action()
        response.success = success
        response.message = msg
        return response

    def compute_action(self):
        if len(self.R_gripper2base) < 3:
            msg = "Not enough samples. Need at least 3."
            self.get_logger().warn(msg)
            return False, msg

        self.get_logger().info(f"Computing calibration with {len(self.R_gripper2base)} samples...")

        try:
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                self.R_gripper2base,
                self.t_gripper2base,
                self.R_target2cam,
                self.t_target2cam,
                method=cv2.CALIB_HAND_EYE_PARK
            )

            self.result_R = R_cam2base
            self.result_t = t_cam2base

            quat = R.from_matrix(R_cam2base).as_quat() # x, y, z, w
            
            result_str = f"\nCalibration Result (Camera in Base Frame):\n"
            result_str += f"Translation: x={t_cam2base[0][0]:.4f}, y={t_cam2base[1][0]:.4f}, z={t_cam2base[2][0]:.4f}\n"
            result_str += f"Rotation (xyzw): x={quat[0]:.4f}, y={quat[1]:.4f}, z={quat[2]:.4f}, w={quat[3]:.4f}\n"
            
            self.get_logger().info(result_str)
            return True, "Calibration computed successfully"

        except Exception as e:
            msg = f"Calibration failed: {str(e)}"
            self.get_logger().error(msg)
            return False, msg

    def compute_callback(self, request, response):
        success, msg = self.compute_action()
        response.success = success
        response.message = msg
        return response

    def reset_action(self):
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []
        msg = "Reset all samples."
        self.get_logger().info(msg)
        return True, msg

    def reset_callback(self, request, response):
        success, msg = self.reset_action()
        response.success = success
        response.message = msg
        return response

    def save_action(self):
        if not hasattr(self, 'result_R'):
            msg = "No calibration result to save. Run compute first."
            self.get_logger().warn(msg)
            return False, msg

        save_path = os.path.join(os.getcwd(), 'handeye_calibration_result.yaml')
        
        quat = R.from_matrix(self.result_R).as_quat()
        
        data = {
            'camera_pose': {
                'frame_id': self.base_frame,
                'child_frame_id': self.camera_frame,
                'position': {
                    'x': float(self.result_t[0][0]),
                    'y': float(self.result_t[1][0]),
                    'z': float(self.result_t[2][0])
                },
                'orientation': {
                    'x': float(quat[0]),
                    'y': float(quat[1]),
                    'z': float(quat[2]),
                    'w': float(quat[3])
                }
            }
        }
        
        with open(save_path, 'w') as f:
            yaml.dump(data, f)
            
        msg = f"Saved result to {save_path}"
        self.get_logger().info(msg)
        return True, msg

    def save_callback(self, request, response):
        success, msg = self.save_action()
        response.success = success
        response.message = msg
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
