import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class YoloTestNode(Node):
    def __init__(self):
        super().__init__('yolo_test_node')
        
        # Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('input_topic', '/camera/camera/color/image_raw')
        
        # Get model path
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        if not model_path:
            try:
                pkg_share = get_package_share_directory('energy_unit_locator')
                model_path = os.path.join(pkg_share, 'assets', '251120_epoch335.pt')
            except Exception:
                # Fallback for local testing if package not installed
                model_path = 'assets/251120_epoch335.pt'
            
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None

        self.conf_thresh = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        self.pub = self.create_publisher(Image, '/yolo_test', 10)
        self.get_logger().info(f'Yolo Test Node Started. Subscribing to {input_topic}')

    def image_callback(self, msg):
        if self.model is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # Inference
        # verbose=False prevents printing to stdout
        results = self.model(cv_img, verbose=False, conf=self.conf_thresh)
        
        # Annotate
        # results[0].plot() returns the annotated image as a numpy array (BGR)
        annotated_frame = results[0].plot()

        # Publish
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
