# 可视化和点云发布模块
import struct
import numpy as np
import cv2
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
from std_msgs.msg import Header
from builtin_interfaces.msg import Time


def publish_masked_pointcloud(publisher_pool, mask: np.ndarray, cv_rgb: np.ndarray,
                              cv_depth: np.ndarray, intrinsics: dict, header,
                              child_frame_id: str = ''):
    """
    发布mask后的点云

    Args:
        publisher_pool: PublisherPool 实例，用于发布消息
        mask: 二值掩膜 (H, W)，尺寸可能与 cv_rgb/cv_depth 不一致
        cv_rgb: RGB图像 (H, W, 3)
        cv_depth: 深度图 (H, W) 毫米单位
        intrinsics: 相机内参 {'fx': float, 'fy': float, 'cx': float, 'cy': float}
        header: ROS2 消息头 (包含时间戳和frame_id)
        child_frame_id: 用于区分不同目标的 topic 后缀，如 'l'/'r'
    """
    h, w = cv_depth.shape[:2]
    if mask.shape[:2] != (h, w):
        mask = cv2.resize(mask.astype(np.uint8), (w, h), interpolation=cv2.INTER_NEAREST).astype(bool)

    # 获取掩膜的像素坐标
    ys, xs = np.where(mask)
    depths = cv_depth[ys, xs]
    
    # 过滤有效深度（> 0）
    valid_mask = depths > 0
    ys = ys[valid_mask]
    xs = xs[valid_mask]
    depths = depths[valid_mask]
    
    if len(ys) == 0:
        return
    
    # 深度转换为米
    z = depths / 1000.0
    
    # 相机内参反投影到3D坐标
    px = (xs - intrinsics['cx']) * z / intrinsics['fx']
    py = (ys - intrinsics['cy']) * z / intrinsics['fy']
    
    # 获取RGB值
    rgbs = cv_rgb[ys, xs]
    
    # RGB打包为uint32 (格式: XRGB，其中X=透明度)
    rgb_uint32 = np.array([
        struct.unpack('I', struct.pack('BBBB', b, g, r, 255))[0]
        for b, g, r in rgbs
    ], dtype=np.uint32)
    
    # 拼接点数据 (x, y, z, rgb)
    points_array = np.column_stack((px, py, z, rgb_uint32))
    points = points_array.tolist()
    
    # 构建点云字段定义
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
    ]
    
    # 转换为numpy数组
    points_np = np.array(points)
    
    # 分离XYZ和RGB通道
    xyz = points_np[:, :3].astype(np.float32)
    rgb = points_np[:, 3].astype(np.uint32)
    
    # 构建结构化数组 (符合ROS2 PointCloud2格式)
    structured = np.zeros(
        points_np.shape[0],
        dtype={'names': ['x', 'y', 'z', 'rgb'],
               'formats': ['<f4', '<f4', '<f4', '<u4']}
    )
    structured['x'] = xyz[:, 0]
    structured['y'] = xyz[:, 1]
    structured['z'] = xyz[:, 2]
    structured['rgb'] = rgb
    
    # 转换为字节流
    cloud_data = structured.tobytes()
    
    # 创建PointCloud2消息
    cloud_msg = PointCloud2()
    cloud_msg.header = header
    cloud_msg.height = 1
    cloud_msg.width = len(points)
    cloud_msg.fields = fields
    cloud_msg.is_bigendian = False
    cloud_msg.point_step = 16  # 4个float32字段，每个4字节
    cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
    cloud_msg.is_dense = True
    cloud_msg.data = cloud_data
    
    # 发布点云
    topic = f'masked_pointcloud_{child_frame_id}' if child_frame_id else 'masked_pointcloud'
    publisher_pool.publish(topic, PointCloud2, cloud_msg)


def publish_image(publisher_pool, image: np.ndarray, quality: int = 80):
    """
    发布图像为CompressedImage
    
    Args:
        publisher_pool: PublisherPool 实例，用于发布消息
        image: RGB或BGR图像 (H, W, 3)
        quality: JPEG压缩质量 (0-100)
    """
    # 转换为JPEG格式
    _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, quality])
    
    # 随便编一个header
    header = Header()
    header.frame_id = 'camera_depth_optical_frame'
    
    compressed_msg = CompressedImage()
    compressed_msg.header = header
    compressed_msg.format = 'jpeg'
    compressed_msg.data = buffer.tobytes()
    
    publisher_pool.publish('annotated_image/compressed', CompressedImage, compressed_msg)
