import struct
import numpy as np
import cv2
from foxglove_msgs.msg import (
    Color, ImageAnnotations, Point2, PointsAnnotation, TextAnnotation)
from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
from std_msgs.msg import Header


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


def publish_yolo_annotation(publisher_pool, results, header,
                            topic: str = 'yolo_annotation',
                            draw_masks: bool = True,
                            draw_boxes: bool = True,
                            draw_labels: bool = True,
                            draw_track_id: bool = True,
                            track_ids=None,
                            mask_outline_color=(0.0, 1.0, 0.0, 1.0),
                            mask_fill_color=(0.0, 1.0, 0.0, 0.25),
                            box_color=(1.0, 0.5, 0.0, 1.0),
                            text_color=(1.0, 1.0, 1.0, 1.0),
                            text_background_color=(0.0, 0.0, 0.0, 0.7),
                            line_thickness: float = 2.0,
                            font_size: float = 12.0):
    """
    把 YOLO 推理结果发布为 foxglove_msgs/ImageAnnotations

    Foxglove 依靠时间戳把标注叠加到对应帧的图像上，所以 messages 的 timestamp
    必须与图像头的时间戳一致（不从 header 的 frame_id 读取）。

    Args:
        publisher_pool: PublisherPool 实例，用于发布消息
        results: YOLO 模型 __call__ 的返回值（List[ultralytics.engine.results.Results]）
        header: 对应图像的 ROS2 消息头，用其中的 stamp
        topic: 发布的话题名
        draw_masks: 是否用分割轮廓画 mask
        draw_boxes: 是否画检测框
        draw_labels: 是否画类别与置信度文本
        draw_track_id: 是否在文本里显示 track id
        track_ids: 与检测一一对应的 track id 列表 (按 results/boxes 顺序展开),
            为 None 时使用 results 里自带的 boxes.id
        mask_outline_color: mask 轮廓 RGBA
        mask_fill_color: mask 填充 RGBA，alpha=0 为不填充
        box_color: 检测框 RGBA
        text_color: 文本 RGBA
        text_background_color: 文本背景 RGBA
        line_thickness: 轮廓与框的线宽，像素
        font_size: 文本字号，像素
    """
    def make_color(values):
        return Color(r=float(values[0]), g=float(values[1]),
                     b=float(values[2]), a=float(values[3]))

    def make_polygon(points, outline_color, fill_color):
        annotation = PointsAnnotation()
        annotation.timestamp = header.stamp
        annotation.type = PointsAnnotation.LINE_LOOP
        annotation.points = [Point2(x=float(x), y=float(y)) for x, y in points]
        annotation.outline_color = make_color(outline_color)
        annotation.fill_color = make_color(fill_color)
        annotation.thickness = line_thickness
        return annotation

    msg = ImageAnnotations()
    # 设置后，单个标注自身的 timestamp 会被忽略
    msg.timestamp = header.stamp

    # track_ids 按检测的扁平顺序消费，因此可以跨多个 Results 正确定位
    track_id_iter = iter(track_ids) if track_ids is not None else None

    for result in results:
        boxes = result.boxes
        if boxes is None or len(boxes) == 0:
            continue

        names = result.names or {}
        # masks.xy 是每个目标的轮廓多边形，坐标为像素
        polygons = result.masks.xy if result.masks is not None else None

        for i in range(len(boxes)):
            x1, y1, x2, y2 = (float(v) for v in boxes.xyxy[i].tolist())

            track_id = None
            if track_id_iter is not None:
                track_id = next(track_id_iter, None)
            elif boxes.id is not None:
                track_id = int(boxes.id[i].item())

            if draw_masks and polygons is not None and i < len(polygons):
                if len(polygons[i]) >= 3:
                    msg.points.append(make_polygon(
                        polygons[i], mask_outline_color, mask_fill_color))

            if draw_boxes:
                msg.points.append(make_polygon(
                    [(x1, y1), (x2, y1), (x2, y2), (x1, y2)],
                    box_color, (0.0, 0.0, 0.0, 0.0)))

            if draw_labels:
                cls_id = int(boxes.cls[i].item())
                confidence = float(boxes.conf[i].item())
                label = names.get(cls_id, str(cls_id))

                # 便于在 foxglove 里确认跨帧是否同一目标
                if draw_track_id and track_id is not None:
                    label = f'#{track_id} {label}'

                text = TextAnnotation()
                text.timestamp = header.stamp
                text.position = Point2(x=x1, y=max(y1 - font_size, 0.0))
                text.text = f'{label} {confidence:.2f}'
                text.font_size = font_size
                text.text_color = make_color(text_color)
                text.background_color = make_color(text_background_color)
                msg.texts.append(text)

    publisher_pool.publish(topic, ImageAnnotations, msg)
