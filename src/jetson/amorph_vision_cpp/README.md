# amorph_vision_cpp

能量单元 3D 定位节点（C++ / TensorRT）。替代 Python 版 `amorph_vision`（后者保留可回退）。

管线：`RGB + Depth` → YOLO 实例分割（TensorRT-YOLO）→ PCA 粗定位 → 降自由度 ICP 精配准 → 3D SORT 跟踪 → TF + Foxglove 标注。

## 依赖

| 依赖 | 说明 |
| --- | --- |
| CUDA + TensorRT | 硬依赖。Jetson Orin NX：JetPack 6.x（TensorRT 10.3） |
| OpenCV | `cv_bridge` 已依赖 |
| Eigen3 | 位姿、卡尔曼、最小二乘 |
| PCL | PLY 读取、体素降采样、法向估计、kd-tree |
| [TensorRT-YOLO](libs/TensorRT-YOLO) | 子模块，`v6.4.0`。只使用其源码 |

TensorRT 路径由 `CMakeLists.txt` 自己 `find_path` / `find_library` 探测，**不需要**任何符号链接或环境变量。
若 TensorRT 装在非标准位置：

```bash
colcon build --packages-select amorph_vision_cpp --cmake-args -DTRT_PATH=/opt/tensorrt
```

`-DTRT_PATH` 指向含 `include/` 与 `lib/` 的目录；留空则自动探测
`/usr/include/<triplet>`、`/usr/lib/<triplet>`、`/usr/local` 等常见布局。

CUDA 架构默认 `87;89`（Jetson Orin + 桌面 Ada）。只在一台机器上构建时收窄可以显著加快编译：

```bash
colcon build --packages-select amorph_vision_cpp --cmake-args -DAMORPH_CUDA_ARCHITECTURES=87
```

## 生成 engine

engine 与 TensorRT 版本、GPU 架构强绑定，**不进仓库**，必须在运行它的那台机器上生成。

1. 导出 trtyolo 版 ONNX（任意一台有 ultralytics 的机器，产物走 LFS 提交）：

   ```bash
   pip install 'trtyolo-export<2.0'
   .venv/bin/python scripts/export_eu_engine.py
   # 产出 src/jetson/amorph_vision_cpp/resource/best-trtyolo.onnx
   ```

   > 导出器版本必须是 **1.x**。`trtyolo-export 2.0.1` 导出的图与 TensorRT-YOLO 6.4 不兼容
   > （mask 系数 Gather 维度错误，TensorRT 解析时会在 `MatMul` 处报维度不匹配），脚本会直接拒绝。
   > 脚本还会校验输出张量为 `num_dets / det_boxes / det_scores / det_classes / det_masks`
   > 且含 `ROIAlign_TRT` 插件 —— 这正是 `postProcessSegment` 期望的布局。

2. 在目标机上构建 engine：

   ```bash
   scripts/build_eu_engine.sh
   # 产出 src/jetson/amorph_vision_cpp/resource/best.engine
   ```

   脚本会把 `build/amorph_vision_cpp/libcustom_plugins.so` 同时作为
   `--staticPlugins` 与 `--setPluginsToSerialize` 传给 `trtexec`。Segment 模型的
   mask 插件必须序列化进 engine，否则反序列化时会报找不到插件。

   **精度**：TensorRT 10（JetPack 6.x）上脚本会传 `--fp16`；TensorRT 11 已移除该选项
   （改为 strongly-typed，精度由图决定），脚本会自动探测并跳过，此时构建的是 FP32 engine。

> 注意：engine 内部的置信度阈值在导出时固化（默认 0.25）。运行参数
> `conf_threshold`（默认 0.85）是在此之上的二次过滤，不能低于 engine 内的值。
>
> 另注：`ROIAlign_TRT` 输出的掩膜**已按检测框裁剪**（bbox 局部 160×160），
> 因此掩膜是「resize 到框尺寸 + 按框左上角偏移粘贴」回原图的，而不是整图缩放。

## 运行

默认参数即指向包内的 engine 与 PLY，直接启动即可：

```bash
ros2 run amorph_vision_cpp amorph_vision_cpp_node --ros-args -p log_timing:=true
```

或通过 bringup：

```bash
ros2 launch amorph_bringup eu_locator_cpp.launch.py
```

需要换 engine 时用绝对路径覆盖（相对路径以进程 CWD 为基准，容易踩坑）：

```bash
ros2 run amorph_vision_cpp amorph_vision_cpp_node --ros-args \
    -p engine_path:=/abs/path/to/best.engine
```

> `engine_path` / `ply_path` 留空（launch 的默认值就是空串）时回退到
> `<share>/amorph_vision_cpp/resource/` 下的同名文件。

## 参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `engine_path` | `<share>/resource/best.engine` | TensorRT engine；**留空即用包内默认** |
| `ply_path` | `<share>/resource/energy_unit.ply` | 能量单元网格模型；留空即用包内默认 |
| `model_input_width` / `model_input_height` | `640` / `640` | engine 静态输入尺寸，用于把掩膜从 letterbox 空间映射回原图 |
| `conf_threshold` | `0.85` | 置信度阈值 |
| `erosion_kernel_size` / `erosion_iterations` | `5` / `1` | 掩膜腐蚀，去除边缘噪声 |
| `track_max_age` | `15` | 轨迹丢失后保留帧数 |
| `track_min_hits` | `3` | 确认发布前所需命中帧数 |
| `track_mahalanobis_thresh` | `15.0` | 关联的 3D 马氏距离门限 |
| `track_iou_weight` | `0.4` | 关联代价里 2D IoU 的权重 |
| `log_timing` | `false` | 打印单帧 YOLO / PCA+ICP 耗时 |
| `class_names` | `["energy_unit"]` | 标注用类别名 |
| `rgb_topic` / `depth_topic` / `camera_info_topic` | `/camera/color/image_raw` / `/camera/depth/image_raw` / `/camera/color/camera_info` | 输入话题 |

## 输出

- TF：`<camera_frame> -> energy_unit_<track_id>`
- `yolo_annotation`：`foxglove_msgs/ImageAnnotations`，叠加掩膜轮廓、检测框与 `#track_id` 标签

## 与 Python 版的差异

- 推理后端从 ultralytics(PyTorch) 换成 TensorRT-YOLO；置信度过滤移到 C++ 侧。
- 模型采样从 open3d 的 Poisson Disk 改为 PLY 三角面面积加权采样（固定种子），
  不再依赖 open3d；因此 ICP 的源点分布与 Python 版不完全相同。
- 去掉了逐目标的 `masked_pointcloud_<id>` 话题（ICP 也用不到 RGB），只保留 TF 与标注。
- 图像订阅使用 `SensorDataQoS`，兼容相机驱动的 RELIABLE / BEST_EFFORT 两种 QoS。

## 测试

```bash
colcon build --packages-select amorph_vision_cpp --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select amorph_vision_cpp
```

> 若本机 `.bashrc` 把 `/opt/MVS/lib/64` 加进了 `LD_LIBRARY_PATH`（海康 MVS SDK），
> 那里的旧 `libusb-1.0.so.0` 缺少 `libusb_set_option`，会让任何链接了
> `libpcl_io` 的可执行文件启动失败。测试前先剔除该路径：
>
> ```bash
> export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v /opt/MVS | paste -sd:)
> ```

## 第三方许可证

`libs/TensorRT-YOLO` 采用 GPL-3.0。本包以源码方式集成并链接它；若将来以二进制形式对外分发整车软件，需履行 GPL-3.0 义务。
