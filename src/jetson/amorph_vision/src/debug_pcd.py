"""单帧点云可视化: 采集一帧 → YOLO+PCA → 导出 PLY → Colored ICP.

用法: 在 WSL2 中运行后查看终端输出; PLY 文件可拖入任意 3D 查看器.
"""

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from ultralytics import YOLO

from amorph_vision.cam import Camera
from amorph_vision.icp_refine import ICPSampler, ICPRefiner, extract_masked_points
from amorph_vision.pca_localization import PCALocator


def pcd_stats(name, pcd: o3d.geometry.PointCloud):
    pts = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors) if pcd.has_colors() else None
    print(f"[{name}] {len(pts)} pts "
          f"x=[{pts[:,0].min():.4f},{pts[:,0].max():.4f}] "
          f"y=[{pts[:,1].min():.4f},{pts[:,1].max():.4f}] "
          f"z=[{pts[:,2].min():.4f},{pts[:,2].max():.4f}]")
    if colors is not None:
        print(f"     color r=[{colors[:,0].min():.3f},{colors[:,0].max():.3f}] "
              f"g=[{colors[:,1].min():.3f},{colors[:,1].max():.3f}] "
              f"b=[{colors[:,2].min():.3f},{colors[:,2].max():.3f}]")


def main():
    cam = Camera()
    cam.open()

    K = cam.get_k()
    intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}
    locator = PCALocator(intrinsics)
    sampler = ICPSampler("energy_unit.ply", intrinsics)
    refiner = ICPRefiner()

    model = YOLO("best.pt")

    print("等待一帧...")
    frame = cam.wait_for_frame(5000)
    if frame is None:
        print("未收到帧")
        cam.close()
        return

    results = model(frame.color, conf=0.6, verbose=False)

    for r in results:
        if r.boxes is None or r.masks is None:
            continue
        for i in range(len(r.boxes)):
            cls_id = int(r.boxes.cls[i].item())
            raw_mask = r.masks.data[i].cpu().numpy().astype(np.uint8)
            h, w = frame.depth.shape[:2]
            if raw_mask.shape[:2] != (h, w):
                raw_mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            mask = raw_mask.astype(bool)
            box = r.boxes.xyxy[i].tolist() + [float(r.boxes.conf[i].item()), float(cls_id)]
            pose = locator.process_mask(mask, np.array(box), frame.depth)
            if pose is None:
                print("PCA 定位失败")
                continue

            target_pts = extract_masked_points(
                frame.depth, frame.color, pose['binary_mask'], intrinsics)
            source_pts = sampler.sample_visible(pose, to_camera=True)

            source = o3d.geometry.PointCloud()
            source.points = o3d.utility.Vector3dVector(source_pts[:, :3].astype(np.float64))
            if source_pts.shape[1] >= 6 and source_pts[:, 3:6].max() > 1.1:
                source.colors = o3d.utility.Vector3dVector(source_pts[:, 3:6].astype(np.float64) / 255.0)

            target = o3d.geometry.PointCloud()
            target.points = o3d.utility.Vector3dVector(target_pts[:, :3].astype(np.float64))
            if target_pts.shape[1] >= 6 and target_pts[:, 3:6].max() > 1.1:
                target.colors = o3d.utility.Vector3dVector(target_pts[:, 3:6].astype(np.float64) / 255.0)

            print("\n=== PCA 初始位姿 ===")
            print(f"position: {pose['position']}")
            print(f"orientation: {pose['orientation']}")

            pcd_stats("source (模型)", source)
            pcd_stats("target (场景)", target)

            # 导出 PLY
            o3d.io.write_point_cloud("debug_source.ply", source)
            o3d.io.write_point_cloud("debug_target.ply", target)
            print("PLY 已保存: debug_source.ply, debug_target.ply")

            # ---- Colored ICP 多尺度配准 (open3d 例程风格) ----
            print("\n=== Colored ICP 多尺度配准 ===")
            voxel_radius = [0.04, 0.02, 0.01]
            max_iter = [50, 30, 14]

            R_init = Rotation.from_quat(pose['orientation']).as_matrix()
            t_init = np.array(pose['position'], dtype=np.float64)
            current_transformation = np.eye(4)
            current_transformation[:3, :3] = R_init
            current_transformation[:3, 3] = t_init

            for scale in range(3):
                radius = voxel_radius[scale]
                iters = max_iter[scale]
                print(f"\n--- scale={scale} voxel={radius:.2f} max_iter={iters} ---")

                source_down = source.voxel_down_sample(radius)
                target_down = target.voxel_down_sample(radius)
                print(f"down-sample: source {len(source_down.points)} pts, target {len(target_down.points)} pts")

                source_down.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
                target_down.estimate_normals(
                    o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

                result = o3d.pipelines.registration.registration_colored_icp(
                    source_down, target_down, radius, current_transformation,
                    o3d.pipelines.registration.TransformationEstimationForColoredICP(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(
                        relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=iters))
                current_transformation = result.transformation
                print(f"fitness={result.fitness:.4f} inlier_rmse={result.inlier_rmse:.4f} "
                      f"correspondences={len(result.correspondence_set)}")

            refined_R = current_transformation[:3, :3]
            refined_t = current_transformation[:3, 3]
            refined_quat = Rotation.from_matrix(refined_R).as_quat()
            print(f"\n=== 精化位姿 ===")
            print(f"position: {refined_t.tolist()}")
            print(f"orientation: {refined_quat.tolist()}")

            # 导出精化后的 source
            source_refined = o3d.geometry.PointCloud(source)
            source_refined.transform(current_transformation)
            o3d.io.write_point_cloud("debug_source_refined.ply", source_refined)
            print("PLY 已保存: debug_source_refined.ply")

            break

    cam.close()


if __name__ == "__main__":
    main()
