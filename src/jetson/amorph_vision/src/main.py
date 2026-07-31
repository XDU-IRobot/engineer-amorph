from ultralytics import YOLO

import numpy as np
from time import time

from amorph_vision.cam import Camera
from amorph_vision.pose_estimator import PoseEstimator
from amorph_vision.visualization import FoxgloveServer

import foxglove

# from viztracer import VizTracer


def main():
    cam = Camera()
    cam.open()

    K = cam.get_k()
    intrinsics = {'fx': K[0], 'fy': K[4], 'cx': K[2], 'cy': K[5]}
    estimator = PoseEstimator("assets/energy_unit.ply", intrinsics)

    model = YOLO("assets/best.pt")

    vis = FoxgloveServer()
    vis.start()

    try:
        while True:
            vis.log("SB", foxglove.messages.LogLevel.Error)
            t0 = time()
            frame = cam.wait_for_frame(1000)
            if frame is None:
                continue

            t1 = time()
            results = model(frame.color, conf=0.6, verbose=False)
            
            t2 = time()
            vis.publish_frame(frame)
            vis.publish_yolo_result(results)
            # vis.publish_point_cloud(frame)

            for r in results:
                if r.boxes is None or r.masks is None:
                    continue
                for i in range(len(r.boxes)):
                    t3 = time()
                    cls_id = int(r.boxes.cls[i].item())
                    raw_mask = r.masks.data[i].cpu().numpy().astype(np.uint8)
                    box = r.boxes.xyxy[i].tolist(
                    ) + [float(r.boxes.conf[i].item()), float(cls_id)]

                    pose = estimator.estimate(
                        frame.color, frame.depth, raw_mask, box)
                    if pose is None:
                        continue

                    t4 = time()
                    child = f"energy_unit_{i}"
                    vis.publish_tf("camera_optical_frame", child,
                                   tuple(pose['position']), tuple(pose['orientation']))
                    refined_pts = estimator.sampler.sample_visible(pose)
                    vis.publish_point_cloud_data(refined_pts, frame_id=f"refined_{i}",
                                                 pcl_name="refined")

                    t5 = time()
                    print(f"Frame {i}: YOLO {t2-t1:.3f}s, PCA+ICP {t4-t3:.3f}s, Viz {t5-t4:.3f}s, Total {t5-t0:.3f}s")
    finally:
        vis.stop()
        cam.close()


if __name__ == "__main__":
    # tracer = VizTracer()
    # tracer.start()
    main()
    # tracer.stop()
    # tracer.save()
