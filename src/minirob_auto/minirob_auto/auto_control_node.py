#!/usr/bin/env python3
"""
lane_detection_node.py

Adapted directly from Triton AI’s DSC190 lane_detection.py (track-detection-DSC190/src/lane_detection) 
to run as a ROS 2 Jazzy node. This node:

1) Captures frames from Oak-D LR via DepthAI.
2) Runs YOLO v8 segmentation (best.pt instance‐segmentation) to produce a binary mask.
3) Computes centroid‐based steering percentage in [-100, +100] with PID smoothing.
4) Publishes steering_pct (std_msgs/Float32) on /steer_pct.

Dependencies (install via “python3 -m pip install --user”):
    depthai, ultralytics, opencv-python, numpy
Model weights “best.pt” should sit in:
    ~/ros2_ws/src/minirob_auto/models/best.pt
"""

import os
import cv2
import numpy as np
import depthai
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32
from ultralytics import YOLO  # YOLO v8 API

# PID gains, identical to Triton AI defaults :contentReference[oaicite:11]{index=11}
_KP = 0.038
_KI = 0.0
_KD = 0.008

# Camera resolution (height, width) matching Triton’s 320×192 pre‑processed input :contentReference[oaicite:12]{index=12}
CAM_W, CAM_H = 320, 192
INFER_HZ     = 30  # Attempt 30 FPS as Triton did :contentReference[oaicite:13]{index=13}

class LaneDetection(Node):
    def __init__(self):
        """
        Initialize the ROS 2 node ‘lane_detection’:
          - Sets up a publisher for /steer_pct (Float32).
          - Loads YOLOv8 model from “best.pt” under models/.
          - Builds DepthAI pipeline to capture CV frames from Oak-D LR.
          - Starts a timer to do inference at ~INFER_HZ.
        """
        super().__init__('lane_detection')

        # 1) Publisher for steering percentage in [-100, +100]
        self.pub_steer = self.create_publisher(Float32, 'steer_pct', 10)

        # 2) PID state variables
        self.prev_error = 0.0
        self.integral   = 0.0

        # 3) Locate the “best.pt” YOLOv8 weights exactly as Triton AI expects :contentReference[oaicite:14]{index=14}
        pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        # pkg_dir == ~/ros2_ws/src/minirob_auto/minirob_auto
        best_pt = os.path.join(pkg_dir, '..', 'models', 'best.pt')
        best_pt = os.path.normpath(best_pt)

        if not os.path.isfile(best_pt):
            self.get_logger().error(f"Missing best.pt at: {best_pt}")
            raise FileNotFoundError(f"Expected YOLOv8 weights at {best_pt}")

        self.get_logger().info(f"Loading YOLOv8 model from: {best_pt}")
        self.model = YOLO(best_pt)  # identical call to Triton AI’s YOLO init :contentReference[oaicite:15]{index=15}

        # 4) Build DepthAI pipeline for Oak-D LR :contentReference[oaicite:16]{index=16}
        pipeline = depthai.Pipeline()
        cam = pipeline.createColorCamera()
        cam.setBoardSocket(depthai.CameraBoardSocket.RGB)
        cam.setPreviewSize(CAM_W, CAM_H)
        cam.setInterleaved(False)

        xout = pipeline.createXLinkOut()
        xout.setStreamName('cam')
        cam.preview.link(xout.input)

        self.device = depthai.Device(pipeline)
        self.q_cam  = self.device.getOutputQueue(name='cam', maxSize=4, blocking=False)

        # 5) Start a timer for inference at ~INFER_HZ Hz :contentReference[oaicite:17]{index=17}
        self.create_timer(1.0 / INFER_HZ, self.timer_callback)
        self.get_logger().info("lane_detection node initialized and running at ~30 FPS.")

    def timer_callback(self):
        """
        Called at ~INFER_HZ. Steps:
          a) Grab a frame (BGR) from Oak-D LR.
          b) Convert BGR→RGB and run self.model(...) for segmentation.
          c) Build a combined mask; compute centroid on the bottom half → error.
          d) Apply PID: steering_corr = KP*error + KI*integral + KD*derivative.
          e) Normalize to [-1.0, +1.0] → steer_pct in [-100, +100].
          f) Publish steer_pct on /steer_pct as Float32.
          g) (Optional) Visualize overlay exactly as Triton AI’s demo. :contentReference[oaicite:18]{index=18}
        """
        # a) Get frame from Oak-D LR
        in_cam = self.q_cam.get()             # depthai.ImgFrame
        frame_bgr = in_cam.getCvFrame()       # shape: (CAM_H, CAM_W, 3)

        # b) Convert to RGB and run YOLOv8 segmentation inference
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        results = self.model(frame_rgb, imgsz=(CAM_H, CAM_W))  # identical API call :contentReference[oaicite:19]{index=19}
        res0 = results[0]

        # c) Build binary mask from instance masks
        if res0.masks is not None and res0.masks.data.shape[0] > 0:
            masks_data = res0.masks.data.cpu().numpy()  # shape (n, H, W), values ∈{0,1}
            combined   = (np.sum(masks_data, axis=0) >= 0.5).astype(np.uint8) * 255
            mask       = combined
        else:
            mask = np.zeros((CAM_H, CAM_W), dtype=np.uint8)

        # d) Overlay (Optional visualization exactly as Triton AI did) :contentReference[oaicite:20]{index=20}
        overlay = cv2.addWeighted(frame_bgr, 0.7,
                                   cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                                   0.3, 0)
        cv2.imshow("Lane Detection – Mask Overlay", overlay)
        cv2.waitKey(1)

        # e) Compute centroid x on bottom half → error
        bottom = mask[CAM_H // 2 :, :]
        M = cv2.moments(bottom)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
        else:
            cx = CAM_W // 2

        error = (CAM_W / 2) - cx
        self.integral   += error
        derivative      = error - self.prev_error
        steering_corr   = _KP * error + _KI * self.integral + _KD * derivative
        self.prev_error = error

        # f) Normalize to [-1.0, +1.0], then to steer_pct [-100, +100]
        steering_frac = max(min(steering_corr, 1.0), -1.0)
        steer_pct     = float(steering_frac * 100.0)

        # g) Publish on /steer_pct
        msg = Float32()
        msg.data = steer_pct
        self.pub_steer.publish(msg)
        self.get_logger().info(f"Published /steer_pct: {steer_pct:.2f} %")

    def destroy_node(self):
        """
        Cleanup: close OpenCV windows, release DepthAI device, then call super.
        """
        cv2.destroyAllWindows()
        self.device.close()
        super().destroy_node()

def main(args=None):
    """
    Main entrypoint: initialize ROS, spin the node until shutdown, then cleanup.
    """
    rclpy.init(args=args)
    node = LaneDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
