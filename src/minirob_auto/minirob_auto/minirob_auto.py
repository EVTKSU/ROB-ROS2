#!/usr/bin/env python3
"""
minirob_auto_node.py

Self-contained ROS 2 Jazzy node for autonomous control:

1) Captures frames from OAK-D LR via DepthAI.
2) Runs YOLO v8 segmentation (best.pt) to generate a binary mask.
3) Computes centroid-based steering in [-100, +100].
4) Uses a throttle% parameter (0–100).
5) Maps steering% and throttle% to “raw” values:
     steer_raw = ((steer_percent + 100) * 7.365) + 350
     thr_raw   = (throttle_percent * 6.75) + 350
6) Publishes a String “thr_raw,steer_raw,0” to `/auto_commands`.

Dependencies (pip‐install via `python3 -m pip install --user`):
    depthai, ultralytics, opencv-python, numpy
"""

import os
import cv2
import numpy as np
import depthai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ultralytics import YOLO  # YOLO v8 API

# PID gains for steering correction
_KP = 0.038
_KI = 0.0
_KD = 0.008

# Camera resolution (must match model training)
CAM_W, CAM_H = 320, 192
INFER_HZ     = 30

class MiniRobAuto(Node):
    def __init__(self):
        super().__init__('minirob_auto')

        # Publisher: /auto_commands (String: "thr_raw,steer_raw,emergency_flag")
        self.publisher_ = self.create_publisher(String, 'auto_commands', 10)

        # PID state variables
        self.prev_error = 0.0
        self.integral   = 0.0

        # Declare throttle_percent parameter (0–100)
        self.declare_parameter('throttle_percent', 50.0)

        # 1) Locate best.pt inside our models folder
        #    __file__ is .../minirob_auto/minirob_auto/minirob_auto_node.py
        pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        # pkg_dir == ~/ros2_ws/src/minirob_auto/minirob_auto
        best_pt = os.path.join(pkg_dir, '..', 'models', 'best.pt')
        best_pt = os.path.normpath(best_pt)

        if not os.path.isfile(best_pt):
            self.get_logger().error(f"Missing YOLOv8 weights at: {best_pt}")
            raise FileNotFoundError(f"Expected best.pt at {best_pt}")

        self.get_logger().info(f"Loading YOLOv8 model from: {best_pt}")
        self.model = YOLO(best_pt)

        # 2) Build DepthAI pipeline for OAK-D LR
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

        # 3) Timer at ~INFER_HZ to run inference
        self.create_timer(1.0 / INFER_HZ, self.timer_callback)
        self.get_logger().info("MiniRobAuto node initialized.")

    def timer_callback(self):
        # 1) Get latest frame from OAK-D LR
        in_cam = self.q_cam.get()                    # depthai.ImgFrame
        frame_bgr = in_cam.getCvFrame()              # shape: (CAM_H, CAM_W, 3)

        # 2) Convert BGR→RGB for YOLO inference
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # 3) Run YOLOv8 segmentation (imgsizes in (H, W) order)
        results = self.model(frame_rgb, imgsz=(CAM_H, CAM_W))
        res0 = results[0]

        # 4) Build a binary mask from instance masks
        if res0.masks is not None and res0.masks.data.shape[0] > 0:
            masks_data = res0.masks.data.cpu().numpy()  # shape (n, H, W)
            combined  = (np.sum(masks_data, axis=0) >= 0.5).astype(np.uint8) * 255
            mask = combined
        else:
            mask = np.zeros((CAM_H, CAM_W), dtype=np.uint8)

        # 5) (Optional) Visualize: overlay mask onto BGR frame
        overlay = cv2.addWeighted(frame_bgr, 0.7,
                                   cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                                   0.3, 0)
        cv2.imshow("MiniRobAuto – Mask Overlay", overlay)
        cv2.waitKey(1)

        # 6) Compute centroid x on bottom half of mask → steering error
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

        # 7) Normalize steering_corr to [-1.0, +1.0], then to steer_pct [-100, +100]
        steering_frac = max(min(steering_corr, 1.0), -1.0)
        steer_pct     = steering_frac * 100.0

        # 8) Get throttle_pct from ROS parameter (0–100)
        throttle_pct = self.get_parameter('throttle_percent').get_parameter_value().double_value
        throttle_pct = max(min(throttle_pct, 100.0), 0.0)

        # 9) Map percentages to raw values:
        #    steer_raw = ((steer_pct + 100) * 7.365) + 350
        #    thr_raw   = (throttle_pct * 6.75) + 350
        steer_raw = ((steer_pct + 100.0) * 7.365) + 350.0
        thr_raw   = (throttle_pct        * 6.75)  + 350.0

        # 10) Publish “thr_raw,steer_raw,0” (emergency_flag=0 for now)
        msg = String()
        msg.data = f"{int(thr_raw)},{int(steer_raw)},{0}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"CMD → {msg.data}")

    def destroy_node(self):
        # Clean up
        cv2.destroyAllWindows()
        self.device.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MiniRobAuto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
