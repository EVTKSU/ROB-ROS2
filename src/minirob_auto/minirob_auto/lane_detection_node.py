#!/usr/bin/env python3
"""
lane_detection_node.py

1) Captures OAK-D LR frames via DepthAI.
2) Runs YOLO v8 segmentation (best.pt) to get a binary mask.
3) Computes centroid-based steering percentage in [-100, +100].
4) Publishes that as a std_msgs/Float32 on /steer_pct.
"""

import os
import cv2
import numpy as np
import depthai
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from ultralytics import YOLO  # YOLO v8 API

# PID gains for smoothing (if needed later). For pure centroid, KP alone may suffice.
_KP = 0.038
_KI = 0.0
_KD = 0.008

# Match the training resolution of best.pt (height,width)
CAM_W, CAM_H = 320, 192
INFER_HZ     = 30  # frames/sec

class LaneDetection(Node):
    def __init__(self):
        super().__init__('lane_detection')

        # Publisher: /steer_pct (Float32)
        self.pub_steer = self.create_publisher(Float32, 'steer_pct', 10)

        # PID state (optional smoothing)
        self.prev_error = 0.0
        self.integral   = 0.0

        # 1) Locate best.pt
        pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        best_pt = os.path.join(pkg_dir, '..', 'models', 'best.pt')
        best_pt = os.path.normpath(best_pt)
        if not os.path.isfile(best_pt):
            self.get_logger().error(f"Missing best.pt at: {best_pt}")
            raise FileNotFoundError(f"Expected best.pt at {best_pt}")

        self.get_logger().info(f"Loading YOLOv8 model from: {best_pt}")
        self.model = YOLO(best_pt)

        # 2) DepthAI pipeline for OAK-D LR
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

        # 3) Timer to run at INFER_HZ
        self.create_timer(1.0 / INFER_HZ, self.timer_callback)
        self.get_logger().info("lane_detection node initialized.")

    def timer_callback(self):
        in_cam = self.q_cam.get()                    # depthai.ImgFrame
        frame_bgr = in_cam.getCvFrame()              # shape: (CAM_H, CAM_W, 3)

        # Convert to RGB for YOLO
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # YOLOv8 segmentation
        results = self.model(frame_rgb, imgsz=(CAM_H, CAM_W))
        res0 = results[0]

        # Build binary mask (H × W)
        if res0.masks is not None and res0.masks.data.shape[0] > 0:
            masks_data = res0.masks.data.cpu().numpy()  # shape: (n, H, W)
            combined  = (np.sum(masks_data, axis=0) >= 0.5).astype(np.uint8) * 255
            mask = combined
        else:
            mask = np.zeros((CAM_H, CAM_W), dtype=np.uint8)

        # (Optional) Visualization
        overlay = cv2.addWeighted(frame_bgr, 0.7,
                                   cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR),
                                   0.3, 0)
        cv2.imshow("Lane Detection – Mask Overlay", overlay)
        cv2.waitKey(1)

        # Compute centroid on bottom half → steering error
        bottom = mask[CAM_H // 2 :, :]
        M = cv2.moments(bottom)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
        else:
            cx = CAM_W // 2

        error = (CAM_W / 2) - cx
        # (Optional) PID smoothing
        self.integral   += error
        derivative      = error - self.prev_error
        steering_corr   = _KP * error + _KI * self.integral + _KD * derivative
        self.prev_error = error

        # Normalize to [-1.0, +1.0], then to steer_pct [-100, +100]
        steering_frac = max(min(steering_corr, 1.0), -1.0)
        steer_pct     = float(steering_frac * 100.0)

        # Publish
        msg = Float32()
        msg.data = steer_pct
        self.pub_steer.publish(msg)
        self.get_logger().info(f"Published /steer_pct: {steer_pct:.2f}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        self.device.close()
        super().destroy_node()

def main(args=None):
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
