#!/usr/bin/env python3
"""
video_recorder_node.py

1) Captures frames from OAK-D LR via DepthAI.
2) Publishes each frame as sensor_msgs/Image on /video_frames.
3) Saves raw frames to a video file (raw_output.avi).
"""

import os
import cv2
import depthai
import rclpy
import numpy as np

from datetime import datetime

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Desired resolution (match your YOLO model or requirements)
CAM_W, CAM_H = 854, 480
FRAME_RATE = 30 # FPS for recording

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')

        # Publisher: /video_frames (sensor_msgs/Image)
        self.pub_frame = self.create_publisher(Image, 'video_frames', 10)
        self.bridge = CvBridge()

        # Get the current date
        self.now: datetime = datetime.now()
        self.formatted_time: str = self.now.strftime('%m_%d-%H:%M:%S')

        # Set up DepthAI pipeline
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

        # Set up OpenCV VideoWriter for raw video
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out_path = os.path.join(os.path.expanduser('~'), f'raw_output_{self.formatted_time}.avi')
        self.video_writer = cv2.VideoWriter(out_path, fourcc, FRAME_RATE, (CAM_W, CAM_H))
        self.get_logger().info(f"Recording raw video to: {out_path}")

        # Timer to grab frames at ~FRAME_RATE
        self.create_timer(1.0 / FRAME_RATE, self.timer_callback)
        self.get_logger().info("video_recorder node initialized.")

    def timer_callback(self):
        # Get latest frame from OAK-D LR
        in_cam = self.q_cam.get()  # depthai.ImgFrame
        frame_bgr = in_cam.getCvFrame()  # shape: (CAM_H, CAM_W, 3)

        # Publish as ROS Image
        ros_img = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        self.pub_frame.publish(ros_img)

        # Write to raw video file
        self.video_writer.write(frame_bgr)

        # (Optional) show a tiny preview
        cv2.imshow("Raw OAK-D Preview", frame_bgr)
        cv2.waitKey(1)

    def destroy_node(self):
        # Clean up
        cv2.destroyAllWindows()
        self.video_writer.release()
        self.device.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
