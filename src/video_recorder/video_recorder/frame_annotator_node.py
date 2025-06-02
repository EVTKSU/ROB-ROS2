#!/usr/bin/env python3
"""
frame_annotator_node.py

1) Subscribes to /video_frames (sensor_msgs/Image) and /telemetry (std_msgs/String).
2) Maintains the latest telemetry (steer, throttle) values.
3) Overlays the telemetry text onto each incoming frame.
4) Writes annotated frames to a new video file (annotated_output.avi).
"""

import os
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Same resolution & framerate as recorder
CAM_W, CAM_H = 320, 192
FRAME_RATE = 30  # fps

class FrameAnnotator(Node):
    def __init__(self):
        super().__init__('frame_annotator')

        # Latest telemetry string (initialize to zeros)
        self.latest_telemetry = "0.00,0.00,0"  # "throttle,steer,emergency"

        # Subscriber: /telemetry (from Teensy, format "throttle,steer,flag")
        self.sub_tele = self.create_subscription(
            String, 'telemetry', self.telemetry_callback, 10)

        # Subscriber: /video_frames
        self.sub_frame = self.create_subscription(
            Image, 'video_frames', self.frame_callback, 10)

        self.bridge = CvBridge()

        # Set up VideoWriter for annotated video
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out_path = os.path.join(os.path.expanduser('~'), 'annotated_output.avi')
        self.video_writer = cv2.VideoWriter(out_path, fourcc, FRAME_RATE, (CAM_W, CAM_H))
        self.get_logger().info(f"Recording annotated video to: {out_path}")

        self.get_logger().info("frame_annotator node initialized.")

    def telemetry_callback(self, msg: String):
        # Store the latest telemetry (string format: "throttle,steer,flag")
        self.latest_telemetry = msg.data
        # Optionally log it:
        self.get_logger().debug(f"Telemetry updated: {self.latest_telemetry}")

    def frame_callback(self, msg: Image):
        # Convert ROS Image to OpenCV BGR
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Overlay the latest telemetry in the top-left corner
        text = f"Telem: {self.latest_telemetry}"
        cv2.putText(frame_bgr,
                    text,
                    (10, 30),              # top-left pixel
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,                   # font scale
                    (0, 255, 0),           # green color
                    2)                     # line thickness

        # Write annotated frame to video
        self.video_writer.write(frame_bgr)

        # (Optional) show preview
        cv2.imshow("Annotated OAK-D Preview", frame_bgr)
        cv2.waitKey(1)

    def destroy_node(self):
        # Clean up
        cv2.destroyAllWindows()
        self.video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FrameAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
