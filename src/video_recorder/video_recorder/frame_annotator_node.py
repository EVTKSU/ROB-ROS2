#!/usr/bin/env python3
"""
frame_annotator_node.py

1) Subscribes to /video_frames (sensor_msgs/Image) and /telemetry (std_msgs/String).
2) Maintains the latest telemetry (steer, throttle) values.
3) Overlays the telemetry text onto each incoming frame.
4) Writes annotated frames to a new video file (annotated_output.avi).
"""

import os

import rclpy
from rclpy.node import Node

import numpy as np

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import String

from datetime import datetime

# Same resolution & framerate as recorder
CAM_W, CAM_H = 854, 480
FRAME_RATE = 30 

class FrameAnnotator(Node):
    """
    ROS node used to recieve frames from the OAK-D LR camera and annotate telemetry on them
    """

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

        # Get the current date
        self.now: datetime = datetime.now()
        self.formatted_time: str = self.now.strftime('%m_%d-%H:%M:%S')

        # Set up VideoWriter for annotated video
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out_path = os.path.join(os.path.expanduser('~'), f'annotated_output_{self.formatted_time}.avi')
        self.video_writer = cv2.VideoWriter(out_path, fourcc, FRAME_RATE, (CAM_W, CAM_H))
        self.get_logger().info(f"Recording annotated video to: {out_path}")

        self.get_logger().info("frame_annotator node initialized.")


    def telemetry_callback(self, msg: String):
        """
        Callback method to use for telemetry updates
        """

        # Store the latest telemetry (string format: "throttle,steer,flag")
        self.latest_telemetry = msg.data

        # Optionally log it:
        self.get_logger().debug(f"Telemetry updated: {self.latest_telemetry}")


    def frame_callback(self, msg: Image):
        """
        Callback method to use for OAK-D LR data
        """
        
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
        """
        Method to use when closing the ROS node
        """

        # Clean up
        cv2.destroyAllWindows()
        self.video_writer.release()
        super().destroy_node()


def main(args = None):
    rclpy.init(args = args)
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
