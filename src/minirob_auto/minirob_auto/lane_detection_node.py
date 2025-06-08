import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
import cv2
from cv_bridge import CvBridge
import os
from ament_index_python import get_package_share_directory
from ultralytics import YOLO
import numpy as np
from shapely.geometry import Polygon, LineString
from shapely.ops import split
import time

# Nodes in this program
NODE_NAME = 'lane_detection_node'

# Topics subscribed/published to in this program
CAMERA_TOPIC_NAME = '/oak/rgb/image_raw'
SIM_CAMERA_TOPIC_NAME = '/camera_link/image/compressed'
CENTROID_ERROR_TOPIC_NAME = '/centroid error'

MODEL_PATH = os.path.join(get_package_share_directory("lane_detection"), "models/best.pt")


class LaneDetection(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.centroid_error_publisher = self.create_publisher(Float32, CENTROID_ERROR_TOPIC_NAME, 10)
        self.camera_subscriber = self.create_subscription(Image, CAMERA_TOPIC_NAME, self.locate_centroid, 10)
        self.bridge = CvBridge()
        self.sim_camera_sub = self.create_subscription(CompressedImage, SIM_CAMERA_TOPIC_NAME, self.sim_locate_centroid, 10)
        self.start_time = None

        self.centroid_error = Float32()

        # Initialize model
        self.model = YOLO(MODEL_PATH)

    def locate_centroid(self, data):
        # Image processing from rosparams
        frame = self.bridge.compressed_imgmsg_to_cv2(data)

        # Find contour with the highest confidence
        results = self.model.predict(frame, conf=0.7)
        conf = results[0].boxes.conf.tolist()

        if len(conf) != 0:
            max_conf_index = np.argmax(conf)
            _, width, _ = frame.shape
            center_x = width // 2

            # Find contour
            coords_xy = results[max_conf_index].masks.xy[0]
            contour = np.array(coords_xy, dtype=np.int32).reshape((-1, 1, 2))

            # Find centroid
            M = cv2.moments(contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Publish centroid error
            centroid_error = float(center_x - cX)
            self.centroid_error.data = centroid_error
            self.centroid_error_publisher.publish(self.centroid_error)

    def sim_locate_centroid(self, data):

        if self.start_time is None:
            self.start_time = time.time()

        # Image processing from rosparams
        frame = self.bridge.compressed_imgmsg_to_cv2(data)

        # Find contour with the highest confidence
        results = self.model.predict(frame, conf=0.7)
        conf = results[0].boxes.conf.tolist()

        if len(conf) != 0:
            max_conf_index = np.argmax(conf)
            _, width, _ = frame.shape
            center_x = width // 2

            # Find contour
            coords_xy = results[max_conf_index].masks.xy[0]
            contour = np.array(coords_xy, dtype=np.int32).reshape((-1, 1, 2))

            # Find centroid
            M = cv2.moments(contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Draw contour
            image_with_contour = cv2.drawContours(frame.copy(), [contour], -1, (0, 255, 0), 2)

            # Draw centroid
            cv2.circle(image_with_contour, (cX, cY), 5, (0, 0, 255), -1)

            # Draw center line
            cv2.line(image_with_contour, (center_x, 0), (center_x, frame.shape[0]), (255, 0, 0), 2)

            # Show the frame
            cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Frame", 854, 480)
            cv2.imshow("Frame", image_with_contour)
            cv2.waitKey(1)  # Adjust the delay as needed

            # Publish centroid error
            centroid_error = float(center_x - cX)
            self.centroid_error.data = centroid_error
            self.centroid_error_publisher.publish(self.centroid_error)
            
            if centroid_error > 0:
                direction = "LEFT"
            elif centroid_error < 0:
                direction = "RIGHT"
            else:
                direction = "STRAIGHT"

            curr_time = time.time() - self.start_time

            print(f"Time: {curr_time}. Error: {self.centroid_error}. Direction: {direction}")

def main(args=None):
    rclpy.init(args=args)
    centroid_publisher = LaneDetection()
    try:
        rclpy.spin(centroid_publisher)
    except KeyboardInterrupt:
        centroid_publisher.get_logger().info(f'Shutting down {NODE_NAME}...')

        # Kill cv2 windows and node
        cv2.destroyAllWindows()
        centroid_publisher.destroy_node()
        rclpy.shutdown()
        centroid_publisher.get_logger().info(f'{NODE_NAME} shut down successfully.')


if __name__ == '__main__':
    main()