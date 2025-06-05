#!/usr/bin/env python3
"""
lane_traversal_node.py

1) Subscribes to /steer_pct (std_msgs/Float32).
2) Reads throttle_pct from a ROS parameter (0–100).
3) Publishes the string "throttle_pct,steer_pct,0" on /percent_cmds.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class LaneTraversal(Node):
    def __init__(self):
        super().__init__('lane_traversal')

        # Subscriber to /steer_pct
        self.sub_steer = self.create_subscription(
            Float32, 'steer_pct', self.steer_callback, 10)

        # Publisher to /percent_cmds
        self.pub_percent = self.create_publisher(String, 'percent_cmds', 10)

        # Declare throttle_percent parameter (0–100)
        self.declare_parameter('throttle_percent', 50.0)
        # Log initialization
        self.get_logger().info("lane_traversal node initialized.")

    def steer_callback(self, msg: Float32):
        steer_pct = msg.data  # float between -100 and +100
        # Get throttle_pct from ROS parameter
        throttle_pct = self.get_parameter('throttle_percent').get_parameter_value().double_value
        throttle_pct = max(min(throttle_pct, 100.0), 0.0)

        # Format as "throttle_pct,steer_pct,0"
        out_str = f"{throttle_pct:.2f},{steer_pct:.2f},{0}"
        percent_msg = String()
        percent_msg.data = out_str

        self.pub_percent.publish(percent_msg)
        self.get_logger().info(f"Published /percent_cmds: \"{out_str}\"")

def main(args=None):
    rclpy.init(args=args)
    node = LaneTraversal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
