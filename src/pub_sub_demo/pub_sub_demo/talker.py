# Import ROS 2 Python client library
import rclpy
from rclpy.node import Node

# Import the message type you want to publish
from std_msgs.msg import String

# Define your custom Node class
class MinimalPublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'talker'
        super().__init__('talker')

        # Create a publisher for the 'chatter' topic, sending String messages
        # The '10' is the queue size: how many messages to buffer if the subscriber is slow
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Set up a timer to call the 'timer_callback' function every second (1.0s)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.count = 0  # Track how many messages we've sent

    def timer_callback(self):
        # Create a new message and fill it with data
        msg = String()
        msg.data = f'Hello, world! {self.count}'

        # Publish the message on the topic
        self.publisher_.publish(msg)

        # Log the message to the terminal
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.count += 1  # Increment the count for next time

# This is the standard Python entry point
def main(args=None):
    rclpy.init(args=args)          # Initialize ROS 2 Python interface
    node = MinimalPublisher()      # Create an instance of your node
    rclpy.spin(node)               # Keep the node running, waiting for timer callbacks
    node.destroy_node()            # Cleanup when node shuts down
    rclpy.shutdown()               # Shutdown the ROS client library
