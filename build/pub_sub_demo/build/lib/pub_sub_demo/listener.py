# Import ROS 2 Python client library
import rclpy
from rclpy.node import Node

# Import the same message type we're subscribing to
from std_msgs.msg import String

# Define your custom Node class
class MinimalSubscriber(Node):
    def __init__(self):
        # Initialize the node with the name 'listener'
        super().__init__('listener')

        # Create a subscription to the 'chatter' topic
        # The callback function is called every time a message is received
        self.subscription = self.create_subscription(
            String,                   # Message type
            'chatter',               # Topic name
            self.listener_callback,  # Callback function
            10                       # Queue size
        )

        # Prevent the subscription object from being garbage-collected
        self.subscription

    # This function runs when a message is received
    def listener_callback(self, msg):
        # Log the message contents to the terminal
        self.get_logger().info(f'I heard: "{msg.data}"')

# Standard Python entry point
def main(args=None):
    rclpy.init(args=args)          # Initialize ROS 2
    node = MinimalSubscriber()     # Create an instance of your node
    rclpy.spin(node)               # Wait and respond to messages
    node.destroy_node()            # Cleanup
    rclpy.shutdown()               # Shutdown ROS 2 interface
