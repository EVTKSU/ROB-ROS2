import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PseudoAutoMode(Node):
    """
    A ROS 2 node that simulates autonomous driving commands by publishing
    formatted strings to the 'auto_commands' topic. Each message contains
    throttle, steering angle, and emergency flag values.
    """

    def __init__(self):
        super().__init__('pseudo_auto_mode')

        # Create a publisher for the 'auto_commands' topic with a queue size of 10
        self.publisher_ = self.create_publisher(String, 'auto_commands', 10)

        # Set the timer period to 1.0 second
        timer_period = 1.0  # seconds 

        # Create a timer that calls the timer_callback function every timer_period seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize counters for throttle, steering angle, and loop count
        self.throttle = 0.0
        self.steering_angle = -50.0
        self.loop_count = 0

    def timer_callback(self):
        """
        Callback function that is called periodically by the timer.
        It updates the throttle and steering angle values, sets the emergency flag,
        formats the message, and publishes it to the 'auto_commands' topic.
        """

        # Determine the emergency flag: set to 1 every 4th loop, else 0
        emergency_flag = 1 if self.loop_count % 4 == 0 else 0

        # Format the message as 'throttle,steering_angle,emergency_flag'
        message = f"{self.throttle:.2f},{self.steering_angle:.2f},{emergency_flag}"

        # Create a String message and assign the formatted string to its data field
        msg = String()
        msg.data = message

        # Publish the message to the 'auto_commands' topic
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Update the throttle and steering angle values for the next iteration
        self.throttle += 5.0
        if self.throttle > 100.0:
            self.throttle = 0.0

        self.steering_angle += 10.0
        if self.steering_angle > 50.0:
            self.steering_angle = -50.0

        # Increment the loop count
        self.loop_count += 1

def main(args=None):
    """
    Main function that initializes the ROS 2 Python client library,
    creates the PseudoAutoMode node, and spins it to process callbacks.
    """
    rclpy.init(args=args)
    node = PseudoAutoMode()
    rclpy.spin(node)

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()
