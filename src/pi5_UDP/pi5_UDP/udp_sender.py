import rclpy
from rclpy.node import Node
import socket

class UDPSender(Node):
    def __init__(self):
        super().__init__('udp_sender')  # Initialize the ROS 2 node with the name 'udp_sender'

        # Create a UDP socket using IPv4 addressing
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Define the target IP and port to send messages to
        self.target_ip = '192.168.1.100'  # Replace with the receiver's IP address
        self.target_port = 5005  # Replace with the receiver's port number

        # Set up a timer to call the send_message function every 1.0 seconds
        self.timer = self.create_timer(1.0, self.send_message)

    def send_message(self):
        # Define the message to be sent
        message = b'Hello from ROS 2 UDP sender'

        # Send the message to the specified IP and port
        self.sock.sendto(message, (self.target_ip, self.target_port))

        # Log the sent message
        self.get_logger().info(f'Sent: {message}')

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = UDPSender()     # Create an instance of the UDPSender node
    rclpy.spin(node)       # Keep the node running, processing callbacks
    node.destroy_node()    # Clean up the node upon shutdown
    rclpy.shutdown()       # Shutdown the ROS 2 Python client library
