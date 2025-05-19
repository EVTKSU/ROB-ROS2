import rclpy
from rclpy.node import Node
import socket

class UDPReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')  # Initialize the ROS 2 node with the name 'udp_receiver'

        # Create a UDP socket using IPv4 addressing
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Bind the socket to all available interfaces on port 5005
        self.sock.bind(('', 5005))  # Replace with desired port if different

        # Set the socket to non-blocking mode to prevent blocking the node
        self.sock.setblocking(False)

        # Set up a timer to call the receive_message function every 0.1 seconds
        self.timer = self.create_timer(0.1, self.receive_message)

    def receive_message(self):
        try:
            # Attempt to receive data from the socket
            data, addr = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes

            # Log the received message along with the sender's address
            self.get_logger().info(f'Received from {addr}: {data.decode()}')
        except BlockingIOError:
            # No data received; non-blocking socket would raise this exception
            pass

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = UDPReceiver()   # Create an instance of the UDPReceiver node
    rclpy.spin(node)       # Keep the node running, processing callbacks
    node.destroy_node()    # Clean up the node upon shutdown
    rclpy.shutdown()       # Shutdown the ROS 2 Python client library
