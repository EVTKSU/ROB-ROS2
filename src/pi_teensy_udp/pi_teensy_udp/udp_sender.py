import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket

class UDPSender(Node):
    def __init__(self):
        super().__init__('udp_sender')

        # Create a subscriber to the 'auto_commands' topic
        self.subscription = self.create_subscription(
            String,
            'auto_commands',
            self.listener_callback,
            10)

        # Set up the UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.target_ip = '192.168.0.177'  # teensy IP
        self.target_port = 8888  
        
    def listener_callback(self, msg):
        # Send the received message via UDP
        message = msg.data.encode('utf-8')
        self.sock.sendto(message, (self.target_ip, self.target_port))
        self.get_logger().info(f'Sent: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = UDPSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
