import rclpy
from rclpy.node import Node
import socket

# Import your custom message
from system_msgs.msg import TeensyTelemetry

class UDPReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')

        # Create a ROS 2 publisher for the teensy_telemetry topic
        self.publisher_ = self.create_publisher(TeensyTelemetry, 'teensy_telemetry', 10)

        # Set up a non-blocking UDP socket on port 5005
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 5005))  # Listen on all interfaces
        self.sock.setblocking(False)

        # Use a timer to poll the socket every 0.1s
        self.timer = self.create_timer(0.1, self.receive_message)

    def receive_message(self):
        try:
            data, addr = self.sock.recvfrom(1024)  # Max 1 KB packet
            message_str = data.decode().strip()

            # Expecting format like: "AUTO,540.2,23.5,49.1,13.2,5.8,3.4,0.8"
            parts = message_str.split(',')

            if len(parts) != 8:
                self.get_logger().warn(f"Invalid telemetry packet: {message_str}")
                return

            # Populate TeensyTelemetry message
            msg = TeensyTelemetry()
            msg.state = parts[0]
            msg.rpm = float(parts[1])
            msg.vesc_voltage = float(parts[2])
            msg.odrv_voltage = float(parts[3])
            msg.avg_motor_current = float(parts[4])
            msg.odrv_current = float(parts[5])
            msg.steering_angle = float(parts[6])
            msg.velocity = float(parts[7])

            # Publish it
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published telemetry from {addr}")

        except BlockingIOError:
            # No UDP packet available — this is normal in non-blocking mode
            pass

def main(args=None):
    rclpy.init(args=args)
    node = UDPReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
