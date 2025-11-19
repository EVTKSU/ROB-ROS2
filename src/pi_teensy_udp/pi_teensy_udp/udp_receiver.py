import rclpy
from rclpy.node import Node
import socket

# Import your updated custom message
from system_msgs.msg import TeensyTelemetry

class UDPReceiver(Node):
    def __init__(self):
        super().__init__('udp_receiver')

        # Create a ROS 2 publisher for the 'teensy_telemetry' topic using your custom message
        self.publisher_ = self.create_publisher(TeensyTelemetry, 'teensy_telemetry', 10)

        # Create and configure a non-blocking UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 5005))        # Listen on all network interfaces
        self.sock.setblocking(False)      # Don't block if no message is ready

        # Call receive_message() every 0.1 seconds
        self.timer = self.create_timer(0.1, self.receive_message)

    def receive_message(self):
        try:
            # Try to receive up to 1024 bytes (1KB) of UDP data
            data, addr = self.sock.recvfrom(1024)
            message_str = data.decode().strip()

            # Expecting format:
            # "1,AUTO,540.2,0.34,49.1,47.9,13.2,12.5,1300,990"
            parts = message_str.split(',')

            if len(parts) != 10:
                self.get_logger().warn(f"Invalid telemetry packet (wrong field count): {message_str}")
                return

            # Create and populate the message
            msg = TeensyTelemetry()
            msg.emergency           = bool(int(parts[0]))      # Convert "1" or "0" to True/False
            msg.state               = parts[1]
            msg.rpm                 = float(parts[2])
            msg.steering_angle      = float(parts[3])
            msg.odrv_voltage        = float(parts[4])
            msg.vesc_voltage        = float(parts[5])
            msg.odrv_current        = float(parts[6])
            msg.vesc_current        = float(parts[7])
            msg.rc_steering_input   = float(parts[8])
            msg.rc_throttle_input   = float(parts[9])

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published telemetry from {addr}: {message_str}")

        except BlockingIOError:
            # Normal behavior when no packet is available on the socket
            pass
        except ValueError as e:
            # Conversion failed due to malformed number (e.g., letters in float field)
            self.get_logger().error(f"Malformed packet: {message_str} | Error: {e}")
        except Exception as e:
            # Catch-all for unexpected issues
            self.get_logger().error(f"Unexpected error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UDPReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
