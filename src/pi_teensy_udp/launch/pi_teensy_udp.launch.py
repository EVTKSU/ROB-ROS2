from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pi_teensy_udp',
            executable='udp_sender',
            name='udp_sender_node',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='pi_teensy_udp',
            executable='udp_receiver',
            name='udp_receiver_node',
            output='screen'
        ),
        Node(
            package='pi_teensy_udp',
            executable='pseudo_auto_mode',
            name='pseudo_auto_mode_node',
            output='screen'
        ),
    ])
