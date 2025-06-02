from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1) Launch the main minirob_auto node
        Node(
            package='minirob_auto',
            executable='minirob_auto_node',
            name='minirob_auto',
            output='screen',
            parameters=[{
                'throttle_percent': 50.0,  # default throttle % (0–100)
            }],
        ),

        # 2) Open an xterm that echoes /auto_commands
        ExecuteProcess(
            cmd=[
                'xterm',
                '-hold',      # keep the xterm open after the command exits
                '-e',         # execute the following command
                'ros2', 'topic', 'echo', '/auto_commands'
            ],
            output='screen'
        ),
    ])
