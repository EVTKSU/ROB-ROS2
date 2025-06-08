from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os 


os.system("python3 /home/evt/Desktop/Startup_Noises.py")   
os.system("python3 /home/evt/Desktop/Initialized.py")   

def generate_launch_description():
    return LaunchDescription([
        # 1) Lane detection node (identical logic to Triton AI’s script)
        Node(
            package='minirob_auto',
            executable='lane_detection_node',
            name='lane_detection',
            output='screen',
            parameters=[{}],
        ),

        # 2) Lane traversal node
        Node(
            package='minirob_auto',
            executable='lane_traversal_node',
            name='lane_traversal',
            output='screen',
            parameters=[{
                'throttle_percent': 50.0
            }],
        ),

        # 3) Auto control node
        Node(
            package='minirob_auto',
            executable='auto_control_node',
            name='auto_control',
            output='screen',
            parameters=[{}],
        ),

        # 4) (Optional) Open an xterm echoing /auto_commands for debugging
        ExecuteProcess(
            cmd=[
                'xterm',
                '-hold',
                '-e',
                'ros2', 'topic', 'echo', '/auto_commands'
            ],
            output='screen'
        ),
    ])
