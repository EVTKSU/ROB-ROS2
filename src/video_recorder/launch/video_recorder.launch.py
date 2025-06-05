from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # 1) video_recorder_node: publishes /video_frames and writes raw video
        Node(
            package='video_recorder',
            executable='video_recorder_node',
            name='video_recorder',
            output='screen',
            parameters=[{}],  # no params needed here
        ),

        # 2) frame_annotator_node: subscribes to /video_frames and /telemetry, writes annotated video
        Node(
            package='video_recorder',
            executable='frame_annotator_node',
            name='frame_annotator',
            output='screen',
            parameters=[{}],  # no params needed here
        ),

        # 3) (Optional) Open a terminal to echo /telemetry so you can see raw telemetry
        ExecuteProcess(
            cmd=[
                'xterm',
                '-hold',
                '-e',
                'ros2', 'topic', 'echo', '/telemetry'
            ],
            output='screen'
        ),
    ])
