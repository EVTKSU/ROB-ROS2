from launch import LaunchDescription
from launch_ros.actions import Node

# This function is called when the launch file is executed
def generate_launch_description():
    # Create and return a LaunchDescription object with both nodes
    return LaunchDescription([
        # Launch the talker node
        Node(
            package='pub_sub_demo',      # Name of the package containing the node
            executable='talker',         # Entry point name from setup.py
            name='talker_node',          # Optional: name of the node as seen in rqt_graph
            output='screen'              # Print output to terminal
        ),
        # Launch the listener node
        Node(
            package='pub_sub_demo',      # Same package
            executable='listener',       # Entry point name from setup.py
            name='listener_node',        # Another visible name for debugging
            output='screen'
        )
    ])
