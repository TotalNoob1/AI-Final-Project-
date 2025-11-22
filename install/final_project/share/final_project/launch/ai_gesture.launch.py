from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'final_project',
            executable = 'gesture_control',
            output = 'screen'
        ),
        Node(
            package = 'final_project',
            executable = 'a1_subscriber',
            output = 'screen'
        ),
        # Node(
        #     package = 'final_project',
        #     executable = 'final_project',
        #     output = 'screen'
        # ),
    ])