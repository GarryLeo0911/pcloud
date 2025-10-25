from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_pcloud',
            executable='preview_publisher_node',
            name='preview_publisher',
            output='screen',
            parameters=[{
                'publish_period_ms': 33,
                'frame_id': 'oakd_frame'
            }]
        )
    ])

