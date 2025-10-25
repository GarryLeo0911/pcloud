from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oakd_pcloud',
            executable='median_filter_node',
            name='median_filter',
            output='screen',
            parameters=[{
                'input_image_topic': 'image',
                'output_image_topic': 'image_filtered',
                'kernel_size': 3,
                'iterations': 1,
            }]
        )
    ])

