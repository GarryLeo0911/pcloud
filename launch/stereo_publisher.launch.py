from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'camera_name',
            default_value='OAK-D',
            description='Name of the camera'
        ),
        DeclareLaunchArgument(
            'camera_param_uri',
            default_value='file://${ROS_HOME}/camera_info/${camera_name}.yaml',
            description='URI to camera calibration file'
        ),
        
        # Stereo publisher node
        Node(
            package='oakd_pcloud',
            executable='stereo_rectified_rgb_node',
            name='stereo_publisher',
            parameters=[{
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_param_uri': LaunchConfiguration('camera_param_uri')
            }],
            output='screen'
        )
    ])