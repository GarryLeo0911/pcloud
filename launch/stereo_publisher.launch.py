from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Camera identification
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
        
        # OAK-D Configuration
        DeclareLaunchArgument(
            'fps',
            default_value='30.0',
            description='Camera FPS'
        ),
        DeclareLaunchArgument(
            'color_width',
            default_value='1280',
            description='Color camera width'
        ),
        DeclareLaunchArgument(
            'color_height',
            default_value='720',
            description='Color camera height'
        ),
        DeclareLaunchArgument(
            'mono_width',
            default_value='640',
            description='Mono cameras width'
        ),
        DeclareLaunchArgument(
            'mono_height',
            default_value='400',
            description='Mono cameras height'
        ),
        DeclareLaunchArgument(
            'confidence_threshold',
            default_value='200',
            description='Stereo confidence threshold'
        ),
        DeclareLaunchArgument(
            'enable_depth',
            default_value='true',
            description='Enable depth stream'
        ),
        DeclareLaunchArgument(
            'enable_rectified',
            default_value='true',
            description='Enable rectified streams'
        ),
        DeclareLaunchArgument(
            'enable_lr_check',
            default_value='true',
            description='Enable left-right consistency check'
        ),
        DeclareLaunchArgument(
            'enable_subpixel',
            default_value='true',
            description='Enable subpixel interpolation'
        ),
        DeclareLaunchArgument(
            'enable_extended_disparity',
            default_value='false',
            description='Enable extended disparity'
        ),
        
        # Stereo publisher node
        Node(
            package='oakd_pcloud',
            executable='stereo_rectified_rgb_node',
            name='stereo_publisher',
            parameters=[{
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_param_uri': LaunchConfiguration('camera_param_uri'),
                'fps': LaunchConfiguration('fps'),
                'color_width': LaunchConfiguration('color_width'),
                'color_height': LaunchConfiguration('color_height'),
                'mono_width': LaunchConfiguration('mono_width'),
                'mono_height': LaunchConfiguration('mono_height'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'enable_depth': LaunchConfiguration('enable_depth'),
                'enable_rectified': LaunchConfiguration('enable_rectified'),
                'enable_lr_check': LaunchConfiguration('enable_lr_check'),
                'enable_subpixel': LaunchConfiguration('enable_subpixel'),
                'enable_extended_disparity': LaunchConfiguration('enable_extended_disparity')
            }],
            output='screen'
        )
    ])