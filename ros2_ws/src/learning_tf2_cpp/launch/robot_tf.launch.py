from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments for camera static transform parameters
        DeclareLaunchArgument(
            'camera_x',
            default_value='0.1',
            description='Camera X offset from arm_end'
        ),
        DeclareLaunchArgument(
            'camera_y',
            default_value='0.0',
            description='Camera Y offset from arm_end'
        ),
        DeclareLaunchArgument(
            'camera_z',
            default_value='0.05',
            description='Camera Z offset from arm_end'
        ),
        DeclareLaunchArgument(
            'camera_roll',
            default_value='0.0',
            description='Camera roll rotation from arm_end'
        ),
        DeclareLaunchArgument(
            'camera_pitch',
            default_value='0.0',
            description='Camera pitch rotation from arm_end'
        ),
        DeclareLaunchArgument(
            'camera_yaw',
            default_value='0.0',
            description='Camera yaw rotation from arm_end'
        ),

        # Robot TF broadcaster node
        Node(
            package='learning_tf2_cpp',
            executable='robots_tfs',
            name='robot_tf_broadcaster',
            parameters=[{
                'camera_x': LaunchConfiguration('camera_x'),
                'camera_y': LaunchConfiguration('camera_y'),
                'camera_z': LaunchConfiguration('camera_z'),
                'camera_roll': LaunchConfiguration('camera_roll'),
                'camera_pitch': LaunchConfiguration('camera_pitch'),
                'camera_yaw': LaunchConfiguration('camera_yaw'),
            }],
            output='screen'
        ),
    ])
