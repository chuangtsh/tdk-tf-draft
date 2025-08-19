from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    # Declare launch arguments
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start the lifecycle node (set to false for manual control)'
    )

    # Camera parameters
    camera_x_arg = DeclareLaunchArgument('camera_x', default_value='0.1')
    camera_y_arg = DeclareLaunchArgument('camera_y', default_value='0.0')
    camera_z_arg = DeclareLaunchArgument('camera_z', default_value='0.05')
    camera_roll_arg = DeclareLaunchArgument('camera_roll', default_value='0.0')
    camera_pitch_arg = DeclareLaunchArgument('camera_pitch', default_value='0.0')
    camera_yaw_arg = DeclareLaunchArgument('camera_yaw', default_value='0.0')

    # Odom parameters
    odom_x_arg = DeclareLaunchArgument('odom_x', default_value='0.0')
    odom_y_arg = DeclareLaunchArgument('odom_y', default_value='0.0')
    odom_z_arg = DeclareLaunchArgument('odom_z', default_value='0.0')
    odom_roll_arg = DeclareLaunchArgument('odom_roll', default_value='0.0')
    odom_pitch_arg = DeclareLaunchArgument('odom_pitch', default_value='0.0')
    odom_yaw_arg = DeclareLaunchArgument('odom_yaw', default_value='0.0')

    # Lifecycle node
    robot_tf_lifecycle_node = LifecycleNode(
        package='learning_tf2_cpp',
        executable='robots_tfs_lifecycle',
        name='robot_tf_broadcaster_lifecycle',
        namespace='',
        parameters=[{
            'camera_x': LaunchConfiguration('camera_x'),
            'camera_y': LaunchConfiguration('camera_y'),
            'camera_z': LaunchConfiguration('camera_z'),
            'camera_roll': LaunchConfiguration('camera_roll'),
            'camera_pitch': LaunchConfiguration('camera_pitch'),
            'camera_yaw': LaunchConfiguration('camera_yaw'),
            'odom_x': LaunchConfiguration('odom_x'),
            'odom_y': LaunchConfiguration('odom_y'),
            'odom_z': LaunchConfiguration('odom_z'),
            'odom_roll': LaunchConfiguration('odom_roll'),
            'odom_pitch': LaunchConfiguration('odom_pitch'),
            'odom_yaw': LaunchConfiguration('odom_yaw'),
        }],
        output='screen'
    )

    return LaunchDescription([
        auto_start_arg,
        camera_x_arg,
        camera_y_arg,
        camera_z_arg,
        camera_roll_arg,
        camera_pitch_arg,
        camera_yaw_arg,
        odom_x_arg,
        odom_y_arg,
        odom_z_arg,
        odom_roll_arg,
        odom_pitch_arg,
        odom_yaw_arg,
        robot_tf_lifecycle_node,
    ])

# To manually control the lifecycle node, use these commands:
# ros2 lifecycle set /robot_tf_broadcaster_lifecycle configure
# ros2 lifecycle set /robot_tf_broadcaster_lifecycle activate
# ros2 lifecycle set /robot_tf_broadcaster_lifecycle deactivate
# ros2 lifecycle set /robot_tf_broadcaster_lifecycle cleanup
# ros2 lifecycle set /robot_tf_broadcaster_lifecycle shutdown

# To check the current state:
# ros2 lifecycle get /robot_tf_broadcaster_lifecycle

# To list available transitions:
# ros2 lifecycle list /robot_tf_broadcaster_lifecycle
