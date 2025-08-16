from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'start_simulator',
            default_value='true',
            description='Whether to start the robot simulator'
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Whether to start RViz2'
        ),
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
        DeclareLaunchArgument(
            'map_x',
            default_value='5.0',
            description='Map X offset from world (robot starting position)'
        ),
        DeclareLaunchArgument(
            'map_y',
            default_value='5.0',
            description='Map Y offset from world (robot starting position)'
        ),
        DeclareLaunchArgument(
            'map_z',
            default_value='0.0',
            description='Map Z offset from world (robot starting position)'
        ),
        DeclareLaunchArgument(
            'map_roll',
            default_value='0.0',
            description='Map roll rotation from world'
        ),
        DeclareLaunchArgument(
            'map_pitch',
            default_value='0.0',
            description='Map pitch rotation from world'
        ),
        DeclareLaunchArgument(
            'map_yaw',
            default_value='0.0',
            description='Map yaw rotation from world'
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
                'map_x': LaunchConfiguration('map_x'),
                'map_y': LaunchConfiguration('map_y'),
                'map_z': LaunchConfiguration('map_z'),
                'map_roll': LaunchConfiguration('map_roll'),
                'map_pitch': LaunchConfiguration('map_pitch'),
                'map_yaw': LaunchConfiguration('map_yaw'),
            }],
            output='screen'
        ),

        Node(
            package='learning_tf2_cpp',
            executable='camera_displacement_publisher',
            name='camera_displacement_publisher',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            output='screen'
        )
    ])


# --- Robot (world -> robot_center) ---
# W/S: Move forward/backward (robot's local frame)
# A/D: Move left/right (robot's local frame)
# Q/E: Rotate left/right (Yaw)

# --- Lift Base (robot_center -> lift_base) ---
# Z/X: Move forward/backward (X)
# C/V: Move left/right (Y)

# --- Arm Base (lift_base -> arm_base) ---
# U/O: Move up/down (Z) - cascade lift

# --- Arm End (arm_base -> arm_end) ---
# I/K: Move forward/backward (X)
# J/L: Move left/right (Y)
# T/G: Move up/down (Z)
# R/Y: Rotate pitch up/down

# Press Ctrl-C to exit
# Step size: 0.1 units, 0.1 radians