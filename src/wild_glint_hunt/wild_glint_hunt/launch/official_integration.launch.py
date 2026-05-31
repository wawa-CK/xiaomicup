from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('wild_glint_hunt')
    params = os.path.join(pkg_share, 'config', 'params.yaml')
    field_map = os.path.join(pkg_share, 'config', 'game_field_map.yaml')

    params_file = LaunchConfiguration('params_file')
    field_map_file = LaunchConfiguration('field_map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    backend = LaunchConfiguration('backend')

    official_overrides = {
        'use_sim_time': use_sim_time,
        'backend': backend,
        'game_field_map_file': field_map_file,
        'use_rgb_camera': True,
        'enable_fisheye_undistortion': False,
        'image_topic': '/image_left',
        'camera_info_topic': '/image_left/camera_info',
        'odom_topic': '/odom_out',
        'official_odom_topic': 'odom_out',
        'imu_topic': '/imu',
        'official_imu_topic': 'imu',
        'ultrasonic_topic': 'ultrasonic_payload',
        'official_ultrasonic_topic': 'ultrasonic_payload',
        'tof_topic': 'head_tof_payload',
        'official_tof_topic': 'head_tof_payload',
        'official_rear_tof_topic': 'rear_tof_payload',
        'cmd_vel_topic': 'motion_servo_cmd',
        'official_motion_servo_topic': 'motion_servo_cmd',
        'target_use_visual_distance_updates': True,
        'target_use_visual_yaw_updates': True,
        'sim_assume_strike_success': False,
    }

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('field_map_file', default_value=field_map),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('backend', default_value='real'),
        Node(
            package='wild_glint_hunt',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[params_file, official_overrides],
        ),
        Node(
            package='wild_glint_hunt',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[params_file, official_overrides],
        ),
        Node(
            package='wild_glint_hunt',
            executable='state_machine_node',
            name='state_machine_node',
            output='screen',
            parameters=[params_file, official_overrides],
        ),
    ])
