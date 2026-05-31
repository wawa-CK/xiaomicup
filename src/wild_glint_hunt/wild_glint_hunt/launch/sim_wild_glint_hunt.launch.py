from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('wild_glint_hunt')
    params = os.path.join(pkg_share, 'config', 'competition_tuned_params.yaml')
    field_map = os.path.join(pkg_share, 'config', 'game_field_map.yaml')

    params_file = LaunchConfiguration('params_file')
    field_map_file = LaunchConfiguration('field_map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    shared_overrides = {
        'use_sim_time': use_sim_time,
        'game_field_map_file': field_map_file,
        'odom_topic': '/odom',
    }
    rgb_overrides = {
        'image_topic': '/rgb_camera/image_raw',
        'camera_info_topic': '/rgb_camera/camera_info',
        'use_rgb_camera': True,
        'enable_fisheye_undistortion': False,
    }

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('field_map_file', default_value=field_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        TimerAction(period=1.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'wild_glint_hunt', 'reset_robot_pose', '--ros-args', '-p', 'model_name:=robot', '-p', 'spawn_x:=2.90', '-p', 'spawn_y:=0.45', '-p', 'spawn_z:=0.35', '-p', 'spawn_yaw:=1.57', '-p', 'delay_ms:=1000'],
                output='screen',
            ),
        ]),
        TimerAction(period=4.0, actions=[
            Node(
                package='wild_glint_hunt',
                executable='simulated_sensors_node',
                name='simulated_sensors_node',
                output='screen',
                parameters=[params_file, shared_overrides],
            ),
            Node(
                package='wild_glint_hunt',
                executable='check_camera_topic',
                name='check_camera_topic',
                output='screen',
                parameters=[{'camera_topic': '/rgb_camera/image_raw', 'camera_timeout_s': 5.0}, shared_overrides],
            ),
            Node(
                package='wild_glint_hunt',
                executable='vision_node',
                name='vision_node',
                output='screen',
                parameters=[params_file, shared_overrides, rgb_overrides],
            ),
        ]),
        TimerAction(period=14.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'wild_glint_hunt', 'reset_robot_pose', '--ros-args', '-p', 'model_name:=robot', '-p', 'spawn_x:=2.90', '-p', 'spawn_y:=0.45', '-p', 'spawn_z:=0.35', '-p', 'spawn_yaw:=1.57', '-p', 'delay_ms:=100'],
                output='screen',
            ),
        ]),
        TimerAction(period=18.0, actions=[
            Node(
                package='wild_glint_hunt',
                executable='path_planner_node',
                name='path_planner_node',
                output='screen',
                parameters=[params_file, shared_overrides],
            ),
            Node(
                package='wild_glint_hunt',
                executable='state_machine_node',
                name='state_machine_node',
                output='screen',
                parameters=[params_file, shared_overrides],
            ),
        ]),
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('wild_glint_hunt')
    params = os.path.join(pkg_share, 'config', 'competition_tuned_params.yaml')
    field_map = os.path.join(pkg_share, 'config', 'game_field_map.yaml')

    params_file = LaunchConfiguration('params_file')
    field_map_file = LaunchConfiguration('field_map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    reset_robot = LaunchConfiguration('reset_robot')
    sensor_delay = LaunchConfiguration('sensor_delay')
    planner_delay = LaunchConfiguration('planner_delay')
    shared_overrides = {
        'use_sim_time': use_sim_time,
        'game_field_map_file': field_map_file,
        'odom_topic': '/odom',
    }
    rgb_overrides = {
        'image_topic': '/rgb_camera/image_raw',
        'camera_info_topic': '/rgb_camera/camera_info',
        'use_rgb_camera': True,
        'enable_fisheye_undistortion': False,
    }

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('field_map_file', default_value=field_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('reset_robot', default_value='false'),
        DeclareLaunchArgument('sensor_delay', default_value='1.0'),
        DeclareLaunchArgument('planner_delay', default_value='3.0'),
        TimerAction(period=1.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'wild_glint_hunt', 'reset_robot_pose', '--ros-args', '-p', 'model_name:=robot', '-p', 'spawn_x:=2.90', '-p', 'spawn_y:=0.45', '-p', 'spawn_z:=0.35', '-p', 'spawn_yaw:=1.57', '-p', 'delay_ms:=1000'],
                output='screen',
                condition=IfCondition(reset_robot),
            ),
        ]),
        TimerAction(period=sensor_delay, actions=[
            Node(
                package='wild_glint_hunt',
                executable='simulated_sensors_node',
                name='simulated_sensors_node',
                output='screen',
                parameters=[params_file, shared_overrides],
            ),
            Node(
                package='wild_glint_hunt',
                executable='check_camera_topic',
                name='check_camera_topic',
                output='screen',
                parameters=[{'camera_topic': '/rgb_camera/image_raw', 'camera_timeout_s': 5.0}, shared_overrides],
            ),
            Node(
                package='wild_glint_hunt',
                executable='vision_node',
                name='vision_node',
                output='screen',
                parameters=[params_file, shared_overrides, rgb_overrides],
            ),
        ]),
        TimerAction(period=14.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'wild_glint_hunt', 'reset_robot_pose', '--ros-args', '-p', 'model_name:=robot', '-p', 'spawn_x:=2.90', '-p', 'spawn_y:=0.45', '-p', 'spawn_z:=0.35', '-p', 'spawn_yaw:=1.57', '-p', 'delay_ms:=100'],
                output='screen',
                condition=IfCondition(reset_robot),
            ),
        ]),
        TimerAction(period=planner_delay, actions=[
            Node(
                package='wild_glint_hunt',
                executable='path_planner_node',
                name='path_planner_node',
                output='screen',
                parameters=[params_file, shared_overrides],
            ),
            Node(
                package='wild_glint_hunt',
                executable='state_machine_node',
                name='state_machine_node',
                output='screen',
                parameters=[params_file, shared_overrides],
            ),
        ]),
    ])
