from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('wild_glint_hunt')
    params = os.path.join(pkg_share, 'config', 'params.yaml')
    bringup_params = os.path.join(pkg_share, 'config', 'bringup_params.yaml')

    params_file = LaunchConfiguration('params_file')
    bringup_params_file = LaunchConfiguration('bringup_params_file')
    backend = LaunchConfiguration('backend')

    overrides = {'backend': backend}

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        DeclareLaunchArgument('bringup_params_file', default_value=bringup_params),
        DeclareLaunchArgument('backend', default_value='real'),
        Node(
            package='wild_glint_hunt',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[params_file, bringup_params_file],
        ),
        Node(
            package='wild_glint_hunt',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[params_file, bringup_params_file],
        ),
        Node(
            package='wild_glint_hunt',
            executable='state_machine_node',
            name='state_machine_node',
            output='screen',
            parameters=[params_file, bringup_params_file, overrides],
        ),
    ])
