from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('wild_glint_hunt')
    params = os.path.join(pkg_share, 'config', 'params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        Node(package='wild_glint_hunt', executable='vision_node', name='vision_node', output='screen', parameters=[LaunchConfiguration('params_file')]),
        Node(package='wild_glint_hunt', executable='path_planner_node', name='path_planner_node', output='screen', parameters=[LaunchConfiguration('params_file')]),
        Node(package='wild_glint_hunt', executable='state_machine_node', name='state_machine_node', output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
