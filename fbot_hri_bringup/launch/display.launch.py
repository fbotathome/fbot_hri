import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    display_config_arg = DeclareLaunchArgument(
        'display_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', 'media_display.yaml']),
        description='Path to the display_node parameter file'
    )

    foxglove_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')),
        launch_arguments={
            'port': '9090'
        }.items())
    
    display_node = Node(
            package='fbot_screen',
            executable='display_node',
            name='media_display_node',
            output='screen',
            parameters=[LaunchConfiguration('display_config')])
    vnc_node = Node(
            package='fbot_screen',
            executable='foxglove_xvnc.sh',
            name='foxglove_xvnc_node',
            output='screen')
        

    return LaunchDescription([
        display_config_arg,
        foxglove_bridge_launch,
        display_node,
        vnc_node
    ])