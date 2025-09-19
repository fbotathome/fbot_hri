import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='fbot_screen',
            executable='foxglove_xvnc.sh',
            name='foxglove_xvnc_node',
            output='screen'
        ),
        Node(
            package='fbot_screen',
            executable='display_node',
            name='media_display_node',
            output='screen'
        ),

        IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')),
        launch_arguments={
            'port': '9090'
        }.items()
        )
    ])