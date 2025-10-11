import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    motors_config = os.path.join(
        get_package_share_directory('fbot_head'),
        'config',
        'motors.yaml'
        )
        
    return launch.LaunchDescription([
        Node(
            package='fbot_head',
            executable='neck_controller',
            name='neck_controller'),
        Node(
            package='fbot_head',
            executable='emotions_bridge',
            name='emotions_bridge',
            parameters = [motors_config])
    ])

