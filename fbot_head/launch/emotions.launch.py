import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    motors_config = os.path.join(
        get_package_share_directory('fbot_head'),
        'config',
        'motors.yaml'
        )
    
    default_blink_arg = DeclareLaunchArgument(
        'default_blink',
        default_value='False',
        description='Defines whether the robot should start with blinking enabled (True) or disabled (False).'
    )
    
    default_blink_config = LaunchConfiguration('default_blink')

    return launch.LaunchDescription([
        default_blink_arg,
        Node(
            package='fbot_head',
            executable='emotions_bridge',
            name='emotions_bridge',

            parameters = [motors_config, {'default_blink': default_blink_config}]),

