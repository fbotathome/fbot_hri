from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    doorbell_config_file_arg = DeclareLaunchArgument(
        'doorbell_config_file',
        default_value='fbot_doorbell_detection.yaml',
        description='Name of the ros parameter file inside the config folder'
    )

    doorbell_config_arg = DeclareLaunchArgument(
        'doorbell_config',
        default_value=PathJoinSubstitution([FindPackageShare('fbot_hri_bringup'), 'config', LaunchConfiguration('doorbell_config_file')]),
        description='Path to the ros parameter file'
    )

    doorbell_node = Node(
        name='doorbell_detector_node',
        package='fbot_speech',
        executable='doorbell_detector_node',
        parameters=[LaunchConfiguration('doorbell_config')]
    )

    return LaunchDescription([
        doorbell_config_file_arg,
        doorbell_config_arg,
        doorbell_node
    ])
