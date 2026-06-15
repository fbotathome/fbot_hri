from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_remote_ssh import NodeRemoteSSH, FindPackageShareRemote

def generate_launch_description():
    use_remote_arg = DeclareLaunchArgument(
        'use_remote',
        default_value='false',
        description="If should run the node on remote"
    )

    ros_config_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_hri_bringup'),
        'config',
        'ros.yaml']
    )

    ros_config_path = PathJoinSubstitution([
        get_package_share_directory('fbot_hri_bringup'),
        'config',
        'ros.yaml']
    )

    ros_config_file_remote_arg = DeclareLaunchArgument(
        'ros_config_remote',
        default_value=ros_config_path_remote,
        description='Path to the ros parameter file'
    )

    ros_config_file_arg = DeclareLaunchArgument(
        'ros_config',
        default_value=ros_config_path,
        description='Path to the ros parameter file'
    )

    stt_config_file_path_remote = PathJoinSubstitution([
        FindPackageShareRemote(remote_install_space='/home/jetson/jetson_ws/install', package='fbot_hri_bringup'),
        'config',
        'asr_riva_gpsr.yaml']
    )

    stt_config_file_path = PathJoinSubstitution([
        get_package_share_directory('fbot_hri_bringup'),
        'config',
        'asr_riva_gpsr.yaml']
    )

    stt_config_file_remote_arg = DeclareLaunchArgument(
        'stt_config_remote',
        default_value=stt_config_file_path_remote,
        description='Path to the stt parameter file'
    )

    stt_config_file_arg = DeclareLaunchArgument(
        'stt_config',
        default_value=stt_config_file_path,
        description='Path to the stt parameter file'
    )

    riva_recognizer_gpsr_remote_node = NodeRemoteSSH(
        name='riva_recognizer_gpsr_node',
        package='fbot_speech',
        executable='asr_riva_gpsr',
        parameters=[LaunchConfiguration('ros_config_remote'),
                    LaunchConfiguration('stt_config_remote')
                    ],
        user='jetson',
        machine="jetson",
        source_paths=[
            "/home/jetson/jetson_ws/install/setup.bash"
        ],
        condition=IfCondition(LaunchConfiguration('use_remote'))
    )

    riva_recognizer_gpsr_node = Node(
        name='riva_recognizer_gpsr_node',
        package='fbot_speech',
        executable='asr_riva_gpsr',
        parameters=[LaunchConfiguration('ros_config'),
                    LaunchConfiguration('stt_config')
                    ],
        condition=UnlessCondition(LaunchConfiguration('use_remote'))
    )

    return LaunchDescription([
        use_remote_arg,
        ros_config_file_arg,
        ros_config_file_remote_arg,
        stt_config_file_arg,
        stt_config_file_remote_arg,
        riva_recognizer_gpsr_remote_node,
        riva_recognizer_gpsr_node
    ])
