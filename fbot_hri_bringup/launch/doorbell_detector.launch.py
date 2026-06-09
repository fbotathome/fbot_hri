from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    doorbell_node = Node(
        name='doorbell_detector_node',
        package='fbot_speech',
        executable='doorbell_detector',
        parameters=[{
            'threshold': 0.05,
            'publish_false': False,
            'device_index': -1,
            'model_path': '/home/fbot_ws/src/fbot_hri/fbot_speech/model/yamnet-tensorflow2-yamnet-v1',
        }]
    )

    return LaunchDescription([
        doorbell_node,
    ])
