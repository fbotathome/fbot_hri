#!/usr/bin/env python3
import os
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool
from termcolor import colored
from speech_plugins.detect_doorbell import DetectDoorbell


class DoorbellDetectorNode(Node):
    """
    @brief ROS node that detects a doorbell sound through the microphone.
    A reference doorbell audio sample is provided by parameter. The node
    continuously listens to the microphone and, whenever the incoming audio
    matches the reference sample, it publishes a message on the detection topic.
    """

    def __init__(self):
        """
        @brief Constructor for the DoorbellDetectorNode class.
        Declares and reads the parameters, sets up ROS communication and starts
        the detection loop.
        """
        super().__init__('doorbell_detector_node')

        self.declareParameters()
        self.readParameters()
        self.initRosComm()

        if not os.path.isfile(self.sample_path):
            self.get_logger().error(f"Doorbell sample not found: {self.sample_path}")
            raise FileNotFoundError(self.sample_path)

        self.detector = DetectDoorbell(
            sample_path=self.sample_path,
            sample_rate=self.sample_rate,
            n_mfcc=self.n_mfcc,
            threshold=self.threshold)
        self.detector.hear()

        self.get_logger().info(colored("Doorbell Detector is on!", "green"))

        # Number of consecutive listening loops to wait before a new detection
        # can be published, to avoid publishing many times for a single ring.
        cooldown_loops = 0

        while rclpy.ok():
            similarity = self.detector.process()
            if cooldown_loops > 0:
                cooldown_loops -= 1
                continue
            if self.detector.is_detected(similarity):
                self.get_logger().warn(
                    f"Doorbell detected! (similarity: {similarity:.2f})")
                self.doorbell_publisher.publish(Bool(data=True))
                cooldown_loops = self.cooldown
            else:
                self.get_logger().error(
                    f"Doorbell not detected (similarity: {similarity:.2f})")

    def initRosComm(self):
        """
        @brief Initialize ROS communication for the node.
        Sets up the publisher used to report doorbell detections.
        """
        self.doorbell_publisher = self.create_publisher(Bool, self.detector_publisher_param, 10)

    def declareParameters(self):
        """
        @brief Declare parameters for the node.
        """
        self.declare_parameter('fbot_doorbell_detection.sample_path', '')
        self.declare_parameter('fbot_doorbell_detection.sample_rate', 16000)
        self.declare_parameter('fbot_doorbell_detection.n_mfcc', 20)
        self.declare_parameter('fbot_doorbell_detection.threshold', 0.85)
        self.declare_parameter('fbot_doorbell_detection.cooldown', 20)
        self.declare_parameter('publishers.fbot_doorbell_detection.topic', '/fbot_speech/doorbell/detected')

    def readParameters(self):
        """
        @brief Read parameters from the ROS parameter server.
        """
        self.sample_path = self.get_parameter('fbot_doorbell_detection.sample_path').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('fbot_doorbell_detection.sample_rate').get_parameter_value().integer_value
        self.n_mfcc = self.get_parameter('fbot_doorbell_detection.n_mfcc').get_parameter_value().integer_value
        self.threshold = self.get_parameter('fbot_doorbell_detection.threshold').get_parameter_value().double_value
        self.cooldown = self.get_parameter('fbot_doorbell_detection.cooldown').get_parameter_value().integer_value
        self.detector_publisher_param = self.get_parameter('publishers.fbot_doorbell_detection.topic').get_parameter_value().string_value


def main(args=None):
    rclpy.init(args=args)

    node = DoorbellDetectorNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
