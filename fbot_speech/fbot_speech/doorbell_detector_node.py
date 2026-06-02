#!/usr/bin/env python3
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import sounddevice as sd
import tensorflow as tf
import tensorflow_hub as hub

# AudioSet class indices related to bell/doorbell sounds.
# 395 = Doorbell, 396 = Bell, 397 = Jingle bell
# NOTE: 42 = Speech — removed to avoid false positives in competition arenas.
DOORBELL_CLASS_INDICES = [395, 396, 397]

SAMPLE_RATE    = 16000   # YAMNet expects 16 kHz mono
CHUNK_DURATION = 0.975   # seconds — YAMNet native window
CHUNK_SAMPLES  = int(SAMPLE_RATE * CHUNK_DURATION)


class DoorbellDetectorNode(Node):
    """
    ROS2 node that listens to the microphone and publishes to
    /fbot_speech/doorbell/detected (std_msgs/Bool) whenever a doorbell
    sound is detected via YAMNet.

    The audio loop runs in a background thread so rclpy.spin() is never
    blocked.

    ROS2 parameters
    ---------------
    threshold     float  (default 0.3)  Confidence threshold [0, 1].
    publish_false bool   (default False) Publish False on non-detections.
    device_index  int    (default -1)   Sounddevice input device; -1 = default.
    model_path    string (default '')   Local path to a saved YAMNet model.
                                        Empty = download from TF Hub on first run
                                        (cached in ~/.cache/tfhub_modules/).
    """

    TFHUB_URL = 'https://tfhub.dev/google/yamnet/1'

    def __init__(self):
        super().__init__('doorbell_detector_node')

        # Parameters
        self.declare_parameter('threshold',     0.05)
        self.declare_parameter('publish_false', False)
        self.declare_parameter('device_index',  -1)
        self.declare_parameter('model_path', '/home/othavio/fbot_ws/src/fbot_hri/fbot_speech/model/yamnet-tensorflow2-yamnet-v1')

        # FIX: read each parameter only once, after all declare_parameter calls
        self.threshold     = self.get_parameter('threshold').value
        self.publish_false = self.get_parameter('publish_false').value
        device_param       = self.get_parameter('device_index').value
        self.device_index  = None if device_param == -1 else device_param
        self.model_path    = self.get_parameter('model_path').value

        # Publisher 
        self.publisher = self.create_publisher(
            Bool, '/fbot_speech/doorbell/detected', 10
        )

        #Load YAMNet 
        source = self.model_path if self.model_path else self.TFHUB_URL
        self.get_logger().info(f'Loading YAMNet from: {source}')
        self.model = hub.load(source)
        self.get_logger().info('YAMNet loaded successfully.')

        # Audio loop in background thread 
        # _run_audio_loop() is blocking (it reads from the mic in a while loop). If called directly in __init__, it would block
        # rclpy.spin() from ever starting, making the node unable to process ROS2 callbacks or receive the shutdown signal.
        self._stop_event = threading.Event()
        self._audio_thread = threading.Thread(
            target=self._run_audio_loop,
            daemon=True,
            name='doorbell_audio_loop',
        )
        self._audio_thread.start()

        self.get_logger().info(
            f'Doorbell detector running | threshold={self.threshold} | '
            f'device={self.device_index if self.device_index is not None else "default"}'
        )

    # Audio loop 

    def _run_audio_loop(self):
        """
        Blocking loop that runs in a background thread.
        Reads audio chunks from the microphone and classifies each one.
        """
        self.get_logger().info('Audio capture thread started.')
        try:
            with sd.InputStream(
                samplerate=SAMPLE_RATE,
                channels=1,
                dtype='float32',
                blocksize=CHUNK_SAMPLES,
                device=self.device_index,
            ) as stream:
                while not self._stop_event.is_set() and rclpy.ok():
                    audio_chunk, _ = stream.read(CHUNK_SAMPLES)
                    self._process_chunk(audio_chunk.flatten())
        except Exception as e:
            self.get_logger().error(f'Audio capture thread error: {e}')

    # Inference 

    def _process_chunk(self, waveform: np.ndarray):
        """
        Runs YAMNet inference on a single audio chunk.
        Publishes Bool(True) when doorbell confidence >= threshold.

        @param waveform: 1-D float32 array, 16 kHz mono.
        """
        scores, _, _ = self.model(tf.constant(waveform, dtype=tf.float32))

        # scores shape: [frames, 521] — max confidence per class across frames
        max_scores = scores.numpy().max(axis=0)
        confidence = float(max(max_scores[i] for i in DOORBELL_CLASS_INDICES))

        self.get_logger().debug(f'Doorbell confidence: {confidence:.3f}')

        detected = confidence >= self.threshold

        if detected:
            self.get_logger().info(f'Doorbell detected! confidence={confidence:.3f}')

        if detected or self.publish_false:
            msg = Bool()
            msg.data = detected
            self.publisher.publish(msg)

    #Cleanup 

    def destroy_node(self):
        """Signals the audio thread to stop before destroying the node."""
        self._stop_event.set()
        self._audio_thread.join(timeout=3.0)
        super().destroy_node()


# Entry point 

def main(args=None):
    rclpy.init(args=args)
    node = DoorbellDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
