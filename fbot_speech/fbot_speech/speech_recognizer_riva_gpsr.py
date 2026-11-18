#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import riva.client
import riva.client.proto.riva_asr_pb2 as rasr
import riva.client.audio_io
import queue
import time
import threading
from rclpy.node import Node
from std_srvs.srv import Empty
from fbot_speech_msgs.srv import RivaToText
from copy import deepcopy

DEFAULT_LANGUAGE = 'en'


class RivaRecognizerGpsrNode(Node):
    def __init__(self):
        super().__init__('riva_recognizer_gpsr_node')
        self.get_logger().info("Initializing Riva Recognizer GPSR Node...")
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        auth = riva.client.Auth(uri=self.riva_url)
        self.riva_asr = riva.client.ASRService(auth)
        self.config = riva.client.StreamingRecognitionConfig(
                                                            config = riva.client.RecognitionConfig(
                                                                    encoding=riva.client.AudioEncoding.LINEAR_PCM,
                                                                    language_code='en-US',
                                                                    max_alternatives=1,
                                                                    profanity_filter=False,
                                                                    enable_automatic_punctuation=False,
                                                                    verbatim_transcripts=False,
                                                                    sample_rate_hertz=16000,
                                                                    audio_channel_count=1,
                                                                ),
                                                            interim_results=True)

        riva.client.add_endpoint_parameters_to_config(
            self.config,
            start_history= self.start_history,
            start_threshold= self.start_threshold,
            stop_history= self.stop_history,
            stop_history_eou= self.stop_history_eou,
            stop_threshold= self.stop_threshold,
            stop_threshold_eou= self.stop_threshold_eou,
        )

        default_device_info = riva.client.audio_io.get_default_input_device_info()
        self.device = default_device_info['index']

        self.get_logger().info("Microphone Opened: {}".format(default_device_info['name']))
        self.mic_stream = riva.client.audio_io.MicrophoneStream(
            rate=16000,
            chunk=512,
            device=self.device,
        )
        self.mic_stream.__enter__()
        self._stop_yielding = False
        self.get_logger().info("Microphone Stream Initialized.")


    def initRosComm(self):
        self.speech_recognition_service = self.create_service(RivaToText, self.recognizer_service_param, self.handleRecognition)
        self.audio_player_beep_service = self.create_client(Empty, self.audio_player_beep_param_service)

    def declareParameters(self):
        self.declare_parameter('riva.url', 'localhost:50051')
        self.declare_parameter('stt_mic_timeout', 25)
        self.declare_parameter('stt_silence_timeout', 4.0)
        self.declare_parameter('stt_configs.start_history', 50)
        self.declare_parameter('stt_configs.start_threshold', -1)
        self.declare_parameter('stt_configs.stop_history', 3500)
        self.declare_parameter('stt_configs.stop_history_eou', 1500)
        self.declare_parameter('stt_configs.stop_threshold', 0.8)
        self.declare_parameter('stt_configs.stop_threshold_eou', 0.9)
        self.declare_parameter('services.audio_player_beep.service', '/fbot_speech/ap/audio_beep')
        self.declare_parameter('services.asr_recognizer.service', '/fbot_speech/sr/asr_recognizer')


    def readParameters(self):
        self.audio_player_beep_param_service = self.get_parameter('services.audio_player_beep.service').get_parameter_value().string_value
        self.recognizer_service_param = self.get_parameter('services.asr_recognizer.service').get_parameter_value().string_value
        self.stt_mic_timeout = self.get_parameter('stt_mic_timeout').get_parameter_value().integer_value
        self.stt_silence_timeout = self.get_parameter('stt_silence_timeout').get_parameter_value().double_value
        self.start_history = self.get_parameter('stt_configs.start_history').get_parameter_value().integer_value
        self.start_threshold = self.get_parameter('stt_configs.start_threshold').get_parameter_value().integer_value
        self.stop_history = self.get_parameter('stt_configs.stop_history').get_parameter_value().integer_value
        self.stop_history_eou = self.get_parameter('stt_configs.stop_history_eou').get_parameter_value().integer_value
        self.stop_threshold = self.get_parameter('stt_configs.stop_threshold').get_parameter_value().double_value
        self.stop_threshold_eou = self.get_parameter('stt_configs.stop_threshold_eou').get_parameter_value().double_value
        self.riva_url = self.get_parameter('riva.url').get_parameter_value().string_value


    def delayStarterRecorder(self):
        time.sleep(0.75)
        self.audio_player_beep_service.call_async(Empty.Request())

    def get_audio_generator(self):
        """Yield audio chunks from the persistent microphone stream until
        self._stop_yielding is set. Drains any backlog first so each request
        starts listening from 'now', not from stale buffered audio."""
        self._stop_yielding = False

        try:
            while True:
                self.mic_stream._buff.get_nowait()
        except queue.Empty:
            pass

        while not self._stop_yielding:
            try:
                data = [self.mic_stream._buff.get(timeout=0.01)]

                while True:
                    try:
                        data.append(self.mic_stream._buff.get_nowait())
                    except queue.Empty:
                        break

                yield b''.join(data)
            except queue.Empty:
                continue

    def handleRecognition(self, req: RivaToText.Request, res: RivaToText.Response):
        """
        @brief Callback for the GPSR speech recognition service.
        Listens until the operator goes quiet for stt_silence_timeout seconds
        (or stt_mic_timeout is reached as a hard cap) and returns every spoken
        segment concatenated, so multiple commands spoken in one breath (with
        pauses shorter than stt_silence_timeout between them) are all captured.
        The last segment is kept even if it was not finalized before the stop.
        """
        config_service = deepcopy(self.config)

        if req.boosted_lm_words:
            speech_context = rasr.SpeechContext()
            speech_context.phrases.extend(req.boosted_lm_words)
            speech_context.boost = req.boost
            config_service.config.speech_contexts.extend([speech_context])

        delay_starter = threading.Thread(target=self.delayStarterRecorder)

        audio_generator = self.get_audio_generator()
        output = self.riva_asr.streaming_response_generator(
                audio_chunks=audio_generator,
                streaming_config=config_service)

        deadline = time.time() + self.stt_mic_timeout
        silence_limit = self.stt_silence_timeout
        delay_starter.start()

        segments = []
        last_partial = ""
        last_change = time.time()
        stop_requested = False
        for response in output:
            now = time.time()
            if not stop_requested:
                for result in response.results:
                    if not result.alternatives:
                        continue
                    transcript = result.alternatives[0].transcript.strip()
                    if not transcript:
                        continue
                    if result.is_final:
                        segments.append(transcript)
                        last_partial = ""
                        self.get_logger().info(f"[GPSR ASR] final segment: {transcript}")
                    elif transcript != last_partial:
                        last_partial = transcript
                        last_change = now

                if now >= deadline:
                    self.get_logger().warn("[GPSR ASR] hard timeout reached")
                    if last_partial:
                        segments.append(last_partial)
                        last_partial = ""
                    self._stop_yielding = True
                    stop_requested = True
                elif now - last_change >= silence_limit:
                    self.get_logger().info("[GPSR ASR] end of speech (silence)")
                    if last_partial:
                        segments.append(last_partial)
                        last_partial = ""
                    self._stop_yielding = True
                    stop_requested = True

        res.text = ' '.join(segments)
        self.get_logger().info(f"[GPSR ASR] full transcript: {res.text}")
        return res


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    riva_recognizer_gpsr_node = RivaRecognizerGpsrNode()

    # Spin the node to keep it running
    rclpy.spin(riva_recognizer_gpsr_node)

    # Clean up before shutting down
    riva_recognizer_gpsr_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
