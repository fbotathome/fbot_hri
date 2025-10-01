#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
import riva.client
import riva.client.proto.riva_asr_pb2 as rasr
import riva.client.audio_io
import os
import time
import threading
from rclpy.node import Node
from std_srvs.srv import Empty
from fbot_speech_msgs.srv import RivaToText
from playsound import playsound
from copy import deepcopy

DEFAULT_LANGUAGE = 'en'
PACK_DIR = os.path.join(os.path.expanduser("~"), 'jetson_ws', 'src', 'fbot_hri', 'fbot_speech')
AUDIO_DIR = os.path.join(PACK_DIR, "audios/")
TALK_AUDIO = os.path.join(AUDIO_DIR, "beep.wav")


class RivaRecognizerNode(Node):
    def __init__(self):
        super().__init__('riva_recognizer_node')
        self.get_logger().info("Initializing Riva Recognizer Node...")
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
                                                            interim_results=False)

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


        with riva.client.audio_io.MicrophoneStream(
                                                        rate =16000,
                                                        chunk=512,
                                                        device=self.device,
                                                    ) as self.audio_chunk_iterator:

            self.output = self.riva_asr.streaming_response_generator(
                    audio_chunks=self.audio_chunk_iterator,
                    streaming_config=self.config)




    def initRosComm(self):
        self.speech_recognition_service = self.create_service(RivaToText, self.recognizer_service_param, self.handleRecognition)
        self.audio_player_beep_service = self.create_client(Empty, self.audio_player_beep_param_service)

    def declareParameters(self):
        self.declare_parameter('riva.url', 'localhost:50051')
        self.declare_parameter('stt_mic_timeout', 10)
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
        self.start_history = self.get_parameter('stt_configs.start_history').get_parameter_value().integer_value
        self.start_threshold = self.get_parameter('stt_configs.start_threshold').get_parameter_value().integer_value
        self.stop_history = self.get_parameter('stt_configs.stop_history').get_parameter_value().integer_value
        self.stop_history_eou = self.get_parameter('stt_configs.stop_history_eou').get_parameter_value().integer_value
        self.stop_threshold = self.get_parameter('stt_configs.stop_threshold').get_parameter_value().double_value
        self.stop_threshold_eou = self.get_parameter('stt_configs.stop_threshold_eou').get_parameter_value().double_value
        self.riva_url = self.get_parameter('riva.url').get_parameter_value().string_value


    def delayStarterRecorder(self):
        time.sleep(0.75)
        self.audio_player_beep_service.call(Empty.Request())
        #playsound(TALK_AUDIO)
    
    def handleRecognition(self, req: RivaToText.Request, res: RivaToText.Response):
        """
        @brief Callback function for the speech recognition service.
        This function is called when a new request is received for speech recognition
        """
            
        good_output = ''
        bad_output = ''
        very_bad_output = ''
        delay_starter = threading.Thread(target=self.delayStarterRecorder)
        
        if req.sentence:
            self.sentence = True
            self.word = False
        else:
            self.sentence = False
            self.word = True
        
        start = time.time() + self.stt_mic_timeout
        delay_starter.start()
        for response in self.output:
            if (start > time.time()):
                if not response.results:
                    continue
                for result in response.results:
                    if not result.alternatives:
                        continue
                if result.is_final:
                    if self.word:
                        if result.alternatives[0].words[0].word in req.boosted_lm_words:
                            if result.alternatives[0].words[0].confidence >=0.6:
                                good_output = result.alternatives[0].words[0].word
                                res.text = good_output
                                self.audio_chunk_iterator.close()
                                return res
                            else:
                                bad_output = result.alternatives[0].words[0].word
                        else:
                            very_bad_output = result.alternatives[0].words[0].word
                    elif self.sentence:
                        for alternative in result.alternatives:
                            for word in alternative.words:
                                if word.word in req.boosted_lm_words:
                                    res.text = result.alternatives[0].transcript 
                                    self.audio_chunk_iterator.close()
                                    return res
                                else:
                                    bad_output = result.alternatives[0].transcript
            else:
                self.audio_chunk_iterator.close()
                if bad_output != '':
                    res.text = bad_output
                    return res
                else:
                    res.text = very_bad_output
                    return res

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    riva_recognizer_node = RivaRecognizerNode()

    # Spin the node to keep it running
    rclpy.spin(riva_recognizer_node)

    # Clean up before shutting down
    riva_recognizer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()