#!/usr/bin/env python3
# coding: utf-8
import numpy as np
import rclpy
import wave
import requests
import io

from fbot_speech_msgs.srv import SynthesizeSpeech, FileSynthesizer
from fbot_speech_msgs.msg import SynthesizeSpeechMessage
from audio_common_msgs.msg import AudioData, AudioInfo
from speech_plugins.wav_to_mouth import WavToMouth
from threading import Event


class FakeResp:
    def __init__(self, audio):
        self.audio = audio


class SpeechSynthesizerNode(WavToMouth):
    def __init__(self):
        super().__init__(node_name='speech_synthesizer')
        self.get_logger().info("Initializing Speech Synthesizer Node...")
        self.declareParameters()
        self.readParameters()
        self.initRosComm()
        self.event = Event()

        self.tts_url = "http://jetson:5002/api/tts"

        self.get_logger().info("Speech Synthesizer Node initialized!")

    def initRosComm(self):
        self.synthesizerSubscriber = self.create_subscription(
            SynthesizeSpeechMessage,
            self.synthesizer_subscriber_param,
            self.synthesizeSpeechCallback,
            10
        )

        self.synthesizerService = self.create_service(
            SynthesizeSpeech,
            self.synthesizer_service_param,
            self.synthesizeSpeech
        )

        self.saveSynthesizerService = self.create_service(
            FileSynthesizer,
            self.save_synthesizer_service_param,
            self.saveSynthesizer
        )

    def declareParameters(self):
        self.declare_parameter('tts_configs.language_code', 'en-US')
        self.declare_parameter('tts_configs.sample_rate_hz', 44100)
        self.declare_parameter('tts_configs.voice_name', 'English-US')

        self.declare_parameter('riva.url', 'localhost:50051')

        self.declare_parameter('services.audio_player_by_data.service', '/fbot_speech/ap/audio_player_by_data')
        self.declare_parameter('services.save_synthesizer.service', '/fbot_speech/ss/save_synthesizer')
        self.declare_parameter('services.speech_synthesizer.service', '/fbot_speech/ss/say_something')
        self.declare_parameter('subscribers.speech_synthesizer.topic', '/fbot_speech/ss/say_something')

    def readParameters(self):
        self.audio_player_by_data_service_param = self.get_parameter(
            'services.audio_player_by_data.service'
        ).get_parameter_value().string_value

        self.synthesizer_service_param = self.get_parameter(
            'services.speech_synthesizer.service'
        ).get_parameter_value().string_value

        self.save_synthesizer_service_param = self.get_parameter(
            'services.save_synthesizer.service'
        ).get_parameter_value().string_value

        self.synthesizer_subscriber_param = self.get_parameter(
            'subscribers.speech_synthesizer.topic'
        ).get_parameter_value().string_value

        self.configs = {
            "language_code": self.get_parameter('tts_configs.language_code').get_parameter_value().string_value,
            "sample_rate_hz": self.get_parameter('tts_configs.sample_rate_hz').get_parameter_value().integer_value,
            "voice_name": self.get_parameter('tts_configs.voice_name').get_parameter_value().string_value,
        }

        self.riva_url = self.get_parameter('riva.url').get_parameter_value().string_value

    def synthesizeSpeech(self, request: SynthesizeSpeech.Request, response: SynthesizeSpeech.Response):
        """
        @brief Synthesize speech from text using Riva TTS.
        @param request: The request object containing the text to synthesize.
        @return: The response object indicating success or failure.
        """
        config = self.configs
        speech = request.text

        try:
            res = requests.get(
                self.tts_url,
                params={"text": speech},
                timeout=120
            )

            if res.status_code != 200:
                response.success = False
                return response

            wav_file = wave.open(io.BytesIO(res.content), 'rb')
            #test
            actual_sample_rate = wav_file.getframerate() 
            
            audio_pcm = wav_file.readframes(wav_file.getnframes())

            self.resp = FakeResp(audio_pcm)

            audio_samples = np.frombuffer(self.resp.audio, dtype=np.int16)

            audio_data = AudioData()
            #audio_data.uint8_data = audio_samples.tobytes()

            #test pc e notebook
            if hasattr(audio_data, 'uint8_data'):
                audio_data.uint8_data = audio_samples.tobytes()
            else:
                audio_data.data = audio_samples.tobytes()

            audio_info = AudioInfo()

            #audio_info.rate = config["sample_rate_hz"]
            #audio_info.channels = 1
            #audio_info.format = 16   
            
            #teste pc e notebook
            if hasattr(audio_info, 'rate'):
                audio_info.rate = config["sample_rate_hz"]
            elif hasattr(audio_info, 'sample_rate'):
                audio_info.sample_rate = config["sample_rate_hz"]

            if hasattr(audio_info, 'channels'):
                audio_info.channels = 1

            if hasattr(audio_info, 'format'):
                audio_info.format = 16 

            '''try:
                if self.streaming:
                    response.success = False
                    return response

                data = audio_data.uint8_data
                info = audio_info
                self.setDataAndInfo(data, info)'''
            
            #aqui test
            try:
                if self.streaming:
                    response.success = False
                    return response

                if hasattr(audio_data, 'uint8_data'):
                    data = audio_data.uint8_data
                else:
                    data = audio_data.data
                    
                class DummyInfo:
                    pass
                
                fake_info = DummyInfo()
                fake_info.rate = actual_sample_rate
                fake_info.sample_rate = actual_sample_rate
                fake_info.channels = wav_file.getnchannels() # puxa variavael
                fake_info.format = 16
                
                self.setDataAndInfo(data, fake_info) #ate aqui teste pc 

                while self.playAllData() != True:
                    continue

                response.success = self.playAllData()
                self.get_logger().info(f"AllData: {response}")

            except Exception as e:
                response.success = False
                self.get_logger().error(f"Error while synthesizing speech voice: {e}")

        except Exception as e:
            response.success = False
            self.get_logger().error(f"Error while synthesizing speech: {e}")

        return response

    def synthesizeSpeechCallback(self, msg: SynthesizeSpeechMessage):
        """
        @brief Callback function for the speech synthesizer subscriber.
        This function is called when a new message is received on the subscriber topic. It extracts the text and language from the message and calls the synthesizeSpeech function.
        @param msg: The message object containing the text and language to synthesize.
        """
        request = SynthesizeSpeech.Request()
        request.text = msg.text
        request.lang = msg.lang
        self.synthesizeSpeech(request, SynthesizeSpeech.Response())

    def saveSynthesizer(self, request: FileSynthesizer.Request, response: FileSynthesizer.Response):
        """
        @brief Save synthesized speech to a file.
        @param request: The request object containing the text to synthesize and the output file path
        @return: The response object indicating success or failure.
        """
        synthesizer = SynthesizeSpeech.Request()
        synthesizer.text = request.text
        synthesizer.lang = 'en-US'

        try:
            out_f = wave.open(request.output_file, 'wb')
            out_f.setnchannels(1)
            out_f.setsampwidth(2)
            out_f.setframerate(self.configs['sample_rate_hz'])

            resultado = self.synthesizeSpeech(synthesizer, SynthesizeSpeech.Response())

            if resultado.success == True and hasattr(self, 'resp'):
                out_f.writeframes(self.resp.audio)
                out_f.close()
                response = FileSynthesizer.Response()
                response.success = True
                return response
            else:
                out_f.close()
                self.get_logger().error("A Jetson não retornou o áudio a tempo. Arquivo não foi salvo.")
                response = FileSynthesizer.Response()
                response.success = False
                return response

        except Exception as e:
            response = SynthesizeSpeech.Response()
            response.success = False
            self.get_logger().error(f"Error while saving file: {e}")
            return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeechSynthesizerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
