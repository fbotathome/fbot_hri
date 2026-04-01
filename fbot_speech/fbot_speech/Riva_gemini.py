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
from copy import deepcopy
import queue # Adicionado para gerenciar o áudio

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
        
        # --- A GRANDE MUDANÇA: O microfone é aberto na inicialização e fica vivo ---
        self.get_logger().info("Abrindo a stream do microfone...")
        self.mic_stream = riva.client.audio_io.MicrophoneStream(
            rate=16000,
            chunk=512,
            device=self.device,
        )
        self.mic_stream.__enter__() # Inicializa a stream manualmente
        self.get_logger().info("Microfone pronto e gravando continuamente em background.")

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
        self.audio_player_beep_service.call_async(Empty.Request()) # Usando call_async para evitar travamentos
    
    # Criamos um gerador customizado para limpar o buffer antigo antes de enviar o áudio
    def get_audio_generator(self):
        self._stop_yielding = False # Flag para parar o envio de áudio
        
        # Esvazia o buffer de áudio que ficou gravando desde a última chamada
        try:
            while True:
                self.mic_stream._buff.get_nowait()
        except queue.Empty:
            pass
            
        # Yield do novo áudio verificando a flag
        while not self._stop_yielding:
            try:
                # O timeout de 0.1s destrava a thread rapidamente para checar a flag
                data = [self.mic_stream._buff.get(timeout=0.1)]
                
                # Coleta todo o restante do áudio disponível instantaneamente
                while True:
                    try:
                        data.append(self.mic_stream._buff.get_nowait())
                    except queue.Empty:
                        break
                        
                yield b''.join(data)
            except queue.Empty:
                continue

    def handleRecognition(self, req: RivaToText.Request, res: RivaToText.Response):
        config_service = deepcopy(self.config)
        self.get_logger().info("Inicio do servico")
        res.text = "" 
        
        speech_context = rasr.SpeechContext()
        good_output = ''
        bad_output = ''
        very_bad_output = ''
        
        delay_starter = threading.Thread(target=self.delayStarterRecorder)
        
        if req.boosted_lm_words != '':
            speech_context.phrases.extend(req.boosted_lm_words)
            speech_context.boost = req.boost
            config_service.config.speech_contexts.extend([speech_context])
            self.get_logger().info("Boosted")
        
        if req.sentence:
            self.sentence = True
            self.word = False
        else:
            self.sentence = False
            self.word = True

        # Usa o gerador que limpa o lixo e escuta áudio fresco
        audio_generator = self.get_audio_generator()

        output = self.riva_asr.streaming_response_generator(
                audio_chunks=audio_generator,
                streaming_config=config_service)
        
        self.get_logger().info("Output do gRPC criado")
        start = time.time() + self.stt_mic_timeout
        delay_starter.start()
        
        stop_requested = False 

        for response in output:
            if time.time() > start and not stop_requested:
                self.get_logger().warn("Timeout do microfone atingido.")
                # Aqui NÃO fechamos o microfone, apenas sinalizamos pro gRPC fechar o gerador
                self._stop_yielding = True # Sinaliza pro gerador parar
                stop_requested = True
                continue 

            if not response.results or stop_requested:
                continue
            
            result = response.results[0]
            if not result.alternatives:
                continue
                
            if result.is_final:
                if self.word:
                    recognized_word = result.alternatives[0].words[0].word
                    confidence = result.alternatives[0].words[0].confidence
                    
                    if recognized_word in req.boosted_lm_words:
                        if confidence >= 0.6:
                            res.text = recognized_word
                            self._stop_yielding = True # Sinaliza pro gerador parar
                            stop_requested = True # Ignora novos resultados
                        else:
                            bad_output = recognized_word
                    else:
                        very_bad_output = recognized_word
                        
                elif self.sentence:
                    transcript = result.alternatives[0].transcript
                    found_boosted = False
                    for alternative in result.alternatives:
                        for word in alternative.words:
                            if word.word in req.boosted_lm_words:
                                res.text = transcript 
                                found_boosted = True
                                break 
                        if found_boosted:
                            break 
                            
                    if found_boosted:
                        self._stop_yielding = True # Sinaliza pro gerador parar
                        stop_requested = True 
                    else:
                        bad_output = transcript

        self.get_logger().info("Processamento da fala encerrado.")
        
        if res.text == "":
            if bad_output != '':
                res.text = bad_output
            else:
                res.text = very_bad_output
                
        self.get_logger().info(f"Retornando reconhecimento: {res.text}")
        return res
        
    def __del__(self):
        # Desliga o microfone de verdade quando o Node morre
        if hasattr(self, 'mic_stream'):
            self.get_logger().info("Fechando stream do microfone...")
            self.mic_stream.__exit__(None, None, None)

def main(args=None):
    rclpy.init(args=args)
    riva_recognizer_node = RivaRecognizerNode()
    try:
        rclpy.spin(riva_recognizer_node)
    except KeyboardInterrupt:
        pass
    finally:
        riva_recognizer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()