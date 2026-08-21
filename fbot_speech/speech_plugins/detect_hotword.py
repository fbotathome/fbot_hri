# -*- coding: utf-8 -*-
from operator import index
import struct
import numpy as np
import pyaudio
#import pvporcupine
from openwakeword.model import Model

#access_key="Tbyk0dhsux2oYz/+GO8IGk05dCGmhTVze760CdDlA/vfLjkuGCqdRQ==" 
#access_key = "IOI/v3Jkh0zwgEL5MEC2I0Dn/BVDZ8zfcQdFJhiYntRGFLf37F2gqw=="

class DetectHotWord():
    """
    @brief Class for detecting hotwords using Porcupine.
    This class initializes the Porcupine library with the specified hotword paths and sensitivities.
    It also handles audio input from the microphone and processes the audio frames to detect hotwords.
    """
    def __init__(self,
                keyword_path: list[str],
                sensitivity: float,
                library_path: str = None,
                model_path: str = None):
        """
        @brief Initialize the hotword detector.
        @param keyword_path: List of paths to the hotword models.
        @param sensitivity: Sensitivities for detecting keywords. Each value should be a number within [0, 1]. A higher
        sensitivity results in fewer misses at the cost of increasing the false alarm rate. If not set 0.5 will be used.
        @param library_path: Path to the Porcupine library (optional).
        @param model_path: Path to the Porcupine model (optional).
        """
        #self.handle = pvporcupine.create(access_key=access_key, keyword_paths=keyword_path, sensitivities=sensitivity)
        self.mic = None
        self.handle = Model(wakeword_model_paths=keyword_path)
        self.sensitivity = sensitivity
        self.sample_rate = 16000
        self.frame_length = int(self.sample_rate * 0.08)

        

    def hear(self):
        """
        @brief Initialize the microphone for audio input.
        This function sets up the microphone stream for audio input using PyAudio.
        """
        self.pa = pyaudio.PyAudio()
        audio_stream = self.pa.open(
            rate=self.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.frame_length)
        self.mic = audio_stream

    def process(self):
        """
        @brief Process audio frames to detect hotwords.
        This function reads audio frames from the microphone and processes them using the Porcupine library.
        @return: Index of the detected hotword (0 for first hotword, 1 for second hotword, etc.).
        If no hotword is detected, it returns -1.
        """
        if self.mic is not None:
            pcm = self.mic.read(self.frame_length, exception_on_overflow=False)
            pcm = np.frombuffer(pcm,dtype=np.int16)
            prediction = self.handle.predict(pcm)

            # Check the predictions
            for index, keyword in enumerate(self.handle.models.keys()):
                score = prediction.get(keyword, 0)
                if score >= self.sensitivity[index]:
                    return index #Return an integer representing the index of the hotword detected, from zero.
        return -1

    # def process(self):
    #     if self.mic is not None:
    #         pcm = self.mic.read(self.frame_length, exception_on_overflow=False)
    #         pcm = np.frombuffer(pcm, dtype=np.int16)
    #         prediction = self.handle.predict(pcm)

    #         for index, keyword in enumerate(self.handle.models.keys()):
    #             score = prediction.get(keyword, 0)
    #             print(f"[DEBUG] max_amp={np.abs(pcm).max()} {keyword}={score:.4f}")
    #             if score >= self.sensitivity[index]:
    #                 return index
    #     return -1

    def __del__(self):
        """
        @brief Clean up resources.
        This function closes the microphone stream and terminates the PyAudio instance.
        """
        self.mic.close()
        self.pa.terminate()

    # def __del__(self):
    #     try:
    #         if self.mic is not None:
    #             self.mic.stop()
    #             self.mic.close()
    #     except Exception:
    #         pass