# -*- coding: utf-8 -*-
import numpy as np
import pyaudio
#import pvporcupine
from openwakeword.model import Model

class DetectHotWord():
    """
    @brief Class for detecting hotwords using OpenWakeWord.
    This class loads the specified wakeword models and sensitivities.
    It also handles microphone input and processes audio frames to detect hotwords.
    """
    def __init__(self,
                keyword_path: list[str],
                sensitivity: float,
                library_path: str = None,
                model_path: str = None):
        """
        @brief Initialize the hotword detector.
        @param keyword_path: List of paths to the OpenWakeWord models.
        @param sensitivity: List of detection thresholds, one for each model. Each value should be in [0, 1].
        @param library_path: Unused compatibility parameter for the previous Porcupine implementation.
        @param model_path: Unused compatibility parameter for the previous Porcupine implementation.
        """
        
        self.mic = None
        self.handle = Model(wakeword_models=keyword_path, inference_framework="onnx")
        self.sensitivity = sensitivity
        self.sample_rate = 16000
        self.frame_length = int(self.sample_rate * 0.08)
        self.cooldown_frames = 25  # ~2s a 80ms/frame, ignora detecções logo após um acerto
        self.frames_since_detection = self.cooldown_frames

        

    def hear(self):
        """
        @brief Initialize the microphone stream for audio input.
        The stream uses mono 16-bit PCM audio at 16 kHz with 80 ms frames.
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
        This function reads one audio frame, obtains a prediction from OpenWakeWord,
        and compares each model score with its configured sensitivity. Detection is
        temporarily suppressed for the cooldown period after a successful detection.
        @return: Index of the detected hotword in keyword_path, or -1 if no hotword
        is detected, the microphone is not initialized, or the detector is cooling down.
        """

        if self.mic is not None:
            pcm = self.mic.read(self.frame_length, exception_on_overflow=False)
            pcm = np.frombuffer(pcm,dtype=np.int16)
            prediction = self.handle.predict(pcm)
            self.frames_since_detection += 1

            if self.frames_since_detection < self.cooldown_frames:
                return -1

            # Check the predictions
            for keyword_index, keyword in enumerate(self.handle.models.keys()):
                score = prediction.get(keyword, 0)
                if score >= self.sensitivity[keyword_index]:
                    self.frames_since_detection = 0
                    self.handle.reset()
                    return keyword_index
        return -1


    def __del__(self):
        """
        @brief Clean up resources.
        This function closes the microphone stream and terminates PyAudio.
        """
        self.mic.close()
        self.pa.terminate()