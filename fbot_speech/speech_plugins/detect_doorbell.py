# -*- coding: utf-8 -*-
import numpy as np
import librosa
import pyaudio


class DetectDoorbell():
    """
    @brief Class for detecting a doorbell sound using MFCC template matching.
    This class loads a reference doorbell audio sample and extracts an MFCC
    based fingerprint from it. It then continuously reads audio from the
    microphone, keeps a rolling buffer with the same duration as the reference
    sample, and compares the fingerprint of the buffer with the reference one
    using cosine similarity. When the similarity is above a threshold the
    doorbell is considered detected.
    """

    def __init__(self,
                 sample_path: str,
                 sample_rate: int = 16000,
                 n_mfcc: int = 20,
                 threshold: float = 0.85,
                 frame_length: int = 2048):
        """
        @brief Initialize the doorbell detector.
        @param sample_path: Path to the reference doorbell audio file (.wav).
        @param sample_rate: Sample rate used to process the audio (Hz).
        @param n_mfcc: Number of MFCC coefficients used to build the fingerprint.
        @param threshold: Cosine similarity threshold within [0, 1]. A higher
        value results in fewer false alarms at the cost of more misses.
        @param frame_length: Number of samples read from the microphone per chunk.
        """
        self.sample_rate = sample_rate
        self.n_mfcc = n_mfcc
        self.threshold = threshold
        self.frame_length = frame_length

        # Load the reference sample and build its fingerprint.
        reference, _ = librosa.load(sample_path, sr=self.sample_rate, mono=True)
        self.reference_duration = len(reference) / float(self.sample_rate)
        self.buffer_size = max(len(reference), self.frame_length)
        self.reference_fingerprint = self._fingerprint(reference)
        # Reference energy is used to gate detection when the mic is (almost) silent.
        self.reference_energy = float(np.sqrt(np.mean(reference ** 2)))

        # Rolling buffer of microphone samples (float32 in [-1, 1]).
        self.buffer = np.zeros(self.buffer_size, dtype=np.float32)

        self.pa = None
        self.mic = None

    def _fingerprint(self, signal: np.ndarray) -> np.ndarray:
        """
        @brief Build a fixed-size fingerprint from an audio signal.
        The fingerprint is the concatenation of the mean and standard deviation
        of the MFCC coefficients, normalized to unit norm so that it can be
        compared with cosine similarity regardless of loudness.
        @param signal: Mono audio signal as a float array.
        @return: 1-D normalized fingerprint vector.
        """
        mfcc = librosa.feature.mfcc(y=signal, sr=self.sample_rate, n_mfcc=self.n_mfcc)
        feature = np.concatenate([mfcc.mean(axis=1), mfcc.std(axis=1)])
        norm = np.linalg.norm(feature)
        if norm > 0.0:
            feature = feature / norm
        return feature

    def hear(self):
        """
        @brief Initialize the microphone for audio input using PyAudio.
        """
        self.pa = pyaudio.PyAudio()
        self.mic = self.pa.open(
            rate=self.sample_rate,
            channels=1,
            format=pyaudio.paInt16,
            input=True,
            frames_per_buffer=self.frame_length)

    def process(self) -> float:
        """
        @brief Read a chunk of audio and check for the doorbell.
        Reads one chunk from the microphone, appends it to the rolling buffer
        and compares the buffer fingerprint with the reference one.
        @return: Cosine similarity in [0, 1] between the current buffer and the
        reference sample, or -1.0 if the microphone is not initialized.
        """
        if self.mic is None:
            return -1.0

        pcm = self.mic.read(self.frame_length, exception_on_overflow=False)
        samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0

        # Shift the rolling buffer and append the new samples.
        self.buffer = np.roll(self.buffer, -len(samples))
        self.buffer[-len(samples):] = samples

        # Ignore near-silence to avoid matching background noise.
        energy = float(np.sqrt(np.mean(self.buffer ** 2)))
        if energy < 0.1 * self.reference_energy:
            return 0.0

        fingerprint = self._fingerprint(self.buffer)
        similarity = float(np.dot(fingerprint, self.reference_fingerprint))
        return similarity

    def is_detected(self, similarity: float) -> bool:
        """
        @brief Check whether a similarity value counts as a detection.
        @param similarity: Similarity returned by process().
        @return: True if the similarity is above the configured threshold.
        """
        return similarity >= self.threshold

    def __del__(self):
        """
        @brief Clean up the microphone and PyAudio resources.
        """
        if self.mic is not None:
            self.mic.close()
        if self.pa is not None:
            self.pa.terminate()
