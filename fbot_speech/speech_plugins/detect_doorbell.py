# -*- coding: utf-8 -*-
import os
import numpy as np
import librosa
import pyaudio


class DetectDoorbell():
    """
    @brief Class for detecting a doorbell sound using MFCC template matching.
    This class loads one or more reference doorbell audio samples and extracts
    an MFCC based fingerprint from each of them. It then continuously reads
    audio from the microphone, keeps a rolling buffer as long as the longest
    reference sample, and compares the fingerprint of the matching-length
    window with every reference fingerprint using cosine similarity. When the
    similarity of any reference is above a threshold the doorbell is considered
    detected.
    """

    def __init__(self,
                 sample_paths,
                 sample_rate: int = 16000,
                 n_mfcc: int = 20,
                 threshold: float = 0.85,
                 frame_length: int = 2048):
        """
        @brief Initialize the doorbell detector.
        @param sample_paths: Path (str) or list of paths to the reference
        doorbell audio files (.wav). Any of them triggers a detection.
        @param sample_rate: Sample rate used to process the audio (Hz).
        @param n_mfcc: Number of MFCC coefficients used to build the fingerprint.
        @param threshold: Cosine similarity threshold within [0, 1]. A higher
        value results in fewer false alarms at the cost of more misses.
        @param frame_length: Number of samples read from the microphone per chunk.
        """
        if isinstance(sample_paths, str):
            sample_paths = [sample_paths]

        self.sample_rate = sample_rate
        self.n_mfcc = n_mfcc
        self.threshold = threshold
        self.frame_length = frame_length

        # One entry per reference sample with its name, window length,
        # fingerprint and RMS energy (used to gate near-silence).
        self.references = []
        for path in sample_paths:
            signal, _ = librosa.load(path, sr=self.sample_rate, mono=True)
            self.references.append({
                'name': os.path.splitext(os.path.basename(path))[0],
                'length': len(signal),
                'fingerprint': self._fingerprint(signal),
                'energy': float(np.sqrt(np.mean(signal ** 2))),
            })

        if not self.references:
            raise ValueError("At least one doorbell sample must be provided.")

        # Rolling buffer sized to the longest reference sample.
        self.buffer_size = max(self.frame_length,
                               max(ref['length'] for ref in self.references))
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

    def process(self):
        """
        @brief Read a chunk of audio and check for any of the doorbell samples.
        Reads one chunk from the microphone, appends it to the rolling buffer
        and compares, for each reference, the fingerprint of the matching-length
        window with the reference fingerprint.
        @return: Tuple (name, similarity) of the best matching reference, where
        similarity is the cosine similarity in [0, 1]. Returns (None, -1.0) if
        the microphone is not initialized.
        """
        if self.mic is None:
            return None, -1.0

        pcm = self.mic.read(self.frame_length, exception_on_overflow=False)
        samples = np.frombuffer(pcm, dtype=np.int16).astype(np.float32) / 32768.0

        # Shift the rolling buffer and append the new samples.
        self.buffer = np.roll(self.buffer, -len(samples))
        self.buffer[-len(samples):] = samples

        best_name = None
        best_similarity = 0.0
        for ref in self.references:
            window = self.buffer[-ref['length']:]

            # Ignore near-silence to avoid matching background noise.
            energy = float(np.sqrt(np.mean(window ** 2)))
            if energy < 0.1 * ref['energy']:
                continue

            fingerprint = self._fingerprint(window)
            similarity = float(np.dot(fingerprint, ref['fingerprint']))
            if similarity > best_similarity:
                best_similarity = similarity
                best_name = ref['name']

        return best_name, best_similarity

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
