#!/usr/bin/env python3
import time
import numpy as np
from queue import Queue, Empty
import threading
import gc

import rclpy
from rclpy.node import Node

from charmie_interfaces.srv import GetSoundClassification

from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import audio as mp_audio
from mediapipe.tasks.python.components import containers
from mediapipe.tasks.python.audio.core import audio_record

from pathlib import Path

class SoundClassificationNode(Node):
    def __init__(self):
        super().__init__('sound_classification')

        # Parameters (kept simple; same defaults as your script)
        # self.declare_parameter('model_path', 'classifier.tflite')
        self.declare_parameter('max_results', 5)
        self.declare_parameter('score_threshold', 0.1)
        self.declare_parameter('overlap', 0.5)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('num_channels', 1)

        # Ensure exclusive mic usage (e.g., when Whisper runs after this)
        self.mic_lock = threading.Lock()

        # Service
        self.srv = self.create_service(GetSoundClassification, 'get_sound', self.handle_get_sound)
        self.get_logger().info('Sound classification service "/get_sound" ready.')

    # ---- core classification (mirrors your Mediapipe loop) ----
    def classify_for(self, duration_s: float):

        # by using self.home it automatically adjusts to all computers home file, which may differ since it depends on the username on the PC
        home = str(Path.home())
        sound_classification_models = "charmie_ws/src/charmie_audio/charmie_audio/sound_classification_models"
        model_path = home+'/'+sound_classification_models+'/'+'classifier.tflite'

        # model_path      = self.get_parameter('model_path').get_parameter_value().string_value
        max_results     = int(self.get_parameter('max_results').value)
        score_threshold = float(self.get_parameter('score_threshold').value)
        overlap         = float(self.get_parameter('overlap').value)
        sample_rate     = int(self.get_parameter('sample_rate').value)
        num_channels    = int(self.get_parameter('num_channels').value)

        # Validations (same as your script)
        if not (0.0 <= overlap < 1.0):
            raise ValueError('overlap must be in [0.0, 1.0).')
        if not (0.0 <= score_threshold <= 1.0):
            raise ValueError('score_threshold must be in [0.0, 1.0].')

        # Thread-safe results buffer
        results_q: "Queue[mp_audio.AudioClassifierResult]" = Queue(maxsize=16)

        def save_result(result: mp_audio.AudioClassifierResult, timestamp_ms: int):
            result.timestamp_ms = timestamp_ms
            try:
                results_q.put_nowait(result)
            except Exception:
                try:
                    _ = results_q.get_nowait()
                except Empty:
                    pass
                results_q.put_nowait(result)

        # Build classifier
        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_audio.AudioClassifierOptions(
            base_options=base_options,
            running_mode=mp_audio.RunningMode.AUDIO_STREAM,
            max_results=max_results,
            score_threshold=score_threshold,
            result_callback=save_result
        )
        classifier = mp_audio.AudioClassifier.create_from_options(options)

        # Audio I/O
        buffer_size = int(0.975 * sample_rate)  # ~0.975 s @ 16kHz -> 15600
        audio_format = containers.AudioDataFormat(num_channels, sample_rate)
        record = audio_record.AudioRecord(num_channels, sample_rate, buffer_size)
        audio_data = containers.AudioData(buffer_size, audio_format)

        # Inference cadence
        input_len_sec = buffer_size / float(sample_rate)
        interval = input_len_sec * (1.0 - overlap)
        last_infer = time.time()

        # Start
        record.start_recording()

        # Collect max score per label across the whole duration
        max_scores = {}  # label -> max score seen

        try:
            start_time = time.time()
            while time.time() - start_time < duration_s:
                now = time.time()
                remaining = interval - (now - last_infer)
                if remaining > 0:
                    time.sleep(remaining)
                last_infer = time.time()

                # Read mic buffer -> float32
                data = record.read(buffer_size)
                if data.dtype != np.float32:
                    # If backend gives int16, you can scale to [-1,1] instead:
                    # if data.dtype == np.int16:
                    #     data = data.astype(np.float32) / 32768.0
                    data = data.astype(np.float32, copy=False)

                audio_data.load_from_array(data)
                classifier.classify_async(audio_data, time.time_ns() // 1_000_000)

                # Drain queue; process newest result only (like your print loop)
                newest = None
                while True:
                    try:
                        newest = results_q.get_nowait()
                    except Empty:
                        break

                if newest and newest.classifications:
                    cats = newest.classifications[0].categories

                    detected_sounds = {c.category_name: round(float(c.score), 2) for c in cats}
                    detected_sounds = dict(sorted(detected_sounds.items(),
                                                  key=lambda kv: kv[1],
                                                  reverse=True))
                    print(detected_sounds)

                    for c in cats:
                        lbl = c.category_name
                        sc = float(c.score)
                        if sc >= score_threshold:
                            if lbl not in max_scores or sc > max_scores[lbl]:
                                max_scores[lbl] = sc

        finally:
            # hard-stop the stream and release native handles
            try:
                record.stop_recording()
            except Exception:
                pass
            try:
                # some backends expose an extra stop(); if absent this is harmless
                record.stop()
            except Exception:
                pass
            try:
                classifier.close()
            except Exception:
                pass
            # drain and drop references so GC can free PortAudio/OS handles
            try:
                while True:
                    results_q.get_nowait()
            except Empty:
                pass
            record = None
            audio_data = None
            audio_format = None
            classifier = None
            gc.collect()

        # Prepare sorted arrays
        if not max_scores:
            return [], []

        sorted_items = sorted(max_scores.items(), key=lambda kv: kv[1], reverse=True)
        labels = [k for k, _ in sorted_items]
        scores = [float(v) for _, v in sorted_items]
        return labels, scores

    # ---- service handler ----
    def handle_get_sound(self, request: GetSoundClassification.Request, response: GetSoundClassification.Response):
        duration = float(request.duration) if request.duration > 0.0 else 2.0
        try:
            # ensure only one request owns the mic at a time
            with self.mic_lock:
                labels, scores = self.classify_for(duration)
            response.labels = labels
            response.scores = [float(s) for s in scores]
            response.success = True
            response.message = "ok" if labels else "no sounds above threshold"
        except Exception as e:
            response.labels = []
            response.scores = []
            response.success = False
            response.message = f"error: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SoundClassificationNode()
    rclpy.spin(node)
    rclpy.shutdown()
