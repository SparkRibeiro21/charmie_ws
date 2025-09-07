#!/usr/bin/env python3
import time
import gc
import threading
import json
import numpy as np
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from charmie_interfaces.srv import GetSoundClassification, GetSoundClassificationContinuous

from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import audio as mp_audio
from mediapipe.tasks.python.components import containers
from mediapipe.tasks.python.audio.core import audio_record

from pathlib import Path

# nao usar ros2 params para configs, receber pelo serviço

class SoundClassificationNode(Node):
    def __init__(self):
        super().__init__('sound_classification')

        # Params
        self.declare_parameter('max_results', 5)
        self.declare_parameter('score_threshold', 0.1)
        self.declare_parameter('overlap', 0.5)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('num_channels', 1)

        # Lock so the mic is never held by two calls
        self.mic_lock = threading.Lock()

        # Simple String publisher for live window results (JSON)
        self.current_pub = self.create_publisher(String, 'sound_current', 10)

        # Services
        self.srv_once = self.create_service(GetSoundClassification, 'get_sound', self.handle_get_sound)
        self.srv_cont = self.create_service(GetSoundClassificationContinuous, 'get_sound_continuous', self.handle_get_sound_continuous)

        self.get_logger().info('Services ready: /get_sound, /get_sound_continuous')

    # --- helpers ---
    def _build_paths_and_params(self):
        home = str(Path.home())
        model_dir = f"{home}/charmie_ws/src/charmie_audio/charmie_audio/sound_classification_models"
        model_path = f"{model_dir}/classifier.tflite"
        return dict(
            model_path=model_path,
            max_results=int(self.get_parameter('max_results').value),
            score_threshold=float(self.get_parameter('score_threshold').value),
            overlap=float(self.get_parameter('overlap').value),
            sample_rate=int(self.get_parameter('sample_rate').value),
            num_channels=int(self.get_parameter('num_channels').value),
        )

    def _open_pipeline(self, model_path, max_results, score_threshold, sample_rate, num_channels):
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

        base_options = mp_python.BaseOptions(model_asset_path=model_path)
        options = mp_audio.AudioClassifierOptions(
            base_options=base_options,
            running_mode=mp_audio.RunningMode.AUDIO_STREAM,
            max_results=max_results,
            score_threshold=score_threshold,
            result_callback=save_result
        )
        classifier = mp_audio.AudioClassifier.create_from_options(options)

        buffer_size = int(0.975 * sample_rate)
        audio_format = containers.AudioDataFormat(num_channels, sample_rate)
        record = audio_record.AudioRecord(num_channels, sample_rate, buffer_size)
        audio_data = containers.AudioData(buffer_size, audio_format)

        record.start_recording()
        return results_q, classifier, record, audio_data, buffer_size

    def _close_pipeline(self, results_q, classifier, record, audio_data):
        try:
            record.stop_recording()
        except Exception:
            pass
        try:
            record.stop()  # harmless if not implemented
        except Exception:
            pass
        try:
            classifier.close()
        except Exception:
            pass
        # clear refs & GC to free mic immediately
        try:
            while True:
                results_q.get_nowait()
        except Empty:
            pass
        results_q = None
        classifier = None
        record = None
        audio_data = None
        gc.collect()

    # --- single-shot (like your mediapipe script) ---
    def classify_for(self, duration_s: float):
        cfg = self._build_paths_and_params()
        if not (0.0 <= cfg['overlap'] < 1.0):
            raise ValueError('overlap must be in [0.0, 1.0).')
        if not (0.0 <= cfg['score_threshold'] <= 1.0):
            raise ValueError('score_threshold must be in [0.0, 1.0].')

        results_q, classifier, record, audio_data, buffer_size = self._open_pipeline(
            cfg['model_path'], cfg['max_results'], cfg['score_threshold'],
            cfg['sample_rate'], cfg['num_channels']
        )

        input_len_sec = buffer_size / float(cfg['sample_rate'])
        interval = input_len_sec * (1.0 - cfg['overlap'])
        last_infer = time.time()

        max_scores = {}
        try:
            start_time = time.time()
            while time.time() - start_time < duration_s:
                now = time.time()
                remaining = interval - (now - last_infer)
                if remaining > 0:
                    time.sleep(remaining)
                last_infer = time.time()

                data = record.read(buffer_size)
                if data.dtype != np.float32:
                    data = data.astype(np.float32, copy=False)

                audio_data.load_from_array(data)
                classifier.classify_async(audio_data, time.time_ns() // 1_000_000)

                newest = None
                while True:
                    try:
                        newest = results_q.get_nowait()
                    except Empty:
                        break

                if newest and newest.classifications:
                    cats = newest.classifications[0].categories
                    detected = {c.category_name: round(float(c.score), 2) for c in cats}
                    detected = dict(sorted(detected.items(), key=lambda kv: kv[1], reverse=True))
                    print(detected)

                    for c in cats:
                        lbl = c.category_name
                        sc = float(c.score)
                        if sc >= cfg['score_threshold']:
                            if lbl not in max_scores or sc > max_scores[lbl]:
                                max_scores[lbl] = sc
        finally:
            self._close_pipeline(results_q, classifier, record, audio_data)

        if not max_scores:
            return [], []
        items = sorted(max_scores.items(), key=lambda kv: kv[1], reverse=True)
        labels = [k for k, _ in items]
        scores = [float(v) for _, v in items]
        return labels, scores

    # --- continuous (publish current window; stop on break or timeout) ---
    def classify_until(self, break_sounds, timeout_s: float):
        cfg = self._build_paths_and_params()
        if not (0.0 <= cfg['overlap'] < 1.0):
            raise ValueError('overlap must be in [0.0, 1.0).')
        if not (0.0 <= cfg['score_threshold'] <= 1.0):
            raise ValueError('score_threshold must be in [0.0, 1.0].')

        results_q, classifier, record, audio_data, buffer_size = self._open_pipeline(
            cfg['model_path'], cfg['max_results'], cfg['score_threshold'],
            cfg['sample_rate'], cfg['num_channels']
        )

        input_len_sec = buffer_size / float(cfg['sample_rate'])
        interval = input_len_sec * (1.0 - cfg['overlap'])
        last_infer = time.time()

        break_set = set(break_sounds or [])
        start_time = time.time()
        hit = False
        label = ""
        score = 0.0

        try:
            while True:
                if timeout_s > 0 and (time.time() - start_time) >= timeout_s:
                    break

                now = time.time()
                remaining = interval - (now - last_infer)
                if remaining > 0:
                    time.sleep(remaining)
                last_infer = time.time()

                data = record.read(buffer_size)
                if data.dtype != np.float32:
                    data = data.astype(np.float32, copy=False)

                audio_data.load_from_array(data)
                classifier.classify_async(audio_data, time.time_ns() // 1_000_000)

                newest = None
                while True:
                    try:
                        newest = results_q.get_nowait()
                    except Empty:
                        break

                if newest and newest.classifications:
                    cats = newest.classifications[0].categories

                    window = {c.category_name: round(float(c.score), 2) for c in cats}
                    window = dict(sorted(window.items(), key=lambda kv: kv[1], reverse=True))
                    print(window)

                    msg = String()
                    msg.data = json.dumps(window, ensure_ascii=False)
                    self.current_pub.publish(msg)

                    if break_set:
                        for c in cats:
                            lbl = c.category_name
                            sc = float(c.score)
                            if lbl in break_set and sc >= cfg['score_threshold']:
                                hit = True
                                label = lbl
                                score = sc
                                break
                        if hit:
                            break
        finally:
            self._close_pipeline(results_q, classifier, record, audio_data)

        return hit, label, float(score)

    # --- service handlers ---
    def handle_get_sound(self, request: GetSoundClassification.Request, response: GetSoundClassification.Response):
        duration = float(request.duration) if request.duration > 0.0 else 2.0
        try:
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

    def handle_get_sound_continuous(self, request: GetSoundClassificationContinuous.Request,
                                    response: GetSoundClassificationContinuous.Response):
        break_sounds = list(request.break_sounds) if request.break_sounds else []
        timeout = float(request.timeout)
        try:
            with self.mic_lock:
                hit, lbl, sc = self.classify_until(break_sounds, timeout)
            response.success = bool(hit)
            response.detected_label = lbl if hit else ""
            response.detected_score = float(sc if hit else 0.0)
            response.message = "break detected" if hit else ("timeout" if timeout > 0 else "stopped")
        except Exception as e:
            response.success = False
            response.detected_label = ""
            response.detected_score = 0.0
            response.message = f"error: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SoundClassificationNode()
    rclpy.spin(node)
    rclpy.shutdown()
