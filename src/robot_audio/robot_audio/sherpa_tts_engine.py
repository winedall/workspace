import os
from pathlib import Path
import subprocess
import tempfile
import threading
import hashlib
import wave
from typing import Any, Optional

import numpy as np


class SherpaOnnxTTSEngine:
    def __init__(
        self,
        model_dir: str,
        num_threads: int = 1,
        provider: str = "cpu",
        logger: Optional[Any] = None,
        cache_enabled: bool = True,
        cache_dir: str = "",
        noise_scale: float = 1.0,
        length_scale: float = 1.0,
        target_peak: int = 20000,
        max_gain: float = 4.0,
    ):
        self.model_dir = Path(model_dir).expanduser().resolve()
        self.num_threads = int(num_threads)
        self.provider = str(provider or "cpu")
        self.logger = logger
        self.cache_enabled = bool(cache_enabled)
        self.cache_dir = Path(cache_dir).expanduser().resolve()
        self.noise_scale = float(noise_scale)
        self.length_scale = float(length_scale)
        self.target_peak = int(target_peak)
        self.max_gain = float(max_gain)
        self._lock = threading.Lock()
        self._current_process = None
        self._stop_flag = False
        self._sherpa = self._import_sherpa()
        self._tts = self._create_tts()
        if self.cache_enabled:
            self.cache_dir.mkdir(parents=True, exist_ok=True)

    @staticmethod
    def _import_sherpa():
        try:
            import sherpa_onnx
        except Exception as exc:
            raise RuntimeError(f"sherpa_onnx未安装或不可用: {exc}") from exc
        return sherpa_onnx

    def _log(self, level: str, msg: str) -> None:
        if self.logger:
            try:
                log_func = getattr(self.logger, level, None)
                if log_func:
                    log_func(msg)
            except Exception:
                pass

    def _required_file(self, name: str) -> str:
        path = self.model_dir / name
        if not path.exists():
            raise FileNotFoundError(f"sherpa-onnx模型文件不存在: {path}")
        return str(path)

    def _existing_rule_fsts(self) -> str:
        return ",".join(
            str(path)
            for path in [
                self.model_dir / "date.fst",
                self.model_dir / "number.fst",
                self.model_dir / "phone.fst",
            ]
            if path.exists()
        )

    def _create_tts(self):
        sherpa = self._sherpa
        config = sherpa.OfflineTtsConfig(
            model=sherpa.OfflineTtsModelConfig(
                matcha=sherpa.OfflineTtsMatchaModelConfig(
                    acoustic_model=self._required_file("model-steps-3.onnx"),
                    vocoder=self._required_file("vocos-22khz-univ.onnx"),
                    lexicon=self._required_file("lexicon.txt"),
                    tokens=self._required_file("tokens.txt"),
                    noise_scale=self.noise_scale,
                    length_scale=self.length_scale,
                ),
                num_threads=self.num_threads,
                provider=self.provider,
            ),
            rule_fsts=self._existing_rule_fsts(),
        )
        self._log(
            "info",
            f"sherpa-onnx TTS初始化: model_dir={self.model_dir}, "
            f"num_threads={self.num_threads}, provider={self.provider}, "
            f"model=matcha, noise_scale={self.noise_scale}, "
            f"length_scale={self.length_scale}, target_peak={self.target_peak}, "
            f"max_gain={self.max_gain}",
        )
        return sherpa.OfflineTts(config)

    def _write_audio_file(self, audio, output_file: str) -> None:
        if hasattr(audio, "save"):
            audio.save(output_file)
        else:
            self._sherpa.write_wave(output_file, audio.samples, audio.sample_rate)
        self._postprocess_wav(
            output_file,
            target_peak=self.target_peak,
            max_gain=self.max_gain,
        )

    @staticmethod
    def _postprocess_wav(
        wav_path: str,
        lead_silence_sec: float = 0.20,
        tail_silence_sec: float = 0.25,
        target_peak: int = 20000,
        max_gain: float = 4.0,
    ) -> None:
        with wave.open(wav_path, "rb") as wav_file:
            channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
            sample_rate = wav_file.getframerate()
            frame_count = wav_file.getnframes()
            frames = wav_file.readframes(frame_count)

        if channels != 1 or sample_width != 2 or not frames:
            return

        audio = np.frombuffer(frames, dtype=np.int16).astype(np.float32)
        peak = float(np.max(np.abs(audio))) if audio.size else 0.0
        if peak > 0:
            scale = min(float(target_peak) / peak, float(max_gain))
            audio = np.clip(audio * scale, -32768, 32767)

        lead = np.zeros(int(sample_rate * lead_silence_sec), dtype=np.float32)
        tail = np.zeros(int(sample_rate * tail_silence_sec), dtype=np.float32)
        padded = np.concatenate([lead, audio, tail]).astype(np.int16)

        with wave.open(wav_path, "wb") as wav_file:
            wav_file.setnchannels(1)
            wav_file.setsampwidth(2)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(padded.tobytes())

    def _cache_path(self, text: str) -> Path:
        acoustic_model = self.model_dir / "model-steps-3.onnx"
        vocoder = self.model_dir / "vocos-22khz-univ.onnx"
        acoustic_mtime = str(acoustic_model.stat().st_mtime_ns) if acoustic_model.exists() else "missing"
        vocoder_mtime = str(vocoder.stat().st_mtime_ns) if vocoder.exists() else "missing"
        key = "\n".join(
            [
                "matcha",
                text,
                str(self.model_dir),
                acoustic_mtime,
                vocoder_mtime,
                str(self.num_threads),
                self.provider,
                str(self.noise_scale),
                str(self.length_scale),
                str(self.target_peak),
                str(self.max_gain),
            ]
        )
        return self.cache_dir / f"{hashlib.sha256(key.encode('utf-8')).hexdigest()}.wav"

    def _synthesize_to_file(self, text: str) -> tuple[Optional[str], bool]:
        if self.cache_enabled:
            cache_path = self._cache_path(text)
            if cache_path.exists() and cache_path.stat().st_size > 0:
                self._log("debug", f"sherpa-onnx TTS缓存命中: {cache_path.name}")
                return str(cache_path), True

        try:
            audio = self._tts.generate(text)
            suffix = ".tmp.wav" if self.cache_enabled else ".wav"
            fd = tempfile.NamedTemporaryFile(suffix=suffix, delete=False)
            temp_file = fd.name
            fd.close()
            self._write_audio_file(audio, temp_file)
            if self.cache_enabled:
                cache_path = self._cache_path(text)
                os.replace(temp_file, cache_path)
                return str(cache_path), True
            return temp_file, False
        except Exception as exc:
            self._log("error", f"sherpa-onnx语音合成失败: {exc}")
            return None, False

    def _play_audio(self, audio_file: str) -> bool:
        try:
            self._current_process = subprocess.Popen(
                # ["aplay", "-q", audio_file],
                ["aplay", "-D", "pulse", "-q", audio_file],
                # ["aplay", "-D", "plughw:1,0", "-q", audio_file],
                # ["aplay", "-D", "plughw:CARD=Device,DEV=0", "-q", audio_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self._current_process.wait()
            self._current_process = None
            return True
        except Exception as exc:
            self._log("error", f"音频播放失败: {exc}")
            return False

    def play_file(self, audio_file: str) -> bool:
        with self._lock:
            if self._stop_flag:
                return False
            return self._play_audio(audio_file)

    def speak(self, text: str, callback: Optional[callable] = None) -> bool:
        with self._lock:
            if self._stop_flag:
                return False
            self._log("info", f"[sherpa-onnx TTS] 开始合成: {text[:30]}...")
            audio_file, cached = self._synthesize_to_file(text)
            if not audio_file:
                return False
            try:
                if not self._stop_flag:
                    played = self._play_audio(audio_file)
                else:
                    played = False
            finally:
                if not cached:
                    try:
                        os.remove(audio_file)
                    except OSError:
                        pass
            if callback:
                callback()
            return played

    def stop(self) -> None:
        self._stop_flag = True
        if self._current_process:
            try:
                self._current_process.terminate()
                self._current_process.wait(timeout=1)
            except Exception:
                pass
            self._current_process = None
        self._stop_flag = False

    def shutdown(self) -> None:
        self.stop()
        self._tts = None
