#!/usr/bin/env python3
import json
import requests
import rclpy
from rclpy.node import Node
import tempfile
import os
import subprocess
import queue
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import threading
import numpy as np
import time

# ----------------------------
# Ollama Configuration
# ----------------------------
OLLAMA_URL = "http://localhost:11434/api/generate"
MODEL = "llama3.2:1b"

# ----------------------------
# Piper Configuration
# ----------------------------
PIPER_EXEC = "piper"  # Make sure Piper is installed and in PATH
PIPER_MODEL = "/home/mr_robot/Desktop/dev_ws/src/llm_ros2/tts/en_US-lessac-low.onnx"
PIPER_CONFIG = PIPER_MODEL.replace(".onnx", ".onnx.json")

# ----------------------------
# Vosk Configuration
# ----------------------------
VOSK_MODEL_PATH = "/home/mr_robot/Desktop/dev_ws/src/llm_ros2/models/vosk-model-en-us-0.22"
vosk_model = Model(VOSK_MODEL_PATH)
audio_queue = queue.Queue()

# ----------------------------
# Global interrupt control
# ----------------------------
interrupted = threading.Event()

# ----------------------------
# Voice Activity Detection
# ----------------------------
def voice_activity_detector(threshold=500, duration=0.2):
    """Monitor mic input. If voice detected, set interrupt flag."""
    global interrupted
    samplerate = 16000
    blocksize = int(samplerate * duration)

    with sd.InputStream(samplerate=samplerate, channels=1, dtype='int16') as stream:
        while not interrupted.is_set():
            try:
                audio = stream.read(blocksize)[0]

                # Skip if no data or malformed
                if audio is None or len(audio) == 0:
                    continue

                # Convert to float and clean invalid values
                audio = np.asarray(audio, dtype=np.float32)
                if not np.any(np.isfinite(audio)):
                    continue

                audio = np.nan_to_num(audio, nan=0.0, posinf=0.0, neginf=0.0)

                # Compute RMS safely
                mean_square = np.mean(np.square(audio))
                if np.isnan(mean_square) or mean_square <= 0:
                    continue

                rms = float(np.sqrt(mean_square))

                if rms > threshold:
                    print("\n⚠️  Interrupt detected — user started speaking.")
                    interrupted.set()
                    break

            except Exception as e:
                # Handle sporadic audio glitches gracefully
                # Comment out the next line if you don't want console output
                # print(f"(VAD warning: {e})")
                continue

            time.sleep(0.05)

# ----------------------------
# Audio Functions
# ----------------------------
def play_audio_interruptible(file_path):
    """Play audio file on Linux using aplay, stop if interrupted."""
    global interrupted
    interrupted.clear()

    vad_thread = threading.Thread(target=voice_activity_detector, daemon=True)
    vad_thread.start()

    process = subprocess.Popen(
        ["aplay", file_path],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    # Monitor both audio playback and user interrupt
    while process.poll() is None:
        if interrupted.is_set():
            process.terminate()
            break
        sd.sleep(100)

    vad_thread.join()


def speak_with_piper(text):
    """Generate TTS with Piper and play immediately with interruption support"""
    if not text.strip():
        return
    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav") as tmpfile:
        wav_path = tmpfile.name

    subprocess.run(
        [PIPER_EXEC, "-m", PIPER_MODEL, "-c", PIPER_CONFIG, "-i", "/dev/stdin", "-f", wav_path],
        input=text.encode("utf-8"),
        check=True
    )

    play_audio_interruptible(wav_path)
    os.remove(wav_path)

# ----------------------------
# Vosk Speech-to-Text
# ----------------------------
def callback(indata, frames, time, status):
    """Put microphone data into queue"""
    if status:
        print(status, flush=True)
    audio_queue.put(bytes(indata))


def recognize_speech(timeout=10):
    """Capture speech from microphone and return recognized text"""
    global interrupted
    interrupted.clear()

    rec = KaldiRecognizer(vosk_model, 16000)
    print("🎤 Listening... Speak now.")

    with sd.RawInputStream(samplerate=16000, blocksize=1600, dtype='int16', channels=1, callback=callback):
        text = ""
        try:
            start_time = time.time()
            while True:
                data = audio_queue.get()
                if rec.AcceptWaveform(data):
                    result = json.loads(rec.Result())
                    text = result.get("text", "")
                    break
                if time.time() - start_time > timeout:
                    break
        except KeyboardInterrupt:
            pass
        return text.strip()

# ----------------------------
# ROS2 Node for Chat
# ----------------------------
class OllamaChat(Node):
    def __init__(self):
        super().__init__('ollama_chat_node')
        self.get_logger().info("Chatting with Ollama (real-time streaming TTS + interrupt detection)")

    def stream_ollama(self, prompt: str):
        """Stream response from Ollama and speak in real-time as text is generated"""
        global interrupted
        try:
            with requests.post(
                OLLAMA_URL,
                json={
                    "model": MODEL,
                    "prompt": prompt,
                    "stream": True,
                    "temperature": 0.8,
                },
                stream=True,
                timeout=120,
            ) as response:
                response.raise_for_status()
                print("Kilo Bot: ", end="", flush=True)

                buffer = ""

                for chunk in response.iter_lines():
                    if interrupted.is_set():
                        print("\n⏹️  Stopped Kilo Bot (user interrupted).")
                        break
                    if not chunk:
                        continue
                    try:
                        data = json.loads(chunk.decode())
                        text = data.get("response", "")
                        if text:
                            buffer += text
                            while "\n\n" in buffer and not interrupted.is_set():  # split by paragraphs
                                paragraphs = buffer.split("\n\n")
                                for p in paragraphs[:-1]:
                                    print(p + "\n")
                                    speak_with_piper(p)
                                buffer = paragraphs[-1]  # keep incomplete paragraph

                    except Exception:
                        continue

                # Speak any remaining text
                if not interrupted.is_set() and buffer.strip():
                    print(buffer, end="", flush=True)
                    speak_with_piper(buffer)

                print()  # newline

        except Exception as e:
            self.get_logger().error(f"Ollama API error: {e}")
            print("\n❌ Connection to Ollama failed.\n")

    def chat(self, user_text):
        """Send user message to Kilo Bot."""
        chat_prompt = (
            "You are a friendly AI named Kilo Bot created by rom dynamics. "
            "Respond conversationally and naturally. "
            "Do not include any JSON or code. "
            "Keep your reply between 3 and 5 sentences.\n\n"
            f"User: {user_text}\nKilo Bot:"
        )
        self.stream_ollama(chat_prompt)

# ----------------------------
# Main Function
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = OllamaChat()
    try:
        while rclpy.ok():
            user_text = recognize_speech()
            if not user_text:
                continue
            print(f"You: {user_text}")
            node.chat(user_text)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
