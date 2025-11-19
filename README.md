# 🎙️ ROS2 Real-Time Voice Chat with Ollama, Vosk STT, Piper TTS, and Interrupt Detection

This project creates a fully offline, real-time voice assistant named
**Kilo Bot**, powered by:

-   **ROS2 (rclpy)**
-   **Ollama** for local LLM inference
-   **Vosk** for speech-to-text
-   **Piper** for natural text-to-speech\
-   **Voice Activity Detection (VAD)** for interrupting TTS when the
    user starts talking\
-   **Real-time streaming LLM responses**

Kilo Bot listens to your voice, sends the text to Ollama, and speaks
responses with Piper --- with live interruption for natural
conversation.

## ✅ Features

-   🎤 Continuous listening (no hotword)
-   🧠 Local LLM (Llama3.2)
-   🗣️ Piper TTS
-   🔊 Interruptible playback
-   ⚡ Streaming paragraph-level TTS
-   🔧 Configurable
-   🐍 ROS2 Python node

## 📁 Project Structure

    llm_ros2/
     ├── llm_chat_node.py
     ├── tts/
     │    ├── en_US-lessac-low.onnx
     │    └── en_US-lessac-low.onnx.json
     └── models/
          └── vosk-model-en-us-0.22/

## 📦 Requirements

### ROS2, Ollama, Vosk, Piper, Python dependencies

## ▶️ Running

``` bash
colcon build
source install/setup.bash
ros2 run llm_ros2 ollama_chat_node
```

## 🚀 Usage

Just speak normally---Kilo Bot listens, responds, and interrupts when you
talk.
