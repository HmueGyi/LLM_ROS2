# INSTALL.md

## ✅ Installation Instructions

This document explains how to install all dependencies for the Kilo Bot
voice assistant system.

------------------------------------------------------------------------

## 1. Install ROS2 (Humble recommended)

Follow the official instructions:\
https://docs.ros.org/en/humble/Installation.html

After installation:

``` bash
source /opt/ros/humble/setup.bash
```

------------------------------------------------------------------------

## 2. Install Ollama

``` bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.2
```

Verify server:

    http://localhost:11434

------------------------------------------------------------------------

## 3. Install Vosk Speech-to-Text

Download model:

``` bash
wget https://alphacephei.com/vosk/models/vosk-model-en-us-0.22.zip
unzip vosk-model-en-us-0.22.zip
```

------------------------------------------------------------------------

## 4. Install Piper Text-to-Speech

``` bash
sudo apt install piper
```

Download voice model:

``` bash
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/low/en_US-lessac-low.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/low/en_US-lessac-low.onnx.json
```

------------------------------------------------------------------------

## 5. Python Dependencies

``` bash
pip install -r requirements.txt
```

------------------------------------------------------------------------

## 6. Build ROS2 Workspace

``` bash
colcon build
source install/setup.bash
```

✅ Installation complete.
