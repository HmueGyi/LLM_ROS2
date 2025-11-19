# USAGE.md

## ✅ Using Kilo Bot Voice Assistant

Kilo Bot is a hands-free conversational agent that listens, thinks, and
speaks locally.

------------------------------------------------------------------------

## 🚀 Running the Node

From your ROS2 workspace:

``` bash
colcon build
source install/setup.bash
ros2 run llm_ros2 ollama_chat_node
```

------------------------------------------------------------------------

## 🎤 How It Works

1.  System listens continuously\
2.  Vosk converts your speech → text\
3.  Ollama generates a response\
4.  Piper speaks the response\
5.  If you start talking → playback stops instantly

------------------------------------------------------------------------

## ✅ Example Conversation

**You:**\
\> hey Kilo Bot how are you

**Kilo Bot:**\
\> I'm doing well! Thanks for asking...

Start speaking anytime → Kilo Bot stops immediately.

------------------------------------------------------------------------

## ⚠️ Notes

-   Works offline\
-   Requires a microphone and speaker\
-   Background noise affects voice detection
