# ARCHITECTURE.md

## 🧠 System Architecture Overview

This document describes how the components of Kilo Bot interact.

------------------------------------------------------------------------

## 🏗️ Architecture Diagram (Text-Based)

     ┌────────────┐
     │  Microphone │
     └──────┬─────┘
            │ audio stream
            ▼
     ┌────────────┐
     │   Vosk STT │
     │  (speech → text)
     └──────┬─────┘
            │ text
            ▼
     ┌────────────┐
     │   ROS2 Node │
     │ (OllamaChat)│
     └──────┬─────┘
            │ prompt
            ▼
     ┌────────────┐
     │   Ollama LLM│
     │ (local model)
     └──────┬─────┘
            │ streamed text chunks
            ▼
     ┌────────────┐
     │ Piper TTS  │
     │ (text → audio)
     └──────┬─────┘
            │ audio
            ▼
     ┌────────────┐
     │  Speaker   │
     └────────────┘

------------------------------------------------------------------------

## ⚙️ Key Components

### **1. Vosk STT**

-   Converts live microphone audio into text\
-   Streams audio chunks\
-   Runs fully offline

### **2. Ollama**

-   Local LLM engine (default: Llama3.2)\
-   Streams tokens for low-latency response

### **3. Piper TTS**

-   Converts text chunks into speech\
-   Generates WAV files\
-   Produces natural, clear audio

### **4. Voice Activity Detector**

-   Monitors mic audio\
-   If user starts speaking → immediately interrupts TTS

### **5. ROS2 Node**

-   Coordinates all components\
-   Handles conversation loop\
-   Sends LLM prompts\
-   Plays streamed audio in real-time

------------------------------------------------------------------------

## 🧪 Conversation Flow

1.  User speaks\
2.  Vosk detects speech → produces text\
3.  ROS2 sends text to Ollama\
4.  Ollama streams a response\
5.  Piper streams spoken audio\
6.  If user speaks again → cutoff

------------------------------------------------------------------------

## ✅ The system is fully local, private, and real-time.
