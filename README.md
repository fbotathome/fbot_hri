<div align="center">

<img width="5122" height="959" alt="fbot_hri_boris" src="https://github.com/user-attachments/assets/3377371a-b0f6-4d79-9824-9d6a6bf69bbe" />

![UBUNTU](https://img.shields.io/badge/UBUNTU-22.04-orange?style=for-the-badsge&logo=ubuntu)
![python](https://img.shields.io/badge/python-3.10-blue?style=for-the-badsge&logo=python)
![ROS2](https://img.shields.io/badge/ROS2-Humble-blue?style=for-the-badsge&logo=ros)
[![Last Commit](https://img.shields.io/github/last-commit/fbotathome/fbot_hri.svg?style=for-the-badsge)](https://github.com/fbotathome/fbot_hri/commits/main)
[![GitHub issues](https://img.shields.io/github/issues/fbotathome/fbot_hri)](https://github.com/fbotathome/fbot_hri/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/fbotathome/fbot_hri)](https://github.com/fbotathome/fbot_hri/pulls)
[![Contributors](https://img.shields.io/github/contributors/fbotathome/fbot_hri.svg)](https://github.com/fbotathome/fbot_hri/graphs/contributors)

**A comprehensive ROS 2 package suite for human-robot interaction featuring speech synthesis, speech recognition, head movement control with facial emotions, and display management for robot operators.**

[Overview](#overview) • [Architecture](#architecture) • [Installation](#installation) • [Usage](#usage) • [fbot_hri message and services](#fbot_hri-message-and-services) • [Commit History](#commit-history) • [available emotions](#available-emotions) • [Contributing](#contributing)

</div>

## Overview

`fbot_hri` is a ROS 2 package suite designed for comprehensive human-robot interaction applications. It provides essential capabilities for robots to communicate naturally with humans through speech, express emotions through facial expressions and head movements, and display information to operators through integrated screen management. 
---

## Architecture

The system consists of four main packages:

```
fbot_hri/
├── 📁 fbot_speech/               # Speech processing and audio interaction
│   ├── 📁 fbot_speech/             # Core speech nodes (TTS, STT, audio player)
│   ├── 📁 scripts/                 # Utility scripts for audio processing
│   └── 📁 audios/               # Audio resources (beep, talk sounds)
├── 📁 fbot_head/                 # Head movement and facial emotion control
│   ├── 📁 emotions_bridge/       # Emotion-to-hardware bridge
│   ├── 📁 neck_controller/       # Neck movement control
│   └── 📁 head_firmware/         # Hardware communication layer
├── 📁 fbot_screen/               # Display and UI management for operators
│   ├── 📁 display_node
└── 📁 fbot_speech_msgs/          # Custom ROS message definitions for speech
└── 📁 fbot_hri_bringup/          # Launch configurations and parameter files
```

---

## Installation

### Prerequisites

- ROS2 Humble
- Python 3.10+
- Ubuntu 22.04
- Dependencies listed in `package.xml` and `requirements.txt`

### Setup

1. **Clone the repository into your ROS workspace:**
   ```bash
   cd ~/fbot_ws/src
   git clone https://github.com/fbotathome/fbot_hri.git
   ```

2. **Install dependencies:**
   ```bash
   cd ~/fbot_ws
   sudo rosdep init  # Skip if already initialized
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   pip install -r src/fbot_hri/fbot_speech/requirements.txt
   ```

3. **Build the workspace:**
   ```bash
   cd ~/fbot_ws
   colcon build --packages-select fbot_speech fbot_head fbot_screen fbot_speech_msgs fbot_hri_bringup
   source install/setup.bash
   ```

---

## Usage

### Speech Synthesis (Text-to-Speech)
```bash
# Launch speech synthesizer with audio player
ros2 launch fbot_hri_bringup synthesizer_speech.launch.py

# Synthesize speech via service
ros2 service call /fbot_speech/ss/say_something \
    fbot_speech_msgs/srv/SynthesizeSpeech "{text: 'Hello, I am Boris!', lang: 'en', force_stream_mode: false}"

# Synthesize speech via topic
ros2 topic pub /fbot_speech/ss/say_something \
    fbot_speech_msgs/msg/SynthesizeSpeechMessage "{text: 'Hello World', lang: 'en', force_stream_mode: false}"
```

### Speech Recognition (Speech-to-Text)
```bash
# Launch speech recognizer
ros2 launch fbot_hri_bringup speech_to_text.launch.py stt_config_file:=fbot_stt_recepcionist.yaml

# Start speech recognition
ros2 service call /fbot_speech/sr/speech_recognizer \
    fbot_speech_msgs/srv/SpeechToText "{prompt: 'Say something', lang: 'en'}"
```

### RIVA Speech Recognition (Speech-to-Text)
```bash
# Launch speech recognizer
ros2 launch fbot_hri_bringup riva_recognizer.launch.py

# Start speech recognition
ros2 service call /fbot_speech/sr/asr_recognizer \
    fbot_speech_msgs/srv/SpeechToText "{string: ['boosted', 'lm', 'words'], boost: 20, sentence: 'True'}"
```

### Audio Player
```bash
# Launch audio player
ros2 launch fbot_hri_bringup audio_player.launch.py

# Play audio file
ros2 service call /fbot_speech/ap/audio_player \
    fbot_speech_msgs/srv/AudioPlayer "{path: '/path/to/audio.wav'}"

# Play beep sound
ros2 service call /fbot_speech/ap/audio_beep std_srvs/srv/Empty
```

### Hotword Detection
```bash
# Launch hotword detection
ros2 launch fbot_hri_bringup hotword_detector.launch.py hotword_config_file:=fbot_hotword_restaurant.yaml

# Monitor hotword detection
ros2 topic echo /fbot_speech/bhd/detected
```

### Head Movement and Emotions
```bash
# Launch neck controller
ros2 launch fbot_head neck.launch.py

# Launch head control system
ros2 launch fbot_head emotions.launch.py

# Look at specific 3D point
ros2 service call /lookat_start \
    fbot_vision_msgs/srv/LookAtDescription3D "{description: 'person', position: {x: 1.0, y: 0.0, z: 1.5}}"

# Stop looking
ros2 service call /lookat_stop std_srvs/srv/Empty

# Send emotion command
ros2 topic pub /fbot_face/emotion std_msgs/msg/String "{data: 'happy'}"

# Control neck position
ros2 topic pub /updateNeck std_msgs/msg/Float64MultiArray "{data: [180.0, 160.0]}"
```


### Display Management
```bash
# Launch display system with Foxglove integration
ros2 launch fbot_screen display.launch.py

# Display text message
ros2 topic pub /display_command std_msgs/msg/String "{data: 'sentence:Hello, I am Boris!\nHow can I help you?'}"

# Display image
ros2 topic pub /display_command std_msgs/msg/String "{data: 'image:/path/to/image.jpg'}"

# Display video
ros2 topic pub /display_command std_msgs/msg/String "{data: 'video:/path/to/video.mp4'}"

# Mirror camera topic
ros2 topic pub /display_command std_msgs/msg/String "{data: 'topic:/camera/image_raw'}"
```

---

## fbot_hri Topics and Services

### Speech System Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/fbot_speech/ss/say_something` | [`SynthesizeSpeechMessage`](fbot_speech_msgs/msg/SynthesizeSpeechMessage.msg) | Speech synthesis commands |
| `/fbot_speech/ap/stream_data` | [`AudioData`](audio_common_msgs/msg/AudioData.msg) | Audio stream data |
| `/fbot_speech/bhd/detected` | [`String`](std_msgs/msg/String.msg) | Hotword detection events |
| `/fbot_speech/bhd/hot_word` | [`String`](std_msgs/msg/String.msg) | Hotword configuration |

### Head Control Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/updateNeck` | [`Float64MultiArray`](std_msgs/msg/Float64MultiArray.msg) | Neck position control |
| `/boris_head/joint_states` | [`JointState`](sensor_msgs/msg/JointState.msg) | Head joint states |
| `/fbot_face/emotion` | [`String`](std_msgs/msg/String.msg) | Facial emotion commands |

### Display System Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/display_command` | [`String`](std_msgs/msg/String.msg) | Display control commands |
| `/ui_display` | [`Image`](sensor_msgs/msg/Image.msg) | Display output for visualization |

### Speech System Services

| Service | Type | Description |
|---------|------|-------------|
| `/fbot_speech/ss/say_something` | [`SynthesizeSpeech`](fbot_speech_msgs/srv/SynthesizeSpeech.srv) | Text-to-speech synthesis |
| `/fbot_speech/sr/speech_recognizer` | [`SpeechToText`](fbot_speech_msgs/srv/SpeechToText.srv) | Speech-to-text recognition |
| `/fbot_speech/sr/asr_recognizer` | [`RivaToText`](fbot_speech_msgs/srv/RivaToText.srv) | Speech-to-text recognition with RIVA ASR |
| `/fbot_speech/ap/audio_player` | [`AudioPlayer`](fbot_speech_msgs/srv/AudioPlayer.srv) | Play audio files |
| `/fbot_speech/ap/audio_player_by_data` | [`AudioPlayerByData`](fbot_speech_msgs/srv/AudioPlayerByData.srv) | Play audio from data |
| `/fbot_speech/ap/audio_beep` | [`Empty`](std_srvs/srv/Empty.srv) | Play beep sound |
| `/fbot_speech/ap/stream_start` | [`AudioStreamStart`](fbot_speech_msgs/srv/AudioStreamStart.srv) | Start audio streaming |
| `/fbot_speech/ap/stream_stop` | [`Empty`](std_srvs/srv/Empty.srv) | Stop audio streaming |

### Head Control Services

| Service | Type | Description |
|---------|------|-------------|
| `/lookat_start` | [`LookAtDescription3D`](fbot_vision_msgs/srv/LookAtDescription3D.srv) | Start looking at 3D target |
| `/lookat_stop` | [`Empty`](std_srvs/srv/Empty.srv) | Stop looking behavior |

### Available Emotions

The system supports the following facial emotions:
- `happy` - Happy expression
- `sad` - Sad expression  
- `neutral` - Neutral/default expression
- `surprised` - Surprised expression
- `angry` - Angry expression
- `suspicious` - Suspicious expression
- `sleepy` - Sleepy expression

---

## Commits history

This repository was created from scratch on August 20, 2025, with files migrated from the `fbot_hri_deprecated` branch. The previous repository was a fork of another repository, and we wanted it to be independent.

To view commits history before this date, please access the deprecated repository: [fbot_hri_deprecated](https://github.com/fbotathome/fbot_hri_deprecated)

## Contributing

1. Create a feature branch (`git checkout -b feat/amazing-feature`)
2. Commit your changes (`git commit -m 'Add amazing feature'`)
3. Push to the branch (`git push origin feat/amazing-feature`)
4. Open a Pull Request
