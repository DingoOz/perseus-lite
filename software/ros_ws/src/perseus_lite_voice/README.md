# perseus_lite_voice

ROS 2 voice assistant for Perseus Lite. Ports the
[`ollama_voice`](https://github.com/DingoOz/ollama_voice) `chat.py` REPL into a
node: Ollama LLM chat with sentence-streamed Piper TTS, optional Whisper STT,
optional `openwakeword` wake-word activation, optional YOLOv8 vision over
`/image_raw`, and a voice→robot intent bridge that publishes `TwistStamped` on
`/joy_vel` and toggles `frontier_explorer`.

## Run

```bash
nix run .#perseus-lite-voice                                 # default
nix run .#perseus-lite-voice -- --ros-args -p wake_enabled:=true
```

The launcher pins `ROS_DOMAIN_ID=42`. Use the same domain in any second
terminal: `ROS_DOMAIN_ID=42 ros2 topic list`.

---

## REPL → ROS mapping

The `chat.py` REPL is replaced by ROS topics, services, and (when wake mode is
on) verbal commands. The table below shows every original REPL function and how
to invoke it now.

| `chat.py` REPL                | ROS equivalent                                                                                                              | Verbal (wake mode)                       |
| ----------------------------- | --------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------- |
| Type a prompt + `<Enter>`     | `ros2 topic pub --once /perseus_lite_voice/prompt std_msgs/String "{data: '...'}"`                                          | _say anything after the wake word_       |
| `<Enter>` on empty / `:say`   | _push-to-talk not exposed standalone — use wake mode instead_                                                               | wake word triggers capture automatically |
| Speak text without LLM        | `ros2 topic pub --once /perseus_lite_voice/say std_msgs/String "{data: '...'}"`                                             | —                                        |
| `:reset` (clear history)      | `ros2 service call /perseus_lite_voice/reset_history std_srvs/srv/Trigger`                                                  | "reset history" / "forget that"          |
| `:see` (run vision)           | `ros2 service call /perseus_lite_voice/detect std_srvs/srv/Trigger`                                                         | "what do you see" / "describe what you see" |
| `:vol` / `:vol N`             | _not exposed yet — set `initial_volume` in `voice_params.yaml` and restart_                                                 | —                                        |
| `:q` / `:quit` / `Ctrl-D`     | `Ctrl-C` the launcher                                                                                                       | —                                        |
| `Ctrl-C` (interrupt speech)   | _no equivalent yet; `Ctrl-C` exits the node entirely_                                                                       | —                                        |
| Wake-word activation          | launch with `-p wake_enabled:=true`                                                                                         | "hey jarvis ..."                         |

---

## Voice → robot intents (wake mode)

When wake mode is on, transcripts are routed through `intents.py` before the
LLM. Phrases matching these patterns dispatch directly without an Ollama
round-trip; everything else falls through to chat.

| Intent              | Trigger phrases                                                       | Action                                                                                                |
| ------------------- | --------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| `stop`              | "stop", "halt", "freeze", "hold"                                      | publish zero `TwistStamped` on `/joy_vel`                                                             |
| `forward`           | "go/move/drive/head forward(s)"                                       | `linear.x = motion_linear_speed` for `motion_pulse_seconds`, then zero (re-published at 10 Hz)        |
| `backward`          | "go/move/drive/head back(ward)(s)"                                    | `linear.x = -motion_linear_speed` pulse                                                               |
| `turn_left`         | "turn/rotate/spin left"                                               | `angular.z = +motion_angular_speed` pulse                                                             |
| `turn_right`        | "turn/rotate/spin right"                                              | `angular.z = -motion_angular_speed` pulse                                                             |
| `start_exploring`   | "start exploring", "begin exploring", "go explore", "explore"         | `SetParameters` on `frontier_explorer`: `planning_period_sec = explorer_default_period_sec`           |
| `stop_exploring`    | "stop exploring"                                                      | `SetParameters` on `frontier_explorer`: `planning_period_sec = explorer_pause_period_sec` (≈ pause)   |
| `see`               | "what do you see", "describe what you see", `:see`                    | run YOLO on the latest `/image_raw` frame, publish JSON to `~/detections`, speak a summary            |
| `reset_history`     | "reset history", "reset conversation", "forget everything/that"       | clear LLM `history` (same as the `~/reset_history` service)                                           |
| `chat` (fall-through) | anything else                                                        | send to Ollama, stream the reply through the speaker, publish on `~/reply`                            |

---

## ROS interface reference

### Subscribed topics

| Topic                                  | Type                | Purpose                                                       |
| -------------------------------------- | ------------------- | ------------------------------------------------------------- |
| `~/say`                                | `std_msgs/String`   | Speak text directly via Piper (no LLM round-trip)             |
| `~/prompt`                             | `std_msgs/String`   | Push prompt through Ollama; reply is spoken and published     |
| `<vision_topic>` (default `/image_raw`) | `sensor_msgs/Image` | Latest-frame buffer for YOLO detections                       |

### Published topics

| Topic                                       | Type                          | Purpose                                                                          |
| ------------------------------------------- | ----------------------------- | -------------------------------------------------------------------------------- |
| `~/heard`                                   | `std_msgs/String`             | Each transcribed user utterance (push-to-talk or wake)                           |
| `~/reply`                                   | `std_msgs/String`             | Final Ollama reply text after streaming completes                                |
| `~/state`                                   | `std_msgs/String`             | Equivalent of the chat.py status bar (`idle`, `thinking`, `recording`, ...)      |
| `~/intent`                                  | `std_msgs/String`             | Matched intent name (or `chat`) for every transcript                             |
| `~/detections`                              | `std_msgs/String` (JSON list) | YOLO detections: `[{label, conf, bbox, frame_id, stamp}, ...]`                   |
| `<cmd_vel_topic>` (default `/joy_vel`)      | `geometry_msgs/TwistStamped`  | Voice-driven velocity (only when `enable_robot_bridge:=true`)                    |

### Services

| Service              | Type                  | Purpose                                                            |
| -------------------- | --------------------- | ------------------------------------------------------------------ |
| `~/reset_history`    | `std_srvs/Trigger`    | Clear LLM history (REPL `:reset`)                                  |
| `~/detect`           | `std_srvs/Trigger`    | Force a YOLO pass on the latest frame (REPL `:see`)                |

### Service clients

| Service                                       | Type                            | Purpose                                                |
| --------------------------------------------- | ------------------------------- | ------------------------------------------------------ |
| `/<explorer_node>/set_parameters`             | `rcl_interfaces/SetParameters`  | Toggle `planning_period_sec` for explore start/stop    |

---

## Parameters

See `config/voice_params.yaml` for defaults. Key knobs:

| Parameter                       | Default                                                            | Notes                                                                 |
| ------------------------------- | ------------------------------------------------------------------ | --------------------------------------------------------------------- |
| `ollama_url`, `ollama_model`    | `http://localhost:11434/api/chat`, `gemma4:e2b`                    | Ollama daemon must be running with the model pulled                   |
| `system_prompt`                 | concise voice-friendly assistant prompt                            | Prepended to every conversation                                       |
| `voice_path`                    | `~/Programming/Piper/voices/en_US-amy-medium.onnx`                 | Piper voice `.onnx` (and adjacent `.onnx.json`)                       |
| `audio_device`, `mic_device`    | `plughw:1,0`, `plughw:0,0`                                         | ALSA device strings; verify with `aplay -l` / `arecord -l`            |
| `initial_volume`                | `0.2`                                                              | Piper volume multiplier; **only read at startup**                     |
| `mic_enabled`, `wake_enabled`   | `true`, `false`                                                    | `wake_enabled:=true` forces `mic_enabled:=true`                       |
| `whisper_model`                 | `tiny`                                                             | Lazy-loaded on first STT use                                          |
| `wake_model`, `wake_threshold`  | `hey_jarvis_v0.1`, `0.5`                                           | openwakeword built-in models; raise threshold for fewer false hits    |
| `vision_enabled`, `vision_topic`, `vision_max_fps` | `true`, `/image_raw`, `2.0`                     | YOLO is throttled to one inference per `1/max_fps` seconds            |
| `yolo_model`                    | `yolov8n.pt`                                                       | Bundled in `share/perseus_lite_voice/models/`                         |
| `enable_robot_bridge`           | `true`                                                             | Set `false` to mute all velocity / explorer side effects              |
| `cmd_vel_topic`                 | `/joy_vel`                                                         | Same slot the joystick uses; goes through twist_mux when enabled      |
| `explorer_node`                 | `frontier_explorer`                                                | Node whose `planning_period_sec` is toggled to (un)pause exploration  |
| `motion_linear_speed`, `motion_angular_speed`, `motion_pulse_seconds` | `0.2`, `0.5`, `1.0`                       | Per-pulse velocity for the `forward`/`backward`/`turn_*` intents      |

---

## Runtime requirements

These are **runtime** dependencies of the node (the Nix package itself does not
pull them):

- `piper` binary at `~/.local/bin/piper` (the `piper-tts` pip package), with the
  `~/.local/lib/python3.10/site-packages/piper` module on the user-site path
- `aplay` and `arecord` (system ALSA tools)
- Ollama daemon listening at `ollama_url`
- For wake mode: `openwakeword` and `tflite_runtime` from the venv at
  `~/Programming/Piper/oww-env/`. The launcher prepends this venv to
  `PYTHONPATH`; override with `OLLAMA_VOICE_VENV=/path/to/venv`.
- For vision: a publisher on `vision_topic` (`v4l2_camera` from the
  `perseus_lite` bringup launches `/image_raw` at 320×240 YUYV)

## Quick smoke test

```bash
# In one shell:
nix run .#perseus-lite-voice

# In another:
ROS_DOMAIN_ID=42 ros2 topic pub --once /perseus_lite_voice/say \
  std_msgs/String "{data: 'hello world'}"
```
