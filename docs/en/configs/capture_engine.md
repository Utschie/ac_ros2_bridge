# capture_engine.ini — In-Game Capture Engine Configuration

Switches of the in-game capture engine. **Normal use never needs to touch
this file** — the shipped configuration already has it enabled. Read once at
game startup; restart the game after changes.

| Key | Shipped | Description |
|---|---|---|
| `CAPTURE_ENABLED` | `1` | Production master switch: off = the game produces no data at all — both recording and live streaming stop. |
| `STREAM_ENABLED` | `1` | Live-stream switch: off affects only the Linux/ROS2 live stream; local recording is unaffected. |

Tip: to "record but not stream", editing `streamer.ini` (no camera sections)
is more direct than this switch.
