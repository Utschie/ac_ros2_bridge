# streamer.ini — Image Forwarding Configuration

Selects which cameras and modalities the data bridge forwards live over
zenoh (what the Linux/ROS2 side receives). Read once at bridge startup —
restart the bridge to apply changes.

Format: one `[CAMERA_<name>]` section per camera to forward (the name = the
camera's NAME in custom_cameras.ini); cameras not listed are not forwarded.

| Key | Default | Description |
|---|---|---|
| `BRIDGE_COLOR` | `0` | Forward the color modality (1/0). |
| `BRIDGE_DEPTH` | `0` | Forward the metric depth modality. |
| `BRIDGE_SEMANTIC` | `0` | Forward the semantic segmentation modality. |
| `HZ` | `10` | Forwarding rate (frames/s), per camera. |

Without this file: all cameras, color only, 10 Hz. File present but nothing
enabled = nothing forwarded.

Bandwidth reference: the default five cameras fully on ≈ 18–23 MB/s — if the link can't
carry that, see the direct-Ethernet setup in [ros2-relay.md](../ros2-relay.md).
