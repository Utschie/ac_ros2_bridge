# control.ini — Control Downlink Configuration

Configures the control downlink (how driving and session commands reach the
game). Read once at bridge startup — restart the bridge to apply changes.

| Key | Shipped | Description |
|---|---|---|
| `MODE` | `shm` | `shm` (default, for models/scripts), `vjoy` (virtual-device mode) or `off`. **Session commands (start race / pause / …) only work with `shm`** — see [game-control.md](../game-control.md). |
| `RATE_HZ` | `100` | Rate at which control commands are written (Hz). |
| `RX_TIMEOUT_MS` | `500` | How long after you stop sending driving commands control falls back to the human (ms). |
| `NEUTRAL_ON_TIMEOUT` | `1` | Whether a neutral input is written once when falling back to the human. |
