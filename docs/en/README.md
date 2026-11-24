# AC Simulator Suite — Documentation (English)

New here? Start with the [quickstart](quickstart.md) — it takes you from
the installer to your first verified data recording.

## Reading order

| # | Document | What it covers |
|---|---|---|
| 1 | [quickstart.md](quickstart.md) | Prerequisites, install, activation, first session, first dump |
| 2 | [activation.md](activation.md) | Product keys, activation, offline grace, changing keys |
| 3 | [launcher.md](launcher.md) | Every launcher button, the status panel, updates, launcher.ini |
| 4 | [game-control.md](game-control.md) | Controlling the race session: buttons / zenoh / ROS2 |

## Configuration reference (`configs\`)

| Document | File it explains |
|---|---|
| [configs/custom_cameras.md](configs/custom_cameras.md) | custom_cameras.ini — cameras, pipelines, effects |
| [configs/custom_lidars.md](configs/custom_lidars.md) | custom_lidars.ini — lidars (solid-state & rotating), noise |
| [configs/capture_engine.md](configs/capture_engine.md) | capture_engine.ini — the in-game capture engine |
| [configs/streamer.md](configs/streamer.md) | streamer.ini — what gets forwarded over zenoh |
| [configs/recorder.md](configs/recorder.md) | recorder.ini — dump recording |
| [data-formats.md](data-formats.md) | Data formats — array order / coordinate frames / field semantics (dump & ROS) |
| [configs/control.md](configs/control.md) | control.ini — control downlink |
| [configs/semantic_classes.md](configs/semantic_classes.md) | semantic_classes.ini + the per-track annotation json chain |

## Tools & viewers

| Document | What it covers |
|---|---|
| `tools\README.md` (next to the tools) | The Python dump-inspection/labeling/control-test scripts |
| [viewer-images.md](viewer-images.md) | Image grid viewer (View Images) |
| [viewer-3d.md](viewer-3d.md) | 3D point-cloud viewer (3D View) |
| [viewer-telemetry.md](viewer-telemetry.md) | Telemetry dashboard (View Telemetry) |
| [annotator.md](annotator.md) | Semantic annotation & scene editing (Annotator) |
| [annotation-files.md](annotation-files.md) | The semantic rule files (json format) |

## Going further

| Document | What it covers |
|---|---|
| [ros2-relay.md](ros2-relay.md) | Streaming to a Linux machine / ROS2 (topology, RViz2, tuning) |
| [troubleshooting.md](troubleshooting.md) | Symptoms → causes → fixes, incl. the bridge stats line |

中文文档在 `docs\zh\`（结构相同）。
