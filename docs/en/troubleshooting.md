# Troubleshooting

First stop for anything that does not work. All suite programs write
timestamped logs into `ac_simulator_suite\logs\` — the newest file of the
program in question is always the first place to look. The launcher's
status area tells you which programs are alive and whether data is flowing.

## Installation & startup

| Symptom | Cause and fix |
|---|---|
| Installer warns about the NVIDIA driver | The GPU must support CUDA 13.x; very old drivers can crash — update the driver from nvidia.com. |
| Installer refuses to continue: "Assetto Corsa is running" | Close the game completely (back to the desktop) and rerun the installer. |
| Cameras come out black, or a camera is "disabled with an error" at game start | Cameras require **in-game HDR on and motion blur off** (Options → Video). Set both and restart the game. |
| Launcher status shows `waiting for game` forever | Make sure you are actually inside a track session; if it still waits, the plugin is not loaded — reinstall the suite. Other graphics-enhancement mods (filters / patch packs) can conflict with this suite and must be removed. |
| A suite program pops up "config file missing" and exits | The `configs\` folder is incomplete — reinstall the suite. |
| Controls greyed out, "License required" | Activate first — see [activation.md](activation.md). |

## Recording

| Symptom | Cause and fix |
|---|---|
| Nothing is being dumped | Was **Record dump data** ticked *before* Start Bridge? Is the game in a session (status `scene OK`)? |
| Dropped-frame counter in the bridge log keeps growing | The disk/writer can't keep up — follow "Recording keeps dropping frames" below. |
| `inspect_dump.py` prints `[warn]` lines | Read the line — it names the exact mismatch. Frame **numbering jumps** in the image folders are normal and are *not* dropped frames. |
| Depth-equipped cameras record at only 1–3 Hz | The image viewer is open — it is a second consumer of the same data. Close it while recording. |
| One modality has more/fewer files than another | Within ±1 file number this is normal; cross-modal pairing is by `sceneFrameId` in `index.jsonl`, which stays exact. |

## Recording keeps dropping frames

The bridge prints a per-second stats line in the launcher's log pane; the
`d` inside `rec[...]` is the dropped-frame counter. If it stays above zero,
try in this order:

1. Set the color format to `jpeg` (`IMAGE_COLOR_FORMAT` in `recorder.ini`);
2. Use `png` or `inferno` for depth (not `raw`);
3. Lower `IMAGE_HZ`, or record fewer cameras/modalities;
4. Point `ROOT_DIR` at a faster disk (SSD);
5. Raise `WRITER_THREADS`.

If it still persists, collect a screenshot of the launcher status area and
the newest logs under `ac_simulator_suite\logs\` and contact the vendor.

## Streaming to Linux / ROS2

| Symptom | Cause and fix |
|---|---|
| Relay looks ready but all topics empty | Relay started without `ZENOH_CONFIG` (not listening on 7447), or bridge.json5 points at the wrong IP, or the bridge wasn't restarted after editing it. Full checklist: [ros2-relay.md](ros2-relay.md). |
| Bridge sends at full rate, Linux receives far less, multi-second gaps | Link bandwidth (typical on WiFi). Use the direct gigabit Ethernet setup in [ros2-relay.md](ros2-relay.md). |
| Color image topic not selectable in RViz2 | It is a CompressedImage; RViz2's picker filters by type — run the republish node (see [ros2-relay.md](ros2-relay.md)). |

## Viewers

| Symptom | Cause and fix |
|---|---|
| Image viewer shows "please start the game first" | Start the game and enter a session first; the viewer is a read-only consumer. |
| 3D viewer shows only an empty grid | No lidar data: is the lidar renderer running (**Start Lidar**) and the game in a session? The camera auto-frames on the first data frame. |
| Telemetry viewer stuck on a waiting hint | The game's telemetry switch is off — the installer sets it automatically; if you moved the game folder, reinstall the suite. |
| Annotator window opens grey / empty | Game not inside a track yet — it reconnects automatically once you enter one. |

## Game control

See the troubleshooting table in [game-control.md](game-control.md)
(greyed buttons, commands not arriving, log lines to check).

## Licensing & updates

| Symptom | Cause and fix |
|---|---|
| Activation fails | Copy-paste the key; check network to the license server; the exact HTTP error is in `logs\license_dbg.log`. A key already bound to another machine is refused — ask the vendor to unbind. See [activation.md](activation.md). |
| Update check shows an error line | The update server is unreachable; the rest of the suite is unaffected. Try later or install manually. |

## Still stuck?

Collect, in this order: (1) the launcher status screenshot, (2) the newest
`ac_simulator_suite\logs\*.log` files, (3) what you did right before the
problem, (4) `python tools\inspect_dump.py <session>` output if it is about
recorded data — and contact the vendor.
