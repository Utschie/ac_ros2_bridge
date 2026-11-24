# Quickstart: From Installer to Your First Data Recording

This guide takes a fresh machine from zero to a recorded, verified data dump.
Each step links to the detailed document for that topic.

## 1. Prerequisites

| Requirement | Notes |
|---|---|
| Assetto Corsa (Steam version) | Can be started from Steam; starting via **Content Manager** is recommended (download <https://acstuff.ru>). Do **not** install Custom Shader Patch (CSP) in Content Manager — this software is incompatible with CSP. |
| Windows 10 / 11 x64 | — |
| NVIDIA GPU (CUDA 13.x capable) | The GPU must support CUDA 13.x. |
| ~2 GB free disk | Plus whatever your dump recordings need. |

Two in-game settings must be set once (Options → Video):

- **HDR: ON**
- **Motion blur: OFF**

Cameras refuse to run without these. Everything else works regardless.

> The installer applies the required in-game settings and installs the VC++
> runtime silently — nothing to do by hand.

### Coordinate convention (one rule everywhere)

The whole suite uses the standard robot convention (REP-103): **X forward,
Y left, Z up**; the world frame has x/y horizontal and **z = altitude**.
Camera/lidar mounts, recorded dumps, ROS topics and TF, and RViz all follow
the same rule — what you write in a config is what you see everywhere.

## 2. Install

1. Run `ACSuiteSetup-<version>.exe`. If Assetto Corsa is running, the
   installer asks you to close it first.
2. Choose components:
   - **Core** (always installed) — in-game plugin, data bridge, launcher,
     image/telemetry viewers, configs, docs, dump tools;
   - **3D viewer** — live point-cloud visualization (optional);
   - **Lidar renderer** — lidar simulation (optional; needs the NVIDIA GPU);
   - **Semantic annotator** — the track annotation tool (optional; needs the
     NVIDIA GPU).
3. The installer finds your Assetto Corsa folder via Steam automatically.
   If you use other graphics-enhancement mods (filters / patch packs), they
   may conflict with this suite — see [troubleshooting.md](troubleshooting.md).

Shortcuts "AC Simulator Suite" land on the desktop and the start menu.

## 3. Activate

Start the launcher (desktop shortcut). On a licensed install with no valid
key yet, all controls are greyed out and the top shows **[Activate]**.

Enter the product key you received → the suite unlocks and shows
`License OK (<expiry date>)`. Details: [activation.md](activation.md).

## 4. First session

1. **Start Assetto Corsa** and drive into any track session.
2. In the launcher, click **Start Bridge**. Once the game is inside a
   session, the status area's *Game plugin* row shows `scene OK (frame N)`
   and *Bridge* shows `running`.
3. Optional, live viewing while you drive:
   - **View Images** — the camera grid (color/depth/semantic);
   - **3D View** — point clouds + ground-truth boxes (needs the lidar
     renderer running: **Start Lidar**);
   - **View Telemetry** — the MoTeC-style dashboard and channel plots.
   Each has its own doc (see the index [README.md](README.md)).
4. To record data: tick **Record dump data** *before* clicking Start Bridge
   (the checkbox edits `recorder.ini`, which is read at bridge startup), then
   drive a lap.
5. Click **Stop Bridge** — it finishes the recording cleanly (this is the
   normal way to end a recording session).
6. Click **Open Dump** and check the session:

```bat
cd /d <AC install folder>
python ac_simulator_suite\tools\inspect_dump.py dump\<latest session folder>
```

The report is mainly per-channel counts and hints; when it flags warnings,
follow the advice in [troubleshooting.md](troubleshooting.md). To look at
the data itself:

```bat
python ac_simulator_suite\tools\view_image.py dump\<session>\runs\epoch_1 --cam front --modality color
python ac_simulator_suite\tools\view_lidar.py dump\<session>\runs\epoch_1
```

The tools need Python with `numpy`, `open3d`, `opencv-python`
(`pip install numpy open3d opencv-python`).

## 5. Where to go next

| I want to… | Read |
|---|---|
| Understand every launcher button and the status panel | [launcher.md](launcher.md) |
| Control the race (start/pause/restart) from buttons, zenoh or ROS2 | [game-control.md](game-control.md) |
| Change cameras / add cameras | [configs/custom_cameras.md](configs/custom_cameras.md) |
| Change lidars | [configs/custom_lidars.md](configs/custom_lidars.md) |
| Tune recording (formats, rates, what to record) | [configs/recorder.md](configs/recorder.md) |
| Tune live streaming (which cameras, how fast) | [configs/streamer.md](configs/streamer.md) |
| Watch the live camera feeds | [viewer-images.md](viewer-images.md) |
| Watch the 3D point cloud and ground-truth boxes | [viewer-3d.md](viewer-3d.md) |
| Watch the telemetry dashboard | [viewer-telemetry.md](viewer-telemetry.md) |
| Stream data live to a Linux / ROS2 machine | [ros2-relay.md](ros2-relay.md) |
| Semantic annotation & scene editing | [annotator.md](annotator.md) |
| Understand the dump files | [configs/recorder.md](configs/recorder.md) |
| Something does not work | [troubleshooting.md](troubleshooting.md) |

Full index: [README.md](README.md). 中文文档：`docs\zh\`。
