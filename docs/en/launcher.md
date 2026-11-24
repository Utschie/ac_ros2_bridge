# Launcher Reference

The launcher is the suite's control panel: it starts and stops the data
programs, shows their status, edits the main config switches, and relays the
bridge's log. The desktop / start-menu shortcut "AC Simulator Suite" opens it.

Child programs run without console windows; the bridge's output appears in
the launcher's bottom log pane. Closing the launcher also stops everything it
started (recordings are finished cleanly first). The launcher is
single-instance — launching it again just restores the window.

## Program control

| Control | What it does |
|---|---|
| **Start / Stop Bridge** | Starts / stops the data bridge (dump recording, zenoh streaming, control downlink and telemetry all live here). Stop finishes the recording cleanly before exiting — always stop the bridge this way after a recording. |
| **Start / Stop Lidar** | Starts / stops the lidar renderer (no window). Only needed when you want lidar point clouds. Options below apply at start time only. |
| **View Images / 3D View / View Telemetry / Annotator** | Launch the four viewers. 3D View is greyed out if the 3D-viewer component was not installed; Annotator needs the annotator component. See the index for each viewer's doc. |

## Startup options (the switches are config editors)

| Option | Config it edits | Meaning |
|---|---|---|
| ☑ **Record dump data** | `recorder.ini [recorder] ENABLED` | Master switch of dump recording. Tick it *before* clicking Start Bridge — while the bridge is running the checkbox is disabled. It always reflects the file's current state. |
| ☑ **Real lidar rate (--real-hz)** | `launcher.ini [optix] REAL_HZ` | Ticked = lidar scans at the configured `SIM_HZ` (real sensor rate); unticked = one scan per rendered frame (smoother clouds). |
| **GPU dropdown (lidar)** | `launcher.ini [optix] GPU` | Which NVIDIA GPU runs the lidar renderer (prefer the one not running the game). |
| **3D viewer GPU dropdown** | `launcher.ini [viewer3d] ADAPTER` | Which GPU runs the 3D viewer (same advice: prefer the one not running the game). Cannot be changed while the viewer is running — it applies at launch. |

## Game Control row

**Start Race / Pause(Resume) / Restart** — one toggle button for
pause/resume; the label next to the buttons shows the live session state
(e.g. `Session: Race | GO`). Requires the bridge running and
`control.ini MODE=shm`; the label explains why when greyed. All three ways to
control the session (buttons / zenoh / ROS2) are documented in
[game-control.md](game-control.md).

## Status area

Refreshed once per second. Each row shows the process state plus the data
heartbeats — this distinguishes "process alive" from "data flowing":

| Item | Shows |
|---|---|
| Game plugin | `scene OK (frame N)` while the game is in a session; `waiting for game` otherwise |
| Bridge | scene/image/control data heartbeats |
| Lidar renderer | lidar data heartbeat + point rate |
| Image viewer / 3D viewer / Telemetry viewer / Semantic annotator | running state |

## Open… / Edit… buttons

- **Open Dump / Open Configs / Open Logs** — Explorer windows into
  `<AC root>\dump`, `ac_simulator_suite\configs\`, `ac_simulator_suite\logs\`.
- **Edit recorder.ini / streamer.ini / control.ini / custom_cameras.ini /
  custom_lidars.ini / classes.json** — opens the file in Notepad. What each
  file does: see [configs/](configs/). Restart the owning program (bridge or
  game, as noted per file) after editing.

## Updates

- **Check Updates** button (plus one automatic check 3 s after launch)
  against the update server configured in `launcher.ini [update] URL`.
- When a newer version exists, the top bar shows `Update available: <version>`
  with an **[Update]** button: click it and the update applies automatically
  (your component selection is kept); relaunch the launcher afterwards.

## launcher.ini keys

`ac_simulator_suite\configs\launcher.ini` (shipped as a template; the
GPU/rate options are written back by the launcher itself):

| Key | Default | Meaning |
|---|---|---|
| `[update] URL` | `https://suite.dym-lab.com` | Update server base URL. Change only when the vendor says so. |
| `[license] URL` | `https://suite.dym-lab.com/api` | License server URL. See [activation.md](activation.md). |
| `[optix] REAL_HZ` | `0` | 1 = run the lidar renderer at real sensor rate. |
| `[optix] GPU` | `1` | GPU number used by the lidar renderer / annotator. |
| `[viewer3d] ADAPTER` | *(empty = auto)* | GPU for the 3D viewer; empty = automatically pick the NVIDIA card not running the game. |
| `[annotator] GPU` | *(empty)* | Annotator GPU (same numbering as above). |
