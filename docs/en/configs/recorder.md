# recorder.ini — Recording (Dump) Configuration

Configures dump-to-disk recording: ground truth + state + lidar + camera
images + expert driving controls. The ini is read once at bridge startup —
restart the bridge to apply changes.

## Session directory layout

Each recording is one session folder; each continuous driving stint is one
run subdirectory, with frame numbers (`sceneFrameId`) restarting at 1 per run.

Online (multiplayer) sessions are split by the same rules as offline, one run
per stint: practice/qualify/race each become their own run, and every server
session switch (practice restart, rolling into the next race, ...) starts a
new run. Inside a race run, the post-finish coasting and screen fade-out stay
in that run; it closes the moment the game switches back to the pit view.
Frames between runs (waiting / voting / pit area) are not recorded. When you
join a server mid-session, that run starts from the moment you join.

```
<dump>\<timestamp>\manifest.json            session summary and counts
<dump>\<timestamp>\spans.jsonl              one line per driving stint
<dump>\<timestamp>\vehicle_state.jsonl      session level
<dump>\<timestamp>\game_state.jsonl         session level
<dump>\<timestamp>\run_state.jsonl          run events (session level)
<dump>\<timestamp>\telemetry_static.jsonl   car/track static snapshot (session level)
<dump>\<timestamp>\runs\epoch_1\            first stint:
    ego_pose.jsonl  detections_3d\  cameras.jsonl  expert_ctrl.jsonl
    telemetry.jsonl  timing.jsonl
    lidar\<name>\  cameras\<cam>\<modality>\  (images + index.jsonl)
```

Inspection tools (shipped in `tools\`):

```bat
python tools\inspect_dump.py <session dir>
python tools\view_image.py   <session dir>\runs\epoch_1 --cam front
```

## Coordinate convention

Recorded data follows the suite-wide convention (X fwd / Y left / Z up;
world z = altitude — see [quickstart.md](../quickstart.md)). Array order,
matrix rows/columns, per-file field semantics, and the coordinate exception
(the world-coordinate fields inside the telemetry block are raw game-physics
values) are all detailed in [data-formats.md](../data-formats.md).

## [recorder] keys

| Key | Shipped | How to change | Description |
|---|---|---|---|
| `ENABLED` | `0` | launcher checkbox | Master switch. Control it with the launcher's **Record dump data** checkbox (tick before starting the bridge). |
| `ROOT_DIR` | `..\..\dump` | manual edit | Parent of the session folders (`..\..` = the AC install root). Point it at a faster disk (absolute path) if the disk can't keep up. |
| `LIDAR` | `1` | leave as is | Record lidar scans (`lidar/<name>/<scan#>.pcd`). |
| `TELEMETRY` | `1` | leave as is | Telemetry channel: live publishing and dumping share this switch. |
| `EXPERT_CTRL` | `1` | leave as is | Expert driving controls for imitation learning, strictly frame-aligned with the ground truth; each continuous driving stint becomes one run automatically. |
| `IMAGES` | `1` | leave as is | Record camera images. |
| `IMAGE_CAMS` | `all` | manual edit | Which cameras to record (`all` or a comma list). |
| `IMAGE_MODALITIES` | `all` | manual edit | Which modalities to record (`all` or a comma list). |

> ⚠ After `ROOT_DIR=` write the path only — never append a comment.

## Per-camera overrides

Optional `[CAMERA_<name>]` sections control that camera's recording only
(forwarding is not affected), overriding the global defaults above:

- `MODALITIES` — which modalities (default `all`);
- `HZ` — that camera's recording rate (falls back to the global `IMAGE_HZ`);
- `FORMAT_COLOR` / `FORMAT_SEMANTIC` / `FORMAT_DEPTH` — see below.

## Recording rate

| Key | Shipped | Description |
|---|---|---|
| `IMAGE_HZ` | `10` | Frames per second per camera (fractional values accepted; per-camera override possible). |

## Image dump formats

Priority: per-camera `FORMAT_*` > explicit global keys > the `IMAGE_FORMAT`
shorthand.

| Key | Shipped | Description |
|---|---|---|
| `IMAGE_FORMAT` | `png` | Shorthand default for color and semantic. |
| `IMAGE_COLOR_FORMAT` | `jpeg` | Color: `png` / `jpeg` / `raw`. |
| `IMAGE_SEMANTIC_FORMAT` | `png` | Semantic: `png` recommended (pixel = class id, loss-free, human-viewable). |
| `IMAGE_DEPTH_FORMAT` | `png` | Depth: `png` = 16-bit metric (for training); `raw` = raw values; `inferno` / `rainbow` = colored visualization. |

## Writer tuning

| Key | Shipped | Description |
|---|---|---|
| `WRITER_THREADS` | `6` | Number of disk-writer threads. |
| `QUEUE_MB` | `1024` | Queue budget; frames are dropped and counted when full — the live-forwarding path is never blocked. |

If dropped frames keep growing, follow the order given in
[troubleshooting.md](../troubleshooting.md).
