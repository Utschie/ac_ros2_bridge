# custom_cameras.ini — Custom Camera Configuration

Defines the extra custom cameras rendered by the game every frame. Loaded
once at game startup from `ac_simulator_suite\configs\custom_cameras.ini`;
the pose and optics parameters support save-to-apply hot reload (see below).

Parsing is strict: a missing/invalid required value produces a startup popup
error and disables the whole camera system. There is no silent fallback.

## Coordinate frame

Mount positions and orientations use the suite-wide convention:
**X forward, Y left, Z up** (see [quickstart.md](../quickstart.md)).
Front-facing cameras use `FORWARD=1,0,0`; right-side cameras use negative y,
left-side cameras positive y.

Reference vehicle: Porsche 911 GT3 RS (992) — 4572 × 1900 × 1322 mm
(half width 0.95 m, roof z ≈ 1.32 m, rear bumper x ≈ -2.8 m).

## [render] — render pipeline

| Key | Default | Description |
|---|---|---|
| `PIPELINE` | *(required)* | Render pipeline: `NATIVE_HDR` (independent rendering, more stable; requires in-game HDR on and motion blur off) or `YEBIS` (follows the game's main-image post-processing; may occasionally flicker at low frame rates). |
| `EXPOSURE` | `1.0` | Exposure multiplier (raise it, e.g. to 1.3, if the image is too dark). |

## [CAMERA_<name>] — one section per camera

The file must contain at least one `[CAMERA_*]` section.

| Key | Default | Description |
|---|---|---|
| `NAME` | *(required)* | Unique camera name; the other configs select cameras by it. |
| `POSITION` | `0,0,0` | Mount position `x,y,z` (meters, X fwd / Y left / Z up). |
| `FORWARD` | `1,0,0` | View-direction vector in the body frame (forward = `1,0,0`); a rear-facing camera uses `-1,0,0`. |
| `VERTICAL_FOV` | `60` | Vertical field of view (degrees). |
| `NEAR` / `FAR` | `0.05` / `2000` | Near / far clip planes (meters) — scenery beyond these ranges is not rendered. |
| `WIDTH` / `HEIGHT` | *(required)* | Output image size (pixels). |
| `EMIT_COLOR` | `1` | Publish the color modality. |
| `EMIT_DEPTH` | `0` | Publish the metric depth modality. |
| `EMIT_DEPTH_VIS` | `0` | Publish the depth-visualization modality. |
| `EMIT_SEMANTIC` | `0` | Publish the semantic segmentation modality. |
| `ENABLED` | `1` | 0 = skip this camera. |

## Hot reload (save to apply)

About one second after saving the file, the following **five parameters**
take effect automatically — no game restart needed:

- `POSITION`, `FORWARD`, `VERTICAL_FOV`, `NEAR`, `FAR`

All other changes (`WIDTH`/`HEIGHT`, `PIPELINE`, `EMIT_*`, `ENABLED`,
adding or removing cameras, the `[render]` and `[depth_vis]` sections)
still require a game restart; the log notes which changes did not take
effect when you save.

- A faulty file (invalid values, duplicate camera names, …) never pops up a
  window: the reason is logged and the previous parameters keep running;
  fix the file and save again to re-enable hot reload.
- After changing `VERTICAL_FOV`/`NEAR`/`FAR`, the ROS-side camera intrinsics
  (`camera_info`) update automatically within a few seconds.

## Depth-visualization colormap

`[depth_vis] CMAP` selects the colormap of the depth-visualization modality:
`inferno` (default) or `rainbow`. This only affects the on-screen view; the
recorded depth format is configured in `recorder.ini` (see
[recorder.md](recorder.md)).
