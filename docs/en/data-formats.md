# Data Format Reference (array order · coordinate frames · field semantics)

Everything in the dumped files and ROS topics that is "positional" (arrays,
matrices, enums) is documented here in one place. Overall convention (see
[quickstart.md](quickstart.md)): **X fwd / Y left / Z up; world Z = height**.

## 1. Coordinate frames: AC native → our output

The game has two native frames internally. **The bridge converts everything
before output**; dump files and ROS topics are consistent — no further
conversion is ever needed:

| Data | AC native | Our output |
|---|---|---|
| World coords: `velocity`, `carCoordinates`, `tyreContact*` | physics world (y = height, x/z horizontal) | unified Z-up world |
| Body components: `accG`, `localVelocity`, `localAngularVel` | body frame (x = left, y = up, z = fwd) | unified body frame (x = fwd, y = left, z = up; forward-accel positive, leftward force positive, left-turn yaw positive) |
| Lidar points (x, y, z) | sensor-local (same as the convention) | unchanged |
| Ego matrix, detections, camera matrices, cloud world poses | — | generated directly in the unified convention |

Angles are not converted; their reading conventions:

- `heading`: compass-style clockwise (0 = fwd, +90° = right); azimuth in the
  unified convention = `-heading`;
- `windDirection`: same reading; `steerAngle`: positive = steering right;
- `pitch` / `roll`: game-native body Euler angles; for angular-motion
  analysis prefer `localAngularVel` directly.

## 2. Wheel arrays: index 0..3 = FL, FR, RL, RR

Every `[4]` wheel array is ordered front-left, front-right, rear-left,
rear-right (the [FL][FR][RL][RR] labels in the telemetry viewer): physics
`wheelSlip` `wheelLoad` `wheelsPressure` `wheelAngularSpeed` `tyreWear`
`tyreDirtyLevel` `tyreCoreTemperature` `camberRAD` `suspensionTravel`
`brakeTemp` `tyreTempI/M/O`, the increment block (valid when `incValid=1`)
`inc_mz` `inc_tyreLoadedRadius` `inc_slipAngle` `inc_slipRatio`, and the
static snapshot's `suspensionMaxTravel` `tyreRadius`.

The `[12]` arrays (`tyreContactPoint/Normal/Heading`) are component-major,
wheel-minor: first 4 = x of the four wheels, middle 4 = y, last 4 = z.

Three non-wheel exceptions: `carDamage[5]` = front, rear, left, right,
overall; `rideHeight[2]` = front, rear; `inc_caster[2]` = front-axle left,
right.

## 3. Matrices (16 floats, row-major 4×4)

| Field | Row semantics |
|---|---|
| `world` in `ego_pose.jsonl` | rows 0/1/2 = the body's **fwd / left / up** axes; row 3 = position |
| `view` in `cameras.jsonl` | rows 0/1/2 = the camera's **right / up / back** axes; row 3 = camera position |

## 4. Detections (`detections_3d/<frame id>.json`)

`pos[3]` = center (unified world); `half[3]` = half extents, order
**(fwd, left, up)**; `rot[9]` = 3×3 rotation (row-major, rows fwd/left/up);
`classId` indexes the semantic class table; an object-identity field keeps
its value across frames (for tracking). The per-image `index.jsonl` carries
2D boxes in pixels (top-left origin).

## 5. Lidar point clouds (`lidar/<name>/<scan id>.pcd`)

5 fields per point: `x` `y` `z` `intensity` (floats) + `instance_id`
(integer; same object identity as detections, 0 when unmatched). Points are
in the **sensor-local frame**; the file header comment carries the scan's
mount pose (body frame) and world pose (unified world). A scan may pair with
a camera/GT frame 1–2 frames away. The ROS point cloud has one extra
`range` field per point.

## 6. State and events

- **`vehicle_state`**: `gear` 0 = reverse, 1 = neutral, 2..8 = gears 1..7;
  `flags` bit 0 = race started, bit 1 = start lights on.
- **`game_state`**: `sessionType` 1..7.
- **`run_state`** (event stream): `event` = `run_begin` / `run_end` / `flag`
  / `barrier`; run_end `reason` 0 = results menu, 1 = restart, 2 = key
  change, 3 = metadata invalid, 4 = scene stale; flag `flagField` 0 = race
  over, 1 = session over, 2 = results menu.
- **`expert_ctrl`**: per-frame expert actions, frame-aligned with ground
  truth.

## 7. Dump files ↔ ROS topics

| Data | Dump | ROS topic | Rate |
|---|---|---|---|
| Ego pose | `ego_pose.jsonl` | `/ac/ego_pose` | every scene frame |
| 3D detections | `detections_3d/` | `/ac/detections_3d` | every scene frame |
| Camera images | `cameras/<cam>/<modality>/` | `/ac/camera/<cam>/image_raw` etc. | per-camera HZ |
| Camera intrinsics | `cameras.jsonl` (full, per frame) | `/ac/camera/<cam>/camera_info` (auto-updates) | — |
| Telemetry (physics) | `telemetry.jsonl` | `/ac/telemetry` | every scene frame |
| Timing / race state | `timing.jsonl` | `/ac/timing` | every scene frame |
| Static snapshot | `telemetry_static.jsonl` | `/ac/telemetry_static` (latched) | on session change |
| Run events | `run_state.jsonl` | `/ac/run_state` | event-driven |
| Expert actions | `expert_ctrl.jsonl` | `/ac/expert_ctrl` | every scene frame |
| Vehicle state | `vehicle_state.jsonl` | `/ac/vehicle_state` | 10 Hz |
| Game state | `game_state.jsonl` | `/ac/game_state` | 10 Hz |
| Lidar clouds | `lidar/<name>/*.pcd` | `/ac/lidar/<name>/points` | per scan |
| Driving-span metadata | `spans.jsonl` | — (dump only) | per driving span |
| Write report / indexes | `manifest.json`, `index.jsonl` | — (dump only) | — |

## 8. Frame id (sceneFrameId)

All per-frame data shares one globally increasing frame id (restarting from 1
within each run). Cross-channel alignment anchors on it.
