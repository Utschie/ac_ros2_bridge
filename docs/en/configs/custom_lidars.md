# custom_lidars.ini — Custom Lidar Configuration

Defines the lidars simulated by the suite. **Parameters hot-reload (both
solid-state and rotary)**: saving the ini applies in about a second, with no
game or program restart — point-cloud changes show up live in the 3D viewer;
editing parameters, adding, removing and renaming lidars all work (after 8
distinct lidar names, restart the lidar renderer to reset the slots). Note:
editing a scan-pattern CSV alone does not trigger a reload — save the ini too.

## Coordinate frame

Mount positions and orientations use the suite-wide convention:
**X forward, Y left, Z up** (see [quickstart.md](../quickstart.md)).
Front-facing lidars use `FORWARD=1,0,0`; a rear-facing lidar uses `-1,0,0`.
Scan-pattern CSVs are unaffected (azimuth 0° = sensor forward).

Reference vehicle: Porsche 911 GT3 RS (992) — 4572 × 1900 × 1322 mm
(half width 0.95 m, roof z ≈ 1.32 m, rear bumper x ≈ -2.8 m).

## [LIDAR_<name>] — one section per lidar

| Key | Default | Description |
|---|---|---|
| `TYPE` | `solid` | `solid` (pattern from `SCAN_FILE`) or `rotating` (360° rotary, pattern generated from parameters). |
| `POSITION` | `0.4,0,1.1` | Mount position `x,y,z` (meters, X fwd / Y left / Z up). |
| `FORWARD` | `1,0,0` | Aim direction; rear-facing uses `-1,0,0`. |
| `SCAN_FILE` | *(required for solid)* | Scan-pattern CSV (columns: #, azimuth, elevation), resolved next to this ini; shipped patterns in `scan_mode\`. Ignored for `rotating`. |
| `SIM_HZ` | `10` | Scans per second (for `rotating`, the spin rate). |
| `NOISE` | `0` | Range-noise standard deviation in meters, mimicking a real sensor's range error; 0 = clean data. |
| `RANGE_MIN` / `RANGE_MAX` | `0.5` / `200` | Range limits (meters). |
| `ENABLED` | `1` | 0 = disable this lidar. |

## TYPE=rotating keys

The 360° pattern is generated from parameters — no recording file needed.
Points per frame = `CHANNELS × COLUMNS_PER_REV`; oversized configurations are
automatically reduced in columns (full coverage kept).

| Key | Default | Description |
|---|---|---|
| `CHANNELS` | `16` | Number of vertical lines (1–128). |
| `VERTICAL_ANGLES` | *(empty)* | Explicit per-line elevation table (degrees, comma-separated) matching a real sensor's non-uniform layout; count must equal `CHANNELS`. |
| `VFOV_MIN` / `VFOV_MAX` | `-15` / `15` | Without the table, lines are spread uniformly over [MIN, MAX]. |
| `COLUMNS_PER_REV` | `1800` | Columns (firings) per revolution — 1800 = 0.2° azimuth resolution. |
| `ROT_DIR` | `cw` | Spin direction viewed from above: `cw` = clockwise / `ccw` = counter-clockwise. |
| `START_AZIMUTH` | `0` | Azimuth of the first column (degrees). |

## Shipped configuration

- `[LIDAR_front]` — solid-state, front roof edge, 10 Hz, 0.5–200 m (enabled)
- `[LIDAR_spin]` — 360° rotary, 32 lines × 0.2°, 10 Hz (enabled)
- `[LIDAR_rear]` — rear solid-state, 20 Hz (not enabled)
- `[LIDAR_spin2]` — 360° rotary with a non-uniform 8-line angle table (not enabled)

Sections that are not enabled are ready-made templates: set `ENABLED=1` and tune the
parameters above to match your sensor; add `[LIDAR_*]` sections for more units.
