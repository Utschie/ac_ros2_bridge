# Image Viewer (launcher: View Images)

A grid viewer for the live camera images rendered inside the game
(color / depth visualization / semantic). It reads the game's shared GPU
textures directly — zero-copy, no impact on the game's frame rate.

Start it with the launcher's **View Images** button. The game must be
running (the viewer waits up to 60 s for it, then tells you to start the
game first).

## The grid

- Grid size 1–4 rows × 1–4 columns, adjustable in the control panel.
- **Click a cell** to cycle through the available textures;
  **hover a cell** to get a dropdown listing them by name.
- Each cell overlays the texture name and a live FPS counter.
- `ESC` quits.

## Modalities you can show

| Name | What it is |
|---|---|
| `COLOR` | the camera's color image |
| `SEMANTIC` | semantic segmentation (each pixel = its class color, from `classes.json`) |
| `DEPTH_VIS` | depth visualization, colormap configured in `custom_cameras.ini [depth_vis] CMAP` |

The raw metric depth modality is intentionally not displayed here — it is
meant for the dump / ROS consumers (see [configs/recorder.md](configs/recorder.md)
for the recorded depth formats).

## 2D vehicle boxes (B key)

Press **B** (or tick `Show 2D bbox` in the panel) to overlay green 2D
bounding boxes of all vehicles, projected from the 3D ground truth — useful
for a quick visual sanity check of the ground truth against the actual image.

## Notes

- **Do not keep this viewer open while recording a dump**: it is a second
  consumer of the same shared textures — while connected, cameras with depth
  can drop to 1–3 Hz. Open it for inspection only when no dump is running.
- Camera topology changes (edited cameras) are picked up automatically —
  no restart needed.
- Log files live in `ac_simulator_suite\logs\`.
