# 3D Viewer (launcher: 3D View)

A live 3D window showing the lidar point clouds and the 3D ground-truth
vehicle boxes. Requires the optional **3D viewer** component (installed with
the suite) and, for points to appear, the game inside a session
(**Start Lidar** can be clicked any time — points appear automatically
once you are on track).

## What you see

- **Point clouds** from all enabled lidars, each lidar toggleable; colored
  either by height (turbo colormap) or by intensity (grayscale) — switch in
  the panel; point-size slider.
- **Vehicle ground-truth boxes** as wireframe OBBs with real orientation:
  your car orange, other cars cyan-blue.
- **Ego axes** — three arrows on your car: **red = forward, green = left,
  blue = up** (the standard robot convention, same colors as RViz).
- Light theme by default (`light bg` checkbox switches to dark); a ground
  grid (1/2/5/10 m density) that follows the tracked car's road height
  (`grid follows target` checkbox), and world coordinate axes.

## Camera controls

| Input | Action |
|---|---|
| Left mouse drag | orbit |
| Wheel | zoom |
| Middle mouse drag | pan |
| **T** | top view |
| **G** | toggle ground grid |
| Follow list (panel) | pick an entity to follow — defaults to Ego; any car can be tracked; falls back to Ego if the tracked car disappears |
| Heading follow (checkbox) | camera also follows the car's heading |

The first frame of data auto-frames the camera — you never start lost.
While the mouse is over an ImGui panel, input goes to the panel, not the
camera.

## HUD

The panel shows the GPU name, FPS, scene frame counter, per-lidar point
counts, a waiting hint when there is no data yet, and all display toggles
(`vsync` default on — untick for lower latency). Close the window with the
X button (no ESC shortcut).

## GPU selection

Use the launcher's **3D viewer GPU dropdown** (`launcher.ini [viewer3d] ADAPTER`). Unset = auto: among NVIDIA cards, prefer the one *not* driving
the display, so the viewer runs on the idle card. Layout settings are
remembered in `configs\imgui_3dviewer.ini`.

Log files live in `ac_simulator_suite\logs\`.
