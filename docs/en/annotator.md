# Semantic Annotation & Scene Editing (launcher: Annotator)

This tool is for **semantic annotation and scene editing**: it renders the
whole track colored **by semantic class** — everything shown **white is still
UNKNOWN** and needs a class assignment. Assignments are saved into per-track
rule files that the game, the semantic camera modality and the lidar all
consume within seconds (hot reload). This is how you teach the suite what
each object on your track is — and remove what should not be there.

Requires the optional **Semantic annotator** component (NVIDIA GPU) and the
game running inside a track. Start it with the launcher's **Annotator**
button. The title bar always shows `FPS | track / layout | nodes=N | UNKNOWN=M`.

## The typical workflow: pause → annotate → continue

1. Drive into the track in-game, start the bridge, click **Annotator**.
2. Pause the game with the launcher's **Pause** button (physics and audio
   freeze; rendering and data keep flowing, and WASD no longer drives the
   car). Resume with the launcher's Resume button.
3. Fly around, jump between UNKNOWN objects, assign classes (preview
   applies instantly).
4. **Save to json…** writes the assignments — the semantic map, this tool's
   view and the tree switch the affected objects to the new ground-truth
   colors within ~1–2 s.

## Controls

| Input | Action |
|---|---|
| Left / right / middle mouse | pan / rotate / move forward-back; wheel = zoom; `+` / `-` = fly speed |
| WASD / QE (E up / Q down) | fly (only while this window has focus; Shift = 4× speed) |
| C | chase-camera toggle (nice default view; any manual move breaks out of it) |
| U / Shift+U | jump to next / previous UNKNOWN object |
| G | triangle-mesh wireframe toggle |
| Tab | show / hide the scene-tree panel |
| Click object | select (magenta halo); click sky = clear |
| Hover object | cyan tint + bottom-left `class name #index` readout |
| H / Shift+H | hide selected / show all (hide occluders to reach objects behind them) |
| Tree row click / double-click | select / select + fly to it |
| ESC | quit |

## The panel (Tab)

- **Tree**: every scene node with its class color dot; `[group]` /
  `[ghost]` tags mark containers and non-rendered LOD shells.
- **Filter**: type a name substring and/or pick a class — matching rows
  only; ancestors expand.
- **UNKNOWN counter**: `UNKNOWN: N nodes | view X.X%` — your remaining
  workload.
- **Selected card**: class dropdown + **Apply** (assign) / Jump / Clear +
  the object's world size.
- **Pending list**: assignments not yet saved; per-item undo, `x all`,
  Re-apply all.

**Apply** colors the node **and everything named like it** immediately
(preview). **Save to json…** turns all pending assignments into permanent
rules:

- Default filename is pre-filled correctly (`<track>__<layout>.json` or
  `<track>.json` — only these two names are loaded); a preview line shows
  which file will be loaded after the save.
- The status line walks through `saved → waiting reload → hot-reload
  confirmed`.
- Saving on a brand-new track seeds the new file with all current default
  rules — nothing suddenly turns UNKNOWN.
- Saved files live in `ac_simulator_suite\semantic_annotation\` (the
  `Open folder` button goes there).

## Removing objects (Discard)

- To remove an object, normally just use right-click **Discard**: the object
  disappears from the game view, cameras, lidar and collision (collision
  changes apply after re-entering the track).
- **LOGIC** is the lightweight option, rarely needed: the object stays in the
  game picture but is excluded from the semantic camera data and the lidar
  point cloud (collision is not affected).

Right-click a tree row or the selected card: `Discard/Restore (single)` or
`(subtree + LOD family)` — the family variant automatically includes the
object's LOD siblings (HI/LOW detail variants), which is usually what you
want. Ctrl+click / Shift+click / Ctrl+A multi-select, then right-click the
selection. The filter dropdown "Discarded" lists everything currently
discarded — the way back when something was removed by mistake.

## Class palette (classes.json)

Class names, colors, dynamic/exclude flags live in
`ac_simulator_suite\configs\classes.json` — editable with the launcher's
**Edit classes.json** button (upgrades never overwrite your edits). Changes
hot-reload in ~1–2 s. Rendering switches live in `semantic_classes.ini` —
see [configs/semantic_classes.md](configs/semantic_classes.md); the rule
files' format and fallback chain are documented in
[annotation-files.md](annotation-files.md).

## Follow the game camera (advanced)

In-game, press **F7** (the game's free camera — arrow keys to move, F1 to
leave), then tick **Follow game camera** in the annotator panel: flying in
the tool mirrors live into the game view and vice versa. With **Game
highlight** ticked, your selected object gets the same magenta halo in the
*game picture* — a quick way to check whether the game itself renders an
object at all.

## Notes

- **Grandstand crowds are particle systems** — they appear in the game
  picture and the semantic camera, but they have no scene nodes, so they
  cannot be shown or annotated in this tool; they render normally in the
  game picture and the semantic camera — no action needed.
- Objects the game does not draw (e.g., invisible walls) still exist here
  and in the lidar — remove them with right-click **Discard** to keep them
  out of your sensor data.
