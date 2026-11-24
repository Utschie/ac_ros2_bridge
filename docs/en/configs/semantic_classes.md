# semantic_classes.ini — Semantic Rendering Switches

Global switches of semantic rendering. The per-track "node name → class"
rules live in the json files under `ac_simulator_suite\semantic_annotation\`
(fallback chain: `<track>__<layout>.json → <track>.json → default.json`),
loaded automatically when a track is entered; how to annotate is described
in [annotator.md](../annotator.md); the rule-file format is documented in
[annotation-files.md](../annotation-files.md).

| Key | Shipped | Description |
|---|---|---|
| `UNKNOWN_POLICY` | `white` | How unclassified pixels display: `white` (easy to spot gaps) or `discard`. |
| `[visual_exclusion] ENABLED` | `1` | Whether objects discarded in the annotation tool are hidden from the game view and cameras. |
| `[collision_exclusion] ENABLED` | `1` | Whether discarded objects are also removed from game collision (applies after re-entering the track). |
