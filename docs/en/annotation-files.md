# Semantic Rule Files (the json files in semantic_annotation)

The json files under `ac_simulator_suite\semantic_annotation\` are the
**per-track ground truth of "object → class / exclusion" rules**: they are
what the annotation & scene-editing tool (Annotator) saves, and they can also
be edited by hand. The game picture, the semantic camera modality and the
lidar point cloud are all driven by them.

## File naming and fallback chain

- `<track>__<layout>.json` — layout level (highest priority), e.g.
  `ks_nordschleife__endurance.json`;
- `<track>.json` — track level (shared by all layouts);
- `default.json` — generic fallback (shipped).

Loaded automatically when a track is entered; the first file hit in the chain
applies **as a whole**. Saving reloads it in about 1–2 s while the game is
running — no restart needed.

## Structure at a glance

```json
{
  "meta": { "track": "ks_barcelona", "config": "layout_gp",
            "note": "human note (not consumed by the loader; kept by the tool)" },
  "classes": {
    "TRACK":  { "exact":  ["Object061"],
                "prefix": ["LOFT"],
                "names":  ["ASPH", "LINE"],
                "regex":  ["^TRM-NRM-\\d+$"],
                "note":   "..." },
    "GRASS":  { "names": ["TERRAIN"], "prefix": ["GR_"] }
  },
  "path":  [ { "contains": "PEOPLE.FBX", "class": "PERSON" } ],
  "tree":  [ { "ancestor": "BLURRED", "class": "PERSON" } ],
  "discard": { "exact": ["OBJECT061"], "names": ["SMOKE"],
               "regex": ["RIM_BLUR_.*"] },
  "lodGroups": [ { "key": "^(.+?)_(?:HI|LOW)_(\\d+)$" } ]
}
```

## Section reference

| Section | Purpose |
|---|---|
| `meta` / per-section `note` | human notes; preserved by the tool on save |
| `classes.<Class>.exact` | full node-name equality (highest priority) — what the tool's Save writes |
| `classes.<Class>.prefix` / `names` | name prefix / name substring |
| `path` / `tree` | match by full-path substring / by ancestor node name (whole subtree) |
| `regex` | regex fallback section (`std::regex`, case-insensitive) |
| `discard` | exclusion section (class-agnostic): matched objects vanish from the game view, cameras, lidar and collision — what the tool's right-click Discard writes |
| `lodGroups` | LOD-variant linkage groups (used by the tool; ignored by the game) |

Match priority: `exact` > `prefix` > `path` > `names` > `tree` > `regex`;
no match at all → car-subtree fallback → UNKNOWN.

## Hand-editing notes

- Class names must exist in the `configs\classes.json` vocabulary (unknown
  names warn and are skipped);
- In regexes, write `\\d` for `\d` (one layer of JSON escaping, then the
  regex layer);
- A broken section warns and is skipped — it never blocks the whole file;
- Hand edits coexist with the tool: the tool only adds/removes `exact` /
  `discard` entries and never overwrites your hand-written sections or notes.

How to annotate is described in [annotator.md](annotator.md); the rendering
switches live in [configs/semantic_classes.md](configs/semantic_classes.md).
