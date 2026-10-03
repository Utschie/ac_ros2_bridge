"""SemanticClassRelay - M3 semantic class vocabulary domain module.

zenoh ac/semantic/classes (UTF-8 JSON text, semcls::tableToJson on the
Windows side - NOT a binary payload like every other key) ->
  1. ctx.semantic_classes   in-memory id -> (name, color, dynamic, exclude)
                            vocabulary; swapped whole-object on update so
                            readers (detections_3d class names, marker
                            colors) always see a consistent table;
  2. /ac/semantic/classes   latched ROS topic (std_msgs/String carries the
                            raw JSON; TRANSIENT_LOCAL depth 1 - late-joining
                            consumers get the newest table immediately).

Why: since M3 the class table is user-editable (configs/classes.json -
custom classes, array order = class id) and detections carry the REAL
semanticClass ids. Without a vocabulary on this side, /ac/detections_3d
could only publish bare numeric ids and the RGBA8 semantic image's colors
were unexplainable - custom classes were invisible to ROS consumers.

Fallback rules mirror the Windows five consumers (shared/
semantic_class_config.h): before the first valid zenoh sample the built-in
11-class table bootstraps; any rejected sample (bad JSON, first entry not
UNKNOWN, duplicate names, > 64 classes) keeps the LAST GOOD table serving
with a WARN - no flicker, same graceful-degradation philosophy.

The pure-logic half (parse_class_table / SemanticClasses / builtin_classes)
deliberately imports nothing ROS-side so test_class_table.py runs on any
OS with plain Python (same trick as test_decode.py).
"""

import json
from collections import namedtuple

MAX_CLASSES = 64   # semcls::kMaxSemanticClasses

ClassEntry = namedtuple("ClassEntry", ["name", "color", "dynamic", "exclude"])


def _parse_color(c):
    """'"#RRGGBB"' or [r,g,b] (0..1) -> (r,g,b) floats; None on bad input."""
    if isinstance(c, str):
        if len(c) == 7 and c[0] == "#":
            try:
                return tuple(int(c[i:i + 2], 16) / 255.0 for i in (1, 3, 5))
            except ValueError:
                return None
        return None
    if isinstance(c, (list, tuple)) and len(c) == 3:
        try:
            vals = [float(v) for v in c]
        except (TypeError, ValueError):
            return None
        return tuple(min(max(v, 0.0), 1.0) for v in vals)
    return None


class SemanticClasses:
    """Immutable vocabulary snapshot: entries[class_id] = ClassEntry.

    Out-of-range ids fall back to UNKNOWN semantics (name "UNKNOWN", white)
    - the Python counterpart of the C++ clamp-to-entry-0 read invariant:
    hot table swaps may briefly leave stale ids in flight; they must resolve
    safely, never crash.
    """

    def __init__(self, entries):
        self.entries = list(entries)

    def __len__(self):
        return len(self.entries)

    def name_of(self, class_id):
        if 0 <= class_id < len(self.entries):
            return self.entries[class_id].name
        return "UNKNOWN"

    def color_of(self, class_id):
        if 0 <= class_id < len(self.entries):
            return self.entries[class_id].color
        return (1.0, 1.0, 1.0)


def parse_class_table(text):
    """Validate + parse the canonical JSON from ac/semantic/classes.

    Returns a SemanticClasses. Raises ValueError with a human reason on any
    rule violation (caller keeps the last good table) - the checks mirror
    semcls::parseClassesText: non-empty, first entry "UNKNOWN", unique
    names, <= 64 entries, color "#RRGGBB"/[r,g,b].
    """
    obj = json.loads(text)   # json.JSONDecodeError subclasses ValueError
    if not isinstance(obj, dict):
        raise ValueError("root must be an object")
    classes = obj.get("classes")
    if not isinstance(classes, list) or not classes:
        raise ValueError("missing or empty 'classes' array")
    if len(classes) > MAX_CLASSES:
        raise ValueError(f"too many classes ({len(classes)}, max {MAX_CLASSES})")
    entries = []
    seen = set()
    for i, c in enumerate(classes):
        if not isinstance(c, dict):
            raise ValueError(f"class #{i} is not an object")
        name = c.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"class #{i} missing 'name'")
        if i == 0 and name != "UNKNOWN":
            raise ValueError("first class must be UNKNOWN (id 0 is load-bearing)")
        if name in seen:
            raise ValueError(f"duplicate class name '{name}'")
        seen.add(name)
        color = _parse_color(c.get("color"))
        if color is None:
            raise ValueError(f"class '{name}': bad color (want \"#RRGGBB\" or [r,g,b])")
        entries.append(ClassEntry(name, color,
                                  bool(c.get("dynamic", False)),
                                  bool(c.get("exclude", False))))
    return SemanticClasses(entries)


def builtin_classes():
    """Bootstrap table before the first zenoh sample - same values as
    semcls::builtinTable() (the shipped classes.json minus custom entries;
    its leading built-ins share ids with this table)."""
    table = [
        ("UNKNOWN",  (1.0, 1.0, 1.0), False, False),
        ("CAR",      (1.0, 0.0, 0.0), True,  False),
        ("PERSON",   (1.0, 1.0, 0.0), True,  False),
        ("TREE",     (0.0, 0.5, 0.0), False, False),
        ("GRASS",    (0.5, 1.0, 0.0), False, False),
        ("TRACK",    (0.2, 0.2, 0.2), False, False),
        ("SKY",      (0.5, 0.7, 1.0), False, False),
        ("OBSTACLE", (0.6, 0.0, 0.8), False, False),
        ("LOGIC",    (0.0, 0.0, 0.0), False, True),
        ("PHYS_OBJ", (0.5, 0.3, 0.0), False, False),
        ("KERB",     (1.0, 0.5, 0.0), False, False),
    ]
    return SemanticClasses([ClassEntry(*row) for row in table])


class SemanticClassRelay:
    """Domain module (ctx + attach(session), wired in relay.py).

    The 2s keepalive re-puts from the bridge re-deliver the same JSON; the
    ROS topic is only re-published when the table actually CHANGED (latched
    QoS already covers late joiners between changes)."""

    def __init__(self, ctx):
        self.ctx = ctx
        # Deferred imports keep the pure-logic half importable without ROS
        # (offline test_class_table.py on any OS).
        from std_msgs.msg import String
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile
        self._string_cls = String
        qos = QoSProfile(depth=1,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = ctx.node.create_publisher(String, "/ac/semantic/classes", qos)
        self._last_raw = None

    def attach(self, session) -> None:
        session.declare_subscriber("ac/semantic/classes", self._on_classes)

    def _on_classes(self, sample) -> None:
        # payload extraction inlined (wire.payload_bytes twin) so this module
        # stays import-free of the wire/ROS stack in its pure half
        p = sample.payload
        raw = (p.to_bytes() if hasattr(p, "to_bytes") else bytes(p)).decode("utf-8")
        try:
            table = parse_class_table(raw)
        except ValueError as ex:
            self.ctx.log.warn(f"semantic classes rejected, keeping last good "
                              f"table: {ex}")
            return
        self.ctx.semantic_classes = table
        if raw != self._last_raw:
            self._last_raw = raw
            msg = self._string_cls()
            msg.data = raw
            with self.ctx.pub_lock:
                self.pub.publish(msg)
            self.ctx.log.info(f"semantic classes: {len(table)} classes active "
                              f"(re-published /ac/semantic/classes)")
