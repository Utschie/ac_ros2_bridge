"""Standalone checks for class_table.py - run on any OS with Python (no ROS2
needed; mirrors test_decode.py's trick of importing the module directly).

Exercises the pure half of class_table.py against the rules of
shared/semantic_class_config.h (semcls::parseClassesText):
  - canonical JSON (what semcls::tableToJson emits on the Windows side)
    parses into the right ids/names/colors/flags
  - both color forms ("#RRGGBB" and [r,g,b]) parse
  - out-of-range ids fall back to UNKNOWN semantics (hot-swap safety)
  - every violation (bad JSON, wrong first entry, duplicate names, >64
    classes, bad color, missing name) raises ValueError -> the relay keeps
    the last good table
  - builtin bootstrap table matches semcls::builtinTable() values
Run:  python ac_ros2_bridge/ac_ros2_bridge/test_class_table.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import class_table as ct  # noqa: E402


def _expect_reject(text, why):
    try:
        ct.parse_class_table(text)
    except ValueError:
        return
    raise AssertionError(f"table should have been rejected ({why})")


def main() -> None:
    # ---- canonical JSON, exactly the shape semcls::tableToJson emits ----
    canonical = """{
  "classes": [
    { "name": "UNKNOWN", "color": "#FFFFFF" },
    { "name": "CAR", "color": "#FF0000", "dynamic": true },
    { "name": "PERSON", "color": "#FFFF00", "dynamic": true },
    { "name": "CONE", "color": "#FF8000" },
    { "name": "LOGIC", "color": "#000000", "exclude": true }
  ]
}
"""
    t = ct.parse_class_table(canonical)
    assert len(t) == 5
    assert t.name_of(0) == "UNKNOWN"
    assert t.name_of(1) == "CAR"
    assert t.name_of(4) == "LOGIC"
    assert t.entries[1].dynamic and not t.entries[0].dynamic
    assert t.entries[4].exclude
    r, g, b = t.color_of(1)   # #FF0000
    assert abs(r - 1.0) < 1e-6 and abs(g) < 1e-6 and abs(b) < 1e-6
    r, g, b = t.color_of(3)   # #FF8000
    assert abs(r - 1.0) < 1e-6 and abs(g - 0x80 / 255.0) < 1e-6

    # ---- array color form + clamping ----
    t2 = ct.parse_class_table(
        '{"classes": [{"name": "UNKNOWN", "color": [0.1, 0.2, 0.3]},'
        ' {"name": "X", "color": [2.0, -1.0, 0.5]}]}')
    assert abs(t2.color_of(0)[0] - 0.1) < 1e-6
    r, g, b = t2.color_of(1)   # clamped to [0,1]
    assert r == 1.0 and g == 0.0 and abs(b - 0.5) < 1e-6

    # ---- out-of-range ids -> UNKNOWN semantics (hot-swap stale-id safety) ----
    assert t.name_of(99) == "UNKNOWN"
    assert t.name_of(-1) == "UNKNOWN"
    assert t.color_of(99) == (1.0, 1.0, 1.0)

    # ---- rejection cases: each keeps the last good table on the relay ----
    _expect_reject("{not json", "malformed JSON")
    _expect_reject("[1,2,3]", "root not an object")
    _expect_reject('{"foo": 1}', "missing classes array")
    _expect_reject('{"classes": []}', "empty classes array")
    _expect_reject('{"classes": [{"name": "CAR"}]}', "first entry not UNKNOWN")
    _expect_reject(
        '{"classes": [{"name": "UNKNOWN"}, {"name": "UNKNOWN"}]}',
        "duplicate names")
    _expect_reject(
        '{"classes": [{"name": "UNKNOWN", "color": "#12345"}]}',
        "bad hex color length")
    _expect_reject(
        '{"classes": [{"name": "UNKNOWN", "color": "red"}]}',
        "bad color string")
    _expect_reject(
        '{"classes": [{"name": "UNKNOWN"}, {"name": "X", "color": [1, 2]}]}',
        "bad array color arity")
    _expect_reject('{"classes": [{"color": "#FFFFFF"}]}', "missing name")
    many = [{"name": "UNKNOWN"}] + [
        {"name": f"C{i}"} for i in range(ct.MAX_CLASSES)]
    _expect_reject(
        '{"classes": [' + ",".join(
            f'{{"name": "{c["name"]}"}}' for c in many) + "]}",
        "over 64 classes")

    # two parses must not share state (whole-object swap keeps tables independent)
    t3 = ct.parse_class_table(canonical)
    assert t3 is not t and t3.name_of(1) == "CAR"

    # ---- builtin bootstrap table == semcls::builtinTable() values ----
    bt = ct.builtin_classes()
    assert len(bt) == 11
    assert bt.name_of(0) == "UNKNOWN"
    assert bt.name_of(1) == "CAR"
    assert bt.name_of(10) == "KERB"
    assert bt.entries[1].dynamic          # CAR
    assert bt.entries[2].dynamic          # PERSON
    assert bt.entries[8].exclude          # LOGIC
    assert bt.color_of(1) == (1.0, 0.0, 0.0)          # CAR red
    assert abs(bt.color_of(3)[1] - 0.5) < 1e-6        # TREE (0, 0.5, 0)
    assert bt.color_of(5) == (0.2, 0.2, 0.2)          # TRACK

    print("test_class_table: all checks passed")


if __name__ == "__main__":
    main()
