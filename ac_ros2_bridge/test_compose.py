"""Sanity checks for frames.py (M20 conventions) - any OS, no ROS needed.

Ground truth (M20 v2, 2026-10-20; see roadmap/m20-unified-ros-frames.md):
payloads arrive with body axes already user-facing (bridge converts at the
source):
  - EgoPosePayload.world rows = fwd / left / up; position = AC-scene world
  - Detection3D half = (fwd, left, up), rot rows = fwd / left / up
  - LidarPayload install = vehicle frame x fwd / y left / z up (ini values)
The WORLD is also bridge-converted: the AC scene world is Y-up; payload
world triples arrive already in the Z-up user world (ac_coords.h v2).

Run: python ac_ros2_bridge/ac_ros2_bridge/test_compose.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import frames  # noqa: E402


def check(label, ego, s_pos, s_rot9, want_t, want_fwd, want_left, want_up):
    t, cols = frames.compose_base_to_sensor(ego, s_pos, s_rot9)
    for got, want, what in ((t, want_t, "t"),
                            (cols[0], want_fwd, "fwd"),
                            (cols[1], want_left, "left"),
                            (cols[2], want_up, "up")):
        assert all(abs(a - b) < 1e-6 for a, b in zip(got, want)), \
            f"{label}: {what} got {tuple(round(v, 6) for v in got)} want {want}"
    qx, qy, qz, qw = frames.sensor_axes_to_ros_quat(*cols)
    assert abs(qx) < 1e-6 and abs(qy) < 1e-6 and abs(qz) < 1e-6 \
        and abs(qw - 1.0) < 1e-6, \
        f"{label}: ros quat got ({qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}) want identity"
    print(f"{label}: OK  t={tuple(round(v, 3) for v in t)}")


# Case 1: identity ego (car upright at origin facing world +X; payload rows
# fwd/left/up = I). Sensor at base-local (1, 0, 2), facing base +x -> S = I.
E1 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
check("identity ego", E1, (1.0, 0.0, 2.0), [1, 0, 0, 0, 1, 0, 0, 0, 1],
      (1, 0, 2), (1, 0, 0), (0, 1, 0), (0, 0, 1))

# Case 2: ego yawed +90 deg (car fwd = world +Y, left = world -X), origin
# (5,6,7). Payload rows: fwd=(0,1,0), left=(-1,0,0), up=(0,0,1).
# Sensor at base (2,0,0) facing car fwd: world pos = (5,8,7); sensor axes in
# world: x=(0,1,0), y=(-1,0,0), z=(0,0,1) -> S rows likewise.
E2 = [0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 5, 6, 7, 1]
check("yawed ego", E2, (5.0, 8.0, 7.0), [0, 1, 0, -1, 0, 0, 0, 0, 1],
      (2, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1))

# Case 3: ego_pose_to_ros - payload identity -> identity pose; positions
# pass through (world triples are ALREADY in the Z-up user world - the
# (z,x,y) conversion happened at the bridge, not here).
pos, quat = frames.ego_pose_to_ros(E1)
assert all(abs(a - b) < 1e-6 for a, b in zip(pos, (0, 0, 0)))
qx, qy, qz, qw = quat
assert abs(qx) < 1e-6 and abs(qy) < 1e-6 and abs(qz) < 1e-6 and abs(qw - 1.0) < 1e-6
pos2, _ = frames.ego_pose_to_ros(
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 2575.01, -710.13, 138.11, 1])
assert all(abs(a - b) < 1e-3 for a, b in zip(pos2, (2575.01, -710.13, 138.11))), pos2
# Yawed ego (case-2 rows) -> 90-deg Z quat
_, q2 = frames.ego_pose_to_ros(E2)
assert abs(q2[2] - math.sin(math.pi / 4)) < 1e-6 \
    and abs(q2[3] - math.cos(math.pi / 4)) < 1e-6, q2
print("ego pose assembly: OK (identity + pass-through pos + 90deg yaw)")

# Case 4: install_pose_to_ros against the M20 ini values (payload = ini
# values verbatim; front is the acceptance-H anchor).
#   front  FWD=(1,0,0)  @ (0.4, 0, 1.1)    -> identity quat  [ACCEPTANCE H]
#   rear   FWD=(-1,0,0) @ (-2.0, 0, 1.1)   -> 180-deg Z
#   left   FWD=(0,1,0)  @ (-0.2, 0.9, 1.1) -> +90-deg Z
#   right  FWD=(0,-1,0) @ (-0.2,-0.9, 1.1) -> -90-deg Z
def _quat_close(q, want, tol=1e-6):
    d = sum(a * b for a, b in zip(q, want))
    return abs(abs(d) - 1.0) < tol

cases = [
    ("front", (0.4, 0.0, 1.1), (1, 0, 0), (0.4, 0.0, 1.1), (0.0, 0.0, 0.0, 1.0)),
    ("rear",  (-2.0, 0.0, 1.1), (-1, 0, 0), (-2.0, 0.0, 1.1), (0.0, 0.0, 1.0, 0.0)),
    ("left",  (-0.2, 0.9, 1.1), (0, 1, 0), (-0.2, 0.9, 1.1),
     (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))),
    ("right", (-0.2, -0.9, 1.1), (0, -1, 0), (-0.2, -0.9, 1.1),
     (0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4))),
]
for nm, pos_i, fwd_i, want_pos, want_quat in cases:
    got_pos, got_quat = frames.install_pose_to_ros(pos_i, fwd_i)
    assert all(abs(a - b) < 1e-6 for a, b in zip(got_pos, want_pos)), \
        f"{nm}: pos {tuple(round(v, 4) for v in got_pos)} want {want_pos}"
    assert _quat_close(got_quat, want_quat), \
        f"{nm}: quat {tuple(round(v, 6) for v in got_quat)} want {want_quat}"
    print(f"install {nm}: OK  pos={tuple(round(v, 3) for v in got_pos)}")

print("frames sanity OK (M20: compose identity+yaw, ego pass-through, "
      "install anchors incl. front=identity)")
