"""Sanity checks for frames.py (AC<->ROS conversion) - any OS, no ROS needed.

Ground truth: AC world/BODYTR +X = physical LEFT (2026-08 verified in-game),
+Y up, +Z fwd. AC->ROS map: (x,y,z) -> (z,x,y). Producer sensor matrices are
built by the RESTORED buildSensorRot (cross(fwd,up) = physical right, sensor
+y -> physical left, matching real hardware).

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
    # The ROS-frame quat from these cols must be the identity rotation:
    # the sensor frame maps canonically onto (X fwd, Y left, Z up).
    qx, qy, qz, qw = frames.sensor_axes_to_ros_quat(*cols)
    assert abs(qx) < 1e-6 and abs(qy) < 1e-6 and abs(qz) < 1e-6 \
        and abs(qw - 1.0) < 1e-6, \
        f"{label}: ros quat got ({qx:.6f},{qy:.6f},{qz:.6f},{qw:.6f}) want identity"
    print(f"{label}: OK  t={tuple(round(v, 3) for v in t)}")


# Case 1: identity ego. Sensor mounted at body-local (1, 0, 2) facing body +Z.
# Producer rows (restored buildSensorRot): right=cross(fwd,up)=(-1,0,0);
# rows = (fwd_i, -right_i, up_i) -> (0,1,0),(0,0,1),(1,0,0);
# columns = axes: fwd=(0,0,1), left=(+1,0,0), up=(0,1,0)  (+X = physical LEFT).
E1 = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
check("identity ego", E1, (1.0, 0.0, 2.0), [0, 1, 0, 0, 0, 1, 1, 0, 0],
      (1, 0, 2), (0, 0, 1), (1, 0, 0), (0, 1, 0))

# Case 2: ego yawed so body fwd (0,0,1) points along world +X, origin at (5,6,7).
# Body axes in world: X(left)=(0,0,1), Y(up)=(0,1,0), Z(fwd)=(1,0,0).
# Sensor at body (0,0,2) facing body +Z: world pos = C @ (0,0,2) + (5,6,7) = (7,6,7);
# axes in world: fwd=(1,0,0), left=body X=(0,0,1), up=(0,1,0)
# -> S rows: r0=(1,0,0), r1=(0,0,1), r2=(0,1,0).
E2 = [0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 5, 6, 7, 1]
check("yawed ego", E2, (7.0, 6.0, 7.0), [1, 0, 0, 0, 0, 1, 0, 1, 0],
      (0, 0, 2), (0, 0, 1), (1, 0, 0), (0, 1, 0))

# Case 3: ac_body_pose_to_ros - axis-aligned AC identity must give the ROS
# identity pose (car upright at origin facing +X).
pos, quat = frames.ac_body_pose_to_ros(E1)
assert all(abs(a - b) < 1e-6 for a, b in zip(pos, (0, 0, 0)))
qx, qy, qz, qw = quat
assert abs(qx) < 1e-6 and abs(qy) < 1e-6 and abs(qz) < 1e-6 and abs(qw - 1.0) < 1e-6
# translation remap sanity under the proper world map (z,x,y)
pos2, _ = frames.ac_body_pose_to_ros(
    [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -710.13, 138.11, 2575.01, 1])
assert all(abs(a - b) < 1e-3 for a, b in zip(pos2, (2575.01, -710.13, 138.11))), pos2
print("body pose remap: OK")

# Case 4: install_pose_to_ros against the REAL (2026-08 corrected) ini values.
#   front  FWD=(0,0,1)  @ (0,1.1,0.4)     -> identity quat, pos (0.4,0,1.1)
#   rear   FWD=(0,0,-1) @ (0,1.1,-2.0)    -> 180-deg Z
#   left   FWD=(1,0,0)  @ (0.9,1.1,-0.2)  -> +90-deg Z (X axis -> ROS +Y = left)
#   right  FWD=(-1,0,0) @ (-0.9,1.1,-0.2) -> -90-deg Z
def _quat_close(q, want, tol=1e-6):
    d = sum(a * b for a, b in zip(q, want))
    return abs(abs(d) - 1.0) < tol

cases = [
    ("front", (0.0, 1.1, 0.4), (0, 0, 1), (0.4, 0.0, 1.1), (0.0, 0.0, 0.0, 1.0)),
    ("rear",  (0.0, 1.1, -2.0), (0, 0, -1), (-2.0, 0.0, 1.1), (0.0, 0.0, 1.0, 0.0)),
    ("left",  (0.9, 1.1, -0.2), (1, 0, 0), (-0.2, 0.9, 1.1),
     (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))),
    ("right", (-0.9, 1.1, -0.2), (-1, 0, 0), (-0.2, -0.9, 1.1),
     (0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4))),
]
for nm, pos_i, fwd_i, want_pos, want_quat in cases:
    got_pos, got_quat = frames.install_pose_to_ros(pos_i, fwd_i)
    assert all(abs(a - b) < 1e-6 for a, b in zip(got_pos, want_pos)), \
        f"{nm}: pos {tuple(round(v, 4) for v in got_pos)} want {want_pos}"
    assert _quat_close(got_quat, want_quat), \
        f"{nm}: quat {tuple(round(v, 6) for v in got_quat)} want {want_quat}"
    print(f"install {nm}: OK  pos={tuple(round(v, 3) for v in got_pos)}")

print("frames sanity OK (compose identity+yaw, sensor quat canonical, "
      "body pose remap, install poses)")
