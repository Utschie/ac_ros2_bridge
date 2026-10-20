"""Coordinate frames - THE single place for axis/frame math (M20, 2026-10-19).

M20 doctrine (supersedes fix-10; see roadmap/m20-unified-ros-frames.md and
shared/ac_coords.h):

1. All USER-FACING surfaces speak REP-103 (x fwd, y left, z up): config ini,
   dumps, zenoh payloads, ROS topics, viewer axis labels.
2. The AC body frame (+X left, +Y up, +Z fwd) is an INTERNAL implementation
   detail of the game side / SHM. ac_sim_bridge converts at the source
   (ac_coords.h boundary #2) BEFORE publishing, so payloads arrive already
   in user-facing convention:
     - EgoPosePayload.world rows = fwd / left / up (was left / up / fwd)
     - Detection3D: half_extents = (fwd, left, up), world_rot rows = fwd/left/up
     - LidarPayload: install_pos / install_fwd = vehicle frame x fwd, y left, z up
3. The WORLD is ALSO already converted at the bridge (ac_coords.h v2):
   the AC SCENE/render world is Y-up (x/z horizontal, y = altitude -
   settled 2026-10-20: parked-car up axis reads exactly +Y, and driving
   data keeps y constant while x/z move; the acpmf physics-block comment
   "X/Y plane, Z height" describes the PHYSICS world, a different frame -
   do not conflate). Payload world triples are already standard Z-up
   (x/y horizontal, z = altitude) via ac.z->x, ac.x->y, ac.y(up)->z.
   (fix-10's world map was right; M20-v1 briefly shipped a pass-through
   Y-up world and was corrected the same day - v2.)
4. Sensor-local point clouds were always ROS-style (x fwd, y left, z up) and
   still pass through untouched.

So this module is now mostly direct assembly: quats come from
quat_from_cols(fwd, left, up) on payload rows as-is, positions pass through
unchanged. If anything looks wrong in RViz (mirrored / tilted / swapped),
fix it HERE - all consumers (ego pose, OBBs, lidar TF) go through these
functions. Covered by test_compose.py (run on any OS, no ROS needed).

Validation discipline (unchanged): canonical RViz view - camera ABOVE the
ground plane, axes reading red=fwd / green=left / blue=up. A mirrored camera
once made a reflected map look correct through four validation rounds
(fix-6..9 war story); never trust a view that is not right-side-up.
"""

import math


def quat_from_cols(c0, c1, c2):
    """Quaternion (x,y,z,w) of a right-handed rotation whose columns are c0,c1,c2."""
    r00, r10, r20 = c0
    r01, r11, r21 = c1
    r02, r12, r22 = c2
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r21 - r12) / s
        qy = (r02 - r20) / s
        qz = (r10 - r01) / s
    elif r00 > r11 and r00 > r22:
        s = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / s
        qx = 0.25 * s
        qy = (r01 + r10) / s
        qz = (r02 + r20) / s
    elif r11 > r22:
        s = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / s
        qx = (r01 + r10) / s
        qy = 0.25 * s
        qz = (r12 + r21) / s
    else:
        s = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / s
        qx = (r02 + r20) / s
        qy = (r12 + r21) / s
        qz = 0.25 * s
    return qx, qy, qz, qw


def ego_pose_to_ros(world16):
    """EgoPosePayload.world (bridge-converted: rows = fwd/left/up, world
    triples already Z-up user world) -> (ros_position, ros_quat). Direct
    assembly; the (z,x,y) world permutation happened at the bridge
    (ac_coords.h v2) - do NOT permute here again."""
    pos_r = tuple(world16[12:15])
    quat = quat_from_cols(world16[0:3], world16[4:7], world16[8:11])
    return pos_r, quat


def pos_rot_to_ros(pos3, rot9):
    """Detection3D world_pos/world_rot (bridge-converted, Z-up user world)
    -> (ros_position, ros_quat). Same direct assembly as ego_pose_to_ros."""
    pos_r = tuple(pos3)
    quat = quat_from_cols(rot9[0:3], rot9[3:6], rot9[6:9])
    return pos_r, quat


def install_pose_to_ros(install_pos, install_fwd):
    """LidarPayload install pose (vehicle frame x fwd, y left, z up - the ini
    POSITION/FORWARD carried through unchanged since M20) ->
    (ros_position, ros_quat) for a base_link -> <sensor>_frame TF. Sensor up
    is the vehicle up axis.

    Deterministic and eyeball-verifiable against the ini: front FORWARD=(1,0,0)
    -> identity quat, rear (-1,0,0) -> 180-deg Z, left (0,1,0) -> +90 deg,
    right (0,-1,0) -> -90 deg."""
    fx, fy, fz = install_fwd
    n = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    fwd = (fx / n, fy / n, fz / n)
    up = (0.0, 0.0, 1.0)
    # cross(up, fwd) in the x-fwd/y-left/z-up frame IS the sensor's left axis
    left = (
        up[1] * fwd[2] - up[2] * fwd[1],
        up[2] * fwd[0] - up[0] * fwd[2],
        up[0] * fwd[1] - up[1] * fwd[0],
    )
    quat = quat_from_cols(fwd, left, up)
    return tuple(install_pos), quat


def sensor_axes_to_ros_quat(fwd, y_axis, up):
    """Sensor axes as world-frame triples (from compose_base_to_sensor; sensor
    convention x fwd, y left, z up) -> ROS quat for a base_link-><sensor>_frame
    TF. World frame passes through unchanged, so this is plain assembly."""
    return quat_from_cols(fwd, y_axis, up)


def compose_base_to_sensor(ego_world, world_pos, world_rot9):
    """T(base -> sensor) = inv(E) . S. Inputs in PAYLOAD convention (M20):
    ego_world rows = fwd/left/up (user-facing body axes in world), world_rot9
    rows = sensor x/y/z axes in world. Returns (t, cols): sensor origin in the
    base frame and the sensor's local x/y/z axis directions in the base frame.
    Numerically the same matrix algebra as before - the relabeling happens
    upstream at the bridge (ac_coords.h boundary #2)."""
    ep = ego_world[12:15]
    e_rows = (ego_world[0:3], ego_world[4:7], ego_world[8:11])
    s_rows = tuple(world_rot9[i:i + 3] for i in (0, 3, 6))
    # cols[j] = sensor axis j (S row j, a WORLD vector) expressed in the base
    # frame: project onto the base axes (E rows). NOTE (M20): the pre-M20 code
    # built cols as columns of E.S (= E . S-column-j), which coincided with
    # the correct answer only because all historical test matrices were
    # symmetric; the rewritten non-symmetric anchor case exposed it.
    cols = tuple(
        tuple(sum(e_rows[i][k] * s_rows[j][k] for k in range(3))
              for i in range(3))
        for j in range(3))
    # t = sensor origin offset expressed in the base frame (project d onto E rows)
    d = tuple(world_pos[i] - ep[i] for i in range(3))
    t = tuple(sum(e_rows[i][k] * d[k] for k in range(3)) for i in range(3))
    return t, cols


# ---- Back-compat aliases (old fix-10-era names; payloads changed meaning in
# M20, so these now operate on the new user-facing payload layout). Kept only
# if some consumer still imports the old names; prefer the names above. ----
def ac_body_pose_to_ros(world16):
    return ego_pose_to_ros(world16)


def ac_pos_rot_to_ros(pos3, rot_cols9):
    return pos_rot_to_ros(pos3, rot_cols9)
