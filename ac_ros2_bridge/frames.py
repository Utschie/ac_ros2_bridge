"""AC <-> ROS coordinate conversion - THE single place for axis/frame math.

FINAL (fix-10, 2026-08; verified in a CANONICAL RViz view - axes red=fwd,
green=left, blue=up; C++ counterpart: shared/ac_coords.h):

1. AC->ROS is ONE proper map for BOTH world and body data:
   (x,y,z) -> (z, x, y)   [det=+1]
   (AC Z fwd->ROS X, AC X left->ROS Y, AC Y up->ROS Z.)
2. BODYTR/body frame: +X = physical LEFT (+Y up, +Z fwd); ini
   POSITION/FORWARD follow this (side sensors: left has x > 0).
3. SHM world-matrix contiguous triples m[0:3]/[4:7]/[8:11] ARE the frame's
   X/Y/Z axis images (settled via raw egoMatrix log, fix-8).
   Orientation quats: quat(B(fwd), B(x_axis), B(up)) with fwd=m[8:11],
   x_axis=m[0:3] - NO negation anywhere.
4. Lidar hits need NO chirality flip: optix's buildSensorRot convention
   matches the install-pose frames axis-for-axis (verified for
   front/rear/left). Do not negate hit y.

ROOT CAUSE NOTE (why fix-6..9 looked right): the world-position map
(z,-x,y) used between fix-6 and fix-10 is a REFLECTION (det=-1). Paired
with the proper ego quat it mirrored the car-vs-world arrangement; all
validations of that era were made through an accidentally mirrored RViz
camera (viewing from below the ground plane), and mirror x mirror looked
correct. When the camera was righted, dets appeared left-right swapped
and lidar clouds mirrored. Always validate with the camera ABOVE the
plane, axes reading red=fwd/green=left/blue=up.

If anything looks wrong in RViz (mirrored / tilted / swapped), fix it HERE -
all consumers (ego pose, OBBs, lidar TF, future camera TF) go through these
functions. Covered by test_compose.py (run on any OS, no ROS needed).
"""

import math


def ac_vec_to_ros(v):
    """WORLD-frame vector -> ROS. Same proper map as the body frame:
    AC Z(fwd)->ROS X, AC X(left)->ROS Y, AC Y(up)->ROS Z (det=+1).
    The earlier (z,-x,y) variant is a REFLECTION (det=-1): paired with the
    proper ego quat it silently mirrored the car-vs-world arrangement."""
    x, y, z = v
    return (z, x, y)


def body_vec_to_ros(v):
    """BODYTR-local vector (ini POSITION/FORWARD, +X = physical LEFT) -> ROS."""
    x, y, z = v
    return (z, x, y)


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


def ac_axes_to_ros_quat(x_axis, up, fwd):
    """Axis triples from SHM matrices (x_axis = the m[0:3] triple, physically
    LEFT) -> ROS quat: quat(B(fwd), B(x_axis), B(up)), proper rotation
    (det=+1). The earlier negated variant paired with the reflecting world
    map produced a chirality-consistent-looking but globally mirrored world
    (all pre-fix-10 validations were made through a mirrored RViz camera)."""
    return quat_from_cols(ac_vec_to_ros(fwd), ac_vec_to_ros(x_axis),
                          ac_vec_to_ros(up))


def ac_body_pose_to_ros(world16):
    """BODYTR world matrix (16 floats, AC row-major storage) ->
    (ros_position, ros_quat) of the car in the ROS ac_world frame."""
    pos_r = ac_vec_to_ros(world16[12:15])
    quat = ac_axes_to_ros_quat(world16[0:3], world16[4:7], world16[8:11])
    return pos_r, quat


def ac_pos_rot_to_ros(pos3, rot_cols9):
    """Separate position (3 floats) + BODYTR-style rotation (column triples,
    e.g. Detection3D world_pos/world_rot) -> (ros_position, ros_quat).
    Same math as ac_body_pose_to_ros, for payloads that split the two."""
    pos_r = ac_vec_to_ros(pos3)
    quat = ac_axes_to_ros_quat(rot_cols9[0:3], rot_cols9[3:6], rot_cols9[6:9])
    return pos_r, quat


def sensor_axes_to_ros_quat(fwd, y_axis, up):
    """Sensor axes as world-frame triples (from compose_base_to_sensor;
    sensor convention x=fwd, y=left, z=up) -> ROS quat for a
    base_link-><sensor>_frame TF. Same convention as ac_axes_to_ros_quat."""
    return quat_from_cols(ac_vec_to_ros(fwd), ac_vec_to_ros(y_axis),
                          ac_vec_to_ros(up))


def install_pose_to_ros(install_pos, install_fwd):
    """custom_lidars.ini install pose (POSITION/FORWARD, BODYTR-local frame:
    +X = physical LEFT - see module docstring) -> (ros_position, ros_quat)
    for a base_link -> <sensor>_frame TF. Sensor up is the body up axis.

    Deterministic and eyeball-verifiable against the ini: front FORWARD=(0,0,1)
    -> identity quat, rear (0,0,-1) -> 180-deg Z, left (1,0,0) -> +90 deg,
    right (-1,0,0) -> -90 deg. Uses the BODYTR map, NOT the world map."""
    import math as _m
    fx, fy, fz = install_fwd
    n = _m.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    fwd = (fx / n, fy / n, fz / n)
    up = (0.0, 1.0, 0.0)
    # algebraic cross(up, fwd) in the X-left body frame IS the sensor's left axis
    left = (
        up[1] * fwd[2] - up[2] * fwd[1],
        up[2] * fwd[0] - up[0] * fwd[2],
        up[0] * fwd[1] - up[1] * fwd[0],
    )
    quat = quat_from_cols(body_vec_to_ros(fwd), body_vec_to_ros(left),
                          body_vec_to_ros(up))
    return body_vec_to_ros(install_pos), quat


def compose_base_to_sensor(ego_world, world_pos, world_rot9):
    """T(base -> sensor) = inv(E) . S, all in AC coordinates.

    ego_world : 16-float BODYTR world matrix (AC storage; triples m[0:3]/[4:7]/
                [8:11] are the body's X/Y/Z axes in world).
    world_pos : sensor position in AC world (3 floats).
    world_rot9: sensor rotation, 9 floats as stored by ac_optix_renderer's mat3f
                (row-major; mul(R, sensor_local) = world).

    Returns (t, cols): sensor origin in the BODYTR frame, and the sensor's
    local x/y/z axis directions in the BODYTR frame (3 AC-frame triples).
    Numerically verified by test_compose.py (identity + yawed cases).
    """
    ep = ego_world[12:15]
    e_rows = (ego_world[0:3], ego_world[4:7], ego_world[8:11])  # C^T rows
    s_rows = tuple(world_rot9[i:i + 3] for i in (0, 3, 6))      # S rows
    # rr = C^T . S; columns of rr are the sensor axes in the body frame
    rr = [[sum(e_rows[i][k] * s_rows[k][j] for k in range(3))
           for j in range(3)] for i in range(3)]
    # t = C^T . (sensor_pos - body_pos)
    d = tuple(world_pos[i] - ep[i] for i in range(3))
    t = tuple(sum(e_rows[i][k] * d[k] for k in range(3)) for i in range(3))
    cols = ((rr[0][0], rr[1][0], rr[2][0]),
            (rr[0][1], rr[1][1], rr[2][1]),
            (rr[0][2], rr[1][2], rr[2][2]))
    return t, cols
