"""AC <-> ROS coordinate conversion - THE single place for axis/frame math.

AC world frame: Y-up LEFT-handed (X right, Y up, Z forward). Stored matrices are
row-major 4x4 whose contiguous triples m[0:3]/m[4:7]/m[8:11] are the frame's
X/Y/Z axes in world (effective transform: world_x = x*m[0]+y*m[4]+z*m[8]+m[12]).
ROS: Z-up RIGHT-handed (X fwd, Y left, Z up).

If anything looks wrong in RViz (mirrored / tilted / swapped), fix it HERE -
all consumers (ego pose, OBBs, lidar TF, future camera TF) go through these
functions. Covered by test_compose.py (run on any OS, no ROS needed).
"""

import math


def ac_vec_to_ros(v):
    """Map one AC-frame vector to ROS: AC Z(fwd)->ROS X, AC X(right)->-ROS Y, AC Y(up)->ROS Z."""
    x, y, z = v
    return (z, -x, y)


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


def ac_axes_to_ros_quat(right, up, fwd):
    """BODYTR-style axes (each an AC-frame triple) -> ROS quat of the same
    physical orientation. Axis-aligned input yields the identity quat
    (car upright, facing ROS +X). Verified in RViz."""
    fwd_r = ac_vec_to_ros(fwd)
    left_r = tuple(-t for t in ac_vec_to_ros(right))
    up_r = ac_vec_to_ros(up)
    return quat_from_cols(fwd_r, left_r, up_r)


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


def sensor_axes_to_ros_quat(fwd, left, up):
    """Sensor-frame axes (each an AC-frame triple; sensor local convention is
    x=fwd, y=left, z=up) -> ROS quat for a base_link-><sensor>_frame TF.
    Wraps the sign juggling so callers never hand-roll it."""
    right = tuple(-t for t in left)
    return ac_axes_to_ros_quat(right, up, fwd)


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
