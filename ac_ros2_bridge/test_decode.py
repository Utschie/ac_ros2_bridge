"""Standalone format check for wire.py — run on any OS with Python (no ROS2 needed).

Verifies the Python struct format strings match shared/zenoh_payload.h (pack(1), LE):
  EgoPosePayload      == 80 bytes
  Detection3DPayload  == 12-byte header + 74 bytes/Detection3D
Run:  python ac_ros2_bridge/ac_ros2_bridge/test_decode.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wire  # noqa: E402


def main() -> None:
    # sizes must match the C static_asserts in shared/zenoh_payload.h
    assert wire.EGO_SIZE == 80, f"EGO_SIZE {wire.EGO_SIZE} != 80"
    assert wire.DET_HDR_SIZE == 12, f"DET_HDR_SIZE {wire.DET_HDR_SIZE} != 12"
    assert wire.DET_SIZE == 74, f"DET_SIZE {wire.DET_SIZE} != 74"

    # ego round-trip: row-major 4x4, translation in m[12],m[13],m[14]
    world = [1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             10.5, 20.5, 30.5, 1.0]
    import struct
    ego_bytes = struct.pack("<QQ16f", 123456789, 4242, *world)
    e = wire.decode_ego(ego_bytes)
    assert e["timestamp_ns"] == 123456789
    assert e["frame_id"] == 4242
    assert abs(e["world"][12] - 10.5) < 1e-6
    assert abs(e["world"][13] - 20.5) < 1e-6
    assert abs(e["world"][14] - 30.5) < 1e-6

    # detections round-trip: 2 boxes
    import struct
    one = struct.pack("<3f3f9fHfQ",
                      1.0, 2.0, 3.0,        # worldPos
                      0.5, 0.4, 0.3,         # halfExtents
                      1, 0, 0, 0, 1, 0, 0, 0, 1,  # worldRot (identity 3x3, 9 values)
                      1, 0.99, 0xAABBCCDD)   # classId, score, bodyNodePtr
    hdr = struct.pack("<QI", 999, 2)
    d = wire.decode_detections(hdr + one + one)
    assert d["timestamp_ns"] == 999
    assert d["count"] == 2
    assert len(d["detections"]) == 2
    det0 = d["detections"][0]
    assert abs(det0["world_pos"][0] - 1.0) < 1e-6
    assert abs(det0["half_extents"][2] - 0.3) < 1e-6
    assert det0["class_id"] == 1
    assert det0["body_node_ptr"] == 0xAABBCCDD

    # lidar: header sizes must match shared/zenoh_payload.h + shared/lidar_shm.h
    assert wire.LIDAR_SIZE == 100, f"LIDAR_SIZE {wire.LIDAR_SIZE} != 100"
    assert wire.LIDAR_HIT_SIZE == 24, f"LIDAR_HIT_SIZE {wire.LIDAR_HIT_SIZE} != 24"

    # lidar round-trip: 2 hits, name, pose
    name32 = b"front".ljust(32, b"\x00")
    lhdr = struct.pack("<QQ32sI3f9f", 777, 55, name32, 2,
                       10.0, 20.0, 30.0,               # worldPos
                       1, 0, 0, 0, 1, 0, 0, 0, 1)      # worldRot identity (row-major)
    hit = struct.pack("<5fI", 42.5, 1.0, 2.0, 3.0, 0.9, 7)
    l = wire.decode_lidar(lhdr + hit + hit)
    assert l["name"] == "front"
    assert l["point_count"] == 2
    assert len(l["hits"]) == 48
    assert abs(l["world_pos"][0] - 10.0) < 1e-6
    # first hit x/y/z at byte offsets 4/8/12 (PointCloud2 passthrough layout)
    hx, hy, hz = struct.unpack_from("<3f", l["hits"], 4)
    assert abs(hx - 1.0) < 1e-6 and abs(hy - 2.0) < 1e-6 and abs(hz - 3.0) < 1e-6

    print(f"decode_check OK: EGO_SIZE={wire.EGO_SIZE} "
          f"DET_HDR_SIZE={wire.DET_HDR_SIZE} DET_SIZE={wire.DET_SIZE} "
          f"LIDAR_SIZE={wire.LIDAR_SIZE} LIDAR_HIT_SIZE={wire.LIDAR_HIT_SIZE} "
          f"(matches shared/zenoh_payload.h + shared/lidar_shm.h)")


if __name__ == "__main__":
    main()
