"""Standalone format check for wire.py — run on any OS with Python (no ROS2 needed).

Verifies the Python struct format strings match shared/zenoh_payload.h (pack(1), LE):
  EgoPosePayload      == 80 bytes
  Detection3DPayload  == 12-byte header + 74 bytes/Detection3D
  LidarPayload        == 124 bytes + 24 bytes/hit
  ImagePayload        == 32-byte header + dataLen
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
    assert wire.LIDAR_SIZE == 124, f"LIDAR_SIZE {wire.LIDAR_SIZE} != 124"
    assert wire.LIDAR_HIT_SIZE == 24, f"LIDAR_HIT_SIZE {wire.LIDAR_HIT_SIZE} != 24"

    # lidar round-trip: 2 hits, name, install + world pose
    name32 = b"front".ljust(32, b"\x00")
    lhdr = struct.pack("<QQ32sI18f", 777, 55, name32, 2,
                       0.0, 1.1, 0.4,                 # installPos (ini POSITION)
                       0.0, 0.0, 1.0,                 # installFwd (ini FORWARD)
                       10.0, 20.0, 30.0,              # worldPos
                       1, 0, 0, 0, 1, 0, 0, 0, 1)     # worldRot identity (row-major)
    hit = struct.pack("<5fI", 42.5, 1.0, 2.0, 3.0, 0.9, 7)
    l = wire.decode_lidar(lhdr + hit + hit)
    assert l["name"] == "front"
    assert l["point_count"] == 2
    assert len(l["hits"]) == 48
    assert abs(l["install_pos"][1] - 1.1) < 1e-6
    assert abs(l["install_fwd"][2] - 1.0) < 1e-6
    assert abs(l["world_pos"][0] - 10.0) < 1e-6
    # first hit x/y/z at byte offsets 4/8/12 (PointCloud2 passthrough layout)
    hx, hy, hz = struct.unpack_from("<3f", l["hits"], 4)
    assert abs(hx - 1.0) < 1e-6 and abs(hy - 2.0) < 1e-6 and abs(hz - 3.0) < 1e-6

    # image: header size must match shared/zenoh_payload.h (ImagePayload 32B)
    assert wire.IMAGE_SIZE == 32, f"IMAGE_SIZE {wire.IMAGE_SIZE} != 32"

    # image round-trip: 4x2 rgba8 pixels (32 bytes) + trailing extra byte is NOT
    # forwarded (decoder must slice exactly data_len bytes)
    pixels = bytes(range(4 * 2 * 4))
    ihdr = struct.pack("<QQIIHHI", 123, 77, 4, 2, 0, 0, len(pixels))
    im = wire.decode_image(ihdr + pixels + b"\xEE")   # trailing junk must be cut
    assert im["timestamp_ns"] == 123
    assert im["frame_id"] == 77
    assert im["width"] == 4 and im["height"] == 2
    assert im["encoding"] == 0 and im["mode"] == 0
    assert im["data_len"] == 32
    assert im["data"] == pixels
    assert len(im["data"]) == im["data_len"]          # step = data_len // height = 16

    # depth variant: encoding=2 (R32F) round-trips
    ihdr_d = struct.pack("<QQIIHHI", 124, 78, 2, 1, 2, 0, 8)
    imd = wire.decode_image(ihdr_d + b"\x00" * 8)
    assert imd["encoding"] == 2 and imd["data_len"] == 8

    # truncation branches must raise
    for bad in (ihdr[:16], ihdr + pixels[:16]):
        try:
            wire.decode_image(bad)
            raise AssertionError("truncated image must raise")
        except ValueError:
            pass

    # wire encoding map must cover every raw encoding the bridge sends today
    assert wire.ENCODING_TO_ROS[0] == "rgba8"
    assert wire.ENCODING_TO_ROS[1] == "bgra8"
    assert wire.ENCODING_TO_ROS[2] == "32FC1"
    assert wire.ENCODING_TO_ROS[3] == "mono8"

    # control: sizes must match shared/zenoh_payload.h (ControlPayload 32B)
    assert wire.CONTROL_SIZE == 32, f"CONTROL_SIZE {wire.CONTROL_SIZE} != 32"
    assert wire.VSTATE_SIZE == 20, f"VSTATE_SIZE {wire.VSTATE_SIZE} != 20"

    # control round-trip: values + bool->buttons bit packing + flags
    c = wire.encode_control(steering=-0.25, throttle=0.3, brake=0.0, clutch=0.0,
                            hand_brake=0.0, requested_gear_index=2,   # 1st gear
                            gear_up=False, gear_dn=True, drs=True, kers=False,
                            handbrake_on=True, override_enabled=True)
    assert len(c) == 32
    import struct as _s
    st, th, br, cl, hb, gear, buttons, flags = _s.unpack(wire.CONTROL_FMT, c)
    assert abs(st - (-0.25)) < 1e-6 and abs(th - 0.3) < 1e-6
    assert gear == 2
    assert buttons == (wire.BTN_GEAR_DN | wire.BTN_DRS | wire.BTN_HANDBRAKE_ON)
    assert flags == wire.FLAG_OVERRIDE
    # all-clear command: nothing pressed, not overriding
    c0 = wire.encode_control(0, 0, 0, 0, 0, wire.GEAR_NO_CHANGE)
    assert _s.unpack(wire.CONTROL_FMT, c0)[6:8] == (0, 0)

    # game control: sizes + bit packing
    assert wire.GAME_CMD_SIZE == 4, f"GAME_CMD_SIZE {wire.GAME_CMD_SIZE} != 4"
    assert wire.GAME_STATE_SIZE == 16, f"GAME_STATE_SIZE {wire.GAME_STATE_SIZE} != 16"
    g = wire.encode_game_control(start_race=True, skip_session=True)
    assert _s.unpack(wire.GAME_CMD_FMT, g)[0] == (wire.GAMECMD_START | wire.GAMECMD_SKIP)
    assert _s.unpack(wire.GAME_CMD_FMT,
                     wire.encode_game_control())[0] == 0
    gs = wire.decode_game_state(
        _s.pack(wire.GAME_STATE_FMT, 111, 3, wire.FLAG_FEEDBACK_GO | 0x2))
    assert gs["timestamp_ns"] == 111
    assert gs["session_type"] == 3   # RACE
    assert gs["flags"] == (wire.FLAG_FEEDBACK_GO | 0x2)
    try:
        wire.decode_game_state(b"\x00" * 8)
        raise AssertionError("truncated game_state must raise")
    except ValueError:
        pass

    # vehicle_state round-trip: gear/speed/flags + GO bit convention
    v = wire.decode_vehicle_state(_s.pack(wire.VSTATE_FMT, 987654321, 2, 12.5,
                                          wire.FLAG_FEEDBACK_GO))
    assert v["timestamp_ns"] == 987654321
    assert v["gear"] == 2 and abs(v["speed"] - 12.5) < 1e-6
    assert v["flags"] & wire.FLAG_FEEDBACK_GO
    # truncation must raise
    try:
        wire.decode_vehicle_state(b"\x00" * 10)
        raise AssertionError("truncated vehicle_state must raise")
    except ValueError:
        pass

    print(f"decode_check OK: EGO_SIZE={wire.EGO_SIZE} "
          f"DET_HDR_SIZE={wire.DET_HDR_SIZE} DET_SIZE={wire.DET_SIZE} "
          f"LIDAR_SIZE={wire.LIDAR_SIZE} LIDAR_HIT_SIZE={wire.LIDAR_HIT_SIZE} "
          f"IMAGE_SIZE={wire.IMAGE_SIZE} CONTROL_SIZE={wire.CONTROL_SIZE} "
          f"VSTATE_SIZE={wire.VSTATE_SIZE} GAME_CMD_SIZE={wire.GAME_CMD_SIZE} "
          f"GAME_STATE_SIZE={wire.GAME_STATE_SIZE} "
          f"(matches shared/zenoh_payload.h + shared/lidar_shm.h)")


if __name__ == "__main__":
    main()
