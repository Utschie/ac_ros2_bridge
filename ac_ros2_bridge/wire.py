"""Wire-format decode for ac_sim_bridge payloads.

Matches shared/zenoh_payload.h: all structs are #pragma pack(1), little-endian.
Python struct with '<' (standard size, no alignment) reproduces the same layout,
so these format strings are the exact counterpart of the C structs.

Used by relay.py (ROS2). test_decode.py exercises these against the C header sizes.
"""

import struct

# EgoPosePayload: uint64 timestampNs, uint64 frameId, float world[16]  (== 80 bytes)
EGO_FMT = "<QQ16f"
EGO_SIZE = struct.calcsize(EGO_FMT)  # 80

# Detection3DPayload header: uint64 timestampNs, uint32 count  (== 12 bytes)
DET_HDR_FMT = "<QI"
DET_HDR_SIZE = struct.calcsize(DET_HDR_FMT)  # 12

# Detection3D: float worldPos[3], float halfExtents[3], float worldRot[9],
#              uint16 classId, float score, uint64 bodyNodePtr  (== 74 bytes)
DET_FMT = "<3f3f9fHfQ"
DET_SIZE = struct.calcsize(DET_FMT)  # 74

CLASS_CAR = 1


def decode_ego(data: bytes) -> dict:
    """Decode an EgoPosePayload (80 bytes). 'world' is the 16-float row-major 4x4."""
    if len(data) < EGO_SIZE:
        raise ValueError(f"ego payload too short: {len(data)} < {EGO_SIZE}")
    ts_ns, frame_id, *world = struct.unpack(EGO_FMT, data[:EGO_SIZE])
    return {"timestamp_ns": ts_ns, "frame_id": frame_id, "world": world}


def decode_detections(data: bytes) -> dict:
    """Decode a Detection3DPayload (header + count * Detection3D)."""
    if len(data) < DET_HDR_SIZE:
        raise ValueError(f"detections header too short: {len(data)} < {DET_HDR_SIZE}")
    ts_ns, count = struct.unpack(DET_HDR_FMT, data[:DET_HDR_SIZE])
    need = DET_HDR_SIZE + count * DET_SIZE
    if len(data) < need:
        raise ValueError(f"detections body truncated: have {len(data)}, need {need}")
    dets = []
    off = DET_HDR_SIZE
    for _ in range(count):
        f = struct.unpack(DET_FMT, data[off:off + DET_SIZE])
        dets.append({
            "world_pos": f[0:3],
            "half_extents": f[3:6],
            "world_rot": f[6:15],
            "class_id": f[15],
            "score": f[16],
            "body_node_ptr": f[17],
        })
        off += DET_SIZE
    return {"timestamp_ns": ts_ns, "count": count, "detections": dets}
