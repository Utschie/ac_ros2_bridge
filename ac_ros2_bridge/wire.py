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

# LidarPayload: uint64 timestampNs, uint64 frameId, char name[32], uint32 pointCount,
#               float installPos[3], installFwd[3], worldPos[3], worldRot[9]  (== 124 bytes)
LIDAR_FMT = "<QQ32sI18f"
LIDAR_SIZE = struct.calcsize(LIDAR_FMT)  # 124

# ac_lidar::LidarHit: float range, x, y, z, intensity; uint32 instanceID (== 24 bytes)
LIDAR_HIT_SIZE = 24

# ImagePayload: uint64 timestampNs, uint64 frameId, uint32 width, uint32 height,
#               uint16 encoding (ImageEncoding), uint16 mode (ImageMode),
#               uint32 dataLen  (== 32 bytes) + trailing uint8 data[dataLen]
IMAGE_FMT = "<QQIIHHI"
IMAGE_SIZE = struct.calcsize(IMAGE_FMT)  # 32

# ac_payload::ImageEncoding -> ROS sensor_msgs/Image encoding string.
# MONO8/H264 reserved on the wire; H264 maps to CompressedImage in a future phase.
ENCODING_TO_ROS = {
    0: "rgba8",    # ENC_RGBA8
    1: "bgra8",    # ENC_BGRA8
    2: "32FC1",    # ENC_R32F (depth)
    3: "mono8",    # ENC_MONO8
}

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


def decode_lidar(data: bytes) -> dict:
    """Decode a LidarPayload (100-byte header + pointCount * LidarHit[24B]).

    The hit bytes are returned as a raw slice: LidarHit's layout
    (x@4, y@8, z@12, intensity@16, instance_id@20 floats/uint32, little-endian) maps
    1:1 onto PointCloud2 fields, so the relay can pass them through without
    per-point conversion.
    """
    if len(data) < LIDAR_SIZE:
        raise ValueError(f"lidar payload too short: {len(data)} < {LIDAR_SIZE}")
    ts_ns, frame_id, name_b, count, *rest = struct.unpack(LIDAR_FMT, data[:LIDAR_SIZE])
    install_pos = rest[0:3]
    install_fwd = rest[3:6]
    world_pos = rest[6:9]
    world_rot = rest[9:18]
    need = LIDAR_SIZE + count * LIDAR_HIT_SIZE
    if len(data) < need:
        raise ValueError(f"lidar hits truncated: have {len(data)}, need {need}")
    hits = data[LIDAR_SIZE:need]
    name = name_b.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    return {"timestamp_ns": ts_ns, "frame_id": frame_id, "name": name,
            "point_count": count, "install_pos": install_pos, "install_fwd": install_fwd,
            "world_pos": world_pos, "world_rot": world_rot,
            "hits": hits}


def decode_image(data: bytes) -> dict:
    """Decode an ImagePayload (32-byte header + dataLen trailing bytes).

    'data' is returned as the raw pixel slice: the relay passes it straight
    into sensor_msgs/Image.msg.data (bytes accepted by rclpy) — same passthrough
    trick as the lidar hits.
    """
    if len(data) < IMAGE_SIZE:
        raise ValueError(f"image payload too short: {len(data)} < {IMAGE_SIZE}")
    ts_ns, frame_id, width, height, enc, mode, data_len = \
        struct.unpack(IMAGE_FMT, data[:IMAGE_SIZE])
    need = IMAGE_SIZE + data_len
    if len(data) < need:
        raise ValueError(f"image data truncated: have {len(data)}, need {need}")
    return {"timestamp_ns": ts_ns, "frame_id": frame_id,
            "width": width, "height": height,
            "encoding": enc, "mode": mode, "data_len": data_len,
            "data": data[IMAGE_SIZE:need]}
