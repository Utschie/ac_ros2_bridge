"""CameraRelay - camera image domain module.

Moved verbatim from relay.py (M5 modularization; behavior unchanged):
  ac/camera/<cam>/<suffix> -> sensor_msgs/Image '/ac/camera/<cam>/<suffix>'
    suffix: image_raw | image_depth | image_classes; pixel bytes passed
    through, encoding mapped via wire.ENCODING_TO_ROS.

Images are NOT frame-paired (their grab timestamp never matches an ego
stamp, and the Image display resolves no TF chain) - they use the clamped
stamp only.

Future: MODE_COMPRESSED / ENC_H264 -> sensor_msgs/CompressedImage branch
(the wire enum is already reserved; the Windows-side encoder seam is
ac_sim_bridge's IFrameEncoder).
"""

import numpy as np

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from . import wire

# Image publishers must be RELIABLE: RViz2's Image display subscribes RELIABLE
# by default, and Pub BEST_EFFORT + Sub RELIABLE is the one incompatible QoS
# combination (DDS refuses to match -> "offering incompatible QoS" and no data).
# Cross-machine loss is handled by the Windows bridge (zenoh DROP congestion
# control); relay->RViz2 is a localhost hop, so RELIABLE costs nothing here.
IMAGE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST,
                       depth=5,
                       reliability=ReliabilityPolicy.RELIABLE)


class CameraRelay:
    # Suffixes carried as raw pixel payloads today. camera_info will be relayed
    # when the bridge starts sending CameraInfoPayload (future phase).
    _IMAGE_SUFFIXES = ("image_raw", "image_depth", "image_classes")
    _BYTES_PER_PIXEL = {"rgba8": 4, "bgra8": 4, "32FC1": 4, "mono8": 1}

    def __init__(self, ctx):
        self.ctx = ctx
        self._img_pubs = {}       # ROS topic -> Image publisher (lazy)
        self._cam_ignored = set()

    def attach(self, session) -> None:
        session.declare_subscriber("ac/camera/**", self._on_camera)

    def _on_camera(self, sample) -> None:
        key = str(sample.key_expr)   # ac/camera/<cam>/<suffix>
        parts = key.split("/")
        if len(parts) != 4 or parts[0] != "ac" or parts[1] != "camera" or not parts[2]:
            self.ctx.log.warn(f"unexpected camera key '{key}', ignored")
            return
        cam, suffix = parts[2], parts[3]
        if suffix not in self._IMAGE_SUFFIXES:
            if key not in self._cam_ignored:
                self._cam_ignored.add(key)
                self.ctx.log.info(f"camera key '{key}' not relayed yet, ignored")
            return

        try:
            img = wire.decode_image(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.ctx.log.warn(f"image decode failed ({key}): {ex}")
            return
        ros_enc = wire.ENCODING_TO_ROS.get(img["encoding"])
        if ros_enc is None or img["mode"] != 0:
            # ENC_H264 / MODE_COMPRESSED -> sensor_msgs/CompressedImage, future phase
            self.ctx.log.warn(f"image encoding={img['encoding']} mode={img['mode']} "
                              f"not relayed yet ({key})")
            return
        expected = img["width"] * img["height"] * self._BYTES_PER_PIXEL[ros_enc]
        if img["data_len"] != expected:
            self.ctx.log.warn(f"image size mismatch ({key}): data_len={img['data_len']} "
                              f"but {img['width']}x{img['height']} {ros_enc} needs {expected}")
            return

        topic = f"/{key}"
        if topic not in self._img_pubs:
            with self.ctx.pub_lock:
                self._img_pubs[topic] = self.ctx.node.create_publisher(
                    Image, topic, IMAGE_QOS)
            self.ctx.log.info(f"image publisher '{topic}' "
                              f"({img['width']}x{img['height']} {ros_enc})")

        msg = Image()
        msg.header = Header()
        msg.header.frame_id = f"{cam}_frame"
        self.ctx.stamp_clamped(msg.header)
        msg.height = img["height"]
        msg.width = img["width"]
        msg.encoding = ros_enc
        msg.is_bigendian = 0
        msg.step = img["data_len"] // img["height"]
        # Direct slot write (see scene_relay): the generated uint8[] setter
        # either rejects ndarrays or loops per-element over bytes (~seconds
        # for a 4MB frame). rclpy's C serializer consumes the ndarray natively.
        msg._data = np.frombuffer(img["data"], dtype=np.uint8)
        with self.ctx.pub_lock:
            self._img_pubs[topic].publish(msg)
