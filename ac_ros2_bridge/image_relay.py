"""CameraRelay - camera image domain module.

ac/camera/<cam>/<suffix> handling. M2 compressed-wire adaptation (2026-10-14):
  image_raw     enc=5  JPEG    -> /ac/camera/<cam>/image_raw/compressed
                                  (sensor_msgs/CompressedImage, format="jpeg";
                                  zero-decode passthrough — consume with
                                  image_transport republish or RViz2's
                                  compressed transport)
  image_depth   enc=6  LZ4 u16 -> /ac/camera/<cam>/image_depth
                                  (sensor_msgs/Image 32FC1 meters,
                                   u16 / 32.7675, 2000 m full scale - M17
                                   contract since 2026-10-18; 65535 decodes
                                   to 2000.0 m = sky / full range)
  image_classes enc=3  LZ4 u8  -> /ac/camera/<cam>/image_classes
                                  (sensor_msgs/Image mono8, pixel = class id;
                                  vocabulary = /ac/semantic/classes)
  camera_info   52B payload    -> /ac/camera/<cam>/camera_info
                                  (sensor_msgs/CameraInfo, K from fovDeg,
                                  latched TRANSIENT_LOCAL)
Legacy MODE_RAW frames (enc 0-3 bare pixels) still relay as before.

Images are NOT frame-paired (their grab timestamp never matches an ego
stamp, and the Image display resolves no TF chain) - they use the clamped
stamp only.
"""

import numpy as np

from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
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
# camera_info is quasi-static (once per camera at first frame): latched so
# late subscribers get it immediately (TRANSIENT_LOCAL pub matches both
# TRANSIENT_LOCAL and VOLATILE subs).
CAMINFO_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST,
                         depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CameraRelay:
    _IMAGE_SUFFIXES = ("image_raw", "image_depth", "image_classes")
    _BYTES_PER_PIXEL = {"rgba8": 4, "bgra8": 4, "32FC1": 4, "mono8": 1}

    def __init__(self, ctx):
        self.ctx = ctx
        self._img_pubs = {}       # ROS topic -> Image publisher (lazy)
        self._cimg_pubs = {}      # ROS topic -> CompressedImage publisher (lazy)
        self._info_pubs = {}      # ROS topic -> CameraInfo publisher (lazy)
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
        if suffix == "camera_info":
            self._relay_camera_info(cam, key, sample)
            return
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
        if img["mode"] == wire.MODE_RAW:
            self._relay_raw(cam, suffix, key, img)
        elif img["mode"] == wire.MODE_COMPRESSED:
            self._relay_compressed(cam, suffix, key, img)
        else:
            self.ctx.log.warn(f"image mode={img['mode']} unknown ({key}), dropped")

    # ---- legacy raw path (enc 0-3 bare pixels; kept for compatibility) ----

    def _relay_raw(self, cam, suffix, key, img) -> None:
        ros_enc = wire.ENCODING_TO_ROS.get(img["encoding"])
        if ros_enc is None:
            self.ctx.log.warn(f"image encoding={img['encoding']} unknown raw "
                              f"encoding ({key}), dropped")
            return
        expected = img["width"] * img["height"] * self._BYTES_PER_PIXEL[ros_enc]
        if img["data_len"] != expected:
            self.ctx.log.warn(f"image size mismatch ({key}): data_len={img['data_len']} "
                              f"but {img['width']}x{img['height']} {ros_enc} needs {expected}")
            return
        self._publish_image(f"/{key}", cam, img, ros_enc,
                            img["data_len"] // img["height"],
                            np.frombuffer(img["data"], dtype=np.uint8))

    # ---- M2 compressed path ----

    def _relay_compressed(self, cam, suffix, key, img) -> None:
        w, h, enc = img["width"], img["height"], img["encoding"]
        try:
            if suffix == "image_raw":
                if enc != wire.ENC_JPEG:
                    self.ctx.log.warn(f"image_raw encoding={enc} != JPEG "
                                      f"({key}), dropped")
                    return
                topic = f"/{key}/compressed"
                if topic not in self._cimg_pubs:
                    with self.ctx.pub_lock:
                        self._cimg_pubs[topic] = self.ctx.node.create_publisher(
                            CompressedImage, topic, IMAGE_QOS)
                    self.ctx.log.info(f"compressed image publisher '{topic}' "
                                      f"({w}x{h} jpeg)")
                msg = CompressedImage()
                msg.header = Header()
                msg.header.frame_id = f"{cam}_frame"
                self.ctx.stamp_clamped(msg.header)
                msg.format = "jpeg"
                msg._data = np.frombuffer(img["data"], dtype=np.uint8)
                with self.ctx.pub_lock:
                    self._cimg_pubs[topic].publish(msg)
            elif suffix == "image_depth":
                if enc != wire.ENC_DEPTH16:
                    self.ctx.log.warn(f"image_depth encoding={enc} != DEPTH16 "
                                      f"({key}), dropped")
                    return
                raw = wire.decompress_lz4(img["data"], w * h * 2)
                # u16 -> meters (dscale contract; 65535 -> exactly 200.0 m = sky).
                # 纯 float32 原地乘（不走 float64 中转）：zenoh 回调单线程，
                # 满配额时这里的内存带宽是 relay 吞吐瓶颈（2026-10-14 实测）。
                meters = np.frombuffer(raw, dtype="<u2").astype(np.float32)
                meters *= np.float32(1.0 / wire.DEPTH16_DSCALE)
                self._publish_image(f"/{key}", cam, img, "32FC1", w * 4, meters)
            elif suffix == "image_classes":
                if enc != 3:
                    self.ctx.log.warn(f"image_classes encoding={enc} != MONO8 "
                                      f"({key}), dropped")
                    return
                raw = wire.decompress_lz4(img["data"], w * h)
                self._publish_image(f"/{key}", cam, img, "mono8", w,
                                    np.frombuffer(raw, dtype=np.uint8))
        except Exception as ex:  # noqa: BLE001
            self.ctx.log.warn(f"compressed image relay failed ({key}): {ex}")

    # ---- camera_info (52B CameraInfoPayload -> sensor_msgs/CameraInfo) ----

    def _relay_camera_info(self, cam, key, sample) -> None:
        try:
            ci = wire.decode_camera_info(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.ctx.log.warn(f"camera_info decode failed ({key}): {ex}")
            return
        topic = f"/{key}"
        if topic not in self._info_pubs:
            with self.ctx.pub_lock:
                self._info_pubs[topic] = self.ctx.node.create_publisher(
                    CameraInfo, topic, CAMINFO_QOS)
            self.ctx.log.info(f"camera_info publisher '{topic}' "
                              f"({ci['width']}x{ci['height']} fov={ci['fov_deg']:.1f})")
        w, h = ci["width"], ci["height"]
        fx = 0.5 * w / np.tan(np.radians(ci["fov_deg"]) / 2.0)
        fy = fx                      # square pixels (sim camera)
        cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
        msg = CameraInfo()
        msg.header = Header()
        msg.header.frame_id = f"{cam}_frame"
        self.ctx.stamp_clamped(msg.header)
        msg.height = h
        msg.width = w
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0] * 5
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        with self.ctx.pub_lock:
            self._info_pubs[topic].publish(msg)

    # ---- shared Image publish helper ----

    def _publish_image(self, topic, cam, img, ros_enc, step, data_nd) -> None:
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
        msg.step = step
        # Direct slot write (see scene_relay): the generated uint8[]/float32[]
        # setter either rejects ndarrays or loops per-element (~seconds for a
        # 4MB frame). rclpy's C serializer consumes the ndarray natively.
        msg._data = data_nd
        with self.ctx.pub_lock:
            self._img_pubs[topic].publish(msg)
