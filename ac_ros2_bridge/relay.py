"""ac_ros2_bridge relay node.

Subscribes Zenoh keys from the Windows ac_sim_bridge, decodes the compact
binary payloads (wire.py, matches shared/zenoh_payload.h), and republishes as
standard ROS2 messages. All AC->ROS coordinate math lives in frames.py:

  ac/ego_pose       -> geometry_msgs/PoseStamped '/ac/ego_pose'
                       + TF ac_world -> base_link
  ac/detections_3d  -> vision_msgs/Detection3DArray '/ac/detections_3d'
                       + visualization_msgs/MarkerArray '/ac/detections_3d_markers'
                       (RViz2 has no built-in Detection3DArray display)
  ac/lidar/*/points -> sensor_msgs/PointCloud2 '/ac/lidar/<name>/points'
                       (points stay in the sensor frame, like a real lidar driver)
                       + TF base_link -> <name>_frame
  ac/camera/<n>/<suffix> -> sensor_msgs/Image '/ac/camera/<n>/<suffix>'
                       (suffix: image_raw | image_depth | image_classes;
                       pixel bytes passed through, encoding mapped in wire.py)

TF tree: ac_world -> base_link -> {front,rear,...}_frame. Points/images stay in
their sensor frames; RViz2 and downstream consumers resolve via the TF chain.

Timestamp policy (fixes RViz2's intermittent "No Transform ..." flicker):
ALL outbound headers are stamped with the relay's LOCAL ROS clock at receive
time, in one clock domain. The producer's wall-clock stamps (Windows, from
ac_sim_bridge) are NOT propagated: the two machines' clocks are not synced, so
a forward skew made RViz2 see future-stamped messages and tf2 refused to
extrapolate. Messages that depend on the ac_world->base_link chain are
additionally clamped to the newest ego-TF time, so a lookup at the message
stamp always finds a same-or-newer transform regardless of arrival order.
FRAME-EXACT PAIRING: the bridge stamps ego/dets/lidar of the SAME simulation
frame with one shared producer timestamp; the relay joins on it and stamps
dets/lidar with the local time of that exact frame's ego TF. Without this,
zenoh callback reordering could bind a box/cloud to the previous frame's pose
(box darts ahead of base_link's axes while moving). Messages whose ego hasn't
arrived yet are held briefly (<= PAIR_TIMEOUT_S) and fall back to the clamped
stamp on timeout. Images are not paired (their grab timestamp never matches
an ego stamp, and image display resolves no TF chain).
Sim time (/clock + use_sim_time) is the documented future upgrade; when it
lands, only the clock source here changes.

vision_msgs field access auto-detects ROS2 Humble vs Jazzy/Iron+.
"""

import os
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField, Image
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import zenoh

from . import frames
from . import wire

FRAME_WORLD = "ac_world"
FRAME_BASE = "base_link"
MARKER_NS = "ac_detections"
MARKER_LIFETIME_NS = 500_000_000  # 0.5s auto-expire
PAIR_TIMEOUT_S = 0.5   # max wait for the same-frame ego before clamped fallback
EGO_MAP_CAP = 512      # pairing map size (~10s @ 50Hz)

# Image publishers must be RELIABLE: RViz2's Image display subscribes RELIABLE
# by default, and Pub BEST_EFFORT + Sub RELIABLE is the one incompatible QoS
# combination (DDS refuses to match -> "offering incompatible QoS" and no data).
# Cross-machine loss is handled by the Windows bridge (zenoh DROP congestion
# control); relay->RViz2 is a localhost hop, so RELIABLE costs nothing here.
IMAGE_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST,
                       depth=5,
                       reliability=ReliabilityPolicy.RELIABLE)


def _field(name, datatype, offset):
    f = PointField()
    f.name = name
    f.datatype = datatype
    f.offset = offset
    f.count = 1
    return f


def _payload_bytes(sample) -> bytes:
    p = sample.payload
    return p.to_bytes() if hasattr(p, "to_bytes") else bytes(p)


class AcRelay(Node):
    def __init__(self):
        super().__init__("ac_relay")

        cfg = zenoh.Config()
        zcfg = os.environ.get("ZENOH_CONFIG")
        if zcfg and os.path.isfile(zcfg):
            cfg = zenoh.Config.from_file(zcfg)
        self._session = zenoh.open(cfg)

        self.pub_ego = self.create_publisher(PoseStamped, "/ac/ego_pose", 10)
        self.pub_dets = self.create_publisher(Detection3DArray, "/ac/detections_3d", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/ac/detections_3d_markers", 10)
        self._pc_pubs = {}   # lidar name -> PointCloud2 publisher (lazy)
        self._img_pubs = {}  # ROS topic -> Image publisher (lazy)
        self._last_ego_world = None   # 16-float AC BODYTR world matrix (for TF composition)
        self.tf = TransformBroadcaster(self)
        self.tf_static = StaticTransformBroadcaster(self)
        # Zenoh callbacks run on zenoh threads, not the rclpy executor thread;
        # rclpy publish() is not guaranteed thread-safe -> serialize all ROS
        # publishes / TF sends through one lock.
        self._pub_lock = threading.Lock()
        self._last_tf_stamp = None    # rclpy Time of newest ac_world->base_link TF
        # lidar name -> (pose_sig, TransformStamped). The FULL set is re-sent
        # in one sendTransform(list) on every change: /tf_static is latched
        # (TRANSIENT_LOCAL) with depth 1, so a per-lidar send would leave late
        # joiners (RViz2 started after the relay) with only the LAST transform
        # latched -- the other lidars' chains break and their clouds pile up
        # in RViz2's message filter ("queue is full", never displayed).
        self._static_tf_cache = {}
        # Frame-exact pairing (see module docstring): producer ts of each ego
        # frame -> local stamp of that frame's TF; messages awaiting their ego.
        # Guarded by _pair_lock (short critical sections, never publish inside).
        self._pair_lock = threading.Lock()
        self._ego_stamp_by_pts = {}   # producer ts_ns -> rclpy Time
        self._ego_pts_order = deque()
        self._pending_msgs = []       # (pts, enqueue_monotonic, publish_fn)
        self._pair_timer = self.create_timer(0.1, self._flush_pending)

        self._sub_ego = self._session.declare_subscriber("ac/ego_pose", self._on_ego)
        self._sub_dets = self._session.declare_subscriber("ac/detections_3d", self._on_dets)
        self._sub_lidar = self._session.declare_subscriber("ac/lidar/*/points", self._on_lidar)
        self._sub_camera = self._session.declare_subscriber("ac/camera/**", self._on_camera)
        self.get_logger().info("ac_relay ready: subscribing ac/ego_pose + ac/detections_3d "
                               "+ ac/lidar/*/points + ac/camera/** "
                               "(AC->ROS axis remap in frames.py)")

    def _now(self):
        """Relay-local ROS clock (single clock domain for every outbound stamp)."""
        return self.get_clock().now()

    def _stamp_clamped(self, header) -> None:
        """Stamp for messages that resolve through the ac_world->base_link chain.

        Clamped to the newest ego-TF time: a consumer looking up TF at this
        stamp is guaranteed a same-or-newer transform in the buffer, killing
        both the cross-machine future-stamp failure and the arrival-order race
        (the two causes of RViz2's intermittent "No Transform" flicker).
        Before the first ego message this degrades to plain local now.
        """
        now = self._now()
        if self._last_tf_stamp is not None and now > self._last_tf_stamp:
            now = self._last_tf_stamp
        header.stamp = now.to_msg()

    # ---- frame-exact pairing (dets/lidar <-> same-frame ego TF) ----

    def _publish_paired(self, pts: int, publish_fn) -> None:
        """Publish now with the same-frame ego stamp if known, else defer.

        The bridge stamps ego/dets/lidar of one simulation frame with one
        shared producer timestamp, so an exact dict join recovers the local
        stamp of THAT frame's TF even when zenoh delivers the two callbacks
        out of order (the cause of boxes darting ahead of base_link's axes).
        """
        with self._pair_lock:
            t = self._ego_stamp_by_pts.get(pts)
            if t is not None:
                deferred = False
            else:
                self._pending_msgs.append((pts, time.monotonic(), publish_fn))
                deferred = True
        if not deferred:
            publish_fn(t)

    def _collect_due(self):
        """Remove and return [(publish_fn, t_or_None)] whose ego arrived (t) or
        that timed out (None -> publish_fn falls back to the clamped stamp)."""
        now_mono = time.monotonic()
        due = []
        with self._pair_lock:
            remain = []
            for pts, enq, fn in self._pending_msgs:
                t = self._ego_stamp_by_pts.get(pts)
                if t is not None or (now_mono - enq) >= PAIR_TIMEOUT_S:
                    due.append((fn, t))
                else:
                    remain.append((pts, enq, fn))
            self._pending_msgs = remain
        return due

    def _publish_due(self, due) -> None:
        for fn, t in due:
            try:
                fn(t)
            except Exception as ex:  # noqa: BLE001
                self.get_logger().warn(f"paired publish failed: {ex}")

    def _flush_pending(self) -> None:
        """Timer hook (executor thread): publish matched / timed-out messages."""
        try:
            self._publish_due(self._collect_due())
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"flush_pending failed: {ex}")

    def _on_ego(self, sample) -> None:
        try:
            e = wire.decode_ego(_payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"ego decode failed: {ex}")
            return
        w = e["world"]
        self._last_ego_world = list(w)   # cached for base_link->sensor TF composition
        # Debug: raw BODYTR matrix triples, throttled to ~1 line / 2s.
        # Park the car straight on a straight and read one line: the triple
        # that matches the car's heading decides the storage orientation.
        import time as _time
        now_t = _time.monotonic()
        if now_t - getattr(self, "_last_mtx_log", 0.0) > 2.0:
            self._last_mtx_log = now_t
            self.get_logger().info(
                "egoMatrix t=[{:.2f},{:.2f},{:.2f}] r0=[{:.3f},{:.3f},{:.3f}] "
                "r1=[{:.3f},{:.3f},{:.3f}] r2=[{:.3f},{:.3f},{:.3f}]".format(
                    w[12], w[13], w[14],
                    w[0], w[1], w[2], w[4], w[5], w[6], w[8], w[9], w[10]))
        (px, py, pz), (qx, qy, qz, qw) = frames.ac_body_pose_to_ros(w)

        # One local-time sample per callback; the message and its TF share it.
        t = self._now()
        ps = PoseStamped()
        ps.header.frame_id = FRAME_WORLD
        ps.header.stamp = t.to_msg()
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = FRAME_BASE
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        # Update the clamp reference BEFORE publishing so any message arriving
        # right after this TF clamps to a time we actually have.
        self._last_tf_stamp = t
        with self._pub_lock:
            self.pub_ego.publish(ps)
            self.tf.sendTransform(tf)

        # Frame-exact pairing: record this frame's local stamp, then release
        # any dets/lidar that were deferred waiting for exactly this frame.
        with self._pair_lock:
            self._ego_stamp_by_pts[e["timestamp_ns"]] = t
            self._ego_pts_order.append(e["timestamp_ns"])
            if len(self._ego_pts_order) > EGO_MAP_CAP:
                self._ego_stamp_by_pts.pop(self._ego_pts_order.popleft(), None)
        self._publish_due(self._collect_due())

    def _on_dets(self, sample) -> None:
        try:
            d = wire.decode_detections(_payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"detections decode failed: {ex}")
            return
        # Stamp with the SAME frame's ego TF (bridge shares one producer ts
        # across ego/dets per simulation frame); defer if it hasn't landed.
        self._publish_paired(
            d["timestamp_ns"], lambda t, _d=d: self._publish_dets(_d, t))

    def _publish_dets(self, d, t) -> None:
        """t = local stamp of this frame's ego TF (None -> clamped fallback)."""
        arr = Detection3DArray()
        arr.header.frame_id = FRAME_WORLD
        if t is not None:
            arr.header.stamp = t.to_msg()
        else:
            self._stamp_clamped(arr.header)

        markers = MarkerArray()
        for i, det in enumerate(d["detections"]):
            (px, py, pz), (qx, qy, qz, qw) = frames.ac_pos_rot_to_ros(
                det["world_pos"], det["world_rot"])
            # AC half-extents order (X=right, Y=up, Z=fwd) -> ROS (X=fwd, Y=right, Z=up)
            he = det["half_extents"]
            sx, sy, sz = 2.0 * he[2], 2.0 * he[0], 2.0 * he[1]  # length / width / height

            d3 = Detection3D()
            d3.bbox.center.position.x = px
            d3.bbox.center.position.y = py
            d3.bbox.center.position.z = pz
            d3.bbox.center.orientation.x = qx
            d3.bbox.center.orientation.y = qy
            d3.bbox.center.orientation.z = qz
            d3.bbox.center.orientation.w = qw
            d3.bbox.size.x = sx
            d3.bbox.size.y = sy
            d3.bbox.size.z = sz
            # vision_msgs ObjectHypothesisWithPose differs by ROS2 distro:
            #   Humble -> flat {id, score};  Jazzy/Iron+ -> nested {hypothesis.{class_id,score}}
            hyp = ObjectHypothesisWithPose()
            if hasattr(hyp, "hypothesis"):
                hyp.hypothesis.class_id = str(det["class_id"])
                hyp.hypothesis.score = float(det["score"])
            else:
                hyp.id = str(det["class_id"])
                hyp.score = float(det["score"])
            d3.results.append(hyp)
            arr.detections.append(d3)

            mk = Marker()
            mk.header = arr.header
            mk.ns = MARKER_NS
            mk.id = i
            mk.type = Marker.CUBE
            mk.action = Marker.ADD
            mk.pose.position.x = px
            mk.pose.position.y = py
            mk.pose.position.z = pz
            mk.pose.orientation.x = qx
            mk.pose.orientation.y = qy
            mk.pose.orientation.z = qz
            mk.pose.orientation.w = qw
            mk.scale.x = sx
            mk.scale.y = sy
            mk.scale.z = sz
            mk.color.r = 0.2
            mk.color.g = 0.9
            mk.color.b = 0.2
            mk.color.a = 0.35
            mk.lifetime.nanosec = MARKER_LIFETIME_NS
            markers.markers.append(mk)

        with self._pub_lock:
            self.pub_dets.publish(arr)
            self.pub_markers.publish(markers)

    def _on_lidar(self, sample) -> None:
        try:
            l = wire.decode_lidar(_payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"lidar decode failed: {ex}")
            return
        # The bridge forwards a scan inside the iteration that noticed it and
        # stamps it with that iteration's shared ts -> pairing attaches the
        # cloud to that frame's ego TF (best available frame alignment).
        self._publish_paired(
            l["timestamp_ns"], lambda t, _l=l: self._publish_lidar(_l, t))

        name = l["name"] or "lidar0"
        frame = f"{name}_frame"

        # Install pose from the ini (POSITION/FORWARD via SHM). The one-shot
        # diagnostic and the static TF below don't depend on the dynamic chain;
        # run them here (not deferred with the cloud).
        (px, py, pz), (qx, qy, qz, qw) = frames.install_pose_to_ros(
            l["install_pos"], l["install_fwd"])

        # One-shot diagnostic per lidar: name <-> ini install <-> frame pose <->
        # frame X direction (car fwd in base_link). Compare against the DEPLOYED
        # custom_lidars.ini to catch stale FORWARD values (fix-10).
        import math as _m
        if name not in getattr(self, "_lidar_diag_logged", set()):
            if not hasattr(self, "_lidar_diag_logged"):
                self._lidar_diag_logged = set()
            self._lidar_diag_logged.add(name)
            fxx = 1.0 - 2.0 * (qy * qy + qz * qz)   # frame X axis from quat
            fxy = 2.0 * (qx * qy + qz * qw)
            fxz = 2.0 * (qx * qz - qy * qw)
            ip, ff = l["install_pos"], l["install_fwd"]
            self.get_logger().info(
                "lidarDiag '{}' install=({:.2f},{:.2f},{:.2f}) fwd=({:.1f},{:.1f},{:.1f}) "
                "framePos=({:.2f},{:.2f},{:.2f}) frameX=({:.2f},{:.2f},{:.2f})".format(
                    name, ip[0], ip[1], ip[2], ff[0], ff[1], ff[2],
                    px, py, pz, fxx, fxy, fxz))

        # TF: base_link -> <name>_frame, directly from the ini install pose
        # (POSITION/FORWARD, carried through SHM+bridge). Deterministic and
        # eyeball-verifiable against custom_lidars.ini. The install pose is
        # body-fixed, i.e. CONSTANT -> /tf_static (latched), sent on first
        # sight / change as part of the FULL static set (see _send_static_tfs).
        pose_sig = ((px, py, pz), (qx, qy, qz, qw))
        cached = self._static_tf_cache.get(name)
        if cached is None or cached[0] != pose_sig:
            tf = TransformStamped()
            tf.header.frame_id = FRAME_BASE
            tf.child_frame_id = frame
            tf.header.stamp = self._now().to_msg()
            tf.transform.translation.x = px
            tf.transform.translation.y = py
            tf.transform.translation.z = pz
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._static_tf_cache[name] = (pose_sig, tf)
            self._send_static_tfs()

    def _send_static_tfs(self) -> None:
        """Publish the COMPLETE static set in one /tf_static message.

        /tf_static is latched with depth 1: only the last sample is delivered
        to late-joining subscribers. Sending per-lidar transforms separately
        left RViz2 (started after the relay) with a single lidar's static TF,
        silently breaking the other lidars' chains (clouds stuck in the
        message filter, "queue is full"). One list publish keeps the latched
        sample always-complete.
        """
        all_tfs = [tf for _, tf in self._static_tf_cache.values()]
        if not all_tfs:
            return
        with self._pub_lock:
            self.tf_static.sendTransform(all_tfs)

    def _publish_lidar(self, l, t) -> None:
        """Publish the scan; t = local stamp of this frame's ego TF
        (None -> clamped fallback)."""
        name = l["name"] or "lidar0"
        frame = f"{name}_frame"
        topic = f"/ac/lidar/{name}/points"

        if name not in self._pc_pubs:
            with self._pub_lock:
                self._pc_pubs[name] = self.create_publisher(PointCloud2, topic, 10)

        # Hits stay in the sensor-local frame (x fwd, y left, z up - ROS-style axes,
        # same as a real lidar driver). LidarHit layout: range@0, x@4, y@8, z@12,
        # intensity@16 f32, instance_id@20 u32, LE, 24B - passed through unchanged.
        fields = [
            _field("range", PointField.FLOAT32, 0),
            _field("x", PointField.FLOAT32, 4),
            _field("y", PointField.FLOAT32, 8),
            _field("z", PointField.FLOAT32, 12),
            _field("intensity", PointField.FLOAT32, 16),
            _field("instance_id", PointField.UINT32, 20),
        ]
        pc = PointCloud2()
        pc.header = Header()
        pc.header.frame_id = frame
        if t is not None:
            pc.header.stamp = t.to_msg()
        else:
            self._stamp_clamped(pc.header)
        pc.height = 1
        pc.width = l["point_count"]
        pc.fields = fields
        pc.is_bigendian = False
        pc.point_step = wire.LIDAR_HIT_SIZE
        pc.row_step = pc.point_step * pc.width
        pc.is_dense = True
        pc.data = bytes(l["hits"])   # rclpy accepts bytes
        with self._pub_lock:
            self._pc_pubs[name].publish(pc)

    # ---- camera images (ac/camera/<cam>/<suffix> -> sensor_msgs/Image) ----
    # Suffixes carried as raw pixel payloads today. camera_info will be relayed
    # when the bridge starts sending CameraInfoPayload (future phase).
    _IMAGE_SUFFIXES = ("image_raw", "image_depth", "image_classes")
    _BYTES_PER_PIXEL = {"rgba8": 4, "bgra8": 4, "32FC1": 4, "mono8": 1}

    def _on_camera(self, sample) -> None:
        key = str(sample.key_expr)   # ac/camera/<cam>/<suffix>
        parts = key.split("/")
        if len(parts) != 4 or parts[0] != "ac" or parts[1] != "camera" or not parts[2]:
            self.get_logger().warn(f"unexpected camera key '{key}', ignored")
            return
        cam, suffix = parts[2], parts[3]
        if suffix not in self._IMAGE_SUFFIXES:
            if key not in getattr(self, "_cam_ignored", set()):
                if not hasattr(self, "_cam_ignored"):
                    self._cam_ignored = set()
                self._cam_ignored.add(key)
                self.get_logger().info(f"camera key '{key}' not relayed yet, ignored")
            return

        try:
            img = wire.decode_image(_payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"image decode failed ({key}): {ex}")
            return
        ros_enc = wire.ENCODING_TO_ROS.get(img["encoding"])
        if ros_enc is None or img["mode"] != 0:
            # ENC_H264 / MODE_COMPRESSED -> sensor_msgs/CompressedImage, future phase
            self.get_logger().warn(f"image encoding={img['encoding']} mode={img['mode']} "
                                   f"not relayed yet ({key})")
            return
        expected = img["width"] * img["height"] * self._BYTES_PER_PIXEL[ros_enc]
        if img["data_len"] != expected:
            self.get_logger().warn(f"image size mismatch ({key}): data_len={img['data_len']} "
                                   f"but {img['width']}x{img['height']} {ros_enc} needs {expected}")
            return

        topic = f"/{key}"
        if topic not in self._img_pubs:
            with self._pub_lock:
                self._img_pubs[topic] = self.create_publisher(
                    Image, topic, IMAGE_QOS)
            self.get_logger().info(f"image publisher '{topic}' "
                                   f"({img['width']}x{img['height']} {ros_enc})")

        msg = Image()
        msg.header = Header()
        msg.header.frame_id = f"{cam}_frame"
        self._stamp_clamped(msg.header)
        msg.height = img["height"]
        msg.width = img["width"]
        msg.encoding = ros_enc
        msg.is_bigendian = 0
        msg.step = img["data_len"] // img["height"]
        msg.data = img["data"]   # raw bytes passthrough, same trick as the lidar hits
        with self._pub_lock:
            self._img_pubs[topic].publish(msg)

    def close(self) -> None:
        try:
            self._pair_timer.cancel()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._session.close()
        except Exception:  # noqa: BLE001
            pass


def main(args=None):
    rclpy.init(args=args)
    node = AcRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
