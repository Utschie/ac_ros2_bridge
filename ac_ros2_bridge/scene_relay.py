"""SceneRelay - ego pose / 3D detections / lidar domain module.

Moved verbatim from relay.py (M5 modularization; behavior unchanged):
  ac/ego_pose       -> geometry_msgs/PoseStamped '/ac/ego_pose'
                       + dynamic TF ac_world -> base_link (and feeds the
                       context's pairing/clamp references)
  ac/detections_3d  -> vision_msgs/Detection3DArray '/ac/detections_3d'
                       + visualization_msgs/MarkerArray '/ac/detections_3d_markers'
                       (RViz2 has no built-in Detection3DArray display)
  ac/lidar/*/points -> sensor_msgs/PointCloud2 '/ac/lidar/<name>/points'
                       (points stay in the sensor frame, like a real lidar driver)
                       + static TF base_link -> <name>_frame (via the context's
                       latched full-set sender)

Cross-cutting policies (clock, pairing, locks, TF) come from RelayContext.
"""

import time

from geometry_msgs.msg import PoseStamped, TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from . import frames
from . import wire

FRAME_WORLD = "ac_world"
FRAME_BASE = "base_link"
MARKER_NS = "ac_detections"
MARKER_LIFETIME_NS = 500_000_000  # 0.5s auto-expire


def _field(name, datatype, offset):
    f = PointField()
    f.name = name
    f.datatype = datatype
    f.offset = offset
    f.count = 1
    return f


class SceneRelay:
    def __init__(self, ctx):
        self.ctx = ctx
        node = ctx.node
        self.pub_ego = node.create_publisher(PoseStamped, "/ac/ego_pose", 10)
        self.pub_dets = node.create_publisher(Detection3DArray, "/ac/detections_3d", 10)
        self.pub_markers = node.create_publisher(MarkerArray, "/ac/detections_3d_markers", 10)
        self._pc_pubs = {}   # lidar name -> PointCloud2 publisher (lazy)
        self._lidar_diag_logged = set()
        self._last_mtx_log = 0.0

    def attach(self, session) -> None:
        session.declare_subscriber("ac/ego_pose", self._on_ego)
        session.declare_subscriber("ac/detections_3d", self._on_dets)
        session.declare_subscriber("ac/lidar/*/points", self._on_lidar)

    # ---- ego pose + dynamic TF ----

    def _on_ego(self, sample) -> None:
        try:
            e = wire.decode_ego(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.ctx.log.warn(f"ego decode failed: {ex}")
            return
        w = e["world"]
        self.ctx.last_ego_world = list(w)   # cached for base_link->sensor TF composition
        # Debug: raw BODYTR matrix triples, throttled to ~1 line / 2s.
        # Park the car straight on a straight and read one line: the triple
        # that matches the car's heading decides the storage orientation.
        now_t = time.monotonic()
        if now_t - self._last_mtx_log > 2.0:
            self._last_mtx_log = now_t
            self.ctx.log.info(
                "egoMatrix t=[{:.2f},{:.2f},{:.2f}] r0=[{:.3f},{:.3f},{:.3f}] "
                "r1=[{:.3f},{:.3f},{:.3f}] r2=[{:.3f},{:.3f},{:.3f}]".format(
                    w[12], w[13], w[14],
                    w[0], w[1], w[2], w[4], w[5], w[6], w[8], w[9], w[10]))
        (px, py, pz), (qx, qy, qz, qw) = frames.ac_body_pose_to_ros(w)

        # One local-time sample per callback; the message and its TF share it.
        t = self.ctx.now()
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
        self.ctx.last_tf_stamp = t
        with self.ctx.pub_lock:
            self.pub_ego.publish(ps)
            self.ctx.tf.sendTransform(tf)

        # Frame-exact pairing: record this frame's local stamp, then release
        # any dets/lidar that were deferred waiting for exactly this frame.
        self.ctx.record_ego_frame(e["timestamp_ns"], t)

    # ---- 3D detections ----

    def _on_dets(self, sample) -> None:
        try:
            d = wire.decode_detections(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.ctx.log.warn(f"detections decode failed: {ex}")
            return
        # Stamp with the SAME frame's ego TF (bridge shares one producer ts
        # across ego/dets per simulation frame); defer if it hasn't landed.
        self.ctx.publish_paired(
            d["timestamp_ns"], lambda t, _d=d: self._publish_dets(_d, t))

    def _publish_dets(self, d, t) -> None:
        """t = local stamp of this frame's ego TF (None -> clamped fallback)."""
        arr = Detection3DArray()
        arr.header.frame_id = FRAME_WORLD
        if t is not None:
            arr.header.stamp = t.to_msg()
        else:
            self.ctx.stamp_clamped(arr.header)

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

        with self.ctx.pub_lock:
            self.pub_dets.publish(arr)
            self.pub_markers.publish(markers)

    # ---- lidar ----

    def _on_lidar(self, sample) -> None:
        try:
            l = wire.decode_lidar(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.ctx.log.warn(f"lidar decode failed: {ex}")
            return
        # The bridge forwards a scan inside the iteration that noticed it and
        # stamps it with that iteration's shared ts -> pairing attaches the
        # cloud to that frame's ego TF (best available frame alignment).
        self.ctx.publish_paired(
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
        if name not in self._lidar_diag_logged:
            self._lidar_diag_logged.add(name)
            fxx = 1.0 - 2.0 * (qy * qy + qz * qz)   # frame X axis from quat
            fxy = 2.0 * (qx * qy + qz * qw)
            fxz = 2.0 * (qx * qz - qy * qw)
            ip, ff = l["install_pos"], l["install_fwd"]
            self.ctx.log.info(
                "lidarDiag '{}' install=({:.2f},{:.2f},{:.2f}) fwd=({:.1f},{:.1f},{:.1f}) "
                "framePos=({:.2f},{:.2f},{:.2f}) frameX=({:.2f},{:.2f},{:.2f})".format(
                    name, ip[0], ip[1], ip[2], ff[0], ff[1], ff[2],
                    px, py, pz, fxx, fxy, fxz))

        # TF: base_link -> <name>_frame, directly from the ini install pose
        # (POSITION/FORWARD, carried through SHM+bridge). Deterministic and
        # eyeball-verifiable against custom_lidars.ini. The install pose is
        # body-fixed, i.e. CONSTANT -> /tf_static via the context (latched,
        # FULL set per message, re-sent only if a pose ever changes).
        pose_sig = ((px, py, pz), (qx, qy, qz, qw))

        def build_tf():
            tf = TransformStamped()
            tf.header.frame_id = FRAME_BASE
            tf.child_frame_id = frame
            tf.header.stamp = self.ctx.now().to_msg()
            tf.transform.translation.x = px
            tf.transform.translation.y = py
            tf.transform.translation.z = pz
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            return tf

        self.ctx.update_static_tf(name, pose_sig, build_tf)

    def _publish_lidar(self, l, t) -> None:
        """Publish the scan; t = local stamp of this frame's ego TF
        (None -> clamped fallback)."""
        name = l["name"] or "lidar0"
        frame = f"{name}_frame"
        topic = f"/ac/lidar/{name}/points"

        if name not in self._pc_pubs:
            with self.ctx.pub_lock:
                self._pc_pubs[name] = self.ctx.node.create_publisher(
                    PointCloud2, topic, 10)

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
            self.ctx.stamp_clamped(pc.header)
        pc.height = 1
        pc.width = l["point_count"]
        pc.fields = fields
        pc.is_bigendian = False
        pc.point_step = wire.LIDAR_HIT_SIZE
        pc.row_step = pc.point_step * pc.width
        pc.is_dense = True
        pc.data = bytes(l["hits"])   # rclpy accepts bytes
        with self.ctx.pub_lock:
            self._pc_pubs[name].publish(pc)
