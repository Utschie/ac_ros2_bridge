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

TF tree: ac_world -> base_link -> {front,rear,...}_frame. Points/images stay in
their sensor frames; RViz2 and downstream consumers resolve via the TF chain.

vision_msgs field access auto-detects ROS2 Humble vs Jazzy/Iron+.
"""

import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import zenoh

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
        self._last_ego_world = None   # 16-float AC BODYTR world matrix (for TF composition)
        self.tf = TransformBroadcaster(self)

        self._sub_ego = self._session.declare_subscriber("ac/ego_pose", self._on_ego)
        self._sub_dets = self._session.declare_subscriber("ac/detections_3d", self._on_dets)
        self._sub_lidar = self._session.declare_subscriber("ac/lidar/*/points", self._on_lidar)
        self.get_logger().info("ac_relay ready: subscribing ac/ego_pose + ac/detections_3d "
                               "+ ac/lidar/*/points (AC->ROS axis remap in frames.py)")

    def _stamp(self, header, ts_ns: int) -> None:
        header.stamp.sec = ts_ns // 1_000_000_000
        header.stamp.nanosec = ts_ns % 1_000_000_000

    def _on_ego(self, sample) -> None:
        try:
            e = wire.decode_ego(_payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"ego decode failed: {ex}")
            return
        w = e["world"]
        self._last_ego_world = list(w)   # cached for base_link->sensor TF composition
        (px, py, pz), (qx, qy, qz, qw) = frames.ac_body_pose_to_ros(w)

        ps = PoseStamped()
        ps.header.frame_id = FRAME_WORLD
        self._stamp(ps.header, e["timestamp_ns"])
        ps.pose.position.x = px
        ps.pose.position.y = py
        ps.pose.position.z = pz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pub_ego.publish(ps)

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
        self.tf.sendTransform(tf)

    def _on_dets(self, sample) -> None:
        try:
            d = wire.decode_detections(_payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"detections decode failed: {ex}")
            return
        arr = Detection3DArray()
        arr.header.frame_id = FRAME_WORLD
        self._stamp(arr.header, d["timestamp_ns"])

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

        self.pub_dets.publish(arr)
        self.pub_markers.publish(markers)

    def _on_lidar(self, sample) -> None:
        try:
            l = wire.decode_lidar(_payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(f"lidar decode failed: {ex}")
            return
        name = l["name"] or "lidar0"
        frame = f"{name}_frame"
        topic = f"/ac/lidar/{name}/points"

        if name not in self._pc_pubs:
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
        self._stamp(pc.header, l["timestamp_ns"])
        pc.height = 1
        pc.width = l["point_count"]
        pc.fields = fields
        pc.is_bigendian = False
        pc.point_step = wire.LIDAR_HIT_SIZE
        pc.row_step = pc.point_step * pc.width
        pc.is_dense = True
        pc.data = bytes(l["hits"])   # rclpy accepts bytes
        self._pc_pubs[name].publish(pc)

        # TF: base_link -> <name>_frame, composed from world poses:
        # T(base->sensor) = inv(ego_world) . T(world->sensor).
        if self._last_ego_world is None:
            return   # no ego pose yet; skip TF until ego arrives
        t_rel, (s_x, s_y, s_z) = frames.compose_base_to_sensor(
            self._last_ego_world, l["world_pos"], l["world_rot"])
        px, py, pz = frames.ac_vec_to_ros(t_rel)
        qx, qy, qz, qw = frames.sensor_axes_to_ros_quat(s_x, s_y, s_z)
        tf = TransformStamped()
        tf.header.frame_id = FRAME_BASE
        tf.child_frame_id = frame
        self._stamp(tf.header, l["timestamp_ns"])
        tf.transform.translation.x = px
        tf.transform.translation.y = py
        tf.transform.translation.z = pz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf.sendTransform(tf)

    def close(self) -> None:
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
