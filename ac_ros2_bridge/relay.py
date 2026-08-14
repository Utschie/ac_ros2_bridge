"""ac_ros2_bridge relay node.

Subscribes Zenoh keys 'ac/ego_pose' and 'ac/detections_3d' (produced by the Windows
ac_sim_bridge), decodes the compact binary payloads (see shared/zenoh_payload.h +
wire.py), and republishes as standard ROS2 messages:
  ac/ego_pose      -> geometry_msgs/PoseStamped '/ac/ego_pose' + TF (ac_world -> base_link)
  ac/detections_3d -> vision_msgs/Detection3DArray '/ac/detections_3d'
                   + visualization_msgs/MarkerArray '/ac/detections_3d_markers'
                     (semi-transparent cubes; RViz2 has no built-in Detection3DArray
                      display, MarkerArray works everywhere)

Coordinate frames: AC's world frame is Y-up left-handed (X right, Y up, Z forward);
ROS is Z-up right-handed (X forward, Y left, Z up). Incoming AC positions/rotations
are remapped here (_ac_to_ros), so everything is published directly in ROS convention
under frame 'ac_world' - no static transform needed. With the car axis-aligned in AC
the result is the identity orientation (car upright, facing +X).

vision_msgs field access auto-detects ROS2 Humble vs Jazzy/Iron+.
"""

import math
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

from . import wire

FRAME_WORLD = "ac_world"
FRAME_BASE = "base_link"
MARKER_NS = "ac_detections"
MARKER_LIFETIME_NS = 500_000_000  # 0.5s auto-expire


def _field(name, datatype, offset):
    from sensor_msgs.msg import PointField as _PF
    f = _PF()
    f.name = name
    f.datatype = datatype
    f.offset = offset
    f.count = 1
    return f


def _ac_to_ros(v):
    """Map an AC-world vector to ROS-world. AC(Z=fwd)->ROS(X), AC(Y=up)->ROS(Z), AC(X=right)->ROS(-Y)."""
    x, y, z = v
    return (z, -x, y)


def _cols_to_quat(c0, c1, c2):
    """c0,c1,c2 = column vectors of a right-handed rotation matrix R (R's columns are
    the images of the frame's X,Y,Z axes). Returns quaternion (x, y, z, w)."""
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


def _ac_rot_to_ros_quat(c0, c1, c2):
    """AC rotation columns c0,c1,c2 = AC-local X(right)/Y(up)/Z(fwd) in AC world.
    Convert each to ROS world and re-express as a ROS body frame (X=fwd, Y=left=-right,
    Z=up) - a proper right-handed rotation."""
    fwd = _ac_to_ros(c2)
    left = tuple(-t for t in _ac_to_ros(c0))
    up = _ac_to_ros(c1)
    return _cols_to_quat(fwd, left, up)


def _rowmajor9_to_cols(r):
    """r = 9 floats, ROW-major 3x3 R with world = R @ sensor_local.
    Returns (col0, col1, col2) - the sensor's local X/Y/Z axes in world."""
    return ((r[0], r[3], r[6]), (r[1], r[4], r[7]), (r[2], r[5], r[8]))


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
        self.tf = TransformBroadcaster(self)

        self._sub_ego = self._session.declare_subscriber("ac/ego_pose", self._on_ego)
        self._sub_dets = self._session.declare_subscriber("ac/detections_3d", self._on_dets)
        self._sub_lidar = self._session.declare_subscriber("ac/lidar/*/points", self._on_lidar)
        self.get_logger().info("ac_relay ready: subscribing ac/ego_pose + ac/detections_3d "
                               "+ ac/lidar/*/points (AC->ROS axis remap enabled)")

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
        px, py, pz = _ac_to_ros(w[12:15])
        qx, qy, qz, qw = _ac_rot_to_ros_quat(w[0:3], w[4:7], w[8:11])

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
            px, py, pz = _ac_to_ros(det["world_pos"])
            qx, qy, qz, qw = _ac_rot_to_ros_quat(
                det["world_rot"][0:3], det["world_rot"][3:6], det["world_rot"][6:9])
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

        # Sensor-local hits pass through byte-for-byte: LidarHit layout
        # (range@0, x@4, y@8, z@12, intensity@16 f32, instance_id@20 u32, LE, 24B)
        # maps directly onto PointCloud2 fields with point_step=24.
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

        # TF: ac_world -> <name>_frame (sensor pose, AC->ROS remapped)
        px, py, pz = _ac_to_ros(l["world_pos"])
        c0, c1, c2 = _rowmajor9_to_cols(l["world_rot"])
        qx, qy, qz, qw = _ac_rot_to_ros_quat(c0, c1, c2)
        tf = TransformStamped()
        tf.header.frame_id = FRAME_WORLD
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
