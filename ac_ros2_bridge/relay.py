"""ac_ros2_bridge relay node.

Subscribes Zenoh keys 'ac/ego_pose' and 'ac/detections_3d' (produced by the Windows
ac_sim_bridge), decodes the compact binary payloads (see shared/zenoh_payload.h +
wire.py), and republishes as standard ROS2 messages:
  ac/ego_pose     -> geometry_msgs/PoseStamped '/ac/ego_pose' + TF (ac_world -> base_link)
  ac/detections_3d -> vision_msgs/Detection3DArray '/ac/detections_3d'

Note: AC's world frame is likely Y-up while ROS is Z-up. This first version publishes
in frame 'ac_world' using AC's matrix as-is; if the car/OBBs look tilted in RViz, add a
static axis transform (documented in README).

vision_msgs field access auto-detects ROS2 Humble vs Jazzy/Iron+.
"""

import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from tf2_ros import TransformBroadcaster
import zenoh

from . import wire

FRAME_WORLD = "ac_world"
FRAME_BASE = "base_link"


def _cols_to_quat(c0, c1, c2):
    """c0,c1,c2 = world-space directions of the body's local X,Y,Z axes (columns of R).
    Returns quaternion (x, y, z, w). Standard rotation-matrix -> quaternion."""
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
        self.tf = TransformBroadcaster(self)

        self._sub_ego = self._session.declare_subscriber("ac/ego_pose", self._on_ego)
        self._sub_dets = self._session.declare_subscriber("ac/detections_3d", self._on_dets)
        self.get_logger().info("ac_relay ready: subscribing ac/ego_pose + ac/detections_3d")

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
        qx, qy, qz, qw = _cols_to_quat(w[0:3], w[4:7], w[8:11])

        ps = PoseStamped()
        ps.header.frame_id = FRAME_WORLD
        self._stamp(ps.header, e["timestamp_ns"])
        ps.pose.position.x = w[12]
        ps.pose.position.y = w[13]
        ps.pose.position.z = w[14]
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pub_ego.publish(ps)

        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = FRAME_BASE
        tf.transform.translation.x = w[12]
        tf.transform.translation.y = w[13]
        tf.transform.translation.z = w[14]
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
        for det in d["detections"]:
            qx, qy, qz, qw = _cols_to_quat(
                det["world_rot"][0:3], det["world_rot"][3:6], det["world_rot"][6:9])
            d3 = Detection3D()
            d3.bbox.center.position.x = det["world_pos"][0]
            d3.bbox.center.position.y = det["world_pos"][1]
            d3.bbox.center.position.z = det["world_pos"][2]
            d3.bbox.center.orientation.x = qx
            d3.bbox.center.orientation.y = qy
            d3.bbox.center.orientation.z = qz
            d3.bbox.center.orientation.w = qw
            d3.bbox.size.x = 2.0 * det["half_extents"][0]
            d3.bbox.size.y = 2.0 * det["half_extents"][1]
            d3.bbox.size.z = 2.0 * det["half_extents"][2]
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
        self.pub_dets.publish(arr)

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
