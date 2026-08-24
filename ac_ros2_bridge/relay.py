"""ac_ros2_bridge relay node (entry point / wiring only).

Subscribes Zenoh keys from the Windows ac_sim_bridge, decodes the compact
binary payloads (wire.py, matches shared/zenoh_payload.h), and republishes as
standard ROS2 messages. Module layout (M5):

  relay.py         THIS FILE - node bootstrap, zenoh session, domain wiring
  relay_context.py RelayContext - cross-cutting policies: local-clock
                  stamping + ego-TF clamp, frame-exact pairing (dets/lidar
                  <-> same-frame ego), publish lock (zenoh threads), dynamic
                  TF + latched full-set static TF. /clock + use_sim_time
                  will land there (swap the clock source only).
  scene_relay.py   SceneRelay  - ego_pose / detections_3d / lidar domain
  image_relay.py   CameraRelay - camera image domain
  control_relay.py ControlRelay - vehicle control downlink (ROS->zenoh,
                  the FIRST reverse-direction module) + state feedback up
  wire.py          pure payload encode/decode (+ payload_bytes helper)
  frames.py        ALL AC->ROS coordinate math (see its docstring)

Topics:
  ac/ego_pose       -> '/ac/ego_pose' (PoseStamped) + TF ac_world->base_link
  ac/detections_3d  -> '/ac/detections_3d' (Detection3DArray)
                       + '/ac/detections_3d_markers' (MarkerArray; RViz2 has
                       no built-in Detection3DArray display)
  ac/lidar/*/points -> '/ac/lidar/<name>/points' (PointCloud2, sensor frame)
                       + static TF base_link-><name>_frame
  ac/camera/<n>/<s> -> '/ac/camera/<n>/<s>' (Image; s = image_raw |
                       image_depth | image_classes)
  /ac/vehicle_cmd   -> zenoh 'ac/vehicle_cmd' (ac_msgs/VehicleControl,
                       RELIABLE; Windows bridge drives the game - shm hook
                       or vJoy per its bridge_control.ini, ROS side unaware)
  ac/vehicle_state  -> '/ac/vehicle_state' (ac_msgs/VehicleState:
                       gear / speed / GO race-start flag)

TF tree: ac_world -> base_link -> {front,rear,...}_frame. Points/images stay in
their sensor frames; RViz2 and downstream consumers resolve via the TF chain.

Timestamp policy (fixes RViz2's intermittent "No Transform ..." flicker):
ALL outbound headers are stamped with the relay's LOCAL ROS clock at receive
time, in one clock domain. The producer's wall-clock stamps (Windows, from
ac_sim_bridge) are NOT propagated: the two machines' clocks are not synced, so
a forward skew made RViz2 see future-stamped messages and tf2 refused to
extrapolate. Messages that depend on the ac_world->base_link chain are
additionally clamped to the newest ego-TF time, and dets/lidar are frame-paired
to the exact ego TF of their own simulation frame (the bridge stamps one frame
with one shared producer timestamp). Details in relay_context.py.
Sim time (/clock + use_sim_time) is the documented future upgrade; when it
lands, only the clock source in relay_context.py changes.

Adding a new data type = write a new domain module taking (ctx) with an
attach(session) - then wire it here. No policy code gets duplicated.

vision_msgs field access auto-detects ROS2 Humble vs Jazzy/Iron+.
"""

import os

import rclpy
from rclpy.node import Node
import zenoh

from .relay_context import RelayContext
from .scene_relay import SceneRelay
from .image_relay import CameraRelay
from .control_relay import ControlRelay


class AcRelay(Node):
    def __init__(self):
        super().__init__("ac_relay")

        cfg = zenoh.Config()
        zcfg = os.environ.get("ZENOH_CONFIG")
        if zcfg and os.path.isfile(zcfg):
            cfg = zenoh.Config.from_file(zcfg)
        self._session = zenoh.open(cfg)

        # Cross-cutting policies, shared by every domain below.
        self._ctx = RelayContext(self)

        # Domain modules (subscribe in attach(); publish their own topics).
        self._scene = SceneRelay(self._ctx)
        self._image = CameraRelay(self._ctx)
        self._control = ControlRelay(self._ctx)
        self._scene.attach(self._session)
        self._image.attach(self._session)
        self._control.attach(self._session)

        self.get_logger().info("ac_relay ready: subscribing ac/ego_pose + ac/detections_3d "
                               "+ ac/lidar/*/points + ac/camera/** + ac/vehicle_state "
                               "+ ac/expert_ctrl; forwarding /ac/vehicle_cmd -> zenoh "
                               "(AC->ROS axis remap in frames.py)")

    def close(self) -> None:
        self._ctx.close()
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
