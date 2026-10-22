"""ControlRelay - vehicle control downlink + state feedback domain module.

The FIRST reverse-direction module (ROS -> zenoh), opposite to scene/image:

  /ac/vehicle_cmd  (ac_msgs/VehicleControl, RELIABLE)  -> zenoh 'ac/vehicle_cmd'
      On the Windows side ac_sim_bridge subscribes the key and drives the game
      via the configured backend (bridge_control.ini MODE=shm|vjoy); the ROS
      side is unaware of which backend is in use.
  zenoh 'ac/vehicle_state'  ->  /ac/vehicle_state (ac_msgs/VehicleState)
      gear / speed / GO flag fed back from the game (plugin feedback block).

Threading model (differs per direction):
  _on_cmd runs on the rclpy executor thread; zenoh session.put is thread-safe
  -> no lock needed.
  _on_state runs on a zenoh callback thread; rclpy publish is NOT thread-safe
  -> always under ctx.pub_lock (relay_context policy 3).

⚠ CODING RULES for future edits of this module (from a past setter-bug
post-mortem):
  1. NEVER assign large arrays through rclpy setters (uint8[] data etc.):
     Humble's generated setters run a per-element Python assertion loop
     (≈1-3 s per MB, once masqueraded as network congestion). Direct-write
     the private slot instead:  msg._data = np.frombuffer(raw, np.uint8)
     (rclpy's C serializer consumes ndarray natively, zero-copy).
     VehicleControl/VehicleState are all-scalar today; if any array field
     is ever added, rule 1 applies immediately.
  2. zenoh callbacks publishing ROS = only under ctx.pub_lock.
  3. When reception "feels slow", suspect this process first (py-spy dump),
     then the network.
"""

from rclpy.qos import QoSProfile, ReliabilityPolicy

from ac_msgs.msg import VehicleControl, VehicleState, GameControl, GameState, ExpertCtrl

from . import wire


class ControlRelay:
    def __init__(self, ctx):
        self.ctx = ctx
        node = ctx.node

        # Commands must not drop: RELIABLE (images use DROP on the zenoh side;
        # control is the opposite trade-off - tiny messages, loss intolerant).
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._sub_cmd = node.create_subscription(
            VehicleControl, "/ac/vehicle_cmd", self._on_cmd, cmd_qos)
        self._sub_game = node.create_subscription(
            GameControl, "/ac/game_cmd", self._on_game_cmd, cmd_qos)
        self._pub_state = node.create_publisher(VehicleState, "/ac/vehicle_state", 10)
        self._pub_game_state = node.create_publisher(GameState, "/ac/game_state", 10)
        # Expert actions per frame (BEST_EFFORT depth 50 — one per scene
        # frame; training/recording consumers can lose one or two, no
        # reliability required)
        self._pub_expert = node.create_publisher(ExpertCtrl, "/ac/expert_ctrl", 50)

        self._session = None
        self._cmd_count = 0
        self._last_diag = 0.0
        self._bad_payload_logged = 0

    def attach(self, session) -> None:
        self._session = session
        session.declare_subscriber("ac/vehicle_state", self._on_state)
        session.declare_subscriber("ac/game_state", self._on_game_state)
        session.declare_subscriber("ac/expert_ctrl", self._on_expert)

    # ---- ROS -> zenoh (rclpy executor thread; session.put is thread-safe) ----

    def _on_cmd(self, msg: VehicleControl) -> None:
        if self._session is None:
            return
        payload = wire.encode_control(
            steering=msg.steering,
            throttle=msg.throttle,
            brake=msg.brake,
            clutch=msg.clutch,
            hand_brake=msg.hand_brake,
            requested_gear_index=msg.requested_gear_index,
            gear_up=msg.gear_up,
            gear_dn=msg.gear_dn,
            drs=msg.drs,
            kers=msg.kers,
            handbrake_on=msg.handbrake_on,
            override_enabled=msg.override_enabled,
        )
        self._session.put("ac/vehicle_cmd", payload)

        # ~1Hz one-liner so a live publisher is visible in the console
        self._cmd_count += 1
        import time as _t
        now = _t.monotonic()
        if now - self._last_diag > 1.0:
            self._last_diag = now
            self.ctx.log.info(
                f"vehicle_cmd -> zenoh  n={self._cmd_count} "
                f"steer={msg.steering:+.2f} thr={msg.throttle:.2f} "
                f"brk={msg.brake:.2f} gear={msg.requested_gear_index} "
                f"override={'on' if msg.override_enabled else 'off'}")

    # ---- ROS -> zenoh: game session control (GameControl, 4B) ----

    def _on_game_cmd(self, msg: GameControl) -> None:
        if self._session is None:
            return
        payload = wire.encode_game_control(
            start_race=msg.start_race,
            pause=msg.pause,
            resume=msg.resume,
            restart_session=msg.restart_session,
            skip_session=msg.skip_session,
        )
        self._session.put("ac/game_cmd", payload)
        self.ctx.log.info(
            f"game_cmd -> zenoh  start={int(msg.start_race)} "
            f"pause={int(msg.pause)} resume={int(msg.resume)} "
            f"restart={int(msg.restart_session)} skip={int(msg.skip_session)}")

    # ---- zenoh -> ROS (zenoh callback thread -> publish under pub_lock) ----

    def _on_state(self, sample) -> None:
        try:
            v = wire.decode_vehicle_state(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self._bad_payload_logged += 1
            if self._bad_payload_logged <= 5 or self._bad_payload_logged % 100 == 0:
                self.ctx.log.warn(f"vehicle_state decode failed ({ex}), "
                                  f"count={self._bad_payload_logged}")
            return

        m = VehicleState()
        m.header.stamp = self.ctx.now().to_msg()
        m.gear = v["gear"]
        m.speed = v["speed"]
        m.flags = v["flags"]
        with self.ctx.pub_lock:
            self._pub_state.publish(m)

    def _on_game_state(self, sample) -> None:
        try:
            g = wire.decode_game_state(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self.ctx.log.warn(f"game_state decode failed: {ex}")
            return
        m = GameState()
        m.header.stamp = self.ctx.now().to_msg()
        m.session_type = g["session_type"]
        m.race_live = (g["flags"] & 0x1) != 0
        m.countdown_lights = (g["flags"] & 0x2) != 0
        with self.ctx.pub_lock:
            self._pub_game_state.publish(m)

    def _on_expert(self, sample) -> None:
        # Expert actions for this scene frame (frame_id pairs with
        # /ac/ego_pose)
        try:
            e = wire.decode_expert_ctrl(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            self._bad_payload_logged += 1
            if self._bad_payload_logged <= 5 or self._bad_payload_logged % 100 == 0:
                self.ctx.log.warn(f"expert_ctrl decode failed ({ex}), "
                                  f"count={self._bad_payload_logged}")
            return
        m = ExpertCtrl()
        m.header.stamp = self.ctx.now().to_msg()
        m.frame_id = e["frame_id"]
        m.steer = e["steer"]
        m.throttle = e["throttle"]
        m.brake = e["brake"]
        m.clutch = e["clutch"]
        m.hand_brake = e["hand_brake"]
        m.gear_up = e["gear_up"]
        m.gear_dn = e["gear_dn"]
        m.override = bool(e["override_active"])
        with self.ctx.pub_lock:
            self._pub_expert.publish(m)
