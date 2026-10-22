"""TelemetryRelay - acpmf telemetry/timing/static + run_state domain module.

zenoh -> ROS (zenoh callback threads; publish only under ctx.pub_lock):

  ac/telemetry        -> /ac/telemetry        (ac_msgs/Telemetry, RELIABLE 50)
                         every scene frame (~game fps); frame_id pairs with
                         /ac/ego_pose exactly. Wheel arrays are FL/FR/RL/RR;
                         velocity / tyre_contact_* are acpmf physics-world
                         (Y-up) - see Telemetry.msg comments for conversion.
  ac/timing           -> /ac/timing           (ac_msgs/Timing, RELIABLE 50)
                         same cadence, published when the graphics snapshot
                         is valid (tears are skipped upstream).
  ac/telemetry_static -> /ac/telemetry_static (latched TRANSIENT_LOCAL; fires
                         on session epoch change - late joiners immediately
                         get the car/track constants snapshot)
  ac/run_state        -> /ac/run_state        (ac_msgs/RunState, RELIABLE 10;
                         event-driven, a handful per session)

Field mapping: wire.py decodes camelCase acpmf names while messages are
snake_case; wire.wire_key_for() resolves the names (a few acpmf ALL-CAPS
suffixes need an explicit overrides table, kept in wire.py). Missing wire
keys log once instead of crashing.
"""

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from ac_msgs.msg import Telemetry, Timing, TelemetryStatic, RunState

from . import wire


class TelemetryRelay:
    def __init__(self, ctx):
        self.ctx = ctx
        node = ctx.node

        # High-rate streams (one per scene frame ~ game fps): RELIABLE
        # depth 50 (free on loopback, compatible with ros2 topic echo /
        # RViz default subscriptions — same reasoning as the images)
        stream_qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._pub_tel = node.create_publisher(Telemetry, "/ac/telemetry", stream_qos)
        self._pub_timing = node.create_publisher(Timing, "/ac/timing", stream_qos)
        # Quasi-static snapshot: latched, late joiners immediately get
        # the most recent copy
        self._pub_static = node.create_publisher(
            TelemetryStatic, "/ac/telemetry_static",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        # Low-rate events must not drop
        self._pub_run = node.create_publisher(RunState, "/ac/run_state", 10)

        self._missing_logged = set()
        self._bad_decodes = {}
        self._first_seen = set()

    def attach(self, session) -> None:
        session.declare_subscriber("ac/telemetry", self._on_telemetry)
        session.declare_subscriber("ac/timing", self._on_timing)
        session.declare_subscriber("ac/telemetry_static", self._on_static)
        session.declare_subscriber("ac/run_state", self._on_run_state)

    # ---- helpers ----

    def _fill(self, msg, data: dict) -> None:
        """Map the wire dict (camelCase acpmf names) onto msg fields
        (snake_case) by name. Payload-header fields (frame_id etc.) already
        match; anything unmatched logs once and keeps its default."""
        for field in msg.get_fields_and_field_types():
            if field == "header":
                continue
            key = wire.wire_key_for(field)
            if key in data:
                setattr(msg, field, data[key])
            elif field in data:
                setattr(msg, field, data[field])
            elif field not in self._missing_logged:
                self._missing_logged.add(field)
                self.ctx.log.warn(
                    f"telemetry field '{field}' has no wire key - stays default")

    def _decode(self, what: str, sample, fn):
        try:
            return fn(wire.payload_bytes(sample))
        except Exception as ex:  # noqa: BLE001
            n = self._bad_decodes.get(what, 0) + 1
            self._bad_decodes[what] = n
            if n <= 5 or n % 100 == 0:
                self.ctx.log.warn(f"{what} decode failed ({ex}), count={n}")
            return None

    def _alive_note(self, key: str, extra: str) -> None:
        if key not in self._first_seen:
            self._first_seen.add(key)
            self.ctx.log.info(f"{key} stream live{extra}")

    # ---- zenoh -> ROS ----

    def _on_telemetry(self, sample) -> None:
        v = self._decode("telemetry", sample, wire.decode_telemetry)
        if v is None:
            return
        self._alive_note("telemetry", f" (frame {v.get('frame_id')})")
        m = Telemetry()
        m.header.stamp = self.ctx.now().to_msg()
        self._fill(m, v)
        with self.ctx.pub_lock:
            self._pub_tel.publish(m)

    def _on_timing(self, sample) -> None:
        v = self._decode("timing", sample, wire.decode_timing)
        if v is None:
            return
        self._alive_note("timing", f" (frame {v.get('frame_id')})")
        m = Timing()
        m.header.stamp = self.ctx.now().to_msg()
        self._fill(m, v)
        with self.ctx.pub_lock:
            self._pub_timing.publish(m)

    def _on_static(self, sample) -> None:
        v = self._decode("telemetry_static", sample, wire.decode_telemetry_static)
        if v is None:
            return
        m = TelemetryStatic()
        m.header.stamp = self.ctx.now().to_msg()
        self._fill(m, v)
        with self.ctx.pub_lock:
            self._pub_static.publish(m)
        # Session-level event: worth a log line each time
        self.ctx.log.info(
            f"telemetry_static: epoch={v.get('sessionEpoch')} "
            f"track={v.get('track', '')} car={v.get('carModel', '')}")

    def _on_run_state(self, sample) -> None:
        v = self._decode("run_state", sample, wire.decode_run_state)
        if v is None:
            return
        m = RunState()
        m.header.stamp = self.ctx.now().to_msg()
        m.timestamp_ns = v["timestamp_ns"]
        m.event_type = v["event_type"]
        m.reason_code = v["reason_code"]
        m.run_id = v["run_id"]
        m.session_epoch = v["session_epoch"]
        m.session_type = v["session_type"]
        m.flags = v["flags"]
        m.flag_field = v["flag_field"]
        m.track = v["track"]
        m.car_model = v["car_model"]
        with self.ctx.pub_lock:
            self._pub_run.publish(m)
        ev = wire.RUN_STATE_EVENT_NAMES.get(v["event_type"],
                                            f"event{v['event_type']}")
        self.ctx.log.info(
            f"run_state: {ev} run={v['run_id']} reason={v['reason_code']} "
            f"epoch={v['session_epoch']} track={v['track']}")
