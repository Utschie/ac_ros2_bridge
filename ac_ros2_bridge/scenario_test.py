"""scenario_test.py - scripted ROS2 regression of every /ac/vehicle_cmd field.

The full-chain counterpart of ac_sim_bridge/tools/vehicle_cmd_scenario.py: this
one drives through the ROS2 relay (Linux) -> zenoh -> Windows bridge -> game,
i.e. it exercises the M4 pipeline end to end including the message definitions.

Run (Linux, relay already running, bridge + AC on the Windows machine):

  ros2 run ac_ros2_bridge scenario                    # race: waits for GO
  ros2 run ac_ros2_bridge scenario --ros-args -p skip_go:=true
                                                      # practice: start now
  ros2 run ac_ros2_bridge scenario --ros-args -p throttle_scale:=0.5

Parameters (ros-args -p name:=value):
  skip_go (false)        don't wait for the race-start GO flag
  go_timeout (300.0)     seconds to wait for GO before giving up
  start_delay (1.0)      idle time after GO before LAUNCH
  throttle_scale (1.0)   multiply every throttle/brake/clutch value (derating)
  rate (50.0)            command publish rate [Hz]

Phases: LAUNCH(1st) -> GEAR2 -> REL_DN -> REL_UP -> STEER_L -> STEER_R ->
BRAKE -> CLUTCH -> HANDBRAKE -> DRS_KERS -> OVERRIDE_OFF -> OVERRIDE_ON ->
disarm. Ctrl-C aborts any time; the bridge disarms ~0.5s after the last
message (bridge RX_TIMEOUT_MS).

NOTE for future edits: all fields here are scalars. If an array field is ever
added to these messages, NEVER assign it through the rclpy setter (per-element
Python assertion loop, see relay README 调试记录) - direct-write the private
slot:  msg._data = np.frombuffer(raw, np.uint8).
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from ac_msgs.msg import VehicleControl, VehicleState

GEAR_NO_CHANGE = -100
FLAG_FEEDBACK_GO = 0x01
FLAG_FEEDBACK_LIGHTS_ON = 0x02


def gear_label(g):
    return {0: "R", 1: "N"}.get(g, str(g - 1) if 2 <= g <= 8 else str(g))


def clamp01(v):
    return max(0.0, min(1.0, v))


# (name, duration_s, command kwargs, in-game expectation)
PHASES = [
    ("LAUNCH",       4.0, dict(throttle=0.25, requested_gear_index=2),
     "1st gear engages (feedback gear -> 2), car creeps forward"),
    ("GEAR2",        4.0, dict(throttle=0.35, requested_gear_index=3),
     "upshift to 2nd (feedback gear -> 3), speed rises"),
    ("REL_DN",       2.5, dict(throttle=0.30, gear_dn=True),
     "relative downshift button (feedback gear -> 2)"),
    ("REL_UP",       2.5, dict(throttle=0.30, gear_up=True),
     "relative upshift button (feedback gear -> 3)"),
    ("STEER_L",      3.0, dict(throttle=0.25, steering=-0.4),
     "steering left; car turns left"),
    ("STEER_R",      3.0, dict(throttle=0.25, steering=0.4),
     "steering right; car comes back / turns right"),
    ("BRAKE",        3.0, dict(throttle=0.0, brake=0.6),
     "brakes hard; speed feedback drops toward 0"),
    ("CLUTCH",       2.5, dict(throttle=0.3, clutch=1.0),
     "clutch in: revs flare, drive disengaged"),
    ("HANDBRAKE",    2.5, dict(throttle=0.2, handbrake_on=True, hand_brake=1.0),
     "handbrake on; rear locks at low speed"),
    ("DRS_KERS",     2.0, dict(throttle=0.25, drs=True, kers=True),
     "DRS+KERS bits flow (non-F1 car: path check only)"),
    ("OVERRIDE_OFF", 3.0, dict(throttle=0.3, override_enabled=False),
     "override DISABLED mid-stream: car snaps back to human control"),
    ("OVERRIDE_ON",  2.0, dict(throttle=0.25),
     "override re-enabled: throttle resumes by itself"),
]


class ScenarioNode(Node):
    # states
    WAIT_FEEDBACK, WAIT_GO, START_DELAY, RUN, DONE = range(5)

    def __init__(self):
        super().__init__("scenario_test")
        for name, default in [("skip_go", False), ("go_timeout", 300.0),
                              ("start_delay", 1.0), ("throttle_scale", 1.0),
                              ("rate", 50.0)]:
            self.declare_parameter(name, default)
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.skip_go = bool(p("skip_go"))
        self.go_timeout = float(p("go_timeout"))
        self.start_delay = float(p("start_delay"))
        self.scale = float(p("throttle_scale"))
        rate = max(1.0, float(p("rate")))

        # RELIABLE to match the relay's subscription (a BEST_EFFORT publisher
        # would NOT be matched - same DDS pitfall class as the image QoS one).
        self.pub = self.create_publisher(
            VehicleControl, "/ac/vehicle_cmd",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(VehicleState, "/ac/vehicle_state",
                                 self._on_state, 10)

        # feedback (subscriber callback thread == executor thread here: single-
        # threaded executor, safe to store directly)
        self.fb_time = None
        self.fb_gear = None
        self.fb_speed = None
        self.fb_flags = 0
        self.gear_history = []   # (monotonic_t, gear, speed, flags) on change

        self.state = self.WAIT_FEEDBACK if not self.skip_go else self.START_DELAY
        self.state_t0 = time.monotonic()
        self.phase_idx = -1
        self.last_wait_log = 0.0
        self.done = False
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"scenario_test ready (skip_go={self.skip_go}, scale={self.scale}); "
            "waiting for /ac/vehicle_state feedback ...")

    # ---- feedback ----

    def _on_state(self, msg: VehicleState):
        now = time.monotonic()
        if self.fb_gear != msg.gear:
            self.gear_history.append((now, msg.gear, msg.speed, msg.flags))
        self.fb_time, self.fb_gear = now, msg.gear
        self.fb_speed, self.fb_flags = msg.speed, msg.flags

    def feedback_alive(self):
        return self.fb_time is not None and time.monotonic() - self.fb_time < 2.0

    def fb_str(self):
        g = f"{gear_label(self.fb_gear)}({self.fb_gear})" if self.fb_gear is not None else "n/a"
        s = f"{self.fb_speed:.2f}" if self.fb_speed is not None else "n/a"
        return f"gear={g} speed={s} flags=0x{self.fb_flags:x}"

    # ---- command packing (scalar fields only - see module docstring) ----

    def _make_cmd(self, phase_cmd: dict, override_default=True) -> VehicleControl:
        m = VehicleControl()
        m.override_enabled = bool(phase_cmd.get("override_enabled", override_default))
        m.steering = float(phase_cmd.get("steering", 0.0))
        m.throttle = clamp01(float(phase_cmd.get("throttle", 0.0)) * self.scale)
        m.brake = clamp01(float(phase_cmd.get("brake", 0.0)) * self.scale)
        m.clutch = clamp01(float(phase_cmd.get("clutch", 0.0)) * self.scale)
        m.hand_brake = clamp01(float(phase_cmd.get("hand_brake", 0.0)) * self.scale)
        m.requested_gear_index = int(phase_cmd.get("requested_gear_index", GEAR_NO_CHANGE))
        m.gear_up = bool(phase_cmd.get("gear_up", False))
        m.gear_dn = bool(phase_cmd.get("gear_dn", False))
        m.drs = bool(phase_cmd.get("drs", False))
        m.kers = bool(phase_cmd.get("kers", False))
        m.handbrake_on = bool(phase_cmd.get("handbrake_on", False))
        return m

    # ---- 50Hz tick: state machine ----

    def _tick(self):
        now = time.monotonic()

        if self.state == self.WAIT_FEEDBACK:
            if self.feedback_alive():
                self.get_logger().info(f"feedback OK: {self.fb_str()}")
                self._enter(self.WAIT_GO)
            elif now - self.state_t0 > 10.0:
                self.get_logger().warn(
                    "no /ac/vehicle_state feedback (relay/bridge/plugin up?). "
                    "Continuing without gear checks ...")
                self._enter(self.WAIT_GO)
            return

        if self.state == self.WAIT_GO:
            if self.fb_flags & FLAG_FEEDBACK_GO:
                self.get_logger().info("*** GO! (lights out) ***")
                self._enter(self.START_DELAY)
            elif now - self.state_t0 > self.go_timeout:
                self.get_logger().error(
                    "GO timeout. For practice use -p skip_go:=true; "
                    "for a race check the session actually started.")
                self._finish()
            else:
                self._wait_log(now, f"waiting for GO... {self.fb_str()} "
                                    f"lights={'on' if self.fb_flags & FLAG_FEEDBACK_LIGHTS_ON else 'off'}")
            return

        if self.state == self.START_DELAY:
            if now - self.state_t0 >= self.start_delay:
                self._enter(self.RUN)   # RUN entry advances to phase 0 in _enter
            return

        if self.state == self.RUN:
            name, dur, cmd, expect = PHASES[self.phase_idx]
            if now - self.state_t0 < dur:
                self.pub.publish(self._make_cmd(cmd))
            else:
                self.get_logger().info(f"[{name}] done. feedback: {self.fb_str()}")
                if self.phase_idx + 1 < len(PHASES):
                    self._enter(self.RUN)
                else:
                    self._disarm_and_finish()

    # ---- transitions ----

    def _enter(self, state):
        self.state = state
        self.state_t0 = time.monotonic()
        if state == self.START_DELAY and self.start_delay > 0:
            self.get_logger().info(f"start delay {self.start_delay:.0f}s (stand by)")
        elif state == self.RUN:
            self.phase_idx += 1
            name, dur, cmd, expect = PHASES[self.phase_idx]
            self.get_logger().info(f"=== [{name}] {dur:.0f}s === expect: {expect}")

    def _wait_log(self, now, text):
        if now - self.last_wait_log >= 1.0:
            self.last_wait_log = now
            self.get_logger().info(text)

    def _disarm_and_finish(self):
        m = VehicleControl()          # all-zero + override off
        m.requested_gear_index = GEAR_NO_CHANGE
        self.pub.publish(m)
        self.get_logger().info(
            "publishing stopped -> bridge disarms in ~0.5s. The car is yours.")
        self._finish()

    def _finish(self):
        self.timer.cancel()
        self.done = True
        # summary is printed by main() after spin exits


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted - bridge will disarm ~0.5s after "
                               "the last message")
    finally:
        # ---- summary ----
        print("\n================ SCENARIO SUMMARY ================")
        for name, dur, cmd, expect in PHASES:
            keys = " ".join(f"{k}={v}" for k, v in cmd.items())
            print(f"[{name:12s}] cmd: {keys}")
            print(f"{'':14s}expect: {expect}")
        print("\nfeedback gear transitions (t, gear, speed, flags):")
        t0 = None
        for t, gear, speed, flags in node.gear_history:
            if t0 is None:
                t0 = t
            print(f"  +{t - t0:6.1f}s  gear={gear_label(gear)}({gear})  "
                  f"speed={speed:7.2f}  flags=0x{flags:x}")
        print("\nCheck each expectation against what you saw in-game; report "
              "any mismatching phase.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
