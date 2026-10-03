"""RelayContext - cross-cutting policies shared by every domain relay module.

One context instance is created by relay.py and handed to each domain module
(scene_relay, image_relay, ...). Adding a new data type = writing a new domain
module + attaching it in relay.py; the policies below never get duplicated.

Policies concentrated here (each documented at its definition):
  1. CLOCK/TIMESTAMP  - all outbound stamps use the relay-local ROS clock;
                        chain-dependent messages clamp to the newest ego-TF
                        time (see stamp_clamped).
  2. FRAME PAIRING    - dets/lidar are stamped with the SAME frame's ego TF
                        via the producer-shared timestamp (see publish_paired
                        / record_ego_frame).
  3. PUBLISH LOCK     - zenoh callbacks run on zenoh threads, not the rclpy
                        executor thread; rclpy publish() is not guaranteed
                        thread-safe -> pub_lock serializes all ROS calls.
  4. TF BROADCASTING  - dynamic (ac_world->base_link, from the ego stream) and
                        static (sensor install poses, FULL SET per message;
                        /tf_static is latched with depth 1 - per-child sends
                        would leave late joiners with only the last sample).

Future /clock + use_sim_time lands HERE: replace the clock source in now()
and everything downstream keeps working.
"""

import threading
import time
from collections import deque

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from .class_table import builtin_classes

# Max wait for the same-frame ego before falling back to the clamped stamp.
PAIR_TIMEOUT_S = 0.5
# Pairing map size (~10s @ 50Hz of ego frames).
EGO_MAP_CAP = 512


class RelayContext:
    def __init__(self, node):
        self.node = node
        self.log = node.get_logger()

        # Policy 4: TF broadcasters (dynamic + latched static).
        self.tf = TransformBroadcaster(node)
        self.tf_static = StaticTransformBroadcaster(node)
        # child frame -> (pose_sig, TransformStamped); full set re-sent on
        # every change (latch depth 1 - see update_static_tf).
        self.static_tf_cache = {}

        # Policy 3: publish serialization.
        self.pub_lock = threading.Lock()

        # Policy 1: clamp reference - rclpy Time of the newest
        # ac_world->base_link TF we published.
        self.last_tf_stamp = None

        # Policy 2: frame-exact pairing state. Guarded by pair_lock (short
        # critical sections; NEVER publish while holding it).
        self.pair_lock = threading.Lock()
        self.ego_stamp_by_pts = {}   # producer ts_ns -> rclpy Time
        self.ego_pts_order = deque()
        self.pending_msgs = []       # (pts, enqueue_monotonic, publish_fn)
        self.pair_timer = node.create_timer(0.1, self.flush_pending)

        # Cross-domain caches.
        self.last_ego_world = None   # 16-float AC BODYTR world matrix
        # M3: semantic class vocabulary (id -> name/color/dynamic/exclude),
        # fed by SemanticClassRelay from zenoh ac/semantic/classes and swapped
        # whole-object on update (readers never see a torn table). Bootstrap =
        # built-in 11 classes until the first valid sample arrives.
        self.semantic_classes = builtin_classes()

    def close(self):
        try:
            self.pair_timer.cancel()
        except Exception:  # noqa: BLE001
            pass

    # ---------------- policy 1: clock / stamping ----------------

    def now(self):
        """Relay-local ROS clock (single clock domain for every outbound stamp)."""
        return self.node.get_clock().now()

    def stamp_clamped(self, header) -> None:
        """Stamp for messages that resolve through the ac_world->base_link chain.

        Clamped to the newest ego-TF time: a consumer looking up TF at this
        stamp is guaranteed a same-or-newer transform in the buffer, killing
        both the cross-machine future-stamp failure and the arrival-order race
        (the two causes of RViz2's intermittent "No Transform" flicker).
        Before the first ego message this degrades to plain local now.
        """
        t = self.now()
        if self.last_tf_stamp is not None and t > self.last_tf_stamp:
            t = self.last_tf_stamp
        header.stamp = t.to_msg()

    # ---------------- policy 2: frame-exact pairing ----------------

    def publish_paired(self, pts: int, publish_fn) -> None:
        """Publish now with the same-frame ego stamp if known, else defer.

        The bridge stamps ego/dets/lidar of one simulation frame with one
        shared producer timestamp, so an exact dict join recovers the local
        stamp of THAT frame's TF even when zenoh delivers the two callbacks
        out of order (the cause of boxes darting ahead of base_link's axes).
        """
        with self.pair_lock:
            t = self.ego_stamp_by_pts.get(pts)
            if t is not None:
                deferred = False
            else:
                self.pending_msgs.append((pts, time.monotonic(), publish_fn))
                deferred = True
        if not deferred:
            publish_fn(t)

    def record_ego_frame(self, pts: int, t) -> None:
        """Called by the ego domain right after publishing an ego frame's TF:
        record the frame's local stamp, then release any dets/lidar that were
        deferred waiting for exactly this frame."""
        with self.pair_lock:
            self.ego_stamp_by_pts[pts] = t
            self.ego_pts_order.append(pts)
            if len(self.ego_pts_order) > EGO_MAP_CAP:
                self.ego_stamp_by_pts.pop(self.ego_pts_order.popleft(), None)
        self.publish_due(self.collect_due())

    def collect_due(self):
        """Remove and return [(publish_fn, t_or_None)] whose ego arrived (t) or
        that timed out (None -> publish_fn falls back to the clamped stamp)."""
        now_mono = time.monotonic()
        due = []
        with self.pair_lock:
            remain = []
            for pts, enq, fn in self.pending_msgs:
                t = self.ego_stamp_by_pts.get(pts)
                if t is not None or (now_mono - enq) >= PAIR_TIMEOUT_S:
                    due.append((fn, t))
                else:
                    remain.append((pts, enq, fn))
            self.pending_msgs = remain
        return due

    def publish_due(self, due) -> None:
        for fn, t in due:
            try:
                fn(t)
            except Exception as ex:  # noqa: BLE001
                self.log.warn(f"paired publish failed: {ex}")

    def flush_pending(self) -> None:
        """Timer hook (executor thread): publish matched / timed-out messages."""
        try:
            self.publish_due(self.collect_due())
        except Exception as ex:  # noqa: BLE001
            self.log.warn(f"flush_pending failed: {ex}")

    # ---------------- policy 4: latched static TF ----------------

    def update_static_tf(self, key: str, pose_sig, build_tf) -> None:
        """Register/update ONE static transform; when anything changed, publish
        the COMPLETE set in a single /tf_static message.

        /tf_static is latched (TRANSIENT_LOCAL) with depth 1: only the last
        sample is delivered to late-joining subscribers. Sending per-child
        transforms separately left RViz2 (started after the relay) with a
        single child's static TF, silently breaking the other lidars' chains
        (clouds stuck in the message filter, "queue is full", never shown).
        One list publish keeps the latched sample always-complete.

        pose_sig: cheap equality key (e.g. ((x,y,z),(qx,qy,qz,qw)));
        build_tf: () -> TransformStamped, only called on change.
        """
        cached = self.static_tf_cache.get(key)
        if cached is not None and cached[0] == pose_sig:
            return
        self.static_tf_cache[key] = (pose_sig, build_tf())
        all_tfs = [tf for _, tf in self.static_tf_cache.values()]
        if not all_tfs:
            return
        with self.pub_lock:
            self.tf_static.sendTransform(all_tfs)
