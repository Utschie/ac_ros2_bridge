#!/usr/bin/env python3
r"""zenoh rate probe.

Linux-side zenoh bus diagnostics: per-key delivery rates and inter-arrival
gaps, separating "link/producer drops" from "consumer can't keep up"
(compare against the relay's ROS publish rates to place the blame).

Usage (no ROS dependency; runs anywhere with python3 + eclipse-zenoh):
    python zenoh_rate_probe.py [key ...]           # default ac/camera/**, 10s
    python zenoh_rate_probe.py ac/camera/** ac/ego_pose --seconds 30
    python zenoh_rate_probe.py --connect tcp/<relay-host>:7447

Modes:
  --connect (default tcp/localhost:7447, client mode) — point at a zenoh
    listener (the relay or any peer); diagnosing the relay locally uses
    this form;
  --listen tcp/0.0.0.0:7447 (peer mode) — listen and wait for the bridge
    to connect (probing the bridge directly from the Windows side; don't
    collide with the relay's port).

Output: per-key count / rate / inter-arrival min/median/max (a large max
gap = burst or starvation). Compare rates manually against the per-camera
HZ quotas in streamer.ini.
"""

import argparse
import statistics
import time

try:
    import zenoh
except ImportError:
    raise SystemExit("eclipse-zenoh required: pip install eclipse-zenoh")


def main():
    ap = argparse.ArgumentParser(
        description="zenoh per-key rate probe (delivery vs consumer diagnosis)")
    ap.add_argument("keys", nargs="*", default=["ac/camera/**"],
                    help="zenoh key expressions (default ac/camera/**)")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="sampling window in seconds (default 10)")
    ap.add_argument("--connect", default="tcp/localhost:7447",
                    help="client-mode endpoint (default tcp/localhost:7447)")
    ap.add_argument("--listen", default=None,
                    help="peer-mode listen endpoint (e.g. tcp/0.0.0.0:7447);"
                         "overrides --connect when given")
    args = ap.parse_args()

    cfg = zenoh.Config()
    if args.listen:
        cfg.insert_json5("mode", '"peer"')
        cfg.insert_json5("listen/endpoints", f'["{args.listen}"]')
    else:
        cfg.insert_json5("mode", '"client"')
        cfg.insert_json5("connect/endpoints", f'["{args.connect}"]')
    session = zenoh.open(cfg)

    counts = {}
    gaps = {}
    last_t = {}
    t0 = time.monotonic()

    def cb(sample):
        k = str(sample.key_expr)
        counts[k] = counts.get(k, 0) + 1
        now = time.monotonic()
        if k in last_t:
            gaps.setdefault(k, []).append(now - last_t[k])
        last_t[k] = now

    subs = [session.declare_subscriber(k, cb) for k in args.keys]
    print(f"probing {args.keys} for {args.seconds:.0f}s "
          f"({'listen ' + args.listen if args.listen else 'connect ' + args.connect}) ...")
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass
    span = time.monotonic() - t0
    for s in subs:
        s.undeclare()
    session.close()

    if not counts:
        print("(no messages - check the peer / key expressions / whether "
              "the bridge is publishing)")
        return
    print("\n| key | count | rate/s | gap min/median/max ms |")
    print("|---|---|---|---|")
    for k in sorted(counts):
        g = gaps.get(k, [])
        if g:
            gstat = (f"{min(g)*1000:.0f}/{statistics.median(g)*1000:.0f}/"
                     f"{max(g)*1000:.0f}")
        else:
            gstat = "-"
        print(f"| {k} | {counts[k]} | {counts[k]/span:.1f} | {gstat} |")
    print(f"\nwindow {span:.1f}s, total {sum(counts.values())} messages, "
          f"{sum(counts.values())/span:.1f}/s")


if __name__ == "__main__":
    main()
