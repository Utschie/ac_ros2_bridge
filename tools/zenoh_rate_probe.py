#!/usr/bin/env python3
r"""zenoh 速率探针（2026-10-14 立，WSL 验收轮的裸计数器固化）.

本仓 tools/ 首个工具：Linux 端 zenoh 总线诊断——按 key 统计真实送达
速率与到达间隔，区分 "链路/生产侧丢帧" 与 "消费端处理跟不上"（对照
relay 的 ROS 发布率即可定责；2026-10-14 实测案例：裸计数满额 14.5/15
而 relay 发布 5.3/15，定谳 relay 单线程处理瓶颈）。

用法（无 ROS 依赖，任意有 python3 + eclipse-zenoh 的机器可跑）:
    python zenoh_rate_probe.py [key ...]           # 默认 ac/camera/**，10s
    python zenoh_rate_probe.py ac/camera/** ac/ego_pose --seconds 30
    python zenoh_rate_probe.py --connect tcp/192.168.0.79:7447

模式：
  --connect（默认 tcp/localhost:7447，client 模式）——对准某个 zenoh
    监听端（relay 或任何 peer）。WSL 里对 relay 诊断即此形态；
  --listen tcp/0.0.0.0:7447（peer 模式）——自起监听等桥连过来
    （Windows 侧直接探桥的形态，注意别与 relay 抢端口）。

输出：逐 key 计数/速率/到达间隔 min/中位/max（大 max 间隔 = 突发
或饥饿信号）。配额对照请人工按 streamer.ini 各 [CAMERA_*] HZ 判读。
"""

import argparse
import statistics
import time

try:
    import zenoh
except ImportError:
    raise SystemExit("需要 eclipse-zenoh：pip install eclipse-zenoh")


def main():
    ap = argparse.ArgumentParser(
        description="zenoh per-key rate probe (delivery vs consumer diagnosis)")
    ap.add_argument("keys", nargs="*", default=["ac/camera/**"],
                    help="zenoh key expr 列表（默认 ac/camera/**）")
    ap.add_argument("--seconds", type=float, default=10.0,
                    help="采样窗长秒（默认 10）")
    ap.add_argument("--connect", default="tcp/localhost:7447",
                    help="client 模式连接端（默认 tcp/localhost:7447）")
    ap.add_argument("--listen", default=None,
                    help="peer 模式自监听端（如 tcp/0.0.0.0:7447）；"
                         "给出即覆盖 --connect")
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
        print("（零消息——检查对端/键表达式/桥是否在发）")
        return
    print(f"\n| key | 计数 | 速率/s | 间隔min/中位/max ms |")
    print("|---|---|---|---|")
    for k in sorted(counts):
        g = gaps.get(k, [])
        if g:
            gstat = (f"{min(g)*1000:.0f}/{statistics.median(g)*1000:.0f}/"
                     f"{max(g)*1000:.0f}")
        else:
            gstat = "-"
        print(f"| {k} | {counts[k]} | {counts[k]/span:.1f} | {gstat} |")
    print(f"\n窗长 {span:.1f}s，总计 {sum(counts.values())} 条，"
          f"{sum(counts.values())/span:.1f}/s")


if __name__ == "__main__":
    main()
