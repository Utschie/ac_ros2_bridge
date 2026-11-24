# Game Control Guide: Three Ways to Control the Race Session

AC Simulator Suite provides four session commands: **Start Race / Pause /
Resume / Restart**. There are three equivalent ways to send them —
pick one for your scenario — all three have exactly the same effect.

**What the five commands do**

| Command | Effect |
|---|---|
| Start Race | Starts the race |
| Pause | Pauses the race (audio stops too) |
| Resume | Continues from pause (audio resumes) |
| Restart | Restarts the session |

> Note: Start Race controls the **race session**, not the game program.
> While paused, audio stops too and resumes automatically on unpause;
> pause/resume is one toggle button (its label switches with state).

## Prerequisites (shared by all three methods)

1. **The game is running** with our plugin loaded (the launcher status
   area's "Game plugin" row shows a healthy heartbeat; if it stays
   `waiting for game` while you are inside a session, reinstalling the
   suite fixes it).
2. **The data bridge is running** — start it with the launcher's
   **Start Bridge** button.
3. Control mode is `shm` (the default): in
   `ac_simulator_suite\configs\control.ini`, `[control] MODE=shm`.
   **Session commands are unavailable with MODE=vjoy or off** (the vJoy
   backend forwards driving input only, not session commands).

---

## Method 1: Launcher buttons (simplest)

1. Open the AC Simulator Suite launcher.
2. Click **Start Bridge** (Start Lidar is not needed — the lidar renderer can
   stay off if you only want to control the game).
3. Once you enter any session in-game, the "Game Control" button row becomes
   available: **Start Race / Pause(Resume) / Restart** — pause/resume
   is one toggle button: it shows Pause while running and Resume while paused.
4. The label next to the buttons shows the current session state in real time
   (e.g. `Session: Race | GO`).

Commands reach the bridge within about 10 ms of a button press — no network
involved.

**Why the buttons may be greyed out** (the label also tells you):

- `game ctrl: start bridge first` — the bridge is not running; click Start
  Bridge first;
- `game ctrl needs MODE=shm` — MODE in control.ini is not shm;
- the software license is not activated.

---

## Method 2: zenoh direct call (models / third-party programs)

Suited for autonomous-driving models, scripts, and remote control from any
machine. No ROS required — just Python with the zenoh library:

```
pip install eclipse-zenoh
```

### Command format

Publish a **4-byte little-endian integer** (a bitmask, values combinable) to
the zenoh key **`ac/game_cmd`**:

| Bit | Value | Command |
|---|---|---|
| bit0 | 0x01 | START (start race) |
| bit1 | 0x02 | PAUSE |
| bit2 | 0x04 | RESUME |
| bit3 | 0x08 | RESTART (restart current session) |
| bit4 | 0x10 | SKIP (next session) |

### One-liner example (on Windows, same network as the bridge)

```bat
python -c "import zenoh,sys; s=zenoh.open(zenoh.Config()); s.put('ac/game_cmd', int(sys.argv[1],0).to_bytes(4,'little'))" 0x01
```

Change the trailing argument to 0x02 pause, 0x04 resume, 0x08 restart.

### Connection configuration

The bridge starts in peer mode (configured in
`ac_simulator_suite\configs\bridge.json5`, by default connecting to
`tcp://<relay IP>:7447`). A zenoh client on a remote machine just needs the
same endpoint to find it:

```python
import zenoh
cfg = zenoh.Config()
cfg.insert_json5("connect/endpoints", '["tcp/192.168.0.79:7447"]')  # your relay address
session = zenoh.open(cfg)
session.put("ac/game_cmd", (0x01).to_bytes(4, "little"))   # start race
```

### Companion state feedback (optional)

- Subscribe to `ac/game_state` (16 bytes `<QII>`: timestamp, sessionType,
  flags) — flags bit0 = GO (green light on), bit1 = start countdown lights on.
- Subscribe to `ac/vehicle_state` (20 bytes `<QifI>`) — current gear / speed /
  GO.
- sessionType: 1=Practice 2=Qualify 3=Race 4=Hotlap 5=TimeAttack 6=Drift
  7=Drag.

### Ready-made tools

The driving-test scripts shipped with the suite (in
`ac_simulator_suite\tools\`; they automatically start the race → wait for GO →
cycle through the driving commands):

```bat
python tools\vehicle_cmd_scenario.py            &:: driving test (start + wait for GO)
python tools\vehicle_cmd_scenario.py --skip-go  &:: practice session: don't wait for GO
python tools\vehicle_cmd_test.py --sub-state    &:: just watch gear/speed/GO feedback
```

Driving controls (throttle/steering/gears, `ac/vehicle_cmd`) are out of scope
here — see `ac_simulator_suite\tools\README.md`.

---

## Method 3: ROS2 (Linux robotics integration)

Requires deploying the ROS relay on the Linux side (provided with the
suite; setup steps in [ros2-relay.md](ros2-relay.md)).

### Sending commands (topic `/ac/game_cmd`, message type `ac_msgs/GameControl`)

```bash
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{start_race: true}"
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{pause: true}"
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{resume: true}"
ros2 topic pub --once /ac/game_cmd ac_msgs/GameControl "{restart_session: true}"
```

(`GameControl.msg` fields: `start_race / pause / resume / restart_session`, all bool; each message is an independent edge trigger — send
another one to trigger again.)

### Watching state

```bash
ros2 topic echo /ac/game_state    # sessionType + GO/countdown lights (relay uplink translation)
ros2 topic echo /ac/vehicle_state # gear/speed/GO
```

Starting the relay (Linux side; point `relay.json5` at the bridge's network):

```bash
ros2 run ac_ros2_bridge relay
```

---

## Command semantics and safety behavior

- **Edge-triggered, repeatable**: each command triggers one action; to trigger
  again, send another. After receiving a command the bridge holds the bits for
  500 ms and then clears them automatically (re-arming), so two consecutive
  presses at least 0.5 s apart count as two.
- **Independent of driving override**: session commands are independent of the
  driving override (`ac/vehicle_cmd`). When a model stops sending driving
  commands for about 0.5 s, control falls back to the human automatically
  (`RX_TIMEOUT_MS`); session commands are not affected by this timeout.
- **Data streams keep flowing while paused**: after a pause the game still
  renders frame by frame; camera/lidar/telemetry data keeps streaming — good
  for a "pause → annotate → continue" data-production flow.
- **The three channels mix freely**: buttons, zenoh and ROS2 can be used at
  the same time; commands queue in the bridge and merge into the same path
  without conflicts.

## Troubleshooting

| Symptom | Check |
|---|---|
| Launcher buttons greyed out | Is the bridge running (Start Bridge); is MODE shm; is the license activated |
| Command sent, nothing happens | Is the game running and inside a session (the waiting screen counts); does the bridge log show a `game_cmd` line (see below). If both are fine, the game most likely hasn't loaded the plugin — reinstalling the suite fixes it |
| Remote zenoh doesn't get through | Reachability of port 7447 on the bridge machine; do bridge.json5 and the client endpoint point at the same relay |
| Confirming in the bridge log | Newest bridge log under `ac_simulator_suite\logs\`: buttons should show `game_cmd (launcher) bits=0x..`; zenoh/ROS2 should show `game_cmd bits=0x..` |
| Button works, bridge logs it, game doesn't react | MODE in control.ini (vjoy doesn't forward); or the version is old — upgrade with the installer |

---
*This document ships with the installer at
`ac_simulator_suite\docs\en\game-control.md` (Chinese version:
`docs\zh\game-control.md`).*
