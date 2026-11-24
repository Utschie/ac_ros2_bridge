# Telemetry Viewer (launcher: View Telemetry)

A MoTeC-style live telemetry dashboard plus a channel browser, reading the
game's telemetry directly. Start it with the launcher's **View Telemetry**
button; it also works while the game sits in the menus, and shows a waiting
hint (rechecking every 0.5 s) until the telemetry is available (the required
setting is enabled automatically by the installer).

## Dashboard

- **Top banner**: LIVE indicator, session, track + configuration, car, fitted
  tires, position + lap, flags, sample rate, and the **Add plot** button.
- **Left column**: big gear / speed / RPM redline bar; throttle / brake /
  clutch / steer / fuel / ERS bars; TC · ABS · DRS · PIT · racing-line
  indicator lights; 2×2 tire panel (core temperature, inner/mid/outer temp
  bars, pressure / life / brake temp); environment and damage.
- **Right column**: stacked CHANNELS panels (pedals %, RPM, speed), tire
  temperature panel (0–140 °C), GG diagram (auto-ranged, with current-point
  crosshair), lap times (current / last / best, delta-colored, sectors,
  penalties), wheel slip / load / speed details.
- **Bottom status line**: shared-memory connection states, packet counter,
  channel count, sample rate.

## Channel browser (Add plot)

**Add plot** opens a floating plot window (up to 8, draggable/resizable,
positions remembered):

- Type a keyword in the filter box (e.g. `slip`, `tyreContact`, `cps.`) and
  click channels to add them to the plot — up to 8 traces per window, 8
  stable colors.
- Channels are **grouped into panels by unit**: same-unit channels share one
  panel with a true scale; different units each get their own panel on a
  shared time axis. Want two same-unit channels apart? Open a second plot.
- Traces plot raw values by default (1-2-5 stepped scales, unit labeled);
  the `normalized %` checkbox switches each trace to 0–100 %.
- All **194 channels** are plottable: `physics.*` (145 — inputs, motion,
  4-wheel details, ERS, environment), `gfx.*` (27 — laps, timing, ranking,
  flags, world coordinates), `cps.*` (22 — self-aligning torque, load
  radius, drivetrain rpm, aero coefficients, caster, slip angles/ratios;
  requires the control plugin shipped with the suite — the plot window
  shows a yellow hint if it is not feeding).

Two typical uses: sanity-checking the data chain (a channel that does not
move is not being delivered) and building your own live views of any channel
combination. The same telemetry is published over zenoh (`ac/telemetry`,
`ac/timing`) and dumped to `telemetry.jsonl` — see
[configs/recorder.md](configs/recorder.md).

Close the window with the X button (no ESC shortcut). Log:
`ac_simulator_suite\logs\`.
