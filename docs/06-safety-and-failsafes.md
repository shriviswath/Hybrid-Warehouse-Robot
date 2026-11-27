# Safety and failsafes

A roughly 2.3 kg vehicle with four 10-inch propellers and a mass cantilevered forward
of the rotor centroid. The list below is the operating procedure, not advice.

## Designed failsafe chain

| Layer | Trigger | Response |
|---|---|---|
| UI | ARM pressed with throttle above minimum | refused |
| UI | Mode change to FLIGHT | wheels stop, throttle reset to minimum |
| UI | E-STOP in flight | `SET_MODE` **LAND** — never a motor cut |
| ESP32 | 400 ms with no browser traffic | ground drive zeroed |
| ESP32 | 500 ms with no browser traffic | RC override released |
| ESP32 | 2 s with no browser traffic | **LAND** commanded explicitly |
| ArduPilot | `RC_OVERRIDE_TIME = 1.0` elapsed | RC failsafe path entered |
| ArduPilot | `FS_THR_ENABLE = 1` | **RTL** — available because a GPS is fitted |
| Hardware | safety switch (`BRD_SAFETYENABLE = 1`) | outputs disabled until press-and-hold |

### What is actually live today

The TELEM2 cable is removed, so **every row that depends on the ESP32 reaching the
autopilot is inert** — the three UI flight rows, the override release, the LAND
command, and `RC_OVERRIDE_TIME`. The 400 ms ground-drive cutout is live and is the
one browser-side failsafe that has been exercised.

Flight runs on an RC transmitter with Mission Planner as the ground station. Exactly
two rows of that table protect the aircraft right now: `FS_THR_ENABLE` and the
hardware safety switch.

Treat the browser failsafe chain as designed and coded, not proven. It is proven the
day the link-loss test in [`03-bringup-procedure.md`](03-bringup-procedure.md) stage 5
passes.

### The gap a transmitter introduces

`FS_THR_ENABLE` does not detect a dead transmitter by itself. It fires only when the
**receiver** drives the throttle channel below `FS_THR_VALUE` (~975 µs by default) on
signal loss. Many receivers ship set to *hold last position* instead, which defeats
the parameter silently — the aircraft keeps the last throttle it saw and flies away.

Verify before the next flight:

1. Power the receiver, power **off** the transmitter, read the throttle channel on
   Mission Planner's radio page. It must drop low or stop pulsing.
2. With the transmitter off, Mission Planner's HUD should annunciate the failsafe. If
   nothing happens, step 1 is misconfigured.

Two minutes, and it is the only thing between a transmitter brownout and an
uncontrolled aircraft.

### Why E-STOP lands instead of cutting motors

A motor cut on an airborne multirotor is a fall. LAND is a controlled descent the
flight controller executes with full state knowledge. There is no software e-stop
that makes a flying aircraft safe instantly — the honest control is the safety
switch, on the ground, before arming.

## Bench rules

- **Props off** for every stage before tethered hover.
- Frame clamped or held down during ESC work.
- Battery disconnected before ESC calibration begins.
- Wheels off the ground for first drive tests; arm unloaded, rack path clear.
- `kill` and `x` on the serial console set an abort flag honoured by every blocking
  wait and every scripted sequence. That is what makes `kill` a real panic word.
- E-stop is latched **true** at boot in both `hybridbot_*` builds. Nothing moves
  until it is cleared deliberately.

## Electrical

See [`02-wiring-and-power.md`](02-wiring-and-power.md). Two that cause real damage:
servos on their own ≥ 6 A BEC — done correctly on this build — and the flight
controller's shared supply rail, which is issue B2 and unmeasured.

## Flight envelope as built

GPS fitted, so ALT_HOLD, LOITER, POSHOLD and RTL are available. Fly within visual
line of sight. Until issues B1 and B2 close, and while A1–A3 remain unmeasured, keep
flights short, keep the ground stage unpowered in the air, and check motor case
temperature after every landing.
