# Browser UI reference

The interface is served by the ESP32 from `webui_fly.h` as a single page over its own
SoftAP. No internet and no app store. It works on Android, iPhone and a laptop; *Add
to Home Screen* makes it behave as an installed app.

Connect to WiFi `HybridBot`, then open `http://192.168.4.1`.

> **The FLIGHT tab has never operated the aircraft.** The TELEM2 cable is removed
> from the vehicle, so everything in the flight half of this document describes
> implemented, compiled, unproven firmware and the tab sits at `NO FC`. See
> [`07-open-issues.md`](07-open-issues.md) issue O1. The GROUND tab — drive, steering
> and manipulator — is validated end to end and is what the demo video shows.

## Ground controls — working

| Control | Behaviour |
|---|---|
| Drive | throttle and steering mixed to differential duty, slew-limited |
| EXTEND / RETRACT | rack-and-pinion lift, 180° retracted to 40° extended |
| OPEN / CLOSE | gripper, 0° closed to 60° open |
| Live drive tuning | max duty, duty floor, deadband, PWM frequency and per-side inversion, all changeable from the UI without a reflash |

That last row is worth knowing about: the weak-motor hunt needs no rebuild, and a
rewired motor is corrected by flipping `invertL` / `invertR` live.

## Flight controls — implemented, not connected

| Control | Behaviour |
|---|---|
| Left stick | X = yaw (springs to centre), Y = throttle |
| Right stick | X = roll, Y = pitch — both spring to centre |
| Throttle in ALT_HOLD / LOITER | centre = **hold altitude**; springs to centre on release |
| Throttle in STABILIZE | does **not** spring; releasing holds the last value |
| ARM | refused unless throttle is at minimum |
| Mode switch | GROUND ⇄ FLIGHT; entering FLIGHT stops the wheels and resets throttle to minimum |
| E-STOP in flight | commands **LAND**. Never a motor cut |

Stick positions become `RC_CHANNELS_OVERRIDE` at 1000–2000 µs with 1500 at centre;
mode changes go out as `SET_MODE`.

### Arm behaviour differs between the two builds

| | `hybridbot` (on the robot) | `hybridbot_fly` (integration build) |
|---|---|---|
| Arm in GROUND | commandable at 90°/s | commandable at 90°/s |
| Arm in FLIGHT | read-only, holds position | **commandable at 25°/s** |

In both, the servos are never detached — a rack-and-pinion gripper back-drives, so a
limp servo drops the cargo the instant the mode changes.

### Flight mode buttons and the GPS

A GPS is fitted, so LOITER, POSHOLD and RTL are all available from the autopilot. The
current `webui_fly.h` replaced the LOITER and RTL buttons with STABILIZE, dating from
when no GPS was on board. The `C,loi` and `C,rtl` commands are implemented in both
builds and the UI shipped with `hybridbot/` still has them — issue O3.

## Link loss

| Elapsed | Action |
|---|---|
| 400 ms | ground drive commands zeroed |
| 500 ms | RC override released; ArduPilot's own failsafe takes over |
| 2 s | `SET_MODE` LAND commanded explicitly |

`RC_OVERRIDE_TIME = 1.0` on the flight controller is the second half of this
mechanism — the ESP32 releasing override only helps if ArduPilot acts on the gap.

## Telemetry shown

Flight mode, armed state, battery voltage and percentage, altitude, climb rate, roll
and pitch, per-side drive duty, connected client count, arm lock state and the live
drive tuning values.

`NO FC` means no MAVLink heartbeat. With the cable removed that is the expected and
current state.

## SoftAP credentials

SSID and passphrase are compile-time constants at the top of each `hybridbot*`
sketch. They are published here deliberately: the AP is an isolated, offline network
with no route to anything, and a reader following the bring-up guide needs them.
Change them before any public demonstration, and do not reuse a passphrase from
elsewhere.
