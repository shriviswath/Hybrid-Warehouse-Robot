# Firmware

One ESP32 on the vehicle, plus the Pixhawk running ArduCopter. The second ESP32 that
earlier revisions used as a flight board has been removed.

GPIO assignment: [README pinout table](../README.md#pinout-authoritative).

---

## Which build is which

| Build | Status |
|---|---|
| **`hybridbot/`** | **The firmware on the robot.** Ground drive, manipulator, browser UI — and a complete MAVLink client that is currently unused |
| `hybridbot_fly/` | **Integration build. Not flashed.** Same program plus UART receive diagnostics, MAVLink data-stream requests, and a rate-limited arm in FLIGHT mode. Flash this when reconnecting TELEM2 |

### The flight code in `hybridbot/` is real, and currently inert

`hybridbot.ino` implements `mavHeartbeat`, `mavPoll`, `mavSendOverride`,
`mavReleaseOverride`, `mavSetMode` and `mavArm`, a FLIGHT mode with stick handling,
and the `C,alt` / `C,loi` / `C,rtl` / `C,land` / `C,stab` mode commands. None of it
reaches an autopilot: the TELEM2 cable is removed from the vehicle, so the FLIGHT tab
sits at `NO FC` and overrides go nowhere.

It is kept rather than stripped because it is written, correct as far as it can be
tested, and it is the work that closing issue O1 builds on.

### What `hybridbot_fly` adds, and why it matters for O1

| | `hybridbot` | `hybridbot_fly` |
|---|---|---|
| UART receive counters | none | `rxBytes` / `rxMsgs` / `rxHeartbeats`, every 2 s |
| `REQUEST_DATA_STREAM` | absent | ATTITUDE, VFR_HUD, SYS_STATUS, re-sent every 5 s |
| Arm in FLIGHT mode | read-only, holds position | commandable at 25°/s |
| Arm rate | fixed 90°/s | 90°/s ground, 25°/s flight |
| HTTP caching | `max-age=86400` | `no-store` |

Two of those are not cosmetic.

**Data streams.** ArduPilot sends `HEARTBEAT` on a telemetry port unprompted, but
`ATTITUDE`, `VFR_HUD` and `SYS_STATUS` only when a ground station asks. Without the
request, a *working* link shows as UP while battery, altitude and climb read zero.

**Link diagnostics.** `hybridbot_fly` prints a receive counter every two seconds, and
its own comments read the result:

- `rxBytes` climbing, `rxMsgs` stuck → wrong baud, or noise on the wire
- `rxBytes` stuck at 0 → nothing arriving: TX/RX swapped, wrong port, no common
  ground, or `SERIAL2_PROTOCOL` not set to 2

The original TELEM2 attempt ran on `hybridbot`, which has none of this, so the
failure produced no evidence. Flash `hybridbot_fly` before touching that cable again.

### Aerial manipulation

The project objective includes manipulation while airborne. Only `hybridbot_fly`
implements it, and with a rate limit rather than a lockout: arm travel shifts the CoG,
and a slow move is a disturbance the attitude controller can track while a fast one is
an impulse it has to catch. A 140° rack stroke at 25°/s takes about 5.6 s.

In both builds the servos are **never detached in any mode** — a rack-and-pinion
gripper back-drives, so a limp servo drops the cargo.

---

## The other sketches

| Sketch | Purpose |
|---|---|
| `drive_arm_combined/` | Bench test: wheels and arm operable simultaneously over serial. Proves hardware before the app |
| `ground_drive_test_v4/` | Drive-only diagnosis. `p` isolates a single dead direction line |
| `manip_bringup/` | Arm-only diagnosis with calibrated travel limits |
| `esc_tuner_v2/` | ESC bench tool from before the Pixhawk. With the Pixhawk fitted, motor **order** is set by MAIN OUT 1–4, so this no longer defines anything about the aircraft |

`archive/` holds superseded sketches. **Do not flash them.**

---

## Build

### Arduino IDE

One sketch per folder, folder name matching the `.ino` name — the tree already
complies. Two `.ino` files in one folder are concatenated into a single translation
unit and fail with duplicate `setup()` and `loop()`.

| Library | Needed by | How |
|---|---|---|
| **WebSockets** (Markus Sattler) | `hybridbot`, `hybridbot_fly` | Library Manager → "WebSockets" |
| **MAVLink `c_library_v2`** | `hybridbot`, `hybridbot_fly` | ZIP from [mavlink/c_library_v2](https://github.com/mavlink/c_library_v2) → unzip into `Arduino/libraries/c_library_v2/` |
| none | all others | deliberately dependency-free |

### PlatformIO — the trap that bit this project

PlatformIO builds **`src_dir` only**. A sketch folder that is not `src_dir` is
invisible to it, so editing one sketch and running `pio run` silently and
successfully builds a different one.

`platformio.ini` here sets `src_dir = hybridbot`, the build that runs on the robot.
Change that line to `hybridbot_fly` to build the integration build. Always confirm
which one is live from the boot banner on the serial monitor.

---

## Conventions worth knowing before editing

- **Explicit LEDC channels.** Every `ledcAttach` names its channel. Channels pair onto
  timers two at a time; a servo auto-allocated onto timer0 reprograms it to 50 Hz and
  the motor PWM becomes a 50 Hz chopper.
- **E-stop is latched true at boot.** Nothing moves until it is cleared.
- **Servo angle is declared, not sensed.** After a brownout the firmware's idea of the
  rack angle is wrong. Re-declare it.
- **Drive inversion is a runtime setting.** Both sides measured reversed on this build;
  `invertL` and `invertR` default true and are flippable live from the UI, so a
  rewired motor needs no reflash. Same for max duty, duty floor, deadband and PWM
  frequency — the weak-motor hunt needs no rebuild.
- **The ESP32 stabilises nothing.** Attitude and altitude belong to ArduPilot — see
  [`../docs/01-system-architecture.md`](../docs/01-system-architecture.md).

## Known stale comments

Both builds carry header text from before the Pixhawk — references to "ESP32-A" as the
flight board, and a boot banner reading `MAVLink to ESP32-A`. `hybridbot_fly` also
recommends `FS_THR_ENABLE = 3` (LAND), correct when no GPS was fitted; **a GPS is now
fitted**, so `FS_THR_ENABLE = 1` (RTL) is the better setting. Pin numbers and logic
are correct; only the comments mislead. Tracked as A9.
