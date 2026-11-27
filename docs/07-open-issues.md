# Open issues

This is an academic demonstrator, not a product. The list is sorted by whether an
item actually has to be dealt with, so the ones that do are not buried among the ones
that do not.

Recording known limitations is deliberate. A project that states what it did not
measure is more credible than one that quietly implies it measured everything.

---

## Blocking — before the next flight

Two items, both cheap, both safety rather than polish.

### B1. Wire bundle unrestrained in the propeller plane

The main harness runs high between the arms, inside the disc. With props turning, a
wire reaching the disc is a strike; it also shifts CG and couples vibration into the
flight controller. Lace it to the frame. Ten minutes, props currently off.

### B2. Flight controller shares a power rail with the drive stage

All distribution is on one perfboard. The MG995 servos are correctly on their own
≥ 6 A BEC — that part was done right — but the Pixhawk takes its supply from the
shared rail that also carries the L298N.

Measure before flying with the ground stage energised:

1. Multimeter at the **Pixhawk's supply input**, not at the battery.
2. ESP32 running `drive_arm_combined`, command `combo`.
3. Capture the minimum, and specifically capture a **drive direction reversal** —
   both bridge legs switch at once, which is worse than a stall.
4. **Below 4.5 V at any point → do not fly with the ground stage energised.** Move
   the Pixhawk to its own power module straight off the pack; the BOM lists one.

Template in `hardware/tests/`.

---

## Open work — the remaining project scope

### O1. The ESP32 ⇄ Pixhawk link has never carried a heartbeat — *future work*

The TELEM2 cable was fitted, never produced an autopilot heartbeat, and has since
been **removed from the vehicle**. Flight moved to an RC transmitter, so the ground
and aerial halves are currently independent systems sharing a frame and a power
board. Closing this converts them into the single-interface claim the project is
built around.

**Flash `hybridbot_fly` before debugging this.** The build on the vehicle during the
original attempt — `hybridbot`, still the flashed firmware — has no receive
instrumentation, so the failure produced no evidence. `hybridbot_fly` prints a counter
every two seconds and its own comments read the result:

- `rxBytes` climbing, `rxMsgs` stuck → wrong baud, or noise on the wire
- `rxBytes` stuck at 0 → nothing arriving: TX/RX swapped, wrong port, no common
  ground, or `SERIAL2_PROTOCOL` not set to 2

`hybridbot_fly` also sends `REQUEST_DATA_STREAM`, which the older build does not.
Without it, even a working link shows as UP while battery, altitude and climb all
read zero — a second failure mode that would have looked like the first.

Diagnosis order once refitted:

1. **`SERIAL2_BAUD`** — ArduPilot's TELEM2 default is **57600**, not 115200. Left
   unchanged, the ESP32 reads correctly-timed garbage. *Most likely cause.*
2. **`BRD_SER2_RTSCTS = 0`** — no flow-control wires fitted; at its default the port
   can stay completely silent while looking correctly configured.
3. `SERIAL2_PROTOCOL = 2`.
4. Port identity — TELEM2 is `SERIAL2`. In TELEM1 every parameter is `SERIAL1_*`.
5. TX/RX orientation, then common ground on TELEM2 pin 6.
6. ESP32 alone: short GPIO13→GPIO34 and watch `rxBytes`. This will **not** clear
   `NO FC` even when working — the loopback returns the ESP32's own heartbeat at
   compid `MAV_COMP_ID_MISSIONPLANNER`, and the receive path filters on
   `MAV_COMP_ID_AUTOPILOT1`. Watch the byte counter, not the flag.

Pass condition: FLIGHT tab shows real mode and battery. No motors, no props.

**Status: future work, by decision.** The link was not pursued further because both
subsystems were working independently and the project's objectives were demonstrable
without it. The firmware was left intact rather than stripped, so the work survives
for anyone — including a later revision of this project — who wants to close it.

### O2. Aerial manipulation not flight-tested

`hybridbot_fly` implements it — arm commandable in FLIGHT at a rate-limited 25°/s,
where the flashed `hybridbot` holds the arm read-only outside GROUND — but FLIGHT
mode has never been entered from the browser (O1), and the attitude
disturbance from a rack traverse has not been measured in the air.

### O3. UI flight modes predate the GPS

A GPS is now fitted, so LOITER, POSHOLD and RTL are available. The current
`webui_fly.h` replaced the LOITER and RTL buttons with STABILIZE, from when no GPS
was on board. The `C,loi` and `C,rtl` commands are already implemented in both
builds, and the UI shipped with `hybridbot/` still has the buttons.

---

## Accepted for a demonstrator

Known, understood, not being fixed. Each would matter for a product.

| # | Item | Why it is accepted |
|---|---|---|
| A1 | **Thrust-to-weight unmeasured** | ~2.3 kg all-up on four A2212 1000KV. Never put on a scale, but the aircraft has flown under RC, so margin is empirically sufficient for short flights. The number would be needed before carrying payload |
| A2 | **A2212 1000KV + 1045 props on 4S** | A 3S combination run on 4S; motors draw past rating. Flights have been short enough that it has not bitten. **Touch a motor case after landing** — too hot to hold means stop and fit 8045/9045 |
| A3 | **Centre of gravity not measured** | Manipulator is cantilevered forward. Flight under RC shows it is trimmable. The arm-extended vs arm-retracted CG shift is the number that matters for O2, and it is unknown |
| A4 | **Pack voltage at the L298N under load** | Never logged. Prime suspect for weak drive motors, but the drive works well enough for the demo |
| A5 | **Strapping-pin pulldowns not fitted** | 10 kΩ from ENA/IN1/IN2/IN3/IN4 to GND. Causes the wheel twitch on upload and occasional flash failures. Annoyance, not a hazard |
| A6 | **Drive motors double as landing gear** | TT gearboxes take landing impact directly on plastic gears. Fine at demo cadence; would need a compliant element for repeated hard landings |
| A7 | **Vibration isolation / baro foam** | Gel pads under the Pixhawk and open-cell foam over the MS5611 are standard practice. Absent, expect altitude drift and a noisier attitude estimate |
| A8 | **GPS model not recorded** | The module supplied with the Pixhawk kit. Works; exact part number never written down. Confirm on Mission Planner's hardware page when convenient |
| A9 | **Stale firmware comments** | Both builds reference "ESP32-A" as the flight board, including the boot banner, and `hybridbot_fly` recommends `FS_THR_ENABLE = 3` from before the GPS. Pin numbers and logic are correct; only the comments mislead |

---

An item moves out of *Accepted* only when a measurement or a passing test closes it —
not when it is expected to pass. The README's status section is written from this file
and the two must not drift apart.
