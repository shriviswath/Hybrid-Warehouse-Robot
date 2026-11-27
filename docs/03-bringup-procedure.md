# Bring-up procedure

Five stages. Each one answers a question the next stage assumes. Do not skip one.

---

## Stage 1 — ground hardware

Flash `firmware/drive_arm_combined/`. Serial 115200, line ending **Newline**.
Wheels off the ground, arm unloaded, nothing in the rack's path.

```
pins          read the boot pin report; no line may show HELD HIGH
l             latch off the dead-man
+ + + +       speed to 100%
w             forward
```

**Pass:** all four wheels turn in both directions, left and right independently
(`1`, `2`), at a duty that does not overheat.

Weak drive? `maxd 200`, then `maxd 255`, then `freq 500`. If nothing moves at 255 it is
not PWM — measure pack voltage at the L298N terminals *while driving*.

Then the arm:

```
det           detach
set r 180     declare the real rack angle
set g 0       declare the real gripper angle
att           attach
ext ret op cl
demo
```

**Pass:** rack travels 40–180°, gripper 0–60°, no buzzing at either end stop.

---

## Stage 2 — combined load

Same sketch:

```
combo
```

Wheels drive while the arm cycles.

**Pass:** no ESP32 reset, no servo jitter. A reset here is a power fault and it will
come back in flight, where it costs more.

---

## Stage 3 — ESCs (bench only, spare board)

Flash `firmware/esc_tuner_v2/`. **Props off. Frame clamped.**

```
ma            select all four
c             calibrate (battery disconnected first)
arm
m1 then s     sweep; press k when motor 1 first turns
m2 m3 m4      repeat
t             motor identification
x
```

Record per motor: which arm, rotation direction, breakaway pulse width.

With the Pixhawk fitted this stage only proves the ESCs are healthy — motor **order**
is set by MAIN OUT 1–4, not by these results. The second ESP32 has been removed from
the vehicle; run this on the bench with a spare board if you need it at all.

---

## Stage 4 — flight controller

Follow [`04-pixhawk-ardupilot-setup.md`](04-pixhawk-ardupilot-setup.md) in full.

**Pass:** tethered hover in STABILIZE from the GCS — bare airframe first, then with
the ground-drive mass fitted.

---

## Stage 5 — the app

Flash `firmware/hybridbot_fly/`. Phone joins WiFi `HybridBot`, opens
`http://192.168.4.1`, *Add to Home Screen*.

**Props off for steps 1–3.**

1. GROUND tab: drive and arm behave exactly as they did over serial.
2. FLIGHT tab: telemetry shows the real flight mode and battery voltage instead of
   `NO FC`. This proves the MAVLink link with no motor involvement.

   > **The TELEM2 cable is currently removed from the vehicle.** Refit it, flash
   > `hybridbot_fly` — *not* `hybridbot`; only the integration build prints the
   > receive diagnostics you will need — and work
   > [`07-open-issues.md`](07-open-issues.md) issue O1. Steps 3–5 cannot pass until
   > this one does.
3. ARM, move the sticks, watch servo outputs 1–4 respond in Mission Planner.
4. **Turn the phone's WiFi off mid-test.** Serial must print `[fs] phone link lost`
   then `[fs] commanding LAND`.
5. Props on, tethered, ALT_HOLD only.

Step 4 is the one people skip. A backgrounded browser tab is enough to drop the link,
and that failsafe is what brings the aircraft down when it happens.

---

## Troubleshooting index

| Symptom | Sketch and command |
|---|---|
| One wheel direction dead | `ground_drive_test_v4`, command `p` |
| Wheels weak at full speed | `drive_arm_combined`, `maxd` then `freq` |
| Servo buzzes at an end stop | `manip_bringup`, narrow with `lim` |
| Arm angle wrong after a reset | `det`, `set`, `att` |
| ESP32 resets under load | power, not code. Reproduce with `combo` |
| Upload fails | hold BOOT, tap EN, release BOOT; disconnect GPIO12 and GPIO2 |
| Motor spins during upload | GPIO5/GPIO14 boot pulses — fit the pulldowns |
| No flight telemetry in the app | TELEM2 wiring, `SERIAL2_BAUD`, and ArduPilot actually running |
