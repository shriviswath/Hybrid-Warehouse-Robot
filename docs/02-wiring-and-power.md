# Wiring, harness and power

GPIO assignment is in the [README pinout table](../README.md#pinout-authoritative) —
the single source of truth. This document covers everything above the pin level.

## Harness — ESP32 to Pixhawk

**Not currently fitted.** The cable was removed when flight moved to an RC
transmitter. To refit for issue O1:

```
ESP32 GPIO13 (TX) ──────────► Pixhawk TELEM2 pin 3 (RX)
ESP32 GPIO34 (RX) ◄────────── Pixhawk TELEM2 pin 2 (TX)
ESP32 GND         ─────────── Pixhawk TELEM2 pin 6 (GND)
                              Pixhawk TELEM2 pin 1 (5V) — NOT CONNECTED
```

Both sides are 3.3 V logic. No level shifter is required or wanted. TELEM2 is
`SERIAL2` in ArduPilot; TELEM1 stays free for a telemetry radio or a laptop.

## Power tree

All distribution is on a **single perfboard** carrying the flight and ground stages
together. As built:

| Rail | Source | Feeds | Status |
|---|---|---|---|
| Pack | 4S Li-Po, 6800 mAh 40C | ESCs, L298N motor input, shared perfboard rail | as built |
| 5–6 V servo rail | **dedicated BEC, ≥ 6 A** | 2× MG995 only | **correct — isolated as designed** |
| Pixhawk supply | **shared perfboard rail** | Pixhawk, buzzer, safety switch, GPS | **coupled to the drive stage — see below** |
| 3.3 V logic | ESP32 onboard regulator | ESP32 only | as built |

The servo isolation is the one most likely to cause damage, and it was done right.
The flight controller sharing a rail with the drive stage is the one still open.

### The flight controller shares a rail with the drive stage

An L298N switching four gearmotors puts commutation transients and sag onto whatever
rail it sits on. On the ground that costs an ESP32 reset. In the air, a supply dip at
the flight controller is a brownout, and ArduPilot cannot recover an aircraft whose
attitude estimator has just rebooted.

Not yet proven to be a problem here. Proven or dismissed by one measurement, tracked
as issue B2 and logged in `hardware/tests/`:

1. Multimeter on the Pixhawk's supply input, ground to a common point.
2. ESP32 running `drive_arm_combined`, command `combo` — wheels driving while the arm
   cycles, the worst case the vehicle can generate.
3. Capture the minimum, and specifically capture a **drive direction reversal**: both
   bridge legs switch at once, which is worse than a stall.
4. **Any excursion below 4.5 V at the flight controller means do not fly with the
   ground stage energised.**

If it sags: give the Pixhawk its own power module wired directly to the pack. The BOM
already lists one. That single change decouples the stages and costs a connector.

### Rules that are not negotiable

- Servos never run off the ESP32's regulator. An MG995 stall is 1.2–2.5 A each.
- All grounds commoned, including the ESP32's.
- Do not back-feed 5 V into TELEM2, and do not draw the ESP32's supply from it.
- An ESP32 reset under load is a sagging supply. Lower `slew` and `maxd` before
  suspecting firmware. Boot prints the reset reason; `BROWNOUT` is conclusive.

## Strapping pins

GPIO5 (ENA), GPIO12 (IN1) and GPIO14 (IN2) all move before user code runs: GPIO12 is
MTDI, GPIO5 pulses at boot, and GPIO14 emits a PWM burst from the ROM bootloader.

**Fit 10 kΩ pulldowns from ENA, IN1, IN2, IN3 and IN4 to GND at the L298N end.** No
firmware change prevents this behaviour.

GPIO2 (rack servo) is also a strapping pin. If a flash fails: hold BOOT, tap EN,
release BOOT, and disconnect GPIO12 and GPIO2 first.

## Flight controller mounting

Two items that are not optional on a vehicle with an arm hanging off the front:

1. **Open-cell foam loosely over the MS5611 barometer.** Without it, light and prop
   wash produce altitude drift that reads as a tuning fault.
2. **Gel pads or damped standoffs under the Pixhawk.** Four motors' vibration
   corrupts the accelerometer, and the attitude estimate follows it down.

Mount as near the centre of gravity as the frame allows, arrow forward. If it must be
rotated, set `AHRS_ORIENTATION` to match.

## Wire routing

The main harness currently runs high, between the arms, inside the propeller disc
plane. Lace it to the frame before props go on — issue B1.
