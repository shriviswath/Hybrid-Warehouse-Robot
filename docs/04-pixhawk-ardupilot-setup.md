# Pixhawk 2.4.8 / ArduCopter setup

Windows only, no Linux toolchain required. **No firmware changes to
`hybridbot_fly.ino`** — it already speaks MAVLink over a UART, and the Pixhawk speaks
the same MAVLink on TELEM2.

## What the Pixhawk resolved

| Previous blocker | Now |
|---|---|
| No barometer responding on the ESP32's I2C bus | MS5611 onboard |
| MPU-6500 with no magnetometer, so no heading | MPU6000 + LSM303D on SPI, compass onboard |
| Custom ArduPilot board definition needed | Official target, nothing to author |
| WSL2 + ESP-IDF build chain | Mission Planner installer |
| Second-ESP32 motor pin remap | Superseded — MAIN OUT 1–4 define motor order |

## 1. Hardware checklist

Pixhawk 2.4.8 · buzzer · safety switch · **power module** (it will not run off the
ESP32) · micro-USB to a Windows PC · 4 ESC signal leads · 3 wires to the ESP32
(TX, RX, GND) · foam over the barometer · gel pads underneath.

## 2. Firmware

1. Install **Mission Planner** from ardupilot.org.
2. Connect by USB only. **No battery, no props.**
3. Setup → Install Firmware → **ArduCopter**.

### The flash-size trap

Many 2.4.8 clones carry a 2 MB flash chip behind a bootloader that only exposes 1 MB,
forcing the feature-stripped `fmuv2` build.

- If Mission Planner offers **fmuv3**, take it — that is the full firmware.
- If only fmuv2 appears, you have the 1 MB limit. It still flies; some optional
  features are compiled out.

Reconnect at 115200 over USB and confirm a heartbeat.

## 3. Frame and motor order

```
FRAME_CLASS = 1        Quad
FRAME_TYPE  = 1        X    (0 for a Plus layout — match the build)
```

Plug the ESCs into **MAIN OUT 1–4** following ArduPilot's quad diagram for the frame
type. Then Setup → Optional Hardware → **Motor Test**, props off:

- Test Motor A → motor **1** spins (front right on a quad X)
- B = 2, C = 3, D = 4, going clockwise
- Diagonal pairs counter-rotate; check against the frame diagram
- Wrong direction → swap any two of the three motor wires
- Wrong motor → move the ESC lead to the correct MAIN OUT

**Do not proceed until Motor Test matches the diagram exactly.**

## 4. Calibrations

In order, Setup → Mandatory Hardware:

1. **Accelerometer** — six orientations.
2. **Compass** — onboard. Spin through all axes, away from steel benches, motors and
   laptops.
3. **ESC calibration** — all-at-once: props off, battery disconnected, throttle to max
   in Mission Planner, connect battery, wait for the tune, throttle to min, confirm
   beeps.

### Radio calibration

An RC transmitter and receiver were borrowed and wired to RCIN, and all flight to date
has been on them. **If the transmitter is available, run the normal radio calibration**
— it is more reliable than hand-set values and it is what the aircraft has flown on.

The hand-set values below matter for the *override* path: `hybridbot_fly.ino` sends raw
PWM, so the ranges and trims it assumes must match the autopilot's. Set them by hand in
Config → Full Parameter List when no transmitter is on hand, and verify them even when
one is:

```
RC1_MIN = 1000   RC1_MAX = 2000   RC1_TRIM = 1500      roll
RC2_MIN = 1000   RC2_MAX = 2000   RC2_TRIM = 1500      pitch
RC3_MIN = 1000   RC3_MAX = 2000   RC3_TRIM = 1000      throttle
RC4_MIN = 1000   RC4_MAX = 2000   RC4_TRIM = 1500      yaw
```

These match exactly what `hybridbot_fly.ino` transmits: 1000–2000 µs, 1500 at centre,
throttle minimum 1000.

## 5. Parameters

```
SERIAL2_PROTOCOL = 2       MAVLink 2 on TELEM2
SERIAL2_BAUD     = 115     115200, matching the ESP32. ArduPilot's DEFAULT IS 57600 --
                           left unchanged, the ESP32 frames nothing and the phone
                           shows NO FC forever. Verify this one by eye
BRD_SER2_RTSCTS  = 0       no flow-control wires fitted; without this the port
                           can stay completely silent

RC_OVERRIDE_TIME = 1.0     a stalled phone link drops override in 1 s
FS_THR_ENABLE    = 1       RTL on failsafe. A GPS IS FITTED, so RTL is available
                           and is the better failsafe than LAND
BATT_MONITOR     = 4       analog V and I via the power module, then run the
                           battery calibration wizard
BRD_SAFETYENABLE = 1       keep the safety switch — a real safety feature
ARMING_CHECK     = 1       leave every check on; read refusal messages, fix causes
```

### GPS — fitted

The GPS/compass module supplied with the Pixhawk 2.4.8 kit is fitted. Exact model not
recorded; kit modules are commonly a u-blox NEO-series with an external compass.
Confirm it on Mission Planner's hardware page and record it in `hardware/README.md`.

With it fitted, **LOITER, POSHOLD and RTL are all available** and `FS_THR_ENABLE = 1`
(RTL) is the right failsafe.

Two consequences:

1. Use the **external** compass, not the internal one, and set compass priority
   accordingly — an internal compass this close to four ESCs and a battery lead reads
   badly. Calibrate with the GPS mast in its final flight position.
2. Do not arm in a position-dependent mode until satellite count and HDOP are
   acceptable. ArduPilot's arming checks enforce this when `ARMING_CHECK = 1`, which
   is why that stays on.

**Firmware note:** `hybridbot_fly.ino` still carries a header comment recommending
`FS_THR_ENABLE = 3` from when no GPS was fitted, and its `webui_fly.h` replaced the
LOITER and RTL buttons with STABILIZE. With a GPS on board those buttons are worth
restoring — the UI shipped with `firmware/hybridbot/` still has them, and the
`C,loi` / `C,rtl` commands are implemented in both builds.

## 6. Bring-up sequence

Props off for steps 1–5.

| # | Step | Pass condition |
|---|---|---|
| 1 | USB to Mission Planner | Heartbeat; HUD horizon follows the board when tilted |
| 2 | Motor test | Each motor matches the diagram, correct direction |
| 3 | Wire TELEM2 to the ESP32 | Phone UI shows real mode and battery, not `NO FC` |
| 4 | ARM from the phone, move sticks | Servo output page shows outputs 1–4 responding |
| 5 | **Phone WiFi off while armed** | Serial prints `[fs] phone link lost` then `[fs] commanding LAND` |
| 6 | Props on, tethered, ALT_HOLD | Stable hover |
| 7 | Free hover, then arm movement in flight | Attitude holds while the rack travels |

## 7. Worth knowing before the first hover

Mission Planner supports a **USB joystick**. If a gamepad can be borrowed, the first
hover on physical spring-return sticks is substantially safer than a touchscreen. Use
it for flight one, then move to the phone once the aircraft is trimmed.
