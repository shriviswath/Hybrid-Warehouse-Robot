# SUPERSEDED — ArduPilot on the ESP32 ("Path B")

> **Abandoned November 2026** in favour of a Pixhawk 2.4.8. See
> [`../04-pixhawk-ardupilot-setup.md`](../04-pixhawk-ardupilot-setup.md). The source
> from this attempt is kept at `firmware/archive/flight_board_bringup/`.

## What was attempted

Running ArduCopter on ESP32-A itself — a supported but secondary ArduPilot target —
with an MPU-6500 IMU and a barometer on I2C (SDA 27 / SCL 14), four ESCs on
GPIO19/21/22/23, and battery sense on IO36. ESP32-A would have been the flight
controller, with ESP32-B relaying phone commands to it.

## Why it was dropped

1. **No magnetometer.** The MPU-6500 has none, so there is no heading reference and no
   compass-dependent mode.
2. **Barometer silent.** An I2C scan on SDA 27 / SCL 14 found only the IMU at `0x68`.
   Without a barometer there is no altitude hold.
3. **Toolchain cost.** A custom board definition plus WSL2 and ESP-IDF, against a
   project deadline, for a target with less flight history.
4. **Shared-core latency.** A WiFi stack and a high-rate attitude loop on the same
   silicon is a risk that a 2.3 kg vehicle with a cantilevered arm should not carry.

## What was kept from it

- The ESC output mapping work, which became `firmware/esc_tuner_v2/`.
- The decision to keep the phone link on ESP32-B rather than on the flight board,
  which survived the change of flight controller unaltered — and is why swapping in
  the Pixhawk required **no change** to `hybridbot_fly.ino`.
