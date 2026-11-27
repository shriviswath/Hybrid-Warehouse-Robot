# Superseded firmware

**Do not flash anything in this directory.** Kept for the project record.

| Sketch | Replaced by | Why |
|---|---|---|
| `flight_board_bringup/` | the Pixhawk 2.4.8 running ArduCopter | ESP32-as-flight-controller attempt. No magnetometer on the MPU-6500, barometer never responded on I2C. See [`../../docs/archive/ardupilot-on-esp32-SUPERSEDED.md`](../../docs/archive/ardupilot-on-esp32-SUPERSEDED.md) |

Also superseded and not carried into this repository, listed so the record is complete:

| Sketch | Replaced by | Why |
|---|---|---|
| `hybridbot_app.ino` + `webui.h` | `hybridbot_fly/` | No flight control, and still carried the old `PIN_ENA = 17` |
| `servo_test.ino` | `manip_bringup/` | Placeholder limits, no drive-pin parking |
| `manip_angle.ino` | `manip_bringup/` | ESP32Servo variant; the raw-LEDC approach was kept instead |
