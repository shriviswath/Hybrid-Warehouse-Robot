# System architecture

## Objective

Design, develop, implement and validate a hybrid warehouse robot combining ground
mobility, multirotor flight and an onboard manipulator, such that cargo can be moved
across a floor, lifted over an obstacle or to an elevated location, and placed.

## The one structural decision

**Nothing on the ESP32 stabilises the aircraft.** An earlier iteration attempted
exactly that — see [`archive/ardupilot-on-esp32-SUPERSEDED.md`](archive/ardupilot-on-esp32-SUPERSEDED.md)
and `firmware/archive/flight_board_bringup/`. It was abandoned because the available
IMU had no magnetometer, the barometer on that bus never responded, and a WiFi stack
sharing silicon with a high-rate attitude loop is a latency gamble a 2.3 kg vehicle
with a cantilevered arm should not take.

With a Pixhawk 2.4.8 running ArduCopter, attitude, altitude and — with the GPS fitted
— position are handled by mature code on dedicated hardware. The ESP32 is a transport
layer and a ground-vehicle controller.

That decision is why the project still has a flying aircraft today. When the ESP32 ⇄
Pixhawk link could not be made to work, the aircraft simply flew on a transmitter
instead. Had the ESP32 been the flight controller, a silent UART would have grounded
it outright.

## Layers

| Layer | Runs on | Responsibility |
|---|---|---|
| Operator, ground | browser (phone or laptop) | Drive, steering, arm controls, telemetry display |
| Transport | ESP32 | SoftAP, WebSocket server, command validation, MAVLink translation |
| Ground actuation | ESP32 | Differential drive ramping, servo slew limiting |
| Flight control | Pixhawk 2.4.8 | Attitude, altitude, position, arming logic, failsafes, mixing |
| Actuation | ESCs / L298N | Power stage |

## As designed vs as built

The table above is the design. The vehicle differs in one place, and it is recorded
rather than smoothed over.

| Layer | As designed | As built today |
|---|---|---|
| Operator, ground | browser over the robot's SoftAP | **as designed** |
| Transport | ESP32, MAVLink over UART2 | implemented in firmware; **TELEM2 cable removed from the vehicle** |
| Operator, flight | the same browser, via the transport layer | **RC transmitter into Pixhawk RCIN, Mission Planner as GCS** |
| Flight control | Pixhawk 2.4.8 / ArduCopter | **as designed**, with GPS |

So the project's distinguishing claim — *one operator interface for both modes* — is
**implemented but not connected**. The two halves are independent systems sharing a
frame and a power board.

This was decided deliberately. With the ground side working from the browser and the
aircraft flying on a transmitter, the project's objectives were demonstrable, and
rewiring a flying aircraft to chase a link it no longer needed carried more risk than
value against the remaining schedule. The MAVLink client was left in the firmware
rather than stripped out, so nothing was lost.

Unifying the two under one interface is recorded as future work — issue O1 in
[`07-open-issues.md`](07-open-issues.md). It is a wiring and parameter problem rather
than a design problem: the firmware side is complete, including the diagnostics
needed to debug it.

## Mode behaviour

The two modes share the manipulator and never share the propulsion. Behaviour differs
between the two firmware builds — see [`../firmware/README.md`](../firmware/README.md).

| Event | `hybridbot` (on the robot) | `hybridbot_fly` (integration build) |
|---|---|---|
| Entering FLIGHT | wheels stop, throttle resets to minimum | same |
| Arm in FLIGHT | read-only, holds position | commandable at 25°/s |
| Servos in any mode | never detached | never detached |
| ARM request | refused unless throttle at minimum | same |

The servos are never detached in either build: a rack-and-pinion gripper back-drives,
so a limp servo drops the cargo.

## What is deliberately out of scope

Autonomous navigation, SLAM, fiducial-based bin identification and automatic mode
selection. The platform is semi-autonomous by design: the operator chooses the mode
and flies the traverse. Autonomy is future scope and is not claimed anywhere in this
repository.
