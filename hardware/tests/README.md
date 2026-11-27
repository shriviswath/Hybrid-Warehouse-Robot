# Physical test records

One file per measurement session. Raw numbers, the conditions, and what was concluded
— so a result can be trusted later or thrown out for a stated reason.

Templates below. Replace the bracketed fields; do not delete the units.

## Static thrust test

Accepted limitation A1 — fill in if you measure it. Props on, frame inverted and restrained, or hung
from a scale.

| Field | Value |
|---|---|
| Date | |
| Battery | 4S 6800 mAh 40C, resting voltage ___ V |
| Propellers | 1045 / 8045 / 9045 |
| All-up mass | ___ g |
| Thrust, all four at 100% | ___ g |
| **Thrust-to-weight** | ___ : 1 — **2:1 minimum** |
| Pack voltage under full throttle | ___ V |
| Motor case temperature after 30 s | ___ °C |
| Verdict | |

Record the motor temperature. On a 4S pack with 1045s this is the number that reveals
the overload, not the thrust figure.

## Centre of gravity

Accepted limitation A3 — fill in if you measure it.

| Field | Value |
|---|---|
| Date | |
| Balance point, measured from rotor centroid | ___ mm fore / aft |
| Arm position during the measurement | retracted (180°) / extended (40°) |
| Battery position | |
| CG shift between arm retracted and extended | ___ mm |
| Verdict | |

Measure in **both** arm positions. A CG that moves when the rack travels is the aerial
manipulation problem in one number.

## Flight controller supply under load

Issue B2 — **blocking**. Probe at the Pixhawk's supply input, not at the battery.

| Field | Value |
|---|---|
| Date | |
| Resting voltage at the FC supply input | ___ V |
| Minimum during `combo` (wheels driving + arm cycling) | ___ V |
| Minimum during a drive direction reversal | ___ V |
| Any Pixhawk reboot, buzzer re-init or Mission Planner link drop | yes / no |
| **Below 4.5 V at any point** | yes / no — **yes means do not fly with the ground stage energised** |
| Verdict | |

A reversal is worse than a stall for transients: both bridge legs switch at once.
Capture it specifically.

## Drive pack voltage under load

Accepted limitation A4. Measure at the L298N motor terminals *while driving*, not at rest.

| Field | Value |
|---|---|
| Date | |
| Resting pack voltage | ___ V |
| Voltage, all four motors at 100% duty, wheels loaded | ___ V |
| Voltage, `combo` running (drive + arm) | ___ V |
| ESP32 reset observed | yes / no |
| Verdict | |
