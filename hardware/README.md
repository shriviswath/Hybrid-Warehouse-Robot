# Hardware

Mechanical design, CAD and physical test data. This half of the project is
**Rishav Kamboj's work** — design, printing, assembly and airframe integration.

> **`cad/` is empty on purpose.** Rishav is added as a collaborator in January, after
> the holiday period, and commits the exported geometry himself so the history
> attributes it to him. Nothing here is missing; it is scheduled.

## Construction

| Assembly | How |
|---|---|
| Quad airframe | **ready-made frame kit** — arms, motor mounts, centre plates |
| Lower platform, legs, drive mounts | **fully 3D-printed**, designed for this project |
| Manipulator — rack, pinion, carriage, gripper | **fully 3D-printed**, designed for this project |

The printed lower assembly carries the four driven wheels, doubles as the landing
gear, and mounts the rack-and-pinion manipulator forward of the rotor centroid.

## What goes here, and in what format

Native SolidWorks files are **not** committed. They are binary, large, unreadable
without a licence, and GitHub cannot preview them. Export instead:

| Directory | Format | Why |
|---|---|---|
| `cad/step/` | `.STEP` / `.STP` | Neutral, exact geometry. Opens in any CAD package. The useful archive |
| `cad/stl/` | `.STL` | GitHub renders STL in the browser — a reviewer sees the part without installing anything. Also the print format |
| `cad/renders/` | `.png` | Isometric and exploded views for the README and the paper |
| `tests/` | `.md` + `.csv` | Weight, thrust, CG and supply measurements |

Keep native `.SLDPRT` / `.SLDASM` in cloud storage and link them from this file. If
they must be versioned, use Git LFS — decide that before the first push, not after.
`.gitignore` already blocks the native formats so an accidental `git add .` cannot
bloat the repository.

## Bill of materials

### Aerial

| Item | Spec | Qty |
|---|---|---|
| Airframe | ready-made quad frame kit | 1 |
| Motor | A2212/13T, 1000 KV | 4 |
| ESC | SimonK, 30 A | 4 |
| Propeller | 1045 — **see accepted limitation A2** | 4 |
| Flight controller | Pixhawk 2.4.8 + buzzer + safety switch | 1 |
| GPS / compass | module supplied with the Pixhawk kit — exact model not recorded (A8) | 1 |
| Power module | for the Pixhawk's 5 V rail — **present in the BOM, not currently wired that way** (B2) | 1 |
| Battery | 4S Li-Po, 6800 mAh, 40C | 1 |
| RC transmitter + receiver | borrowed; wired to Pixhawk RCIN. All flight to date | 1 |

### Ground and manipulator

| Item | Spec | Qty |
|---|---|---|
| Gearmotor | TT, 1:48 — four driven wheels, one per leg | 4 |
| Motor driver | HW-095 (L298N) | 1 |
| Servo | MG995 — rack-and-pinion lift, gripper | 2 |
| Servo BEC | 5–6 V, ≥ 6 A | 1 |
| Printed structure | PLA — lower platform, legs, manipulator | — |

### Control

| Item | Spec | Qty |
|---|---|---|
| MCU | ESP32 DevKit — one on the vehicle | 1 |
| Pulldown | 10 kΩ, on each L298N control line — **not yet fitted** (A5) | 5 |

All-up mass as built: **approximately 2.3 kg**, not weighed since final assembly.

> A second ESP32 served as a flight board in earlier revisions and has been removed.
> Propeller choice is unresolved: A2212 1000 KV with 1045 props is a 3S combination
> run on a 4S pack. See [`../docs/07-open-issues.md`](../docs/07-open-issues.md).

## Open mechanical and electrical items

Issue B2 must be measured before the next flight. A1 and A3–A4 are accepted
unmeasured for this demonstrator. Record anything you do measure in `tests/`.

## Licence

MIT, same as the rest of the repository — see [`../LICENSE`](../LICENSE).
