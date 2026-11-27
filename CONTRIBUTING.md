# Working in this repository

Two people, two areas: firmware and control on one side, mechanical design and build
on the other. One rule keeps the history honest.

**Current state:** the repository is being populated solo with the firmware,
documentation and media. Rishav is added as a collaborator in January, after the
holiday period, and pushes the mechanical half himself. `hardware/cad/` is empty
until then by design, not by omission.

## Commit your own work

Mechanical and CAD work is committed by its author. Do not hand files to someone else
to upload — the commit history is the only durable record of who built what, and a
bulk upload erases that permanently.

Adding CAD or test data:

```bash
git clone https://github.com/shriviswath/hybrid-warehouse-robot.git
cd hybrid-warehouse-robot
git checkout -b hardware/manipulator-cad

# export STEP + STL + a render from SolidWorks into hardware/cad/
git add hardware/
git commit -m "hardware: manipulator assembly, STEP + STL + isometric render"
git push -u origin hardware/manipulator-cad
```

Then open a pull request. For a two-person project a PR is not bureaucracy — it is
the reviewable record, and it is what makes the repository credible later.

## File formats

See [`hardware/README.md`](hardware/README.md). Short version: **no native
`.SLDPRT` / `.SLDASM`.** Export STEP for geometry, STL for browser preview and
printing, PNG for renders.

## Commit message style

```
<area>: <what changed>

firmware: request MAVLink data streams every 5 s
hardware: printed base v3 STEP, drive mounts moved 12 mm aft
docs: record FC supply minimum during combo, 4.8 V
```

Areas: `firmware`, `hardware`, `docs`, `media`, `repo`.

## Measurements

A number in this repository either has a test record under `hardware/tests/` or it
does not belong in the README. Items in [`docs/07-open-issues.md`](docs/07-open-issues.md)
move out when a measurement closes them — not when they are expected to pass.

## Before changing firmware

Read [`firmware/README.md`](firmware/README.md) first, in particular which of the two
`hybridbot_*` builds you are editing and the PlatformIO `src/main.cpp` trap described
there. Three things will bite silently if removed: the explicit LEDC channel
assignments, the non-blocking structure, and the latched boot e-stop.

After any firmware change affecting the flight path, re-run the link-loss test
([`docs/03-bringup-procedure.md`](docs/03-bringup-procedure.md), stage 5). It is the
one test that cannot be inferred from a successful hover.
