# ikt467project
Languages for modeling and simulation exam project

## Run the model

Open `Pedestrians.nlogox` in NetLogo 7.0.4, click **setup**, then **go**.
Click **go** again to pause, or **step** to advance one integration step.

## Scenarios and shared modules

The pedestrian model is a module that each scenario includes:

| Model | Scenario |
| --- | --- |
| `Pedestrians.nlogox` | Hallway (open or periodic), Figure 3 lane experiment |
| `scenarios/Door.nlogox` | Corridor split by a wall with one door, crowds on one or both sides |
| `scenarios/TrafficLight.nlogox` | Walkway crossing a road with a light (colour + `switch-time`) |
| `scenarios/Crossing.nlogox` | Horizontal and vertical corridors crossing at right angles |

Each model starts with, for example,
`__includes [ "../core/pedestrians.nls" "../core/contact.nls" "../core/checks.nls" "../core/stats.nls" ]`
and owns only its geometry, spawning, `setup` and `go`:

- `core/pedestrians.nls`: forces, integration and walls. Geometry is data: any
  list of `[x1 y1 x2 y2]` wall segments, each exerting the wall force and
  impenetrable to centres closer than 0.2 m. Optional per-pedestrian
  `waypoints` (used for the door) and `stop-x` (used for the red light).
  Including models must declare the interface variables listed in its header
  and define a `scenario-exit-check` hook.
- `core/checks.nls`: correctness checks: pass-throughs, smallest separation,
  wall clearance, speed cap, overlap count and the lane counter.
- `core/stats.nls`: mean, standard deviation and variance (sample and
  population), plotting helpers and CSV logging on the first completed step
  at or after each 0.1 s boundary, timestamped with its actual elapsed time.

### Minimum pedestrian separation

All four scenarios now enforce **0.5 m centre-to-centre separation**, including
between ticks and across periodic seams. Initial/spawn clearance remains 0.6 m.
The displayed radius and wall-contact radius remain 0.2 m, so the new pair
constraint leaves a 0.1 m gap between the drawn circles at closest contact.

`core/contact.nls` adds a hard geometric constraint after the ordinary
social-force step. It projects only unsafe candidate motion onto joint
pedestrian, wall, stop-line and speed constraints, seeking the smallest total
squared displacement change. It checks the entire swept paths again after
correction. It does not strengthen or replace the paper's repulsive potential.
Unaffected proposals retain their original preferred and actual velocities.
For corrected motion, velocity is displacement / dt and preferred velocity
is set consistently with the same individual `v = cap(w)` relation. Arrival
and door-passage decisions use the final corrected paths.

This is an additional modelling assumption, not an equation from the original
paper. All original force formulas and parameters remain unchanged. Crowds
may queue or jam instead of passing through each other; a perfectly symmetric
head-on pair need not spontaneously choose a passing side. The constraint
is deterministic and does not add randomness.

`pair-contact-enabled?` defaults to true and `minimum-separation` to 0.5 at
setup. For a raw soft-model comparison only, use `set pair-contact-enabled?
false` after setup in the command centre. The saved independent-solver replay
does this explicitly because its starting fixture already overlaps. Normal
production tests require the constraint to remain enabled. Invalid manual
placements closer than the minimum are rejected. `contact-solver-fallbacks`
reports any step where the bounded solver had to use a safe stopped state.

Each model has a `run-checks?` switch, check monitors, plots and a BehaviorSpace
experiment. The door, traffic-light and crossing models also have `log-csv?`
switches. Run an experiment
headless with, for example:

```bash
JAVA_HOME=/opt/NetLogo-7.0.4/lib/runtime /opt/NetLogo-7.0.4/netlogo-headless.sh \
  --model "$PWD/scenarios/Door.nlogox" --experiment door-width-sweep \
  --table "$PWD/work/door-width-sweep.csv" --threads 8
```

To run all four experiments (`figure-3-lanes`, `door-width-sweep`,
`light-cycle`, `crossing-flow-sweep`) with one command, use
`run-experiments.sh` from the repository root (bash; on Windows use WSL or
Git Bash):

```bash
./run-experiments.sh                                   # all experiments
./run-experiments.sh door-width-sweep light-cycle      # only the named ones
THREADS=8 ./run-experiments.sh                         # limit parallel runs
NETLOGO_HOME=/path/to/NetLogo ./run-experiments.sh     # other install location
```

- Arguments: experiment names to run; with none, all run in the order above.
- `THREADS`: parallel BehaviorSpace runs (default: number of CPU cores).
- `NETLOGO_HOME`: NetLogo install directory (default `/opt/NetLogo-7.0.4`).
- `JAVA_HOME`: Java runtime (default `$NETLOGO_HOME/lib/runtime`).

Each experiment's table output is written to `work/<experiment>.csv`,
overwriting any earlier result.

### Scenario corrections and regression tests

The force formulas, parameters, Gaussian desired speeds and individual
`v = cap(w)` relation are unchanged. The existing relative/original prediction
switch is retained. Hard wall contact and the red-light braking target are
scenario additions to the social-force equations, not terms from the paper.

- Wall contact now tests the whole step against segment faces and round
  endpoints, and resolves simultaneous contact constraints together.
- Held pedestrians cannot cross their red stop line. The smooth approach
  braking remains; a hard boundary handles fast arrivals and crowd pressure.
  Light changes occur at exact scheduled times, splitting a step if needed.
  Everyone uses the same state at the start of each substep.
- Door passages are sorted by crossing time within the step, with pedestrian
  ID used only to break an exact time tie.
- Checks, statistics, plots and logging do not consume the simulation's
  random stream. Enabling them therefore does not change physical trajectories.
- Pass-through detection uses stored start/end paths in any direction,
  including periodic seams and pedestrians removed at exits. This diagnostic
  uses a closest centre distance below 0.2 m; ordinary circle overlap is
  measured separately below 0.4 m. The added 0.5 m constraint prevents both
  in ordinary runs; the raw soft force law alone permits them.

`WallContactChecks`, `DiagnosticsChecks` and `ScenarioRegressionChecks` cover
these regressions. The last suite includes bounded crowd runs in all four
models with both prediction options, alongside the existing mathematical tests.

## Files

- `Pedestrians.nlogox`: runnable hallway scenario: interface, documentation,
  and the hallway-specific `setup`, `go` and spawning code. It loads the shared
  model with the four shared `core/*.nls` includes.
- `core/pedestrians.nls`: the shared pedestrian model (breed, state, forces,
  integration, general wall contact, waypoints, stop lines). It defines no
  `setup`/`go`; scenarios call `ped-init-constants`, `ped-check-parameters`
  and `ped-step`, and define the `scenario-exit-check` hook. Its header lists
  the interface variables every including model must declare. In NetLogo it
  opens as a separate code tab and is the only copy of this code.
- `core/checks.nls`, `core/stats.nls`: shared correctness checks and
  statistics/plotting/logging (see *Scenarios and shared modules*).
- `core/contact.nls`: joint deterministic contact projection, 0.5 m pair
  clearance, swept paths, periodic images and consistent corrected velocities.
- `tests/SeparationChecks.java`: independent Java swept-distance checks,
  head-on/oblique/periodic encounters, coupled walls and stop lines, creation
  order, corrected arrivals, free-motion preservation and timestep sensitivity.
- `scenarios/Door.nlogox`, `scenarios/TrafficLight.nlogox`,
  `scenarios/Crossing.nlogox`: the other scenario models, each with its own
  Info tab and BehaviorSpace experiment.
- `run-experiments.sh`: runs every BehaviorSpace experiment headless in one
  command and writes the tables to `work/` (see *Scenarios and shared
  modules* for arguments).
- `tests/ScenarioChecks.java`: vertical, oblique and door-post wall contact;
  door waypoint and 0.8 m blocking; red-light stop without overshoot and
  green recovery; crossing exits; lane counting and the 1.5 m rule;
  pass-through detection; statistics helpers.
- `tests/WallContactChecks.java`: swept post/corner contacts, simultaneous
  wall constraints, wall-order invariance, stop lines and periodic seams.
- `tests/DiagnosticsChecks.java`: deterministic observation, swept pair
  diagnostics, departed agents and elapsed-time sampling without drift.
- `tests/ScenarioRegressionChecks.java`: exact light-event timing, red
  boundary and release, chronological door passages, and crowd invariants in
  all four models with both prediction options. `--quick` skips crowd runs.
- `tests/BoundaryChecks.java`: wall/exit regression checks using NetLogo's own
  headless API. Run from the repository root with NetLogo's Java runtime and
  jar; add `--stress` for crowded simulations and a larger-timestep check.
  Incomplete draining is reported explicitly as a model outcome. Add
  `--require-drain` alongside `--stress` to make it a failing acceptance check.
- `tests/PeriodicChecks.java`: periodic seam-force equivalence, wrapping/state
  retention, fixed population, placement, repeatability and mode-switch checks.
  Add `--stress` for runs with 150 and 200 pedestrians.
- `tests/SpeedChecks.java`: Gaussian sampling, positive speeds, repeatable/fixed
  individual values, per-person driving forces and speed caps, and new defaults.
- `tests/MathChecks.java`: independent potential gradients, angular weights,
  isolated-motion solution, and wall-contact speed-cap consistency. It also
  prints near-singularity force diagnostics.
- `tests/EncounterChecks.java`: compares a saved interior encounter with the
  independent solver in both prediction modes, including all neighbors. It
  checks that the relative mode avoids this particular recorded overlap.
  Passing it is not a general collision-free guarantee.
- `tests/AnticipationChecks.java`: relative-velocity signs and lateral
  components, equal-velocity circular limit, common-boost invariance, raw
  pair-force antisymmetry, both periodic seams, original-mode preservation,
  saved default and setup-only mode changes.
- `tests/BehaviorComparison.java`: paired actual-NetLogo simulations of both
  prediction modes. Checks bounds, population and `v = cap(w)` every step;
  samples separation and behavior every 0.1 s. Arguments are seed, mode
  (`original` or `relative`), and simulated seconds; defaults run both modes
  for seeds 42, 1 and 7 over 60 s. Results go to `work/`.
- `tests/diagnostics/independent_solver.py`: NumPy-only reference solver for
  the saved encounter. Default: first-order and RK4 timestep/softening checks.
  `--quick` runs a shorter comparison; `--variants` tests deliberate prediction
  and perception changes without editing the production model.
- `tests/diagnostics/full_variant_audit.py`: matched 60 s comparison of three
  predictions using the independent solver; this is a diagnostic experiment.
- `tests/fixtures/`: fixed 150-person states from seed 42 at 0.02 and 20.52 s.
  These reproduce the investigation without requiring old ignored CSV files.

For the default Windows installation, run in PowerShell:

```powershell
$netLogoHome = 'C:\Program Files\NetLogo 7.0.4'
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/BoundaryChecks.java --stress
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/PeriodicChecks.java --stress
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/SpeedChecks.java
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/MathChecks.java
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/EncounterChecks.java
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/AnticipationChecks.java
foreach ($suite in 'ScenarioChecks', 'WallContactChecks', 'SeparationChecks', 'DiagnosticsChecks', 'ScenarioRegressionChecks') {
  & "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" "tests/$suite.java"
  if ($LASTEXITCODE -ne 0) { throw "$suite failed" }
}
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/BehaviorComparison.java 42 relative 60
```

On Linux, with NetLogo installed in `/opt/NetLogo-7.0.4`:

```bash
N=/opt/NetLogo-7.0.4
for t in BoundaryChecks PeriodicChecks SpeedChecks MathChecks EncounterChecks AnticipationChecks ScenarioChecks WallContactChecks SeparationChecks DiagnosticsChecks ScenarioRegressionChecks; do
  "$N/lib/runtime/bin/java" -Djava.awt.headless=true -classpath "$N/lib/app/netlogo-7.0.4.jar" tests/$t.java || break
done
```

The independent diagnostics require Python with NumPy:

```powershell
python tests/diagnostics/independent_solver.py --quick
python tests/diagnostics/independent_solver.py --variants
python tests/diagnostics/full_variant_audit.py
```

Generated diagnostic results are written to `work/`; alternative prediction
experiments never change `Pedestrians.nlogox` or its controls.

The `work/` directory and generated `preview.png` are local files excluded from Git.

## TODO
- ~~Add scenarios~~
  - ~~Pedestrians must move through a door which is only wide enough for one pedestrian at a time.~~
  - ~~Pedestrians must wait at a traffic light which has attributes color (Green or red) and Switch time (Time before switching colors)~~
  - ~~A crossing where pedestrians will walk both horizontally and vertically. A
    crossing is not an object. We are just going to position pedestrians and
    obstacles such that a crossing occurs. We will observe how this impact
    walking speed and travel time, and want to see if the pedestrians are able
    to walk though the crossing without stopping~~
- ~~Correctness~~
  - ~~In order to determine correctness of our model, we will compare the data from
    the simulations with our expected results. We expect that pedestrians move
    at their desired speed and do not exceed it, but slow down if there are other
    pedestrians or obstacles in their way. With no obstacles and few people, we
    expect the pedestrians to move directly to their desired position. With a door
    and a significant amount of people, we expect pedestrians to slow down or wait
    for the person in front to move through the door, and if it is very crowded, for
    people to alternate moving through the door from each side. When there is a
    red traffic light, we expect pedestrians to slow down and wait at the position
    right before the light, and for them to speed up to their desired speed when the
    light turns green. The model will fail the correctness check if pedestrians move
    through each other or obstacles. Lastly, we except pedestrians to move in lanes
    when it is very crowded. We consider pedestrians to be part of a lane if the
    shortest path between their position and the line formed of the linear regression
    of the position of all members of the lane is less than 1.5 meters. To validate
    our implementation we will redo the experiment presented in figure 3 of [1]. We
    will create a walkway with a pedestrian density of 0.3 per m2 and examine if the
    number of lanes increases linearly with the width of the walkway.~~
- ~~Logging and plots~~
  - ~~Mean, standard deviation and variance of numerical variables with respect to time?~~_
- Check for errors and run experiments
