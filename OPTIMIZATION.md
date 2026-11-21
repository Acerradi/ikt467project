# Performance changes with exact baseline replay

The baseline is Git commit `5464d309dcd2570c390991c9a61820c4da55937c`.
These changes reduce repeated work in the existing model. They do not change
the social forces, contact constraint, solver tolerances, timestep, spawning,
random draws, scenario settings or diagnostic definitions. Every pedestrian
pair is still evaluated; there is no force cutoff or approximate neighbour
selection. The crossing remains uncontrolled and can still jam.

## Implementation

- `core/pedestrians.nls`: sort pedestrians once per step, snapshot neighbour
  state once per acceleration pass, and reuse the view-angle cosine. Accumulate
  pair forces without allocating a temporary two-element force list for every
  directed pair. Keep the original scalar expressions and neighbour summation
  order. The public single-pedestrian calculation still reads live state.
- `core/contact.nls`: snapshot positions and proposed displacements once per
  contact-discovery pass. Retain the original pair and constraint order. Compute
  the active pedestrian list once per projection solve, since membership does
  not change during that solve. All validity checks, projections, convergence
  limits and fallback rules remain the same.
- `core/checks.nls`: compute minimum swept separation and pass-through counts
  together, and visit each unordered pair once. Snapshot coordinates once for
  endpoint separation and overlap counts. Keep the original distance formulas
  and the endpoint check in the accumulated minimum.

Snapshots are local to a calculation, so later calls see subsequent state
changes. Updates remain synchronous. The four `.nlogox` files, `core/stats.nls`
and the entire `README.md`, including its TODO list, are unchanged.

## Validation

Tested in the actual NetLogo 7.0.4 headless runtime on Windows, against a pristine
copy of the baseline. Numeric replay comparisons require identical IEEE-754
bits, rather than an error tolerance.

| Replay fixture | Steps | Simulated duration | CSV comparison |
| --- | ---: | ---: | --- |
| Periodic hallway, 150 people | 250 | 5 s | Not enabled |
| Periodic hallway, 40 people, original anticipation | 250 | 5 s | Not enabled |
| Open hallway, noise and trails enabled | 300 | 9 s | Header only; identical |
| Door, 20 people on each side, 1.2 m opening | 500 | 10 s | Identical bytes |
| Traffic light, `dt = 0.07 s`, 2 s light intervals | 200 | 14 s | Identical bytes |
| Crossing, 1.8 horizontal + 1.8 vertical attempts/s | 1500 | 30 s | Identical bytes |

All six fixtures passed: **3,000 steps and 10,299,453 scalar/list comparisons**.
The harness compares the original globals, pedestrian variables, positions,
velocities, accelerations, contact-solver state, paths, counters, monitors and
simulation clock after every step. Log file paths are normalized to their
open/closed state because baseline and candidate write separate files. The next
32 random draws also match in every fixture. Door, traffic and crossing CSVs
contain matching time series. The hallway has no automatic `log-row` call, so
its CSV comparison covers only the header; its full state is still checked at
every step. These counts describe repeated state comparisons, not millions of
independent experiments.

Focused tests also passed:

- `ExactForceChecks`: 269,996 comparisons, both anticipation modes, open/periodic
  geometry, manual state changes, and four 100-step baseline replays with the
  contact constraint disabled to isolate the force update.
- `ExactContactChecks`: 75,559 comparisons of pair cuts, discovery order,
  corrected motion, internal solver state, counters and future random draws.
- `ExactDiagnosticsChecks`: 2,944 numeric comparisons across 120 synthetic
  states/path sets, including periodic seams, long paths and unsorted path IDs.
- Existing suites: `MathChecks`, `AnticipationChecks`, `BoundaryChecks`,
  `PeriodicChecks`, `SpeedChecks`, `EncounterChecks`, `ScenarioChecks`,
  `WallContactChecks`, `SeparationChecks`, `DiagnosticsChecks` and
  `ScenarioRegressionChecks --quick`.

This establishes exact equality for the tested cases. It is not an exhaustive
proof for every possible configuration or future code change. Keep the replay
tests when editing either the reference force reporters or their snapshot
implementation.

## Measured performance

Full `go` calls, with `run-checks?` enabled and CSV logging disabled. After an
untimed warmup run of each version, three repetitions ran serially, alternating
which version ran first. Each repetition reset the same seed and replayed to
the start of the measured interval. Setup, warmup and state comparisons are
excluded from the timings; the final state and future random draws matched
after every timed pair. No other test jobs ran during these measurements.

| Fixture | Simulated interval measured | Baseline median | Optimized median | Median paired speedup |
| --- | --- | ---: | ---: | ---: |
| Periodic hallway, 150 people | 1-3 s | 5.854 s | 3.491 s | 1.67x |
| Original anticipation hallway, 40 people | 1-3 s | 0.449 s | 0.264 s | 1.71x |
| Open hallway, light population | 4.5-9 s | 0.106 s | 0.090 s | 1.18x |
| Door | 5-10 s | 1.834 s | 1.346 s | 1.37x |
| Traffic light | 7-14 s | 0.498 s | 0.330 s | 1.53x |
| Crossing, 1.8 + 1.8 attempts/s | 25-30 s | 3.705 s | 2.181 s | 1.56x |

The speedup column is the median of the three paired baseline/optimized
ratios, not the ratio of the two separately computed time medians. Crossing
ratios ranged from 1.51x to 1.70x. The short, lightly populated open-hallway
timing is particularly sensitive to runtime noise. These are local headless
measurements; desktop rendering and other populations can change the gains.

## Reproducing the checks

From the repository root, create the baseline in an unused directory:

```powershell
New-Item -ItemType Directory -Force work | Out-Null
git archive --format=zip --output=work/baseline-5464d30.zip 5464d309dcd2570c390991c9a61820c4da55937c
Expand-Archive -LiteralPath work/baseline-5464d30.zip -DestinationPath work/exact-optimization-baseline
```

Then run the comparisons using the Java bundled with NetLogo (adjust the two
installation paths if needed):

```powershell
$netlogoJava = 'C:\Program Files\NetLogo 7.0.4\runtime\bin\java.exe'
$netlogoJar = 'C:\Program Files\NetLogo 7.0.4\app\netlogo-7.0.4.jar'
& $netlogoJava '-Djava.awt.headless=true' -cp $netlogoJar tests/ExactOptimizationChecks.java --mode parity
& $netlogoJava '-Djava.awt.headless=true' -cp $netlogoJar tests/ExactForceChecks.java --baseline work/exact-optimization-baseline/Pedestrians.nlogox
& $netlogoJava '-Djava.awt.headless=true' -cp $netlogoJar tests/ExactContactChecks.java --baseline work/exact-optimization-baseline
& $netlogoJava '-Djava.awt.headless=true' -cp $netlogoJar tests/ExactDiagnosticsChecks.java work/exact-optimization-baseline
```

The baseline directory can be changed with `--baseline` in the combined and
contact harnesses. The force harness takes the baseline model file, and the
diagnostics harness takes the directory as its first positional argument.
The combined harness also supports `--cases crossing` and `--quick` for a short
smoke check; the results above use the full replay, not `--quick`.

Run the full-step benchmark separately, without concurrent test jobs:

```powershell
& $netlogoJava '-Djava.awt.headless=true' -cp $netlogoJar tests/ExactOptimizationChecks.java --mode benchmark --repeats 3
```

The combined harness writes replay results, logging comparisons and benchmark
timings under the ignored `work/exact-optimization/` directory.
