# ikt467project
Languages for modeling and simulation exam project

## Run the model

Open `Pedestrians.nlogox` in NetLogo 7.0.4, click **setup**, then **go**.
Click **go** again to pause, or **step** to advance one integration step.

**Relative-velocity anticipation is now the default.** The switch
`relative-anticipation?` selects the pair prediction on **setup**:

- On: `s_ij = T * (v_j - v_i)`, using both pedestrians' actual x/y velocities.
- Off: original 1995 prediction `s_j = T * |v_j| * e_j`.

The `active prediction` monitor shows the formula currently running. Changing
the switch while walking takes effect only on the next setup. Both modes use
identical initial positions and sampled attributes for the same seed.

This is a single relative-velocity extension from
[Johansson, Helbing and Shukla, Eq. (12)](https://arxiv.org/html/0810.4587#S3.SS3.SSS1),
not the complete calibrated later model. The ellipse, exponential potential,
analytic gradient, perception weighting, preferred/actual velocity distinction,
wall forces and previous parameter values are retained. The interaction is
still soft; this extension does not guarantee nonoverlapping physical bodies.

The hallway has two boundary modes. Choose `periodic?`, then click **setup**.
The active-mode monitor shows which mode is running. Orange pedestrians walk
right; blue pedestrians walk left. The top and bottom remain solid walls.

**Periodic mode** is the default. It starts with `population = 150` pedestrians
randomly scattered throughout the hallway, split equally between directions.
Pedestrians crossing one end reappear at the other with their identity, y,
actual/preferred velocities, walking direction and force bias retained. The
population stays fixed and no new pedestrians are spawned. Their desired
directions are along the hallway, rather than towards random terminal points.
This avoids repeatedly steering them across the hallway after each wrap.

The x period is exactly `50` m, with coordinates wrapped into `[0,50)`. Pair
forces and initial-placement clearance use the minimum-image displacement
across the seam. NetLogo's built-in wrapping is disabled because the view has
51 patches, while the modeled hallway length is 50 m. `seam crossings` counts
boundary crossings in either direction. Density is population / `(50 * 10)`;
150 pedestrians give exactly `0.3` people/m^2. This uses the 500 m^2 physical
hallway area, not the patch-grid area or the smaller area available to centres.

This comparison is motivated by the discussion of periodic boundaries and lane
stability in [Section IV of Helbing & Molnar](https://arxiv.org/html/cond-mat/9805244#S4).
No lanes are assigned. Lane stability remains an experimental outcome. The
paper's illustrated simulations omit fluctuations, so `noise-strength` defaults
to `0`. Each pedestrian has an individual Gaussian desired speed, sampled once
at creation with mean `1.34 m/s` and standard deviation `0.26 m/s`. Nonpositive
draws are resampled; this rejects an extremely small tail at the default values.
The anticipation horizon is `2` seconds, separate from the integration timestep.

**Open mode** starts empty. Pedestrians enter near either end and get an
independently sampled destination at the opposite end. That destination stays
fixed. They leave when they reach the target or cross the opposite exit plane.
Anyone pushed back out of their entrance is counted separately as `returned`.

- `periodic?`: selects the boundary mode on **setup**.
- `relative-anticipation?`: on by default; selects relative actual velocity
  instead of the original desired-direction prediction on **setup**.
- `population`: initial fixed population in periodic mode; requires **setup**.
- `spawn-rate`: total attempted arrivals per simulated second, shared by both
  ends in open mode; default `1`. Set it to `0` to let that mode empty. Ignored
  in periodic mode.
- `dt`: simulated seconds per integration step; default `0.02`.
- `mean-desired-speed`: mean of the desired-speed distribution; default `1.34 m/s`.
- `desired-speed-sd`: its standard deviation; default `0.26 m/s`. Set to `0`
  for identical speeds. Both distribution controls apply at creation. Changing
  them does not alter existing pedestrians; use **setup** to resample a periodic
  population. Each pedestrian retains their sampled `desired-speed` across
  wraps, and their own speed cap is `1.3` times that individual value.
- `noise-strength`: once-assigned random-force magnitude in m/s^2; default
  `0` (off). Initial positions and desired speeds remain random.
- `simulation-seed`: seed for initial placement and, in open mode, arrivals and
  random-force directions; default `42`. Click **setup** after changing it.
  The same seed and settings reproduce the same run.
- `show-trails?`: draw pedestrian paths; off by default. **setup** clears them.

In open mode, arrivals follow a Poisson process with an expected
`spawn-rate * dt` arrivals per step. Starting and destination y-coordinates are sampled uniformly from `2.25`
to `10.75` m. A new pedestrian starts at rest at `x = 1` or `x = 49` and must
initially be at least `0.6` m from existing pedestrians. An arrival is skipped
and counted as blocked if no clear position is found in 20 attempts. Periodic
initialization uses uniform positions across the whole hallway with `0.6` m
clearance, including across the seam.

The model includes desired motion, elliptical pedestrian repulsion, wall
repulsion and the once-assigned random force. Attraction is deferred.
The force bias is inactive at the zero-strength default. The mean desired-speed
and desired-speed SD monitors show the current population's sample statistics,
which will differ slightly from the configured distribution parameters.

Read the model's **Info** tab for the equation mapping, parameters and numerical
choices. Pedestrian repulsion is soft, so circles can overlap. The two hallway
walls additionally enforce solid contact: centres remain at least `0.2` m from
the walls. The preferred velocity into a wall is removed, its tangential
component is retained, and actual velocity is recalculated from the paper's
speed cap before updating along-wall position.
This boundary rule supplements the social-force equations. The open ends remove
departing pedestrians before they can leave the NetLogo world. Counts satisfy
`spawned = walking + arrived + returned` in open mode. In periodic mode,
`spawned = walking = initial population`, with arrivals and returns both zero.

Starting positions do not prescribe lanes. Higher arrival rates can increase
crowding and the computational cost of all-pairs interactions.

With the original prediction and 2 s horizon, crowded open-mode runs can retain interacting
pedestrians near their different exit targets after arrivals stop. In the earlier
30-by-9 m geometry, seed 42 at spawn-rate 5 for 120 s left two pedestrians after
a further 300 s without arrivals. That historical result is not a validation
result for the new dimensions. Wall bounds and population accounting still held. This flow
limitation remains visible in the walking count; it is not counted as arrival.
The default periodic mode is covered separately by fixed-population tests.

## Mathematical audit and original-model diagnosis

### Validation of the implemented relative option

The actual NetLogo model was tested for 60 simulated seconds in both modes
with seeds 42, 1 and 7: six completed runs, each with 150 pedestrians in the
50-by-10 m periodic hallway, `dt = 0.02`, `T = 2`, and no random force. Bounds,
fixed population/density, individual speed caps and `v = cap(w)` held at every
step. Individual desired speeds remained unchanged. Behavior was sampled every
0.1 s; speed statistics exclude the first 10 s.

| Metric, pooled equally across three runs per mode | Original | Relative |
| --- | ---: | ---: |
| Mean overlapping pairs per sampled frame (distance < 0.4 m) | 4.8667 | 0.00278 |
| Mean absolute lateral speed, 10–60 s | 0.2548 m/s | 0.0781 m/s |
| Mean forward speed, 10–60 s | 1.2489 m/s | 1.2808 m/s |
| Minimum sampled centre separation | 0.0260 m | 0.3732 m |
| Pair-sample observations closer than 0.1 m | 45 | 0 |

The reductions are about 69% in mean lateral speed and 99.94% in mean sampled
overlapping pairs. Five overlapping pair-sample observations remain in relative
mode across 1,800 sampled frames. These finite runs and sampled distances do
not guarantee absence of overlap or events between samples.

The saved two-pedestrian encounter, simulated with all 150 neighbors, was also
replayed in NetLogo. Relative anticipation keeps the pair at least 0.6388666 m
apart at every integration step and agrees with independent solver endpoints
within `1e-8 m`. The original reference replay remains unchanged. The 29 new
anticipation checks, 40 mathematical checks, and existing boundary, speed and
periodic suites all pass. An additional 30 s open-mode smoke run at 3 spawn
attempts/s completed with valid bounds and population accounting; this is not
a full crowded-flow/draining validation.

Per-run CSVs are in `work/behavior-comparison-seed{seed}-{mode}-60.00s.csv`.
Rerun the commands below to regenerate them from the current model. The
independent one-seed experiments later in this section predate this production
validation and use a different sampling schedule.

The force gradient, front/rear weighting, wall potential, individual speed cap,
and isolated-motion update are checked independently in `tests/MathChecks.java`.
The negative gradient is compared with finite differences of the original
potential, rather than a copy of the implementation's gradient formula.
The wall-contact code now reapplies `v = w * min(1, v-max / |w|)` after projecting
`w`. Previously it retained an actual tangential velocity capped using the old
norm, which was inconsistent with that relation.

Passing these checks does not establish collision-free motion. In a 60 s
diagnostic of the pre-fix default 150-person hallway (seed 42, no noise), the
minimum centre separation was `0.01022 m`, with a recorded opposite-direction
pass-through. At `dt = 0.005` instead of `0.02`, the minimum was still `0.05148 m`.
These independent full runs diverge, so their individual collisions are not
matched convergence measurements. Replaying the same interior encounter from
the same saved state at `dt = 0.001` and `0.0002` also retained pass-through,
with minimum separations `0.01448 m` and `0.01446 m`. This local refinement
does not support a large-timestep explanation for that particular encounter.
The wall-contact fix does not establish a remedy for
these interior encounters.

### Traced causes and independent reproduction

The recorded original-mode encounter of pedestrians 3 and 52 at about 21 s was reconstructed
term by term. At 20.72 s, other pedestrians contribute lateral accelerations
of `-9.36` and `+6.23 m/s^2`, pushing the pair towards each other. Their mutual
lateral avoidance is only `+0.31` and `-0.30 m/s^2`. Both are turning sideways,
but the literal 1995 prediction `s_j = T * |v_j| * e_j` remains aligned with
the desired hallway direction. The force-direction perception rule also gives
their mutual interaction half weight during much of the encounter. This is
not a force-sign transcription error. Other neighbors' forces overwhelm the
pair's avoidance before it can maintain the drawn 0.4 m diameter.

An independent NumPy implementation reproduces NetLogo's first-order replay
to about `1e-12 m`. Fourth-order Runge-Kutta with `h = 0.0002 s` still gives
minimum separation `0.01452 m`. Removing the `1e-6 m` softening changes that
minimum by only `6.3e-8 m`. No hard-wall contact occurs in this replay. These
tests isolate this encounter from NetLogo, the wall projection, and ordinary
timestep/softening explanations; they do not prove every trajectory accurate.

The longest sampled sideways episode has a different detailed sequence.
Pedestrian 136 receives `+7.17 m/s^2` sideways from an anticipated ellipse
while its source is still 2.92 m away. A second anticipated endpoint later
produces about `+11 m/s^2`. Preferred speed accumulates to 3.38 m/s, while
actual speed is capped at 0.956 m/s. It therefore continues sideways after
acceleration starts braking it, and the soft wall force subsequently reverses
it. This episode never reaches the added hard-contact boundary.

Controlled prediction changes help distinguish the cause from a numerical
bug. For the same saved encounter, at RK4 timestep 0.001 s:

| Prediction displacement | Minimum centre separation |
| --- | ---: |
| Original: `T * |v_j| * e_j` | 0.0148 m |
| Actual velocity: `T * v_j` | 0.6400 m |
| Relative velocity: `T * (v_j - v_i)` | 0.6368 m |
| Original prediction, full perception weight | 0.0585 m |

Changing a prediction is a model change. In particular, substituting actual
velocity alone is not a reliable fix: a matched 60 s, seed-42 comparison using
the independent first-order solver made crowd-wide behavior worse. The
relative-velocity variant performed much better in that one comparison:

| Prediction | Mean overlapping pairs | Mean absolute lateral speed, 10–60 s |
| --- | ---: | ---: |
| Original desired direction | 4.627 | 0.253 m/s |
| Actual velocity | 4.838 | 0.338 m/s |
| Relative velocity | 0.00267 | 0.0711 m/s |

The relative variant still had a minimum separation of 0.381 m, slightly less
than the drawn diameter. These are one-seed diagnostic results, not proof of
collision-free motion or a calibrated replacement for the paper's model.
All comparison arms keep the same parameters, force-direction weights, and
separate preferred/actual velocity. Only the prediction displacement changes.

Helbing's later coauthored paper explains that elliptical specification I
ignores the affected walker's own speed/direction and introduces relative
velocity in specification II:
[Johansson, Helbing and Shukla, Section 3.3.1](https://arxiv.org/html/0810.4587#S3.SS3.SSS1).
A numerical analysis also documents collisions and oscillations at small
timesteps in social-force models:
[Koster, Treml and Godel (2013)](https://doi.org/10.1103/PhysRevE.87.063305).
Those sources support examining model limitations; they do not establish the
precise undocumented settings used in the original 1995 simulations.

The relative-velocity prediction is now implemented in NetLogo and enabled by
default. The original 1995 prediction remains available by switching
`relative-anticipation?` off before setup. The actual-velocity-only substitution
remains diagnostic-only because it worsened the comparison above.

The elliptical potential is not differentiable on the segment between another
pedestrian's current and anticipated positions. Its gradient can be very large
near the endpoints. The `1e-6 m` softening avoids division by zero but does not
make `dt = 0.02` a validated integration step. The paper evolves preferred `w`
and caps actual `v`; resetting `w` to `v` would change those equations. The
paper also has no pedestrian body-radius exclusion. For a perfectly symmetric
head-on pair at desired speed `1.34 m/s`, the stationary force balance is
`d = 0.3 * ln(7 / 2.68) = 0.288 m`, smaller than our fixed circle diameter
`0.4 m`. The paper's plotted circle diameters instead represent actual speed.
These distinctions and the unspecified numerical integrator in the paper
remain relevant when interpreting overlap and sideways motion. No additional
pedestrian contact force or arbitrary force cap was introduced by this audit.

## Files

- `Pedestrians.nlogox`: runnable model, interface and documentation.
- `pedestrians.nls`: readable copy of the code embedded in the model. It is not
  a separate runnable model; keep both code copies in sync when editing.
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
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -classpath "$netLogoHome\app\netlogo-7.0.4.jar" tests/BehaviorComparison.java 42 relative 60
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
- Add scenarios
  - Pedestrians must move through a door which is only wide enough for one pedestrian at a time.
  - Pedestrians must wait at a traffic light which has attributes color (Green or red) and Switch time (Time before switching colors)
  - A crossing where pedestrians will walk both horizontally and vertically. A
crossing is not an object. We are just going to position pedestrians and
obstacles such that a crossing occurs. We will observe how this impact
walking speed and travel time, and want to see if the pedestrians are able
to walk though the crossing without stopping
- Correctness
  - In order to determine correctness of our model, we will compare the data from
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
number of lanes increases linearly with the width of the walkway.**
- Logging and plots
  - Mean, standard deviation and variance of numerical variables with respect to time?
  
