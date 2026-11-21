# ikt467project
Languages for modeling and simulation exam project

## Run the model

Open `Pedestrians.nlogox` in NetLogo 7.0.4, click **setup**, then **go**.
Click **go** again to pause, or **step** to advance one outer timestep.
Reopen the model after updating the shared includes and interface files.

The main models now use the **Anticipation Velocity Model (AVM)** with a
**0.20 m body radius**, following Xu, Chraibi and Seyfried (2021).
[AVM.md](AVM.md) describes the equations, parameter set, fixed-step integration,
scenario additions, tests and observed limitations.

## Scenarios

| Model | Scenario |
| --- | --- |
| `Pedestrians.nlogox` | Open or periodic hallway; default 150 people in 50 by 10 m |
| `scenarios/Door.nlogox` | Corridor divided by a wall with one door; one or both directions |
| `scenarios/TrafficLight.nlogox` | Walkway crossing a road with timed red/green stops |
| `scenarios/Crossing.nlogox` | Uncontrolled horizontal/vertical crossing; tested at 1.8 attempts/s per stream |

The original hallway geometry is retained for comparison; changing the
pedestrian dynamics to AVM means it is no longer a reproduction of the
Helbing-Molnar force-model experiment. Random arrivals, destinations and
routing belong to these project scenarios.

## Shared implementation

Each scenario includes these modules and supplies geometry, spawning, setup,
go and an exit hook:

- `core/pedestrians.nls`: state, synchronous integration, placement and lifecycle.
- `core/avm.nls`: anticipated steering, heading relaxation and headway speed.
- `core/boundaries.nls`: explicit wall-gliding and traffic-stop extensions.
- `core/checks.nls`: separation, walls, speed, swept paths and lane diagnostics.
- `core/stats.nls`: statistics, plotting and CSV logging.

The speed law uses the body diameter (0.4 m); speed never exceeds each
person's sampled free speed. **There is no added pedestrian-distance limit:**
no pair-position projection, overlap-based step rejection, timestep halving,
distance-dependent step cap or minimum-separation abort. Fixed Euler steps are
committed even when bodies overlap. Diagnostics measure the resulting
separation so the AVM equations can be evaluated without that extra safeguard.
Wall and traffic-stop boundaries remain separate scenario rules in AVM.md.

The `run-checks?` switch measures overlaps and separation without changing
trajectories, the random stream or whether an overlapping step is committed. Door, traffic-light and crossing models also have a
`log-csv?` switch. Logging retains the existing 0.1 s sampling schedule,
timestamped at the actual completed step.

## Validation

Run the AVM tests from the repository root using NetLogo's bundled Java:

```powershell
$netLogoHome = 'C:\Program Files\NetLogo 7.0.4'
foreach ($suite in 'AvmMathChecks', 'AVMBoundaryChecks', 'AVMLifecycleChecks') {
  & "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -cp "$netLogoHome\app\netlogo-7.0.4.jar" "tests/$suite.java"
  if ($LASTEXITCODE -ne 0) { throw "$suite failed" }
}
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -cp "$netLogoHome\app\netlogo-7.0.4.jar" tests/AVMChecks.java --mode pairs --model-root . --output work/avm-radius020/validation/pairs
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -cp "$netLogoHome\app\netlogo-7.0.4.jar" tests/AVMChecks.java --mode order --model-root . --output work/avm-radius020/validation/order
& "$netLogoHome\runtime\bin\java.exe" '-Djava.awt.headless=true' -cp "$netLogoHome\app\netlogo-7.0.4.jar" tests/AVMChecks.java --mode crowds --model-root . --seeds 42,1,7 --seconds 30 --output work/avm-radius020/validation/crowds
```

`tests/AvmMathChecks.java` checks independently written equations.
`tests/AVMBoundaryChecks.java` checks wall/stop geometry and bookkeeping.
`tests/AVMLifecycleChecks.java` checks observation invariance and timed events.
`tests/AVMChecks.java` measures every swept path in encounters and crowds,
continues after overlaps, and reports separation and flow separately from
execution errors. See AVM.md for the measured
results and exact protocols.

Earlier social-force, relative-anticipation and hard-contact tests and
`core/contact.nls` remain as historical files. They are not compiled by the AVM
models and are not the AVM test suite. Their old calibration and expected
behavior do not apply to this implementation. The pre-AVM working model is
also backed up locally under `work/avm/before-avm/`. The subsequent AVM
version with overlap rejection is backed up under `work/avm-no-limit/before/`;
its results in `work/avm/validation/` are historical and do not establish
collision avoidance by AVM alone.

## BehaviorSpace experiments

Each model retains an experiment: `figure-3-lanes`, `door-width-sweep`,
`light-cycle`, or `crossing-flow-sweep`. The first name is historical; its
current pedestrian model is AVM. Use `run-experiments.sh` from Git Bash,
MSYS2, Cygwin, WSL, Linux or macOS:

```bash
./run-experiments.sh
./run-experiments.sh door-width-sweep light-cycle
THREADS=8 ./run-experiments.sh
NETLOGO_HOME=/path/to/NetLogo ./run-experiments.sh
```

Results go to `work/<experiment>.csv`, overwriting the corresponding previous
table. The script prefers NetLogo's bundled Java; `NETLOGO_JAVA` can override
it. Unknown experiment names fail explicitly. The `work/` directory and
generated `preview.png` are excluded from Git.


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
