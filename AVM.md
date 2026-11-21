# Anticipation Velocity Model

The four NetLogo scenarios use the Anticipation Velocity Model (AVM) of
Xu, Chraibi and Seyfried (2021), *Anticipation in a velocity-based model for
pedestrian dynamics*, Transportation Research Part C 133, 103464.
[Paper](https://arxiv.org/abs/2107.00624),
[DOI](https://doi.org/10.1016/j.trc.2021.103464).

This replaces the previous social-force integration and pair-position
projection. Each person retains a position and movement heading; AVM selects
a scalar speed from free space. The body radius is **0.20 m**, so two touching
people have a centre distance of **0.4 m**. Drawing, placement, pair headway,
walls and traffic-light stops use this same radius.

## Parameters

The starting point is the paper's Table 3 bidirectional calibration. The
same parameters apply in every scenario; no scenario-specific tuning was used.

| Parameter | Value |
| --- | --- |
| Steering strength k | 6 |
| Interaction range D | 0.01 m |
| Time gap T | 0.6 s |
| Heading relaxation time tau | 0.3 s |
| Anticipation time | 0.75 s |
| Individual free speed | Gaussian, mean 1.55 m/s, SD 0.18 m/s |
| Body radius | **0.20 m**, requested override of the paper's 0.18 m |
| Default outer timestep | 0.02 s |

Nonpositive free-speed draws are resampled at creation. Individual free speed
is then retained. The radius change means this is not a reproduction of the
paper's calibrated results. The old 1.3-times-free-speed cap and random
acceleration do not apply.

## Equation implementation

`core/avm.nls` implements Eqs. (1)-(10) as pure reporters operating on an old
state snapshot. In this description, q = x_j - x_i, u = q / |q|, e_i is the
retained movement heading, e0_i is the desired direction, and p_i is its
perpendicular.

1. A neighbour is visible when e_i dot q > 0 or e0_i dot q > 0.
2. Anticipated gap is max(0, |q| - 2r + t_a (v_j-v_i) dot u).
3. Steering magnitude is k [1 + (1-e0_i dot e_j)/2] exp(-gap/D).
4. Its lateral sign is opposite to (q + t_a v_j) dot p_i. An exactly aligned
   neighbour chooses either side with equal probability.
5. Normalize the desired direction plus the summed lateral contributions.
6. Relax the previous heading toward that direction using normalized Euler.
7. In the new heading's forward strip of half-width 2r, find the smallest
   radial centre distance minus 2r. Speed is min(v0, max(0, gap/T)). With no
   blocker, speed is v0. Advance position by h times the selected velocity.

The paper's Eq. (6) has inconsistent pair indices; the implementation uses
the influence of j on i defined in Eqs. (3)-(5), consistent with the authors'
implementation. Normalized Euler follows that implementation as well.
The speed gap is radial, not a longitudinal projection. Steering prediction
uses relative velocity in Eq. (2), but future j relative to current i in
Eq. (5). Those two predictions intentionally differ.

## Numerical choices

All pedestrians use one immutable old-state snapshot per Euler step. New states
are committed only after everyone has been calculated. Neighbours are summed
in ascending ID order. Random passing-side choices are assigned to directed
pairs in that same canonical order before the calculations. Reversing actor
calculation order therefore does not change the intended state or random
stream. Relabeling IDs is not promised to retain the same random realization.

Exact alignment uses a 1e-12 m floating-point tolerance. The pair equations use
no interaction cutoff, extra pair buffer, creep velocity or preferred passing
side. Scenario target selection is separate from those equations.

**No added pedestrian-distance constraint is used.** The integrator takes the
selected fixed dt without pair position projection, overlap rejection,
distance-dependent timestep caps, automatic halving or a minimum-separation
abort. It commits the paper-model velocity even if the resulting paths bring
centres closer than 2r. The body diameter remains in AVM's own headway formula;
there is no separate acceptance condition enforcing it after that calculation.

The traffic-light scenario alone splits an outer dt at scheduled light events.
All motion segments, including departures, are retained for diagnostics. Door
passages are sorted by their within-step crossing times. The numerical method
is normalized forward Euler; the physical time-gap, anticipation and heading
relaxation times remain independent of dt.

Overlap detection is observational. A run is allowed to continue after body
overlap so minimum separation, penetration and subsequent flow can be measured.
Finite-state, individual free-speed and wall/stop checks remain separate. Fixed
smaller timesteps can be compared to distinguish discretization effects from
behavior that persists as dt decreases; no distance-based safeguard is used in
those comparisons.

## Scenario additions

The preprint does not specify the project's walls, destinations, arrivals,
traffic lights or yielding controllers. It leaves tactical target selection
outside the operational model. The AVM pedestrian-pair equations and their
parameters remain unchanged; these are explicit scenario additions:

- A solid wall hides a neighbour from both steering and headway calculation,
  following the authors' JuPedSim `NoGeometryBetween` filter. Visibility tests
  the straight line between centres against the literal wall segments, with
  periodic images when needed. It adds no pair radius or distance cutoff.
  Visible neighbours retain their original sorted order. This geometry filter
  is an author-implementation convention, not an extra term in the paper's
  Eq. (1). Removing an occluded aligned pair also removes its passing-side
  random draw. Thus a common seed across model versions does not guarantee
  identical later arrival draws; actor-order invariance is tested within each
  version.
- Wall gliding follows the idea in the authors' JuPedSim implementation:
  within r + 0.1 m, remove an inward heading component, add a 0.3 outward
  component, then normalize. We apply this **before** computing pair headway
  so the speed calculation uses the final direction. This ordering differs
  from the current JuPedSim C++ implementation.
- Swept segment/capsule geometry limits speed along that direction if a wall
  or round door post would be crossed. No endpoint is repositioned.
- A red traffic light or closed admission gate adds an external scalar speed
  rule. Let `q` be the centre's x or y coordinate, `q_stop` its assigned stop
  coordinate, `d` its travel sign (+1 or -1), `h` the integration step, and
  `c = d * heading_component` the actual unit heading toward the stop plane.
  With `gap = max(0, d * (q_stop - q))`, the proposed speed is reduced to
  `min(speed, gap / T)`. If `c > 0`, it is also limited by `gap / (h * c)`.
  Thus the centre cannot advance through its assigned stop plane in that
  step. The scenario places that plane with body clearance; the traffic-light
  centre stops r before the curb. This cap reduces the whole scalar speed,
  including lateral motion while held. It does not reposition a pedestrian
  or enforce a minimum distance from another pedestrian.
- The visible `door-yielding?` and `crossing-yielding?` switches enable
  explicit admission coordination; changing the setting takes effect on
  setup. Door directions receive finite batches, and Crossing alternates
  horizontal and vertical admission phases. Closing admission holds only
  upstream pedestrians who have not been admitted. Admitted pedestrians
  retain permission to finish, and conflicting admission waits until the
  preceding pedestrians have physically cleared the shared region. These
  controllers are project rules, not part of the AVM paper. Disabling them
  restores uncontrolled admission; neither setting guarantees freedom from
  congestion or numerical overlap.
- Door and Crossing select obstacle-clear portal waypoints before resuming
  each pedestrian's original destination. Holding routes and admission gates
  belong to the yielding policy, not to AVM's pair steering or headway law.
  Existing Poisson arrivals, random destinations and the periodic hallway
  remain scenario choices. Coordination does not change the requested arrival
  attempt rates or remove waiting pedestrians from the accounting.

[Author model documentation](https://pedestriandynamics.org/models/anticipation_velocity_model/)
and [JuPedSim source](https://github.com/PedestrianDynamics/jupedsim/blob/master/libsimulator/src/OperationalModels/AnticipationVelocityModel/AnticipationVelocityModel.cpp)
clarify implementation conventions. This implementation does not copy its
3 m interaction cutoff, 0.02 m pair buffer, random creep or reversal snap.

## Current radius 0.20 m: bounded verification

The default was reduced to 0.20 m on 2026-11-13. The AVM equations and other
parameters are unchanged. Touching bodies now have centre distance 0.40 m;
there is still no extra pair-distance enforcement.

All four scenarios completed 5 simulated seconds at dt 0.02 s, seed 42,
with 250 fixed steps each, no execution errors and no observed body overlap.
Minimum swept centre distances were 0.406699168 m (hallway), 0.400089968 m
(door), 0.403004043 m (traffic light), and 0.486451209 m (crossing). These short
startup checks do not establish long-run separation or resolve congestion.

The boundary/GUI-name suite passed 52 checks, the independent equation suite
passed 3,617 assertions, and the lifecycle event checks passed 28 assertions.
The analytical equation and wall fixtures explicitly retain radius 0.25 m;
the scenario setup checks verify the new 0.20 m default. The artificial
side-pinch stress fixture still commits overlapping steps (minimum 0.399480 m),
confirming there is no added 0.40 m hard limit. Results are stored under
`work/avm-radius020/validation/`.

## Historical validation at radius 0.25 m without the added pair limit

These experiments tested whether AVM's equations maintain reasonable pedestrian
separation at radius 0.25 m. The current default is 0.20 m; the results below
are historical and do not describe the new radius. Minimum distance is an
observed result, not a timestep acceptance criterion. Tests independently inspect each pair's
entire linear motion segment, including periodic images and people removed at
exits. They continue after overlap and report it separately from execution
errors and from flow outcomes such as arrivals, queues and jams.

The following unconstrained results were measured on 2026-11-13 in NetLogo
7.0.4, using radius 0.25 m, fixed dt 0.02 s and seeds 42, 1 and 7. Initial
placement retains the existing 0.6 m clearance; this is an initial condition,
not a constraint on subsequent movement. The equation kernel and wall/stop
rules are unchanged from the previous version. Only the extra pair safeguard
and its timestep selection have been removed from the integration.

| Scenario | Configuration | Time per seed | Minimum centre distance across seeds | Maximum body overlap | Flow |
| --- | --- | --- | --- | --- | --- |
| Hallway | Periodic 50 by 10 m, 150 people | 30 s | 0.499232651 m | 0.767349 mm | 114 / 117 / 114 seam crossings |
| Door | 20 per side, 1.2 m door, 6 m corridor, no arrivals | 30 s | approximately 0.500000000 m | None beyond roundoff | Jammed: zero final-exit arrivals; 37 / 37 / 38 of 40 static |
| Traffic light | 20 per side, 0.5 attempts/s, 10 s phases, initially red | 30 s | 0.500002812 m | None observed | 27 / 19 / 15 final-exit arrivals |
| Crossing | 4 m corridors, one-way streams, 1.8 horizontal + 1.8 vertical attempts/s | 60 s | 0.499374482 m | 0.625518 mm | 88 / 87 / 89 final-exit arrivals; congested |

Slash-separated values are in seed order 42 / 1 / 7. These 12 scenario runs
completed 450 simulated seconds and 22,500 fixed integration intervals, with
no execution, free-speed, wall, stop-line or population-accounting failures.
No interval was rejected, shortened because of pedestrian distance, or
position-corrected. An overlap threshold of 1e-8 m excludes floating-point
roundoff from overlap counts.

Hallway seed 1 had two pair-by-step overlap observations; crossing seed 42
had 17. The other ten scenario runs had no overlap above that tolerance.
These counts are observations, not counts of distinct collisions. Static
means mean path speed over the last 10 s below 1% of individual free speed,
among people present for that full interval. No hallway, traffic or crossing
pedestrian met that strict static criterion; this does not imply free flow.
The crossing ended with 111 / 135 / 121 people still inside, and 0 / 2 / 1
blocked entrance attempts. These startup runs do not establish sustainable
long-run throughput.

**Conclusion for the measured runs:** the unguarded AVM maintained roughly
0.5 m body spacing on its own, with small numerical overlaps at dt 0.02 s.
It did not provide strict non-overlap, and it did not resolve the opposing
door-crowd jam. This is evidence for the tested configurations, not a universal
collision-avoidance guarantee.

### Matched timestep refinement

The crossing seed-42 state was exported at 55.04 s, before its first overlap.
Each branch imports that same physical and random-generator state, freezes
new arrivals, and integrates one second at a different fixed timestep. State
and future-RNG equality are checked before the branches; the RNG probe is
undone by reimporting the world. The same pair (75, 88) gives the minimum.

| Fixed dt | Minimum centre distance | Maximum overlap |
| --- | --- | --- |
| 0.02 s | 0.499374482 m | 0.625518 mm |
| 0.01 s | 0.499740979 m | 0.259021 mm |
| 0.005 s | 0.499932650 m | 0.067350 mm |
| 0.0025 s | 0.500189250 m | None observed |

This supports a discretization explanation for that local overlap. It is not
a full-scenario convergence proof, and the four branches are not separate
unmatched Poisson-arrival experiments. The model's saved default remains
0.02 s; no smaller step is selected automatically.

### Encounters and regression checks

- Seed-42 head-on moving/rest, offset and both right-angle encounters pass
  without overlap over 10 s. Head-on minimum is 1.239648789 m; offset minimum
  is 1.255900282 m; right-angle minima are 0.798501009 and 0.776113304 m.
- Exact head-on behavior is sensitive to the passing-side choice and numerical
  grid: seed 1 also passes at dt 0.02, while seed 7 does not pass within 10 s
  (minimum 0.500068569 m). Seed 42 at dt 0.01 also does not pass within 10 s
  (minimum 0.500034189 m). Do not infer reliable passing from one symmetric run.
- The 12 s overtaking fixture follows the slower walker at a final centre
  distance of 1.220000002 m, rather than overtaking it.
- A deliberately artificial near-tangent side-pinch fixture, using tau = 1e9 s
  to isolate the changing blocker strip, completes even when it overlaps.
  Minima at dt 0.2 / 0.02 / 0.005 / 0.001 s are 0.493900000 / 0.499480000 /
  0.499945000 / 0.500006771 m. This proves the simulation is not enforcing
  0.5 m through a hidden acceptance test. It is a numerical stress fixture,
  not a run with the paper's normal turning time.
- Reverse actor calculation order preserves physical state, traces and future
  randomness exactly over 6 s. Checks/logging on and off give 1,250,235
  identical raw-bit state comparisons across all four scenarios.
- The GUI-name/boundary/measurement suite passes 49 checks; exact light-event
  and fixed-step door-bookkeeping tests pass. A synthetic overlapping trace is
  measured without throwing an overlap error or changing the stored path.
- The AVM equation module remains byte-identical to the independently tested
  kernel (3,617 scalar-equation, periodic-image, tie and RNG assertions).

The earlier head-on result changed even where the old guard rejected no
steps: the new integrator uses h = dt directly instead of subtracting elapsed
time from a floating-point end time. Exact symmetry and the side-selection
rule make that numerical detail relevant. It would be incorrect to attribute
every trajectory difference solely to removing rejected steps.

Do not reuse the earlier guarded minimum-distance table as evidence for AVM
alone. Those runs included an overlap-rejection rule; their raw artifacts
remain under `work/avm/validation/`, and their model/documentation backup is
under `work/avm-no-limit/before/`.

The equation tests compare independently written scalar formulas. GUI-name
regression checks verify that every slider's display and variable names match:
NetLogo 7.0.4's GUI uses the display name, while headless compilation uses the
variable field. A mismatch such as `outer dt` versus `dt` must fail the test.

### Recorded commands for the 0.25 m experiments

These commands were run with the previous 0.25 m default. Running them with
the current model uses 0.20 m and does not reproduce the historical tables.
Use the commands and separate output folders in README.md for current-radius
runs. The arguments below are retained as the historical experiment record
for `tests/AVMChecks.java`, after the NetLogo classpath and source filename:

```text
--mode pairs --model-root . --output work/avm-no-limit/validation/pairs
--mode order --model-root . --output work/avm-no-limit/validation/order
--mode crowds --cases hallway,door,traffic --seeds 42,1,7 --seconds 30 --model-root . --output work/avm-no-limit/validation/crowds
--mode crowds --cases crossing --seeds 42,1,7 --seconds 60 --model-root . --output work/avm-no-limit/validation/crossing
--mode pairs --cases side-pinch --dt .2 --seconds 1 --model-root . --output work/avm-no-limit/validation/side-pinch
```

For the matched crossing refinement, run this source with the same NetLogo
Java/classpath prefix as the other tests:

```text
tests/AVMFreeReplay.java 42 55.0797184477 work/avm-no-limit/validation/refinement
```

This replay explicitly restores radius 0.25 m and validates imported worlds
against that historical radius. It exports the pre-overlap world and runs all
four fixed timesteps. To reuse
that saved world, append `--world work/avm-no-limit/validation/refinement/pre-overlap-world.csv`.

CSV summaries, motion traces, initial/final states and worst-approach snapshots
are saved under the selected output folder. A safe jam is not successful
evacuation, and a completed run is not automatically collision-free. Numerical
errors, measured overlaps and congestion are reported as separate outcomes.
Older social-force/contact tests remain historical and are not AVM acceptance
tests. The README's TODO section is preserved byte-for-byte.
