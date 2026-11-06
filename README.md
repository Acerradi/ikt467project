# ikt467project
Languages for modeling and simulation exam project

## Run the model

Open `Pedestrians.nlogox` in NetLogo 7.0.4, click **setup**, then **go**.
Click **go** again to pause, or **step** to advance one integration step.

The default test starts two pedestrians directly opposite each other with no
lateral offset. Coloured trails show their paths. The model includes desired
motion, elliptical pedestrian repulsion, wall repulsion and a small random force
assigned once per pedestrian. Attraction is deferred.

- `population`: number of pedestrians; click **setup** after changing it.
- `dt`: simulated seconds per integration step; default `0.02`.
- `desired-speed`: target speed in m/s. The speed limit is `1.3` times this value.
- `noise-strength`: random-force magnitude in m/s^2; default `0.05`. Set it to
  `0` to reproduce the symmetric head-on stall.
- `noise-seed`: seed for the force directions; default `42`. Click **setup**
  after changing it. The same seed and settings reproduce the same run.

Read the model's **Info** tab for the equation mapping, parameters and numerical
choices. Repulsion is soft: the displayed circles do not define solid-body
collision boundaries. Larger populations start in preset lanes, so this setup
does not demonstrate spontaneous lane formation.

## Files

- `Pedestrians.nlogox`: runnable model, interface and documentation.
- `pedestrians.nls`: readable copy of the code embedded in the model. It is not
  a separate runnable model; keep both code copies in sync when editing.

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
