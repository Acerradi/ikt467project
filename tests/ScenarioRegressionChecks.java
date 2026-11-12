import java.nio.file.Path;
import java.util.Arrays;
import org.nlogo.headless.HeadlessWorkspace;

/** Regression tests for scenario event timing, stop boundaries and passage order.
 * Run from the repository root with NetLogo's Java runtime and jar.
 * --quick omits the bounded crowd runs; no files or model defaults are changed.
 */
public class ScenarioRegressionChecks {
    private static HeadlessWorkspace world;
    private static int assertions;

    private static void open(String path) throws Exception {
        if (world != null) world.dispose();
        world = HeadlessWorkspace.newInstance();
        world.open(Path.of(path).toAbsolutePath().toString());
    }

    private static void require(String condition, String message) throws Exception {
        if (!Boolean.TRUE.equals(world.report(condition))) {
            throw new AssertionError(message + ": " + condition);
        }
        assertions++;
    }

    private static void near(String expression, double expected, double tolerance, String message)
            throws Exception {
        double actual = ((Number) world.report(expression)).doubleValue();
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance) {
            throw new AssertionError(message + ": " + expression + " = " + actual + ", expected " + expected);
        }
        assertions++;
    }

    private static final String INVARIANTS =
        "if spawned-count != count pedestrians + arrived-count + returned-count "
        + "[error \"Population accounting changed\"] "
        + "foreach sort pedestrians [p -> ask p ["
        + "let wnorm vector-length wx wy "
        + "let factor min (list 1 (1.3 * desired-speed / max (list 1e-12 wnorm))) "
        + "if abs (vx - factor * wx) > 1e-8 or abs (vy - factor * wy) > 1e-8 "
        + "[error \"Actual velocity is not capped preferred velocity\"] "
        + "if vector-length vx vy > 1.3 * desired-speed + 1e-8 [error \"Speed cap exceeded\"] "
        + "if wall-clearance < body-radius - 1e-8 [error \"A wall was penetrated\"] "
        + "]] "
        + "if abs (last-step-start-seconds + last-step-duration - elapsed-seconds) > 1e-8 "
        + "[error \"Last-step path time does not match simulation time\"] "
        + "if length last-step-paths < count pedestrians [error \"Missing pedestrian motion paths\"] "
        + "if not pair-contact-enabled? or abs (minimum-separation - 0.5) > 1e-12 "
        + "[error \"Production contact defaults changed\"] "
        + "if swept-min-pair-separation < 0.5 - 1e-8 or min-pair-separation < 0.5 - 1e-8 "
        + "[error \"Pedestrians came closer than 0.5 metres\"] ";

    private static void lightTiming() throws Exception {
        open("scenarios/TrafficLight.nlogox");
        world.command("set initial-per-side 0 set spawn-rate 0 set switch-time 10 "
            + "set dt 0.07 set start-green? false set log-csv? false setup repeat 1000 [go]");
        near("elapsed-seconds", 70, 1e-8, "Non-dividing timestep lost simulated time");
        near("light-switches", 7, 0, "Light cycle accumulated timestep remainder");
        near("light-timer", 0, 1e-8, "Light timer did not retain exact cycle phase");
        near("dt", .07, 1e-14, "An event substep changed the user's timestep");
        require("light-green?", "Seventh switch should leave initially red light green");

        // One outer interval straddles the switch: 1.96..2.00 green, 2.00..2.03 red.
        // Straight motion, no other pedestrians and no force overrides. Before
        // red: v=1.34. At red: gap=.82, target speed=.41, acceleration=-1.86.
        world.command("set initial-per-side 0 set spawn-rate 0 set switch-time 2 set dt 0.07 "
            + "set start-green? true set desired-speed-sd 0 setup add-pedestrian 13.3 4.5 1 "
            + "ask pedestrians [set goal-y ycor set wx 1.34 set vx 1.34] repeat 28 [go]");
        near("elapsed-seconds", 1.96, 1e-10, "Fixture did not reach last green interval");
        near("[xcor] of one-of pedestrians", 15.9264, 1e-9, "Green approach changed unexpectedly");
        require("light-green?", "Light changed before its scheduled time");
        world.command("go");
        near("elapsed-seconds", 2.03, 1e-10, "Split interval changed total step duration");
        near("[vx] of one-of pedestrians", 1.2842, 1e-9, "Wrong light phase applied to movement");
        near("[xcor] of one-of pedestrians", 16.018526, 1e-9, "Motion was not split at the light event");
        near("light-timer", .03, 1e-10, "Time after switch was discarded");
        near("dt", .07, 1e-14, "Split motion did not restore requested timestep");
        require("not light-green? and light-switches = 1", "Switch state incorrect after split interval");
        world.command(INVARIANTS);
        System.out.println("PASS: exact light cycles, remainder retention, physical switch phase and timestep restoration.");
    }

    private static void redStopAndRelease() throws Exception {
        // Start at rest on green and arrive 1cm before the stop line exactly
        // when red begins at t=2. The original smooth braking law overshot.
        double dt = .02, q = 1 - dt / .5;
        double displacement = dt * 1.34 * (100 - q * (1 - Math.pow(q, 100)) / (1 - q));
        double startX = 16.79 - displacement;
        for (boolean relative : new boolean[]{true, false}) {
            world.command("set relative-anticipation? " + relative
                + " set initial-per-side 0 set spawn-rate 0 set switch-time 2 set dt 0.02 "
                + "set start-green? true set desired-speed-sd 0 setup add-pedestrian " + startX + " 4.5 1 "
                + "ask pedestrians [set goal-y ycor] repeat 100 [go]");
            near("[xcor] of one-of pedestrians", 16.79, 1e-9, "Green-to-red fixture position");
            require("not light-green?", "Red did not activate at the event boundary");
            world.command("repeat 99 [go " + INVARIANTS + "]");
            require("max-stop-overshoot <= 1e-9 and held-on-road-count = 0",
                "A lone pedestrian crossed its red stop boundary");
            require("count pedestrians = 1 and all? pedestrians [gap-to-stop-line >= -1e-9 and vector-length vx vy < 0.02]",
                "Red-held pedestrian did not stop before the curb");
            world.command("go");
            require("light-green?", "Green did not activate at t=4");
            world.command("repeat 250 [go " + INVARIANTS + "]");
            require("count pedestrians = 1 and all? pedestrians [stop-x = false and vector-length vx vy > 0.9 * desired-speed]",
                "Pedestrian failed to resume after green or was stopped again on the road");

            // A person already on the road when red is active may finish crossing.
            world.command("set switch-time 40 set start-green? false setup add-pedestrian 17.5 4.5 1 "
                + "ask pedestrians [set goal-y ycor set wx 1.34 set vx 1.34] repeat 100 [go]");
            require("count pedestrians = 1 and all? pedestrians [stop-x = false and xcor > 19]",
                "A pedestrian already on the road was incorrectly held");
        }
        System.out.println("PASS: natural late-red approach, no curb crossing, green recovery and already-on-road continuation in both prediction modes.");
    }

    private static void doorOrdering() throws Exception {
        open("scenarios/Door.nlogox");
        world.command("set initial-per-side 0 set spawn-rate 0 set desired-speed-sd 0 set dt 0.05 "
            + "set door-width 2 set log-csv? false setup set potential-strength 0 set wall-strength 0 "
            + "add-pedestrian 9.99 4.0 1 add-pedestrian 9.95 4.5 1 add-pedestrian 10.03 5.0 -1 "
            + "ask pedestrians [set waypoints [] set goal-y ycor set wx 1.34 * travel-direction set vx wx]");
        require("min-pair-separation >= minimum-separation", "Passage-order fixture starts below required clearance");
        world.command("go");
        require("door-sequence = [1 -1 1]", "Door sequence uses pedestrian ID instead of crossing time");
        near("alternation-fraction", 1, 0, "Door alternation differs from chronological crossings");
        world.command("go");
        near("door-passes", 3, 0, "Crossings were counted more than once");

        // End exactly on the door plane, then move away; record only one passage.
        for (int direction : new int[]{1, -1}) {
            world.command("setup set potential-strength 0 set wall-strength 0 "
                + "add-pedestrian (door-x - " + direction + " * 1.34 * dt) door-center-y " + direction + " "
                + "ask pedestrians [set waypoints [] set goal-y ycor set wx 1.34 * travel-direction set vx wx] go");
            near("[xcor] of one-of pedestrians", 10, 1e-12, "Door-plane endpoint fixture");
            world.command("go");
            require("door-passes = 1 and first door-sequence = " + direction,
                "Exact-plane crossing was omitted or counted twice");
        }
        // Motion along the plane is tangential, not a passage across it.
        world.command("setup set potential-strength 0 set wall-strength 0 "
            + "add-pedestrian door-x 4.0 1 ask pedestrians [set waypoints [] "
            + "set goal-x door-x set goal-y 5.0 set wx 0 set vx 0 set wy 1.34 set vy 1.34] go");
        near("door-passes", 0, 0, "Tangential motion on the plane counted as a passage");
        System.out.println("PASS: chronological door passages, exact-plane endpoints and tangential motion.");
    }

    private static void crowdStress() throws Exception {
        for (String model : new String[]{"Pedestrians.nlogox", "scenarios/Door.nlogox",
                "scenarios/TrafficLight.nlogox", "scenarios/Crossing.nlogox"}) {
            open(model);
            for (boolean relative : new boolean[]{true, false}) {
                String shared = "set relative-anticipation? " + relative
                    + " set simulation-seed 42 set dt 0.02 set mean-desired-speed 1.34 "
                    + "set desired-speed-sd 0.26 set noise-strength 0 set show-trails? false set run-checks? true ";
                int steps;
                if (model.equals("Pedestrians.nlogox")) {
                    world.command(shared + "set periodic? true set population 150 set hall-width 10 setup");
                    steps = 500;
                } else if (model.contains("Door")) {
                    world.command(shared + "set log-csv? false set initial-per-side 20 set spawn-rate 0 "
                        + "set two-sided? true set door-width 1.2 set room-width 6 setup");
                    steps = 1000;
                } else if (model.contains("TrafficLight")) {
                    world.command(shared + "set log-csv? false set initial-per-side 20 set spawn-rate 0 "
                        + "set start-green? false set switch-time 60 set road-width 6 set walkway-width 6 setup");
                    steps = 1500;
                } else {
                    world.command(shared + "set log-csv? false set horizontal-rate 0.8 set vertical-rate 0.8 "
                        + "set two-way? true set corridor-width 4 setup");
                    steps = 2000;
                }
                world.command("repeat " + steps + " [go " + INVARIANTS + "]");
                near("elapsed-seconds", steps * .02, 1e-7, "Stress duration changed");
                require("speed-cap-violations = 0 and worst-wall-clearance >= body-radius - 1e-8",
                    "Crowd violated speed or wall constraints");
                require("worst-min-separation >= 0.5 - 1e-8 and pass-through-events = 0",
                    "Crowd violated the minimum pedestrian separation during a step");
                require("contact-solver-fallbacks = 0",
                    "Crowd needed the contact solver's safe-stop fallback");
                if (model.contains("TrafficLight")) {
                    require("held-on-road-count = 0 and max-stop-overshoot <= 1e-9",
                        "Crowd pressure pushed a held pedestrian over the stop boundary");
                    require("all? pedestrians with [is-number? stop-x] [gap-to-stop-line >= -1e-9]",
                        "Held crowd centre passed its stop line");
                }
                if (model.contains("Crossing")) {
                    require("arrived-count > 0 and arrived-count = length horizontal-results + length vertical-results",
                        "Crossing direction-specific arrival records do not conserve arrivals");
                }
                System.out.println("PASS bounded crowd: " + model + ", relative=" + relative + ", "
                    + world.report("(list elapsed-seconds count pedestrians arrived-count returned-count pass-through-events worst-min-separation contact-step-count contact-solver-fallbacks)"));
            }
        }
        System.out.println("PASS: the added contact constraint preserves 0.5 m minimum separation throughout all bounded crowd runs.");
    }

    public static void main(String[] args) throws Exception {
        try {
            lightTiming();
            redStopAndRelease();
            doorOrdering();
            if (!Arrays.asList(args).contains("--quick")) crowdStress();
            System.out.println("PASS: " + assertions + " scenario regression assertions.");
        } finally {
            if (world != null) world.dispose();
        }
    }
}
