import java.nio.file.Path;
import org.nlogo.headless.HeadlessWorkspace;

/**
 * Checks for the shared-module features used by the scenario models:
 * general wall contact (vertical, oblique, end point), stop lines, door
 * waypoints, crossing exits, lane counting, pass-through detection and the
 * statistics helpers. Run from the repository root with NetLogo's Java
 * runtime and jar, like the other suites.
 */
public class ScenarioChecks {
    private static HeadlessWorkspace world;

    private static void open(String model) throws Exception {
        if (world != null) world.dispose();
        world = HeadlessWorkspace.newInstance();
        world.open(Path.of(model).toAbsolutePath().toString());
    }

    private static void require(String condition, String message) throws Exception {
        if (!Boolean.TRUE.equals(world.report(condition))) {
            throw new AssertionError(message + ": " + condition);
        }
    }

    /** One pedestrian (who 0) in an otherwise empty door model, custom walls. */
    private static void contactFixture(String walls, String state) throws Exception {
        world.command("set initial-per-side 0 set spawn-rate 0 set dt 0.02 setup "
            + "add-pedestrian 5 4.5 1 set wall-segments " + walls + " "
            + "ask pedestrians [set desired-speed 1.34 set ax 0 set ay 0 set vx 0 set vy 0 "
            + state + " calculate-next-state]");
    }

    private static final String CAPPED =
        "[abs (vector-length next-vx next-vy - vector-length next-wx next-wy * "
        + "min (list 1 (1.3 * desired-speed / max (list 1e-12 vector-length next-wx next-wy)))) < 1e-9] of one-of pedestrians";

    private static void contactChecks() throws Exception {
        open("scenarios/Door.nlogox");
        // Vertical wall, approached from the left with a huge preferred velocity.
        contactFixture("[[10 0 10 9]]", "setxy 9.79 4 set wx 100 set wy 0.5");
        require("[abs (next-x - 9.8) < 1e-12 and next-wx = 0 and next-wy = 0.5] of one-of pedestrians",
            "Vertical wall from the left: not clamped to body-radius or inward w kept");
        require(CAPPED, "Vertical wall contact broke v = cap(w)");
        require("[next-y > ycor] of one-of pedestrians", "Vertical wall contact lost tangential motion");
        // ... and from the right.
        contactFixture("[[10 0 10 9]]", "setxy 10.21 4 set wx -100 set wy 0");
        require("[abs (next-x - 10.2) < 1e-12 and next-wx = 0] of one-of pedestrians", "Vertical wall from the right");

        // Oblique wall y = x, pedestrian above it moving into it.
        contactFixture("[[0 0 10 10]]", "setxy 5 5.3 set wx 3 set wy -3");
        require("[distance-to-segment next-x next-y [0 0 10 10] >= 0.2 - 1e-9] of one-of pedestrians",
            "Oblique wall: centre closer than body-radius");
        require("[next-y > next-x] of one-of pedestrians", "Oblique wall: crossed to the other side");
        require("[(- next-wx + next-wy) / sqrt 2 >= -1e-12] of one-of pedestrians",
            "Oblique wall: inward preferred velocity kept");
        require(CAPPED, "Oblique wall contact broke v = cap(w)");

        // End point of a segment (a door post), approached from straight above.
        contactFixture("[[10 0 10 4]]", "setxy 10 4.21 set wx 0 set wy -100");
        require("[distance-to-segment next-x next-y [10 0 10 4] >= 0.2 - 1e-9] of one-of pedestrians",
            "Door post from above: centre closer than body-radius");
        require("[next-wy = 0] of one-of pedestrians", "Door post from above: inward w kept");
        // ... and diagonally.
        contactFixture("[[10 0 10 4]]", "setxy 10.15 4.15 set wx -50 set wy -50");
        require("[distance-to-segment next-x next-y [10 0 10 4] >= 0.2 - 1e-9] of one-of pedestrians",
            "Door post diagonally: centre closer than body-radius");
        require(CAPPED, "Door post contact broke v = cap(w)");
        System.out.println("PASS: vertical, oblique and end-point wall contact keep body-radius, "
            + "remove inward motion and keep v = cap(w).");
    }

    private static void doorChecks() throws Exception {
        open("scenarios/Door.nlogox");
        // A lone walker starting next to the wall must steer through the opening.
        world.command("set door-width 1.2 set initial-per-side 0 set spawn-rate 0 setup "
            + "add-pedestrian 3 2.2 1 repeat 1500 [go]");
        require("arrived-count = 1 and door-passes = 1 and returned-count = 0",
            "Lone walker did not pass the 1.2 m door once and arrive");
        require("worst-wall-clearance >= 0.2 - 1e-9 and wall-penetration-count = 0",
            "Lone walker came closer than body-radius to a wall");
        // Below about 1.0 m the two posts out-push the driving force (see Info).
        world.command("set door-width 0.8 setup ask pedestrians [die] "
            + "add-pedestrian 3 4.5 1 repeat 1500 [go]");
        require("door-passes = 0 and count pedestrians = 1",
            "0.8 m door no longer blocks a lone walker: Info tab analysis is stale");
        require("[xcor < door-x - 0.2 and vector-length vx vy < 0.01] of one-of pedestrians",
            "Blocked walker is not at rest in front of the 0.8 m door");
        // Default crowd: invariants every step.
        world.command("set door-width 1.2 set initial-per-side 20 set run-checks? true setup "
            + "repeat 1000 [go check-invariants]");
        require("speed-cap-violations = 0 and worst-wall-clearance >= 0.2 - 1e-9",
            "Door crowd broke the speed cap or a wall");
        require("door-passes > 0 and length door-sequence = door-passes",
            "Door crowd made no recorded passages");
        System.out.println("PASS: door waypoint, 0.8 m blocking, crowd invariants.");
    }

    private static void trafficLightChecks() throws Exception {
        open("scenarios/TrafficLight.nlogox");
        // A lone walker approaching a red light stops at its line, critically damped.
        world.command("set initial-per-side 0 set spawn-rate 0 set start-green? false "
            + "set switch-time 40 set desired-speed-sd 0 set dt 0.02 setup "
            + "add-pedestrian 5 4.5 1 repeat 1500 [go]");
        require("light-colour = \"red\" and count pedestrians = 1", "Fixture: light or walker missing");
        require("max-stop-overshoot < 0.005",
            "Lone walker overshot its stop line (should be critically damped)");
        require("[gap-to-stop-line >= 0 and gap-to-stop-line < 0.05 and vector-length vx vy < 0.02] "
            + "of one-of pedestrians", "Lone walker is not waiting right before the curb");
        // Green: back to (near) desired speed within 5 s.
        world.command("repeat 500 [go]");
        require("light-colour = \"green\"", "Light did not switch after switch-time");
        world.command("repeat 250 [go]");
        require("[vector-length vx vy > 0.9 * desired-speed and stop-x = false] of one-of pedestrians",
            "Walker did not speed up to its desired speed within 5 s of green");
        // A walker already past its line when red starts keeps going.
        world.command("setup add-pedestrian 17.5 4.5 1 "
            + "ask pedestrians [set vx 1.34 set wx 1.34] repeat 300 [go]");
        require("[xcor > road-right and stop-x = false] of one-of pedestrians",
            "Walker already past the stop line was held");
        System.out.println("PASS: red light stop without overshoot, green recovery, past-line walkers continue.");
    }

    private static void crossingChecks() throws Exception {
        open("scenarios/Crossing.nlogox");
        world.command("set horizontal-rate 0 set vertical-rate 0 set run-checks? true setup "
            + "repeat 3 [spawn-walker true] repeat 1800 [go check-invariants]");
        require("count pedestrians = 0 and length vertical-results = arrived-count "
            + "and arrived-count > 0 and empty? horizontal-results",
            "Vertical walkers were not recorded as vertical arrivals");
        require("min delay-ratios-of \"vertical\" >= 1", "Delay ratio below free walking");

        // A downward walker pushed into the left horizontal arm (seen stuck in
        // crowded runs): its goal is straight below, behind the arm's wall.
        world.command("set horizontal-rate 0 set vertical-rate 0 setup "
            + "create-pedestrians 1 [initialize-pedestrian 12.07 13.68 13.7 1 -1 "
            + "set vertical? true set free-time 15]");
        world.command("go");
        require("[waypoints = [[15 13.68]]] of one-of pedestrians",
            "Displaced walker was not routed back to its corridor's centre line");
        world.command("repeat 1500 [go check-invariants]");
        require("count pedestrians = 0 and arrived-count = 1",
            "Walker displaced into the other corridor's arm did not get out and arrive");
        // Walkers inside their own band never get a detour.
        world.command("setup repeat 2 [spawn-walker true] repeat 2 [spawn-walker false] repeat 200 [go]");
        require("all? pedestrians [empty? waypoints]", "Walker inside its own corridor got a detour");
        System.out.println("PASS: crossing exits, per-direction travel records, displaced-walker detour.");
    }

    private static void laneAndStatsChecks() throws Exception {
        open("Pedestrians.nlogox");
        world.command("set periodic? true set population 2 setup checks-reset ask pedestrians [die] "
            // three clean lanes: right at y=3, left at y=6, right at y=9
            + "foreach [3 6 9] [y -> foreach range 6 [i -> create-pedestrians 1 "
            + "[initialize-pedestrian (i * 8 + y) y 0 y ifelse-value (y = 6) [-1] [1]]]]");
        require("lane-count = 3 and lanes-for-direction 1 = 2 and lanes-for-direction -1 = 1",
            "Three separated lanes not counted as 3");
        world.command("create-pedestrians 2 [initialize-pedestrian 20 7.4 0 7.4 1]");
        require("lane-count = 3", "Two stragglers counted as a lane (min members is 5)");
        // One wide band (2 .. 6 m): every reported lane must obey the 1.5 m rule.
        world.command("ask pedestrians [die] foreach range 9 [i -> create-pedestrians 1 "
            + "[initialize-pedestrian (i * 5) (2 + i * 0.5) 0 0 1]]");
        require("not empty? lane-clusters 1 and "
            + "reduce [[a b] -> a and b] fput true map [c -> max-lane-residual c < lane-max-distance] lane-clusters 1",
            "A reported lane has a member 1.5 m or more from its regression line");
        // Pass-through detection uses actual stored paths, not inferred vx * dt.
        world.command("set last-step-paths [[0 24.9 6 25.1 6] [1 25.2 6 25 6]]");
        require("pass-through-pairs = 1", "Head-on pass-through not detected");
        world.command("set last-step-paths [[0 24.9 6 25.1 6] [1 25.2 6.3 25 6.3]]");
        require("pass-through-pairs = 0", "Passing 0.3 m apart sideways counted as pass-through");
        // Statistics helpers.
        require("abs (stat-variance [1 2 3 4] - 5 / 3) < 1e-12 and "
            + "abs (stat-population-variance [1 2 3 4] - 1.25) < 1e-12 and "
            + "abs (stat-sd [1 2 3 4] - sqrt (5 / 3)) < 1e-12 and stat-mean [] = 0 and stat-sd [1] = 0",
            "Statistics helpers wrong");
        System.out.println("PASS: lane counting, 1.5 m lane rule, pass-through detection, statistics.");
    }

    public static void main(String[] args) throws Exception {
        try {
            contactChecks();
            doorChecks();
            trafficLightChecks();
            crossingChecks();
            laneAndStatsChecks();
        } finally {
            if (world != null) world.dispose();
        }
        System.exit(0);
    }
}
