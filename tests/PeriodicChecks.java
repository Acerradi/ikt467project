import java.nio.file.Path;
import java.util.Arrays;
import org.nlogo.headless.HeadlessWorkspace;

/** Checks the periodic mode using the actual runnable model. */
public class PeriodicChecks {
    private static HeadlessWorkspace world;

    private static void require(String condition, String message) throws Exception {
        if (!Boolean.TRUE.equals(world.report(condition))) {
            throw new AssertionError(message + ": " + condition);
        }
    }

    private static void configure(int population, int seed, double noise) throws Exception {
        world.command("set periodic? true set population " + population
            + " set simulation-seed " + seed + " set noise-strength " + noise
            + " set dt 0.02 set mean-desired-speed 1.34 set desired-speed-sd 0.26 "
            + "set spawn-rate 5 set show-trails? false setup");
    }

    private static String metadata() throws Exception {
        return world.report("map [p -> [(list who travel-direction goal-x goal-y desired-speed "
            + "noise-unit-x noise-unit-y)] of p] sort pedestrians").toString();
    }

    private static String state() throws Exception {
        return world.report("(list elapsed-seconds spawned-count wrap-count "
            + "(map [p -> [(list who xcor ycor desired-speed wx wy vx vy noise-unit-x noise-unit-y)] "
            + "of p] sort pedestrians))").toString();
    }

    private static void unitChecks() throws Exception {
        configure(2, 42, 0.05);
        world.command("ask pedestrians [set desired-speed 1.34]");
        require("active-periodic? and count pedestrians = 2", "Periodic setup failed");
        require("abs (periodic-delta-x 49.6 + 0.4) < 1e-9", "Wrong displacement period");
        require("abs (periodic-delta-x -49.6 - 0.4) < 1e-9", "Wrong reverse displacement");
        String initialMetadata = metadata();

        world.command("set show-trails? true ask pedestrian 0 ["
            + "setxy 49.99 6 set wx 1 set wy 0.2 set ax 0 set ay 0 calculate-next-state]");
        require("abs ([next-x] of pedestrian 0 - 0.01) < 1e-9", "Right overshoot lost");
        require("abs ([next-y] of pedestrian 0 - 6.004) < 1e-9", "Wrap changed y");
        require("[next-wrap-count = 1 and next-vx = 1 and next-vy = 0.2 "
            + "and next-wx = 1 and next-wy = 0.2] of pedestrian 0", "Wrap changed velocity");
        world.command("ask pedestrian 0 [commit-next-state]");
        require("wrap-count = 1 and count pedestrians = 2", "Wrap created/deleted a pedestrian");
        require("[pen-mode = \"up\"] of pedestrian 0", "Trail jumps across seam");
        world.command("ask pedestrian 0 [calculate-next-state commit-next-state]");
        require("[pen-mode = \"down\"] of pedestrian 0", "Trails did not resume");

        world.command("ask pedestrian 1 [setxy 0.01 6 set wx -1 set wy -0.2 "
            + "set ax 0 set ay 0 calculate-next-state]");
        require("abs ([next-x] of pedestrian 1 - 49.99) < 1e-9", "Left overshoot lost");
        require("abs ([next-y] of pedestrian 1 - 5.996) < 1e-9", "Wrap changed y");
        world.command("ask pedestrian 1 [commit-next-state]");
        require("wrap-count = 2 and spawned-count = 2 and arrived-count = 0 and returned-count = 0",
            "Wrap affected open-mode counters");
        if (!metadata().equals(initialMetadata)) {
            throw new AssertionError("Wrap changed identities, directions, goals or force biases");
        }

        // This comparison fails if coordinates wrap but the pair force does not.
        world.command("ask pedestrian 0 [pen-up setxy 0.2 6 set vx 0.8 set vy 0 set ex 1 set ey 0] "
            + "ask pedestrian 1 [pen-up setxy 49.8 6 set vx -0.8 set vy 0 set ex -1 set ey 0] "
            + "let seam-force [pair-force pedestrian 1] of pedestrian 0 "
            + "if item 0 seam-force <= 0.1 [error \"Missing seam repulsion\"] "
            + "ask pedestrian 0 [set xcor 25.2] ask pedestrian 1 [set xcor 24.8] "
            + "let local-force [pair-force pedestrian 1] of pedestrian 0 "
            + "if abs (item 0 seam-force - item 0 local-force) > 1e-9 "
            + "or abs (item 1 seam-force - item 1 local-force) > 1e-9 "
            + "[error \"Seam force differs from equivalent local geometry\"]");

        configure(150, 42, 0.05);
        require("count pedestrians with [travel-direction = 1] = 75 "
            + "and count pedestrians with [travel-direction = -1] = 75", "Unbalanced initial directions");
        require("abs (pedestrian-density - 150 / 500) < 1e-9", "Wrong density area");
        world.command("foreach sort pedestrians [p -> ask p [foreach sort other pedestrians [q -> "
            + "if separation-from ([xcor] of q) ([ycor] of q) < 0.6 - 1e-9 "
            + "[error \"Initial clearance violated across seam\"]]]]");
        world.command("repeat 200 [go]");
        String firstState = state();
        configure(150, 42, 0.05);
        world.command("repeat 200 [go]");
        if (!state().equals(firstState)) {
            throw new AssertionError("Repeated setup did not reproduce the same run");
        }
        world.command("set population 200 set periodic? false repeat 10 [go]");
        require("active-periodic? and count pedestrians = 150 and spawned-count = 150",
            "Controls changed the boundary mode/population without setup");
        world.command("setup");
        require("not active-periodic? and not any? pedestrians", "Could not switch to open mode");
        System.out.println("PASS: both wraps, state retention, trails, seam forces, placement, balance, density, seed and mode switching.");
    }

    private static final String INVARIANTS =
        "if count pedestrians != population or spawned-count != population "
        + "or arrived-count != 0 or returned-count != 0 or blocked-spawn-count != 0 "
        + "[error \"Periodic population/counters changed\"] "
        + "foreach sort pedestrians [p -> ask p ["
        + "if xcor < 0 or xcor >= hall-length [error \"X is outside the periodic domain\"] "
        + "if ycor < hall-bottom + body-radius - 1e-9 or ycor > hall-top - body-radius + 1e-9 [error \"Wall crossed\"] "
        + "if ex != travel-direction or ey != 0 [error \"Desired direction changed\"] "
        + "if vector-length vx vy > 1.3 * desired-speed + 1e-9 [error \"Speed limit exceeded\"] "
        + "]]";

    private static void stressChecks() throws Exception {
        int[] populations = {150, 200};
        int[] steps = {6000, 3000};
        for (int i = 0; i < populations.length; i++) {
            configure(populations[i], i == 0 ? 42 : 1, i == 0 ? 0 : 0.05);
            String before = metadata();
            world.command("repeat " + steps[i] + " [go " + INVARIANTS + "]");
            require("wrap-count > 0", "Stress test never crossed the seam");
            require("abs (elapsed-seconds - " + (steps[i] * 0.02) + ") < 1e-6", "Clock stopped advancing");
            if (!metadata().equals(before)) {
                throw new AssertionError("Run changed the original population metadata");
            }
            System.out.println("PASS: periodic population=" + populations[i] + ": "
                + world.report("(list elapsed-seconds count pedestrians wrap-count pedestrian-density)"));
        }
    }

    public static void main(String[] args) throws Exception {
        world = HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            unitChecks();
            if (Arrays.asList(args).contains("--stress")) {
                stressChecks();
            }
        } finally {
            world.dispose();
        }
    }
}
