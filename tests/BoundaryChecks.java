import java.nio.file.Path;
import java.util.Arrays;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.Element;
import org.nlogo.headless.HeadlessWorkspace;

/** Run from the repository root using NetLogo's Java runtime and jar. */
public class BoundaryChecks {
    private static HeadlessWorkspace world;
    private static boolean requireDrain;

    private static void interfaceChecks() throws Exception {
        // The desktop UI reads control names differently from the headless
        // loader when display and variable disagree. Keep their identifiers equal.
        var document = DocumentBuilderFactory.newInstance().newDocumentBuilder()
            .parse(Path.of("Pedestrians.nlogox").toFile());
        for (String tag : new String[] {"slider", "switch"}) {
            var controls = document.getElementsByTagName(tag);
            for (int i = 0; i < controls.getLength(); i++) {
                var control = (Element) controls.item(i);
                if (!control.getAttribute("display").equals(control.getAttribute("variable"))) {
                    throw new AssertionError("Desktop/headless control name mismatch: "
                        + control.getAttribute("variable"));
                }
            }
        }
        System.out.println("PASS: desktop and headless interface identifiers agree.");
    }

    private static void require(String condition, String message) throws Exception {
        if (!Boolean.TRUE.equals(world.report(condition))) {
            throw new AssertionError(message + ": " + condition);
        }
    }

    private static void fixture(String commands) throws Exception {
        world.command("set periodic? false set spawn-rate 0 set dt 0.02 "
            + "set mean-desired-speed 1.34 set desired-speed-sd 0.26 setup spawn-pedestrian "
            + "ask pedestrian 0 [set desired-speed 1.34 set ax 0 set ay 0 " + commands + "]");
    }

    private static void unitChecks() throws Exception {
        fixture("setxy 25 1.71 set goal-x 49 set goal-y 6 "
            + "set wx 0.5 set wy -100 calculate-next-state");
        require("abs ([next-y] of pedestrian 0 - 1.7) < 1e-9", "Lower wall crossed");
        require("[next-vy = 0 and next-wy = 0] of pedestrian 0", "Outward wall velocity retained");
        require("[next-vx > 0 and next-x > xcor] of pedestrian 0", "Along-wall motion lost");
        world.command("ask pedestrian 0 [commit-next-state]");

        fixture("setxy 25 11.29 set goal-x 1 set goal-y 6 "
            + "set wx -0.5 set wy 100 calculate-next-state");
        require("abs ([next-y] of pedestrian 0 - 11.3) < 1e-9", "Upper wall crossed");
        require("[next-vy = 0 and next-wy = 0] of pedestrian 0", "Outward wall velocity retained");
        require("[next-vx < 0 and next-x < xcor] of pedestrian 0", "Along-wall motion lost");
        world.command("ask pedestrian 0 [commit-next-state]");

        // Miss the destination disk by several metres, but cross the exit plane.
        fixture("setxy 48.99 3 set goal-x 49 set goal-y 9 "
            + "set wx 100 set wy 0 calculate-next-state commit-next-state");
        require("not any? pedestrians and arrived-count = 1 and returned-count = 0", "Right exit missed");
        fixture("setxy 1.01 3 set goal-x 1 set goal-y 9 "
            + "set wx -100 set wy 0 calculate-next-state commit-next-state");
        require("not any? pedestrians and arrived-count = 1 and returned-count = 0", "Left exit missed");

        // Even a buffered coordinate beyond the NetLogo world must be removed
        // before setxy; the original implementation raised a runtime error here.
        fixture("setxy 50.49 3 set goal-x 49 set goal-y 9 "
            + "set wx 100 set wy 0 calculate-next-state commit-next-state");
        require("not any? pedestrians and arrived-count = 1", "Out-of-world departure failed");
        fixture("setxy -0.49 3 set goal-x 1 set goal-y 9 "
            + "set wx -100 set wy 0 calculate-next-state commit-next-state");
        require("not any? pedestrians and arrived-count = 1", "Out-of-world departure failed");

        fixture("setxy 1.01 6 set goal-x 49 set goal-y 6 "
            + "set wx -100 set wy 0 calculate-next-state commit-next-state");
        require("not any? pedestrians and arrived-count = 0 and returned-count = 1", "Return counted as arrival");
        fixture("setxy 48.99 6 set goal-x 1 set goal-y 6 "
            + "set wx 100 set wy 0 calculate-next-state commit-next-state");
        require("not any? pedestrians and arrived-count = 0 and returned-count = 1", "Return counted as arrival");

        world.command("set periodic? false setup repeat 10 [go]");
        require("ticks = 10 and not any? pedestrians", "Empty hallway stopped ticking");
        System.out.println("PASS: both walls, tangential motion, exit planes, world edges, return counts and empty clock.");
    }

    private static final String INVARIANTS =
        "if spawned-count != count pedestrians + arrived-count + returned-count "
        + "[error \"Pedestrian count mismatch\"] "
        + "foreach sort pedestrians [p -> ask p ["
        + "if ycor < hall-bottom + body-radius - 1e-9 or ycor > hall-top - body-radius + 1e-9 [error \"Wall crossed\"] "
        + "if xcor < open-left or xcor > open-right [error \"Open-end departure was not removed\"] "
        + "if vector-length vx vy > 1.3 * desired-speed + 1e-9 [error \"Speed limit exceeded\"] "
        + "]]";

    private static void stressChecks() throws Exception {
        for (int seed : new int[] {42, 1}) {
            world.command("set periodic? false set spawn-rate 5 set dt 0.02 set simulation-seed " + seed
                + " set mean-desired-speed 1.34 set desired-speed-sd 0.26 set noise-strength "
                + (seed == 42 ? "0" : "0.05") + " setup "
                + "repeat 6000 [go " + INVARIANTS + "]");
            require("abs (elapsed-seconds - 120) < 1e-6", "Run stopped advancing");
            System.out.println("PASS: seed " + seed + ", 120 crowded seconds: "
                + world.report("(list spawned-count arrived-count returned-count count pedestrians)"));
            world.command("set spawn-rate 0 let drain-steps 0 "
                + "while [any? pedestrians and drain-steps < 15000] [go " + INVARIANTS
                + " set drain-steps drain-steps + 1]");
            if (!Boolean.TRUE.equals(world.report("not any? pedestrians"))) {
                System.out.println("INCOMPLETE DRAIN: seed " + seed + ", pedestrians remaining after 300 s: "
                    + world.report("count pedestrians") + ". Boundary/count checks still hold.");
                if (requireDrain) {
                    throw new AssertionError("Hallway failed to empty after arrivals stopped");
                }
            } else {
                System.out.println("PASS: open hallway cleared after arrivals stopped.");
            }
        }
        world.command("set periodic? false set spawn-rate 2 set dt 0.1 set simulation-seed 42 "
            + "set mean-desired-speed 1.34 set desired-speed-sd 0.26 set noise-strength 0 setup "
            + "repeat 600 [go " + INVARIANTS + "]");
        require("abs (elapsed-seconds - 60) < 1e-6", "Larger-step run stopped advancing");
        System.out.println("PASS: dt=0.1 for 60 seconds, wall bounds and population accounting preserved.");
    }

    public static void main(String[] args) throws Exception {
        requireDrain = Arrays.asList(args).contains("--require-drain");
        interfaceChecks();
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
