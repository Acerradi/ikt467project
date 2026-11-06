import java.nio.file.Path;
import org.nlogo.headless.HeadlessWorkspace;

/** Regression checks for individual, lifetime-fixed Gaussian desired speeds. */
public class SpeedChecks {
    private static HeadlessWorkspace world;

    private static void require(String condition, String message) throws Exception {
        if (!Boolean.TRUE.equals(world.report(condition))) {
            throw new AssertionError(message + ": " + condition);
        }
    }

    private static String speeds() throws Exception {
        return world.report("map [p -> [(list who desired-speed)] of p] sort pedestrians").toString();
    }

    public static void main(String[] args) throws Exception {
        world = HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            require("noise-strength = 0 and mean-desired-speed = 1.34 and desired-speed-sd = 0.26",
                "Saved interface defaults differ from the requested settings");
            world.command("set periodic? true set population 150 set simulation-seed 42 setup");
            require("hall-length = 50 and hall-top - hall-bottom = 10 "
                + "and count pedestrians = 150 and abs (pedestrian-density - 0.3) < 1e-9",
                "Paper-sized hallway/population/density defaults are incorrect");
            require("anticipation-time = 2", "Anticipation horizon is not two seconds");
            require("min [desired-speed] of pedestrians > 0 and variance [desired-speed] of pedestrians > 0",
                "Population has no positive individual speed variation");
            String initial = speeds();
            world.command("repeat 300 [go]");
            if (!speeds().equals(initial)) {
                throw new AssertionError("Desired speeds changed while walking");
            }
            world.command("setup");
            if (!speeds().equals(initial)) {
                throw new AssertionError("Setup did not reproduce the same sampled speeds");
            }
            world.command("set mean-desired-speed 1.8 set desired-speed-sd 0 repeat 50 [go]");
            if (!speeds().equals(initial)) {
                throw new AssertionError("Changing distribution controls altered existing pedestrians");
            }

            // Test the distribution at scale without creating a crowded model.
            world.command("set mean-desired-speed 1.34 set desired-speed-sd 0.26 random-seed 123 "
                + "let samples n-values 20000 [sample-desired-speed] "
                + "if min samples <= 0 [error \"Nonpositive desired speed\"] "
                + "if abs (mean samples - 1.34) > 0.01 [error \"Gaussian mean is incorrect\"] "
                + "if abs (standard-deviation samples - 0.26) > 0.01 [error \"Gaussian SD is incorrect\"] "
                + "set mean-desired-speed 0.2 set desired-speed-sd 0.8 "
                + "if min (n-values 5000 [sample-desired-speed]) <= 0 "
                + "[error \"Negative-tail resampling failed\"]");

            // The force and cap must use each agent's value, not the mean slider.
            world.command("set periodic? true set population 2 set mean-desired-speed 1.34 "
                + "set desired-speed-sd 0.26 setup set potential-strength 0 set wall-strength 0 "
                + "ask pedestrian 0 [set desired-speed 0.8] ask pedestrian 1 [set desired-speed 1.8] "
                + "ask pedestrians [set ex 1 set ey 0 set vx 0 set vy 0 calculate-acceleration]");
            require("abs ([ax] of pedestrian 0 - 1.6) < 1e-9 "
                + "and abs ([ax] of pedestrian 1 - 3.6) < 1e-9", "Driving force ignores individual speeds");
            world.command("ask pedestrians [set wx 100 set wy 0 set ax 0 set ay 0 calculate-next-state]");
            require("abs ([next-vx] of pedestrian 0 - 1.04) < 1e-9 "
                + "and abs ([next-vx] of pedestrian 1 - 2.34) < 1e-9", "Individual speed caps are incorrect");

            // New arrivals use the current distribution; existing agents keep theirs.
            world.command("set periodic? false set mean-desired-speed 1.1 set desired-speed-sd 0 "
                + "setup spawn-pedestrian set mean-desired-speed 1.8 spawn-pedestrian");
            require("[desired-speed] of pedestrian 0 = 1.1 and [desired-speed] of pedestrian 1 = 1.8",
                "Birth did not use the current distribution parameters");
            System.out.println("PASS: defaults, anticipation, Gaussian mean/SD, positive tail, fixed lifetime speeds, seed, individual forces/caps and new arrivals.");
        } finally {
            world.dispose();
        }
    }
}
