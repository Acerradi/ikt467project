import java.nio.file.Path;
import org.nlogo.headless.HeadlessWorkspace;

/** Independent pair-force checks for both selectable anticipation formulations. */
public class AnticipationChecks {
    private static HeadlessWorkspace world;
    private static int checks;

    private static double number(String expression) throws Exception {
        return ((Number) world.report(expression)).doubleValue();
    }

    private static void require(String expression, String label) throws Exception {
        if (!Boolean.TRUE.equals(world.report(expression))) throw new AssertionError(label);
        checks++;
    }

    private static void close(double actual, double expected, double tolerance, String label) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance) {
            throw new AssertionError(label + ": actual=" + actual + ", expected=" + expected);
        }
        checks++;
    }

    private static void configure(boolean relative) throws Exception {
        world.command("set periodic? true set population 2 set noise-strength 0 "
            + "set relative-anticipation? " + relative + " setup");
    }

    private static double[] pairForce() throws Exception {
        return new double[]{number("[item 0 (pair-force pedestrian 1)] of pedestrian 0"),
            number("[item 1 (pair-force pedestrian 1)] of pedestrian 0")};
    }

    private static double potential(double x, double y, double sx, double sy) {
        double sum = Math.hypot(x, y) + Math.hypot(x - sx, y - sy);
        double b = 0.5 * Math.sqrt(sum * sum - sx * sx - sy * sy);
        return 2.1 * Math.exp(-b / 0.3);
    }

    private static void expectedGradient(double qx, double qy, double sx, double sy, String label)
            throws Exception {
        double h = 1e-6;
        double fx = -(potential(qx+h, qy, sx, sy)-potential(qx-h, qy, sx, sy))/(2*h);
        double fy = -(potential(qx, qy+h, sx, sy)-potential(qx, qy-h, sx, sy))/(2*h);
        double[] actual = pairForce();
        close(actual[0], fx, 2e-6 * Math.max(1, Math.abs(fx)), label + " x");
        close(actual[1], fy, 2e-6 * Math.max(1, Math.abs(fy)), label + " y");
    }

    private static void fixture() throws Exception {
        world.command("ask pedestrian 0 [setxy 24.2 7.1 set vx 0.9 set vy 0.4 set ex 1 set ey 0] "
            + "ask pedestrian 1 [setxy 25 6.5 set vx -0.5 set vy -0.2 set ex 0.6 set ey 0.8]");
    }

    private static void relativeChecks() throws Exception {
        configure(true);
        fixture();
        // q=(-.8,.6), s=T*(v_j-v_i)=2*((-.5,-.2)-(.9,.4))=(-2.8,-1.2).
        // Nonzero lateral components and opposing velocities distinguish signs and branches.
        expectedGradient(-0.8, 0.6, -2.8, -1.2, "Signed relative actual-velocity gradient");
        double[] baseline = pairForce();
        close(number("[item 0 (pair-force pedestrian 0)] of pedestrian 1"), -baseline[0], 1e-12,
            "Swapping pedestrians reverses raw relative force x before perception weights");
        close(number("[item 1 (pair-force pedestrian 0)] of pedestrian 1"), -baseline[1], 1e-12,
            "Swapping pedestrians reverses raw relative force y before perception weights");
        world.command("ask pedestrians [set vx vx + 0.3 set vy vy - 0.25]");
        double[] boosted = pairForce();
        close(boosted[0], baseline[0], 1e-12, "Common velocity boost leaves raw relative force unchanged x");
        close(boosted[1], baseline[1], 1e-12, "Common velocity boost leaves raw relative force unchanged y");
        world.command("ask pedestrian 1 [set ex -1 set ey 0]");
        double[] redirected = pairForce();
        close(redirected[0], baseline[0], 1e-12, "Relative prediction ignores desired direction x");
        close(redirected[1], baseline[1], 1e-12, "Relative prediction ignores desired direction y");

        world.command("ask pedestrian 0 [setxy 25.3 6.9 set vx 0.9 set vy 0.4] "
            + "ask pedestrian 1 [setxy 25 6.5 set vx 0.9 set vy 0.4]");
        double magnitude = 7 * Math.exp(-0.5 / 0.3);
        double[] circular = pairForce();
        close(circular[0], 0.6*magnitude, 1e-9, "Equal moving velocities give circular limit x");
        close(circular[1], 0.8*magnitude, 1e-9, "Equal moving velocities give circular limit y");
    }

    private static void originalChecks() throws Exception {
        configure(false);
        fixture();
        double speed = Math.hypot(-0.5, -0.2);
        expectedGradient(-0.8, 0.6, 2*speed*0.6, 2*speed*0.8,
            "Original actual speed times desired direction remains unchanged");
        double[] baseline = pairForce();
        world.command("ask pedestrian 0 [set vx -1.2 set vy 0.8]");
        double[] changed = pairForce();
        close(changed[0], baseline[0], 1e-12, "Original prediction ignores subject velocity x");
        close(changed[1], baseline[1], 1e-12, "Original prediction ignores subject velocity y");
    }

    private static void setupCaptureChecks() throws Exception {
        configure(true);
        fixture();
        double[] relative = pairForce();
        world.command("set relative-anticipation? false");
        require("active-relative-anticipation?", "Changing switch altered captured true mode before setup");
        double[] unchanged = pairForce();
        close(unchanged[0], relative[0], 1e-12, "Pending false switch preserves force x");
        close(unchanged[1], relative[1], 1e-12, "Pending false switch preserves force y");
        world.command("setup");
        require("not active-relative-anticipation?", "Setup failed to activate original anticipation");
        fixture();
        double[] original = pairForce();
        if (Math.hypot(original[0]-relative[0], original[1]-relative[1]) < 1e-3) {
            throw new AssertionError("Switch fixture does not distinguish the two models");
        }
        checks++;
        world.command("set relative-anticipation? true");
        require("not active-relative-anticipation?", "Changing switch altered captured false mode before setup");
        unchanged = pairForce();
        close(unchanged[0], original[0], 1e-12, "Pending true switch preserves force x");
        close(unchanged[1], original[1], 1e-12, "Pending true switch preserves force y");
        world.command("setup");
        require("active-relative-anticipation?", "Setup failed to activate relative anticipation");
    }

    private static void seamChecks() throws Exception {
        for (boolean relative : new boolean[]{false, true}) {
            configure(relative);
            world.command("ask pedestrian 0 [setxy 0.2 6.8 set vx 0.6 set vy 0.2 set ex 1 set ey 0] "
                + "ask pedestrian 1 [setxy 49.8 6.2 set vx -0.7 set vy -0.3 set ex -1 set ey 0]");
            double[] seam = pairForce();
            world.command("ask pedestrian 0 [set xcor 25.2] ask pedestrian 1 [set xcor 24.8]");
            double[] local = pairForce();
            close(seam[0], local[0], 1e-10, "Periodic seam matches local force x, relative=" + relative);
            close(seam[1], local[1], 1e-10, "Periodic seam matches local force y, relative=" + relative);
        }
    }

    private static void identicalInitialPopulationCheck() throws Exception {
        String stateReporter = "map [p -> [(list who xcor ycor desired-speed travel-direction "
            + "wx wy vx vy noise-unit-x noise-unit-y)] of p] sort pedestrians";
        world.command("set periodic? true set population 150 set simulation-seed 42 "
            + "set mean-desired-speed 1.34 set desired-speed-sd 0.26 set noise-strength 0 "
            + "set relative-anticipation? false setup");
        String originalInitial = world.report(stateReporter).toString();
        world.command("set relative-anticipation? true setup");
        String relativeInitial = world.report(stateReporter).toString();
        if (!originalInitial.equals(relativeInitial)) {
            throw new AssertionError("Changing anticipation mode altered the seeded initial population or RNG sequence");
        }
        checks++;
    }

    public static void main(String[] args) throws Exception {
        world = HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            require("relative-anticipation?", "Saved model must default to relative anticipation");
            relativeChecks();
            originalChecks();
            setupCaptureChecks();
            seamChecks();
            identicalInitialPopulationCheck();
            System.out.println("PASS: " + checks + " checks of relative velocity, lateral prediction, circular limit, "
                + "common boosts, original branch, setup capture, both periodic seams and identical seeded populations.");
        } finally { world.dispose(); }
    }
}
