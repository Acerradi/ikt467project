import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Independent geometric checks for the optional 0.5 m pair-contact constraint.
 * Run from the repository root using NetLogo's Java runtime and jar.
 * Force-free fixtures isolate contact geometry; the normal-force no-contact
 * replay verifies that the extension is inactive when no correction is needed.
 */
public class SeparationChecks {
    private static HeadlessWorkspace world;
    private static int checks;
    private static final double DISTANCE = .5;
    private static final double TOL = 2e-8;

    private static double value(LogoList list, int i) {
        return ((Number) list.apply(i)).doubleValue();
    }
    private static double number(String expression) throws Exception {
        return ((Number) world.report(expression)).doubleValue();
    }
    private static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
        checks++;
    }
    private static void near(double actual, double expected, double tolerance, String label) {
        require(Double.isFinite(actual) && Math.abs(actual - expected) <= tolerance,
            label + ": expected " + expected + ", got " + actual);
    }
    private static List<double[]> rows(String expression, int width) throws Exception {
        LogoList list = (LogoList) world.report(expression);
        var result = new ArrayList<double[]>();
        for (int i = 0; i < list.size(); i++) {
            LogoList row = (LogoList) list.apply(i);
            double[] values = new double[width];
            for (int j = 0; j < width; j++) values[j] = value(row, j);
            result.add(values);
        }
        return result;
    }

    private static void setup(boolean periodic, double dt, boolean enabled) throws Exception {
        world.command("set periodic? " + periodic + " set population 0 set spawn-rate 0 "
            + "set dt " + dt + " set mean-desired-speed 1.34 set desired-speed-sd 0 "
            + "set noise-strength 0 set simulation-seed 42 set run-checks? false setup "
            + "set pair-contact-enabled? " + enabled
            + " set wall-segments [] set potential-strength 0 set wall-strength 0");
    }

    private static void add(double x, double y, double vx, double vy,
                            double goalX, double goalY, int direction) throws Exception {
        world.command("create-pedestrians 1 [initialize-pedestrian " + x + " " + y + " "
            + goalX + " " + goalY + " " + direction + " set vx " + vx + " set vy " + vy
            + " set wx vx set wy vy] set spawned-count spawned-count + 1");
    }

    private static double pointSegment(double px, double py, double ax, double ay,
                                        double bx, double by) {
        double dx = bx - ax, dy = by - ay, n = dx * dx + dy * dy;
        double t = n == 0 ? 0 : Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / n));
        return Math.hypot(px - ax - t * dx, py - ay - t * dy);
    }

    private static double segmentDistance(double ax, double ay, double bx, double by,
                                           double cx, double cy, double dx, double dy) {
        double ux = bx - ax, uy = by - ay, vx = dx - cx, vy = dy - cy;
        double determinant = ux * vy - uy * vx;
        if (Math.abs(determinant) > 1e-15) {
            double qx = cx - ax, qy = cy - ay;
            double t = (qx * vy - qy * vx) / determinant;
            double s = (qx * uy - qy * ux) / determinant;
            if (t >= 0 && t <= 1 && s >= 0 && s <= 1) return 0;
        }
        return Math.min(Math.min(pointSegment(ax, ay, cx, cy, dx, dy),
                                 pointSegment(bx, by, cx, cy, dx, dy)),
                        Math.min(pointSegment(cx, cy, ax, ay, bx, by),
                                 pointSegment(dx, dy, ax, ay, bx, by)));
    }

    // Independent closest point of the relative-motion segment to the origin.
    private static double sweptDistance(double[] a, double[] b, double period) {
        double qx = a[1] - b[1], qy = a[2] - b[2];
        double dx = (a[3] - a[1]) - (b[3] - b[1]);
        double dy = (a[4] - a[2]) - (b[4] - b[2]);
        int first = 0, last = 0;
        if (period > 0) {
            first = (int) Math.floor(-Math.max(qx, qx + dx) / period) - 1;
            last = (int) Math.ceil(-Math.min(qx, qx + dx) / period) + 1;
        }
        double result = Double.POSITIVE_INFINITY;
        for (int k = first; k <= last; k++) {
            double x = qx + k * period;
            result = Math.min(result, pointSegment(0, 0, x, qy, x + dx, qy + dy));
        }
        return result;
    }

    private static double verifyStep(String label) throws Exception {
        boolean periodic = Boolean.TRUE.equals(world.report("active-periodic?"));
        double period = periodic ? number("hall-length") : 0;
        List<double[]> paths = rows("last-step-paths", 5);
        double smallest = Double.POSITIVE_INFINITY;
        for (int i = 0; i < paths.size(); i++) for (int j = i + 1; j < paths.size(); j++) {
            double d = sweptDistance(paths.get(i), paths.get(j), period);
            require(d >= DISTANCE - TOL, label + ": swept separation " + d + " for IDs "
                + paths.get(i)[0] + " and " + paths.get(j)[0]);
            smallest = Math.min(smallest, d);
        }
        for (double[] p : rows("map [p -> [(list vx vy wx wy desired-speed)] of p] sort pedestrians", 5)) {
            double norm = Math.hypot(p[2], p[3]);
            double factor = norm == 0 ? 1 : Math.min(1, 1.3 * p[4] / norm);
            near(p[0], p[2] * factor, TOL, label + " cap x");
            near(p[1], p[3] * factor, TOL, label + " cap y");
            require(Math.hypot(p[0], p[1]) <= 1.3 * p[4] + TOL, label + " speed cap");
        }
        double radius = number("body-radius");
        for (double[] path : paths) for (double[] wall : rows("wall-segments", 4)) {
            int first = periodic ? -1 : 0, last = periodic ? 1 : 0;
            for (int k = first; k <= last; k++) {
                double shift = k * period;
                double d = segmentDistance(path[1], path[2], path[3], path[4],
                    wall[0] + shift, wall[1], wall[2] + shift, wall[3]);
                require(d >= radius - TOL, label + ": corrected path entered a wall: " + d);
            }
        }
        require(Boolean.TRUE.equals(world.report("all? pedestrians with [is-number? stop-x] "
            + "[(stop-x - xcor) * travel-direction >= -2e-8]")), label + ": crossed held stop line");
        return smallest;
    }

    private static void headOn(double angle, double dt, int steps) throws Exception {
        setup(false, dt, true);
        double c = Math.cos(angle), s = Math.sin(angle);
        add(25 - .3 * c, 6.5 - .3 * s, 1.34 * c, 1.34 * s, 25 + 4 * c, 6.5 + 4 * s, 1);
        add(25 + .3 * c, 6.5 + .3 * s, -1.34 * c, -1.34 * s, 25 - 4 * c, 6.5 - 4 * s, -1);
        for (int i = 0; i < steps; i++) {
            world.command("ped-step");
            verifyStep("Head-on angle=" + angle + ", dt=" + dt);
        }
        List<double[]> ps = rows("map [p -> [(list xcor ycor)] of p] sort pedestrians", 2);
        near((ps.get(0)[0] + ps.get(1)[0]) / 2, 25, TOL, "Symmetric centre x");
        near((ps.get(0)[1] + ps.get(1)[1]) / 2, 6.5, TOL, "Symmetric centre y");
        near(Math.hypot(ps.get(0)[0] - ps.get(1)[0], ps.get(0)[1] - ps.get(1)[1]),
            DISTANCE, 2e-7, "Symmetric encounter reaches the separation boundary");
    }

    private static void geometricCases() throws Exception {
        for (double angle : new double[]{0, Math.PI / 2, Math.PI / 4}) headOn(angle, .02, 10);
        headOn(0, .5, 1); // Endpoints would be clear again after a full pass-through.
        setup(false, .5, true);
        add(24.7, 6.4, 1.34, 0, 40, 6.4, 1);
        add(25.3, 6.6, -1.34, 0, 10, 6.6, -1);
        world.command("ped-step"); verifyStep("Oblique passing/tunnelling");

        setup(false, .02, true);
        add(24.74, 6.5, 1.742, 0, 40, 6.5, 1);
        add(25.26, 6.5, -1.742, 0, 10, 6.5, -1);
        world.command("ask pedestrians [set wx ifelse-value (vx > 0) [100] [-100]] ped-step");
        verifyStep("Saturated preferred velocities at contact");

        setup(true, .02, true);
        add(49.7, 6.5, 1.34, 0, 50, 6.5, 1);
        add(.3, 6.5, -1.34, 0, 0, 6.5, -1);
        for (int i = 0; i < 10; i++) {world.command("ped-step"); verifyStep("Periodic seam");}
        require(number("min-pair-separation") >= DISTANCE - TOL, "Wrapped endpoint separation");

        setup(false, .02, true);
        add(24.74, 6.5, 1.34, 0, 45, 6.5, 1);
        add(25.26, 6.5, 1.34, 0, 45, 6.5, 1);
        world.command("repeat 10 [ped-step]"); verifyStep("Close co-moving pedestrians");
        near(number("[xcor] of first sort pedestrians"), 24.74 + 10 * .02 * 1.34, TOL,
            "Co-moving pair must retain ordinary free motion");
        setup(false, .02, true);
        add(24.74, 6.5, 0, 1.34, 24.74, 10.5, 1);
        add(25.26, 6.5, 0, -1.34, 25.26, 2.5, -1);
        world.command("ped-step"); verifyStep("Safe tangential separation");
        near(number("[xcor] of first sort pedestrians"), 24.74, 1e-12,
            "Safe close pair must not be pushed sideways");
        near(number("[ycor] of first sort pedestrians"), 6.5 + .02 * 1.34, 1e-12,
            "Safe tangential motion must not be slowed");
        System.out.println("PASS: head-on rotations, swept tunnelling, oblique paths, periodic seams and co-moving pairs.");
    }

    private static void chain(boolean stopLine) throws Exception {
        setup(false, .02, true);
        if (!stopLine) world.command("set wall-segments [[25.72 1.5 25.72 11.5]]");
        add(24.48, 6.5, 1.34, 0, 49, 6.5, 1);
        add(25, 6.5, 1.34, 0, 49, 6.5, 1);
        add(25.52, 6.5, 0, 0, 49, 6.5, 1);
        if (stopLine) world.command("ask last sort pedestrians [set stop-x 25.52]");
        for (int i = 0; i < 25; i++) {
            world.command("ped-step"); verifyStep(stopLine ? "Chain at stop line" : "Chain at wall");
        }
        require(number("count pedestrians") == 3, "Contact must not delete blocked pedestrians");
    }

    private static List<double[]> orderedState() throws Exception {
        var states = rows("map [p -> [(list goal-x goal-y xcor ycor wx wy vx vy)] of p] sort pedestrians", 8);
        states.sort(Comparator.<double[]>comparingDouble(x -> x[0]).thenComparingDouble(x -> x[1]));
        return states;
    }

    private static List<double[]> symmetricCross(boolean reverse) throws Exception {
        setup(false, .02, true);
        for (int j = 0; j < 4; j++) {
            int i = reverse ? 3 - j : j;
            double angle = i * Math.PI / 2, c = Math.cos(angle), s = Math.sin(angle);
            add(25 + .4 * c, 6.5 + .4 * s, -1.34 * c, -1.34 * s,
                25 - 4 * c, 6.5 - 4 * s, c > 0 ? -1 : 1);
        }
        for (int i = 0; i < 15; i++) {world.command("ped-step"); verifyStep("Four simultaneous contacts");}
        return orderedState();
    }

    private static void coupledCases() throws Exception {
        chain(false); chain(true);
        List<double[]> forward = symmetricCross(false), reverse = symmetricCross(true);
        for (int i = 0; i < forward.size(); i++) for (int k = 0; k < 8; k++)
            near(forward.get(i)[k], reverse.get(i)[k], 2e-7, "Creation-order independence");

        // The candidate enters its goal disk, but pair contact keeps it outside.
        // Arrival hooks must see final corrected coordinates, not raw candidates.
        setup(false, .04, true);
        add(24.65, 6.5, 1.34, 0, 25, 6.5, 1);
        add(25.17, 6.5, 0, 0, 49, 6.5, 1);
        world.command("ask last sort pedestrians [set stop-x 25.17] ped-step");
        verifyStep("Pair contact before arrival decision");
        require(number("count pedestrians") == 2 && number("arrived-count") == 0,
            "Blocked goal candidate incorrectly recorded as an arrival");
        System.out.println("PASS: coupled wall/stop chains, simultaneous contacts, order independence and corrected arrival decisions.");
    }

    private static String freeMotion(boolean enabled) throws Exception {
        setup(false, .02, enabled);
        // Restore normal paper forces: hard contact must leave this replay exact.
        world.command("set potential-strength 2.1 set wall-strength 50");
        add(10, 3, 0, 0, 49, 3, 1);
        add(40, 10, 0, 0, 1, 10, -1);
        world.command("repeat 100 [ped-step]");
        if (enabled) verifyStep("No-contact normal-force replay");
        return world.report("map [p -> [(list xcor ycor wx wy vx vy ax ay)] of p] sort pedestrians").toString()
            + world.report("n-values 16 [random-float 1]").toString();
    }

    private static double[] unaffectedWalker(boolean enabled) throws Exception {
        setup(false, .02, enabled);
        add(24.74, 6.5, 1.34, 0, 40, 6.5, 1);
        add(25.26, 6.5, -1.34, 0, 10, 6.5, -1);
        add(10, 9, 1.742, 0, 40, 9, 1);
        world.command("ask last sort pedestrians [set wx 100] ped-step");
        if (enabled) verifyStep("Distant walker alongside active contact");
        return rows("(list [(list xcor ycor wx wy vx vy)] of last sort pedestrians)", 6).get(0);
    }

    private static void preservationAndTimesteps() throws Exception {
        require(freeMotion(false).equals(freeMotion(true)),
            "No-contact physics or RNG changed when pair contact was enabled");
        double[] baseline = unaffectedWalker(false), constrained = unaffectedWalker(true);
        for (int i = 0; i < baseline.length; i++)
            near(constrained[i], baseline[i], 1e-12, "Distant free walker changed due to another pair's contact");
        for (double dt : new double[]{.005, .01, .02, .04, .1}) headOn(0, dt, (int) Math.round(1 / dt));
        System.out.println("PASS: exact free-motion/RNG preservation, unaffected distant walker, and dt=.005/.01/.02/.04/.1 contact invariants.");
    }

    public static void main(String[] args) throws Exception {
        world = HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            world.command("set periodic? false set spawn-rate 0 setup");
            require(Boolean.TRUE.equals(world.report("pair-contact-enabled?")), "Pair contact must default on");
            near(number("minimum-separation"), DISTANCE, 0, "Default minimum separation");
            geometricCases(); coupledCases(); preservationAndTimesteps();
            System.out.println("PASS: " + checks + " independent pair-separation assertions.");
        } finally {world.dispose();}
    }
}
