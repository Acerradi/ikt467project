import java.nio.file.Path;
import org.nlogo.headless.HeadlessWorkspace;

/** Independent mathematical checks, not just boundary or population invariants. */
public class MathChecks {
    private static HeadlessWorkspace world;
    private static int checks;

    private static double number(String expression) throws Exception {
        return ((Number) world.report(expression)).doubleValue();
    }

    private static void close(double actual, double expected, double tolerance, String label) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance) {
            throw new AssertionError(label + ": actual=" + actual + ", expected=" + expected);
        }
        checks++;
    }

    private static String args(double... values) {
        StringBuilder out = new StringBuilder();
        for (double value : values) out.append(" (").append(value).append(")");
        return out.toString();
    }

    // Evaluate the original, unsoftened scalar potential independently in Java.
    private static double potential(double qx, double qy, double sx, double sy) {
        double sum = Math.hypot(qx, qy) + Math.hypot(qx - sx, qy - sy);
        double b = 0.5 * Math.sqrt(sum * sum - sx * sx - sy * sy);
        return 2.1 * Math.exp(-b / 0.3);
    }

    private static void gradientChecks() throws Exception {
        double[][] geometries = {
            {0.3, 0.4, 0, 0}, {-0.8, 0.4, 0, 0},
            {1.2, 0.6, 2.68, 0}, {3.3, 0.2, 2.68, 0},
            {-0.4, 0.3, 2.68, 0}, {0.4, -0.3, -2.68, 0},
            {0.6, 1.1, 0.9, 1.7}, {-0.2, -0.5, -1.2, -0.7},
            {1.34, 0.01, 2.68, 0}, {0.005, 0.006, 2.68, 0}
        };
        double h = 1e-6;
        for (double[] g : geometries) {
            double dx = -(potential(g[0] + h, g[1], g[2], g[3])
                        - potential(g[0] - h, g[1], g[2], g[3])) / (2 * h);
            double dy = -(potential(g[0], g[1] + h, g[2], g[3])
                        - potential(g[0], g[1] - h, g[2], g[3])) / (2 * h);
            String expression = "(elliptical-force" + args(g) + ")";
            close(number("item 0 " + expression), dx, 2e-5 * Math.max(1, Math.abs(dx)),
                "Ellipse negative-gradient x at " + args(g));
            close(number("item 1 " + expression), dy, 2e-5 * Math.max(1, Math.abs(dy)),
                "Ellipse negative-gradient y at " + args(g));
        }
        double magnitude = 7 * Math.exp(-0.5 / 0.3);
        close(number("item 0 (elliptical-force 0.3 0.4 0 0)"), magnitude * 0.6, 1e-9,
            "Stationary circular limit x");
        close(number("item 1 (elliptical-force 0.3 0.4 0 0)"), magnitude * 0.8, 1e-9,
            "Stationary circular limit y");
    }

    private static void angularChecks() throws Exception {
        world.command("set relative-anticipation? false set population 2 setup "
            + "set noise-strength 0 set wall-strength 0 "
            + "ask pedestrian 0 [setxy 10 6.5 set desired-speed 1 set ex 1 set ey 0 set vx 1 set vy 0] "
            + "ask pedestrian 1 [setxy 11 6.5 set vx 0 set vy 0]");
        double raw = number("[item 0 (pair-force pedestrian 1)] of pedestrian 0");
        world.command("ask pedestrian 0 [calculate-acceleration]");
        close(number("[ax] of pedestrian 0"), raw, 1e-12, "Front receives full repulsion");
        world.command("ask pedestrian 1 [setxy 9 6.5]");
        raw = number("[item 0 (pair-force pedestrian 1)] of pedestrian 0");
        world.command("ask pedestrian 0 [calculate-acceleration]");
        close(number("[ax] of pedestrian 0"), 0.5 * raw, 1e-12,
            "Rear receives half repulsion, using negative-force direction");
        world.command("ask pedestrian 1 [setxy 10 7.5]");
        raw = number("[item 1 (pair-force pedestrian 1)] of pedestrian 0");
        world.command("ask pedestrian 0 [calculate-acceleration]");
        close(number("[ay] of pedestrian 0"), raw, 1e-12, "90-degree side is within 100-degree half angle");
    }

    private static double wallPotential(double x, double y) {
        double nearX = Math.max(0, Math.min(50, x));
        return 10 * Math.exp(-Math.hypot(x - nearX, y - 1.5) / 0.2);
    }

    private static void wallChecks() throws Exception {
        world.command("set wall-strength 50 set wall-range 0.2");
        double[][] positions = {{10, 2}, {10, 1}, {-0.3, 1.8}};
        double h = 1e-6;
        for (double[] p : positions) {
            world.command("ask pedestrian 0 [setxy" + args(p) + "]");
            double fx = -(wallPotential(p[0] + h, p[1]) - wallPotential(p[0] - h, p[1])) / (2 * h);
            double fy = -(wallPotential(p[0], p[1] + h) - wallPotential(p[0], p[1] - h)) / (2 * h);
            close(number("[item 0 (wall-force [0 1.5 50 1.5])] of pedestrian 0"), fx, 1e-8,
                "Finite wall potential gradient x");
            close(number("[item 1 (wall-force [0 1.5 50 1.5])] of pedestrian 0"), fy, 1e-8,
                "Finite wall potential gradient y");
        }
    }

    private static void isolatedMotionChecks() throws Exception {
        world.command("set periodic? true set population 2 setup ask pedestrian 1 [die] "
            + "set potential-strength 0 set wall-strength 0 set noise-strength 0 set dt 0.02 "
            + "ask pedestrian 0 [setxy 10 6.5 set desired-speed 1.34 set travel-direction 1 "
            + "set wx 0 set wy 0 set vx 0 set vy 0] repeat 100 [go]");
        double r = 1 - 0.02 / 0.5;
        double speed = 1.34 * (1 - Math.pow(r, 100));
        double x = 10 + 0.02 * 1.34 * (100 - r * (1 - Math.pow(r, 100)) / (1 - r));
        close(number("[vx] of pedestrian 0"), speed, 1e-12, "Discrete relaxation solution");
        close(number("[wx] of pedestrian 0"), speed, 1e-12, "Uncapped actual/preferred agreement");
        close(number("[xcor] of pedestrian 0"), x, 1e-11, "Position uses new velocity");
        close(number("[ycor] of pedestrian 0"), 6.5, 1e-12, "No spurious lateral motion");
    }

    private static void singularityDiagnostics() throws Exception {
        world.command("set potential-strength 2.1");
        System.out.println("DIAGNOSTIC: b=0 along the other pedestrian's anticipated path. "
            + "The original potential is not differentiable there; smoothing selects zero on its exact midpoint.");
        for (double y : new double[]{0, 1e-6, 1e-4, 0.01}) {
            System.out.println("  q=(1.34," + y + "), s=(2.68,0): "
                + world.report("elliptical-force" + args(1.34, y, 2.68, 0)));
        }
        System.out.println("  At anticipated focus q=s=(2.68,0): "
            + world.report("elliptical-force 2.68 0 2.68 0"));
    }

    private static void wallCapConsistencyCheck() throws Exception {
        world.command("set periodic? true set population 2 setup set dt 0.02 "
            + "ask pedestrian 0 [set desired-speed 1.34 setxy 25 (hall-top - body-radius - 0.001) "
            + "set wx 10 set wy 10 set ax 0 set ay 0 calculate-next-state]");
        double wx = number("[next-wx] of pedestrian 0");
        double wy = number("[next-wy] of pedestrian 0");
        double factor = Math.min(1, 1.3 * 1.34 / Math.hypot(wx, wy));
        close(number("[next-vx] of pedestrian 0"), wx * factor, 1e-12,
            "Wall projection must preserve v=cap(w), including tangential component");
        close(number("[next-vy] of pedestrian 0"), wy * factor, 1e-12,
            "Wall projection must preserve v=cap(w), normal component");
        close(number("[next-x] of pedestrian 0"), 25 + 0.02 * wx * factor, 1e-12,
            "Wall-contact position must use corrected capped velocity");
        close(number("[next-y] of pedestrian 0"), number("hall-top - body-radius"), 1e-12,
            "Wall-contact position remains at clearance boundary");
        close(wy, 0, 1e-12, "Wall contact removes preferred normal velocity");
    }

    public static void main(String[] args) throws Exception {
        world = HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            world.command("set periodic? true set population 2 setup");
            gradientChecks();
            angularChecks();
            wallChecks();
            isolatedMotionChecks();
            wallCapConsistencyCheck();
            singularityDiagnostics();
            System.out.println("PASS: " + checks + " independent checks of elliptical gradients, stationary limit, "
                + "perception weights, wall potential gradient and discrete isolated motion.");
        } finally {
            world.dispose();
        }
    }
}
