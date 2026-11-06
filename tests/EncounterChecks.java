import java.nio.file.Files;
import java.nio.file.Path;
import org.nlogo.headless.HeadlessWorkspace;

/** Compare the production model with an independently solved saved encounter.
 * This verifies equations and replay, NOT collision-free behavior.
 */
public class EncounterChecks {
    private static HeadlessWorkspace world;

    private static double number(String reporter) throws Exception {
        return ((Number) world.report(reporter)).doubleValue();
    }

    private static void close(String reporter, double expected, double tolerance) throws Exception {
        double actual = number(reporter);
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance) {
            throw new AssertionError(reporter + ": expected " + expected + ", got " + actual);
        }
    }

    private static void restoreFixture(boolean relative) throws Exception {
        var rows = Files.readAllLines(Path.of("tests/fixtures/collision-state.csv"));
        var columns = rows.get(0).split(",");
        world.command("set periodic? true set population " + (rows.size() - 1)
            + " set relative-anticipation? " + relative + " set dt 0.02 set noise-strength 0 "
            + "setup set elapsed-seconds 20.52");
        for (String row : rows.subList(1, rows.size())) {
            var values = row.split(",");
            var command = new StringBuilder("ask pedestrian ")
                .append((int) Double.parseDouble(values[0])).append(" [");
            for (int i = 1; i < columns.length; i++) {
                command.append(" set ").append(columns[i]).append(" ").append(values[i]);
            }
            world.command(command.append(" ]").toString());
        }
    }

    public static void main(String[] args) throws Exception {
        world = HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            restoreFixture(false);
            world.command("foreach sort pedestrians [p -> ask p [calculate-acceleration]]");
            // Independent reconstruction at the same state, including every neighbor.
            close("[ax] of pedestrian 3", -0.1355563924, 1e-9);
            close("[ay] of pedestrian 3", -3.2253388676, 1e-9);
            close("[ax] of pedestrian 52", -1.6853792510, 1e-9);
            close("[ay] of pedestrian 52", 1.2743641967, 1e-9);
            // Independent Python first-order integration; all 150 agents are dynamic.
            world.command("repeat 40 [go]");
            close("[xcor] of pedestrian 3", 41.8776045567435, 1e-8);
            close("[ycor] of pedestrian 3", 5.394641734349123, 1e-8);
            close("[xcor] of pedestrian 52", 42.63779842075603, 1e-8);
            close("[ycor] of pedestrian 52", 5.600796601852353, 1e-8);
            System.out.println("PASS: original-anticipation forces and replay match the independent solver.");

            // Start from the identical complete crowd state, changing only anticipation.
            restoreFixture(true);
            double closest = Double.POSITIVE_INFINITY;
            for (int step = 0; step < 40; step++) {
                world.command("go");
                closest = Math.min(closest, number("[separation-from ([xcor] of pedestrian 52) "
                    + "([ycor] of pedestrian 52)] of pedestrian 3"));
            }
            close("[xcor] of pedestrian 3", 42.12943426876623, 1e-8);
            close("[ycor] of pedestrian 3", 5.4613663939871495, 1e-8);
            close("[xcor] of pedestrian 52", 42.73707397590889, 1e-8);
            close("[ycor] of pedestrian 52", 4.962696288756059, 1e-8);
            if (!Double.isFinite(closest) || closest <= 0.6
                    || Math.abs(closest - 0.6388666176789007) > 1e-8) {
                throw new AssertionError("Relative-anticipation encounter clearance differs from independent reference: "
                    + closest);
            }
            System.out.println("PASS: relative-anticipation replay matches independent solver; "
                + "previously colliding pair stays " + closest + " m apart.");
            System.out.println("LIMITATION: this verifies one encounter, not collision-free behavior in every crowd.");
        } finally {
            world.dispose();
        }
    }
}
