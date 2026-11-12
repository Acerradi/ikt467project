import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import org.nlogo.headless.HeadlessWorkspace;

/** Observational diagnostics: orientation-independent paths, RNG, and real-time sampling. */
public class DiagnosticsChecks {
    private static HeadlessWorkspace world;
    private static int checks;
    private static final String BASE =
        "set simulation-seed 42 set periodic? true set population 20 set dt .02 "
        + "set noise-strength 0 set run-checks? false setup";
    private static final String STATE =
        "(list spawned-count arrived-count returned-count "
        + "(map [p -> [(list who xcor ycor wx wy vx vy desired-speed goal-x goal-y "
        + "ex ey stop-x waypoints birth-seconds started? slow-seconds)] of p] sort pedestrians))";

    private static void open(String model) throws Exception {
        if (world != null) world.dispose();
        world=HeadlessWorkspace.newInstance();
        world.open(Path.of(model).toAbsolutePath().toString());
    }

    private static void require(String expression, String label) throws Exception {
        if (!Boolean.TRUE.equals(world.report(expression))) throw new AssertionError(label);
        checks++;
    }
    private static void close(double actual, double expected, double tolerance, String label) {
        if (!Double.isFinite(actual) || Math.abs(actual - expected) > tolerance)
            throw new AssertionError(label + ": expected " + expected + ", got " + actual);
        checks++;
    }
    private static double number(String expression) throws Exception {
        return ((Number) world.report(expression)).doubleValue();
    }
    private static String rng() throws Exception {
        return world.report("n-values 16 [random-float 1]").toString();
    }
    private static String quoted(Path path) {
        return "\"" + path.toAbsolutePath().toString().replace('\\','/') + "\"";
    }

    private static void pathFixture(String paths, boolean periodic, int events, double distance,
                                    String label) throws Exception {
        world.command("set active-periodic? " + periodic + " set last-step-paths " + paths);
        close(number("pass-through-pairs"), events, 0, label + " event count");
        close(number("swept-min-pair-separation"), distance, 1e-10, label + " swept distance");
    }

    private static void pathChecks() throws Exception {
        world.command(BASE + " ask pedestrians [die]");
        pathFixture("[[0 25 6 25.02 6] [1 25.02 6 25 6]]", false, 1, 0,
            "Horizontal deep pass");
        pathFixture("[[0 25 6 25.01 6] [1 25.02 6 25.01 6]]", false, 1, 0,
            "Closest approach exactly at step end is counted");
        pathFixture("[[0 25.01 6 25.02 6] [1 25.01 6 25 6]]", false, 0, 0,
            "Following step does not double-count boundary encounter");
        pathFixture("[[0 25 6 25 6.02] [1 25 6.02 25 6]]", false, 1, 0,
            "Same encounter rotated ninety degrees");
        pathFixture("[[0 25 6 25.02 6.02] [1 25.02 6.02 25 6]]", false, 1, 0,
            "Diagonal deep pass");
        pathFixture("[[0 25 6 25.02 6] [1 25.02 6.3 25 6.3]]", false, 0, .3,
            "Ordinary lateral overlap is not a deep pass");
        pathFixture("[[0 25 6 25.01 6] [1 25.15 6 25.14 6]]", false, 0, .13,
            "Approaching overlap has no interior closest approach");
        pathFixture("[[0 25 6 25.125 6] [1 25.125 6 25.25 6]]", false, 0, .125,
            "Persistent co-moving overlap");
        pathFixture("[[0 25 6 24.99 6] [1 25.1 6 25.11 6]]", false, 0, .1,
            "Separating overlap");
        pathFixture("[[0 49.99 6 50.01 6] [1 .01 6 -.01 6]]", true, 1, 0,
            "Periodic seam, unwrapped endpoints");
        pathFixture("[[0 24.99 6 25.01 6] [1 0 6 0 6]]", true, 0, 24.99,
            "Half-period image switch does not create a collision");
        pathFixture("[[0 0 6 100 6] [1 25 6 25 6]]", true, 1, 0,
            "Multiple periodic images counted as one passing pair");
        world.command("checks-reset set active-periodic? false "
            + "set last-step-paths [[4 25 6 25.02 6] [9 25.02 6 25 6]] checks-update");
        require("count pedestrians = 0 and pass-through-events = 1 and worst-min-separation = 0",
            "Removed pedestrians must remain in step diagnostics");

        // Real integration: both walkers reach their exit disks and die at commit.
        world.command("set periodic? false set spawn-rate 0 set dt .02 set desired-speed-sd 0 setup "
            + "create-pedestrians 1 [initialize-pedestrian 48.99 6 49 6 1 set wx 1.34 set vx 1.34] "
            + "create-pedestrians 1 [initialize-pedestrian 48.99 6.6 49 6.6 1 set wx 1.34 set vx 1.34] go");
        require("count pedestrians = 0 and length last-step-paths = 2 and arrived-count = 2",
            "Core must capture paths before removing arrivals");
        require("swept-min-pair-separation < 0.61", "Removed pair lost from closest-distance check");
        close(number("last-step-start-seconds"), 0, 0, "Recorded step start");
        close(number("last-step-duration"), .02, 0, "Recorded step duration");
        System.out.println("PASS: rotated passes, persistent overlap, periodic images and departed-agent snapshots.");
    }

    private static void rngChecks(Path log) throws Exception {
        String[] reporters = {"speeds", "speed-ratios", "forward-speeds", "lateral-speeds",
            "stopped-fraction", "arrivals-ever-stopped-fraction", "min-pair-separation",
            "overlap-pair-count", "min-wall-clearance", "wall-penetration-count",
            "speed-cap-violation-count", "above-desired-speed-count", "lane-count",
            "pass-through-pairs", "swept-min-pair-separation"};
        for (String reporter : reporters) {
            world.command(BASE);
            String expected = rng();
            world.command(BASE);
            world.report(reporter);
            if (!expected.equals(rng())) throw new AssertionError(reporter + " consumed simulation RNG");
            checks++;
        }
        // Exercise equal-y tie handling explicitly.
        world.command(BASE + " ask pedestrians [set ycor 6]"); String expected = rng();
        world.command(BASE + " ask pedestrians [set ycor 6]"); world.report("lane-count");
        if (!expected.equals(rng())) throw new AssertionError("Lane ties consumed RNG");
        checks++;
        for (String command : new String[]{"checks-update", "check-invariants", "update-plots",
                "log-start " + quoted(log) + " log-row log-row log-stop"}) {
            world.command(BASE); expected = rng();
            world.command(BASE); world.command(command);
            if (!expected.equals(rng())) throw new AssertionError(command + " consumed simulation RNG");
            checks++;
        }
        System.out.println("PASS: diagnostic reporters, tied lanes, checks, plots and CSV preserve RNG.");
    }

    private static String runOpen(boolean withChecks, boolean withLog, Path log) throws Exception {
        world.command("set periodic? false set spawn-rate 5 set simulation-seed 42 set dt .02 "
            + "set noise-strength 0 set run-checks? " + withChecks + " setup");
        if (withLog) world.command("log-start " + quoted(log));
        world.command("repeat 500 [go log-row]");
        String state = world.report(STATE).toString();
        String futureRng = rng();
        world.command("log-stop");
        return state + futureRng;
    }

    private static void wholeRunChecks(Path log) throws Exception {
        String baseline = runOpen(false, false, log);
        for (boolean[] switches : new boolean[][]{{true,false},{false,true},{true,true}}) {
            String actual = runOpen(switches[0],switches[1],log);
            if (!baseline.equals(actual))
                throw new AssertionError("Diagnostics changed physical run or future RNG: checks="
                    + switches[0] + ", logging=" + switches[1]);
            checks++;
        }
        System.out.println("PASS: seeded open arrivals, full physical states and future RNG match with checks/logging toggled.");
    }

    private static void samplingChecks(Path log) throws Exception {
        world.command("set periodic? true set population 0 set dt .03 set run-checks? false setup "
            + "log-start " + quoted(log) + " log-row log-row");
        var samples = new ArrayList<Double>();
        for (int i=0;i<100;i++) {
            world.command("go");
            boolean sampled = Boolean.TRUE.equals(world.report("stats-sample?"));
            require("stats-sample? = stats-sample?", "Repeated sampling query must be idempotent");
            if (sampled) samples.add(number("elapsed-seconds"));
            world.command("log-row log-row");
        }
        if (samples.size()!=30) throw new AssertionError("Expected 30 sampling boundaries, got " + samples.size());
        checks++;
        close(samples.get(0),.12,1e-10,"First sample after .1 s");
        close(samples.get(1),.21,1e-10,"Second sample after .2 s");
        close(samples.get(2),.30,1e-10,"Third sample at .3 s");
        for (int i=0;i<samples.size();i++) {
            double lateness=samples.get(i)-.1*(i+1);
            if (lateness < -1e-10 || lateness >= .03+1e-10)
                throw new AssertionError("Sampling drifted: target="+.1*(i+1)+", actual="+samples.get(i));
        }
        world.command("log-stop");
        var lines=Files.readAllLines(log);
        if (lines.size()!=32) throw new AssertionError("Duplicate/missing rows: expected header + t0 + 30 samples; got " + lines.size());
        checks++;
        close(Double.parseDouble(lines.get(2).split(",")[0]),.12,1e-10,"CSV uses actual sample time");
        // Changing dt after a completed step must not change its sample decision.
        require("stats-sample?", "Final t=3 sample missing");
        world.command("set dt .017");
        require("stats-sample?", "Changing next timestep altered the previous sample decision");
        System.out.println("PASS: dt=.03 sampling uses .12,.21,.30,..., no cumulative drift or duplicate CSV rows.");
    }

    private static String scenarioSetup(String name, boolean withChecks) {
        String common="set simulation-seed 42 set dt .03 set noise-strength 0 "
            + "set relative-anticipation? true set run-checks? " + withChecks + " set log-csv? false ";
        if (name.equals("Door")) return common
            + "set door-width 1.4 set room-width 6 set initial-per-side 2 set two-sided? true "
            + "set spawn-rate 2 setup";
        if (name.equals("TrafficLight")) return common
            + "set initial-per-side 2 set spawn-rate 2 set start-green? false set switch-time .07 setup update-light";
        return common + "set horizontal-rate 1 set vertical-rate 1 set corridor-width 4 set two-way? true "
            + "setup repeat 3 [spawn-walker false spawn-walker true]";
    }

    private static String scenarioState(String name) throws Exception {
        String extra=name.equals("Door") ? "(list door-sequence emptied-seconds)"
            : name.equals("TrafficLight") ? "(list light-green? light-timer light-switches max-stop-overshoot)"
            : "(list horizontal-results vertical-results)";
        return world.report(STATE).toString()+world.report(extra).toString();
    }

    private static void scenarioReporterChecks(String name) throws Exception {
        String[] reporters;
        if (name.equals("Door")) reporters=new String[]{"door-passes", "passes-in-direction 1",
            "alternation-fraction", "door-flow", "waiting-at-door"};
        else if (name.equals("TrafficLight")) reporters=new String[]{"light-colour", "waiting-count",
            "mean-waiting-gap", "held-on-road-count", "[gap-to-stop-line] of first sort pedestrians"};
        else reporters=new String[]{"speeds-of \"horizontal\"", "speeds-of \"vertical\"",
            "travel-times-of \"horizontal\"", "delay-ratios-of \"vertical\"",
            "stopped-fraction-of \"vertical\"", "walkers-of \"horizontal\"", "in-crossing-count"};
        String setup=scenarioSetup(name,false);
        for (String reporter : reporters) {
            world.command(setup);String expected=rng();
            world.command(setup);String before=scenarioState(name);world.report(reporter);
            if (!before.equals(scenarioState(name))) throw new AssertionError(name+" reporter changed state: "+reporter);
            if (!expected.equals(rng())) throw new AssertionError(name+" reporter consumed RNG: "+reporter);
            checks++;
        }
        String[] commands=name.equals("TrafficLight")
            ? new String[]{"draw-light", "track-stop-lines", "update-plots"}
            : new String[]{"update-plots"};
        for (String command : commands) {
            world.command(setup);String expected=rng();
            world.command(setup);String before=scenarioState(name);world.command(command);
            if (!before.equals(scenarioState(name))) throw new AssertionError(name+" bookkeeping changed state: "+command);
            if (!expected.equals(rng())) throw new AssertionError(name+" bookkeeping consumed RNG: "+command);
            checks++;
        }
    }

    private static String runScenario(String name, boolean withChecks, boolean withLog, Path log) throws Exception {
        world.command(scenarioSetup(name,withChecks));
        if (withLog) world.command("log-start "+quoted(log));
        world.command("repeat 200 [go]");
        String result=scenarioState(name)+rng();
        world.command("log-stop");
        return result;
    }

    private static void scenarioObserverChecks(Path log) throws Exception {
        for (String name : new String[]{"Door", "TrafficLight", "Crossing"}) {
            open("scenarios/"+name+".nlogox");
            scenarioReporterChecks(name);
            String baseline=runScenario(name,false,false,log);
            for (boolean[] switches : new boolean[][]{{true,false},{false,true},{true,true}}) {
                String actual=runScenario(name,switches[0],switches[1],log);
                if (!baseline.equals(actual)) throw new AssertionError(name
                    +" physics, scenario records or future RNG changed with checks="+switches[0]+", logging="+switches[1]);
                checks++;
            }
            System.out.println("PASS: "+name+" reporters, plotting/bookkeeping and six-second checks/logging variants preserve state and RNG.");
        }
    }

    public static void main(String[] args) throws Exception {
        Path log=Path.of("work/diagnostics-checks.csv").toAbsolutePath();
        Files.createDirectories(log.getParent());
        try {
            open("Pedestrians.nlogox");
            pathChecks();
            rngChecks(log);
            wholeRunChecks(log);
            samplingChecks(log);
            scenarioObserverChecks(log);
            System.out.println("PASS: " + checks + " diagnostics checks.");
        } finally {world.dispose();}
    }
}
