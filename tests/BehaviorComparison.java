import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Locale;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Actual NetLogo behavior comparison; run one arm per process for parallel work.
 * Usage: BehaviorComparison.java SEED original|relative [SECONDS=60]
 * Without arguments runs seeds 42,1,7 for both modes sequentially.
 * Bounds/caps/counts are checked every step; behavior is sampled every 0.1 s.
 * Distances are centre-to-centre and use the periodic minimum image.
 */
public class BehaviorComparison {
    private static final String INVARIANTS =
        "if count pedestrians != 150 [error \"Population changed\"] "
        + "if abs (pedestrian-density - 0.3) > 1e-9 [error \"Density changed\"] "
        + "foreach sort pedestrians [p -> ask p ["
        + "if xcor < 0 or xcor >= hall-length [error \"Periodic position outside world\"] "
        + "if ycor < hall-bottom + body-radius - 1e-9 or ycor > hall-top - body-radius + 1e-9 "
        + "[error \"Wall clearance violated\"] "
        + "let preferred-norm vector-length wx wy "
        + "let factor ifelse-value (preferred-norm > 0) "
        + "[min (list 1 (1.3 * desired-speed / preferred-norm))] [1] "
        + "if abs (vx - factor * wx) > 1e-8 or abs (vy - factor * wy) > 1e-8 "
        + "[error \"Actual velocity differs from capped preferred velocity\"] "
        + "if vector-length vx vy > 1.3 * desired-speed + 1e-9 "
        + "[error \"Individual speed cap exceeded\"] "
        + "]] ";

    private static double value(LogoList row, int column) {
        return ((Number)row.apply(column)).doubleValue();
    }

    private static void runArm(int seed, boolean relative, double duration) throws Exception {
        double dt = 0.02;
        int steps = (int)Math.round(duration / dt);
        if (duration <= 10 || Math.abs(steps * dt - duration) > 1e-9) {
            throw new IllegalArgumentException("Seconds must exceed 10 and be a multiple of 0.02.");
        }
        var world = HeadlessWorkspace.newInstance();
        String mode = relative ? "relative" : "original";
        String tag = "seed" + seed + "-" + mode + "-" + String.format(Locale.ROOT, "%.2fs", duration);
        long start = System.nanoTime();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            world.command("set periodic? true set relative-anticipation? " + relative
                + " set population 150 set simulation-seed " + seed
                + " set dt 0.02 set noise-strength 0 set mean-desired-speed 1.34 "
                + "set desired-speed-sd 0.26 setup");
            if (!Boolean.valueOf(relative).equals(world.report("active-relative-anticipation?"))) {
                throw new AssertionError("Requested anticipation mode was not captured by setup");
            }
            if (!Boolean.TRUE.equals(world.report("hall-length = 50 and hall-top - hall-bottom = 10 "
                    + "and anticipation-time = 2"))) {
                throw new AssertionError("Geometry or anticipation horizon changed");
            }
            String fixedSpeeds = world.report("map [p -> [desired-speed] of p] sort pedestrians").toString();
            double minSeparation = Double.POSITIVE_INFINITY;
            double minSteadySeparation = Double.POSITIVE_INFINITY;
            long overlapPairSamples = 0, nearPairSamples = 0, frames = 0;
            long steadyOverlapPairSamples = 0, steadyNearPairSamples = 0, steadyFrames = 0;
            int peakOverlapPairs = 0;
            double forwardSum = 0, absLateralSum = 0, actualSpeedSum = 0;
            long steadyAgentSamples = 0, lateralDominantSamples = 0, backwardsSamples = 0;
            double maxPreferred = 0, maxAcceleration = 0;
            for (int step = 0; step < steps;) {
                int batch = Math.min(5, steps - step);
                world.command("repeat " + batch + " [go " + INVARIANTS + "]");
                step += batch;
                boolean steady = step * dt > 10 + 1e-9;
                var rows = (LogoList) world.report("map [p -> [(list xcor ycor vx vy wx wy ax ay travel-direction)] of p] sort pedestrians");
                double[][] data = new double[rows.size()][9];
                for (int i = 0; i < rows.size(); i++) {
                    var row = (LogoList)rows.apply(i);
                    for (int c = 0; c < 9; c++) data[i][c] = value(row, c);
                    double[] r = data[i];
                    maxPreferred = Math.max(maxPreferred, Math.hypot(r[4], r[5]));
                    maxAcceleration = Math.max(maxAcceleration, Math.hypot(r[6], r[7]));
                    if (steady) {
                        forwardSum += r[2] * r[8];
                        absLateralSum += Math.abs(r[3]);
                        actualSpeedSum += Math.hypot(r[2], r[3]);
                        steadyAgentSamples++;
                        if (Math.abs(r[3]) > Math.abs(r[2])) lateralDominantSamples++;
                        if (r[2] * r[8] < -0.1) backwardsSamples++;
                    }
                }
                int overlaps = 0, near = 0;
                double frameMinimum = Double.POSITIVE_INFINITY;
                for (int i = 0; i < data.length; i++) {
                    for (int j = i + 1; j < data.length; j++) {
                        double dx = data[i][0] - data[j][0];
                        dx -= 50 * Math.floor((dx + 25) / 50);
                        double distance = Math.hypot(dx, data[i][1] - data[j][1]);
                        frameMinimum = Math.min(frameMinimum, distance);
                        if (distance < 0.4) overlaps++;
                        if (distance < 0.1) near++;
                    }
                }
                minSeparation = Math.min(minSeparation, frameMinimum);
                overlapPairSamples += overlaps;
                nearPairSamples += near;
                peakOverlapPairs = Math.max(peakOverlapPairs, overlaps);
                frames++;
                if (steady) {
                    minSteadySeparation = Math.min(minSteadySeparation, frameMinimum);
                    steadyOverlapPairSamples += overlaps;
                    steadyNearPairSamples += near;
                    steadyFrames++;
                }
            }
            double elapsed = ((Number)world.report("elapsed-seconds")).doubleValue();
            if (Math.abs(elapsed - duration) > 1e-7) throw new AssertionError("Run did not finish");
            if (!fixedSpeeds.equals(world.report("map [p -> [desired-speed] of p] sort pedestrians").toString())) {
                throw new AssertionError("Individual desired speeds changed");
            }
            String header = "seed,mode,duration_s,dt_s,metric_sample_interval_s,warmup_s,sampled_min_separation_m,"
                + "mean_overlap_pairs,peak_overlap_pairs,near_pair_samples_lt_0_1m,"
                + "steady_sampled_min_separation_m,steady_mean_overlap_pairs,steady_near_pair_samples_lt_0_1m,"
                + "steady_mean_forward_speed_m_s,steady_mean_abs_lateral_speed_m_s,steady_mean_speed_m_s,"
                + "steady_lateral_dominant_fraction,steady_backwards_fraction,"
                + "sampled_max_preferred_speed_m_s,sampled_max_acceleration_m_s2,frames,steady_frames,"
                + "elapsed_simulation_s,wall_clock_s,invariants";
            String values = String.format(Locale.ROOT,
                "%d,%s,%.8f,%.8f,%.8f,10,%.12g,%.12g,%d,%d,%.12g,%.12g,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%d,%d,%.12g,%.6f,PASS",
                seed, mode, duration, dt, 0.1, minSeparation,
                (double)overlapPairSamples / frames, peakOverlapPairs, nearPairSamples,
                minSteadySeparation, (double)steadyOverlapPairSamples / steadyFrames, steadyNearPairSamples,
                forwardSum / steadyAgentSamples, absLateralSum / steadyAgentSamples, actualSpeedSum / steadyAgentSamples,
                (double)lateralDominantSamples / steadyAgentSamples, (double)backwardsSamples / steadyAgentSamples,
                maxPreferred, maxAcceleration, frames, steadyFrames, elapsed, (System.nanoTime() - start) / 1e9);
            Files.createDirectories(Path.of("work"));
            Path output = Path.of("work", "behavior-comparison-" + tag + ".csv");
            Files.writeString(output, header + System.lineSeparator() + values + System.lineSeparator());
            if (relative && seed == 42 && Math.abs(duration - 60) < 1e-9) {
                world.command("export-view \"work/relative-verified.png\"");
            }
            System.out.println(header);
            System.out.println(values);
            System.out.println("PASS: per-step population/density, position bounds and v=cap(w); fixed desired speeds. "
                + "Behavior metrics sampled every 0.1 s; warmup 10 s. Saved " + output);
        } finally {
            world.dispose();
        }
    }

    public static void main(String[] args) throws Exception {
        if (args.length == 0) {
            for (int seed : new int[]{42, 1, 7}) {
                runArm(seed, false, 60);
                runArm(seed, true, 60);
            }
            return;
        }
        if (args.length < 2 || args.length > 3
                || !(args[1].equals("original") || args[1].equals("relative"))) {
            throw new IllegalArgumentException("Usage: BehaviorComparison.java SEED original|relative [SECONDS=60]");
        }
        runArm(Integer.parseInt(args[0]), args[1].equals("relative"),
            args.length == 3 ? Double.parseDouble(args[2]) : 60);
    }
}
