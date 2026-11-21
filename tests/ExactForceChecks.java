import java.nio.file.Path;
import java.util.Arrays;
import java.util.Locale;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Exact regression and warmed microbenchmark for cached force snapshots.
 * No force cutoff or numeric tolerance is allowed by these comparisons.
 * Optional: --baseline path/to/Pedestrians.nlogox for whole-step replays.
 */
public class ExactForceChecks {
    static long checks;
    static final String ACCEL="map [p -> [(list ax ay)] of p] sort pedestrians";
    static final String BATCH="calculate-all-accelerations sort pedestrians";
    // The pre-optimization directed loop, kept independent of snapshot helpers.
    static final String REFERENCE="foreach sort pedestrians [p -> ask p ["
        +"set ax (desired-speed * drive-factor * ex - vx) / tau "
        +"set ay (desired-speed * drive-factor * ey - vy) / tau "
        +"foreach sort other pedestrians [q -> let force pair-force q "
        +"let fx item 0 force let fy item 1 force let magnitude vector-length fx fy let weight rear-weight "
        +"if ex * (- fx) + ey * (- fy) >= magnitude * cos view-half-angle [set weight 1] "
        +"set ax ax + weight * fx set ay ay + weight * fy] "
        +"foreach wall-segments [wall -> let force wall-force wall set ax ax + item 0 force set ay ay + item 1 force] "
        +"set ax ax + noise-strength * noise-unit-x set ay ay + noise-strength * noise-unit-y]]";
    static final String STATE="(list elapsed-seconds ticks spawned-count arrived-count returned-count wrap-count "
        +"travel-times arrival-slow-seconds last-step-paths "
        +"(map [p -> [(list who xcor ycor wx wy vx vy ax ay desired-speed ex ey noise-unit-x noise-unit-y "
        +"next-x next-y next-vx next-vy next-wx next-wy reached-goal? returned? waypoints stop-x "
        +"birth-seconds started? slow-seconds)] of p] sort pedestrians))";
    static void same(Object a,Object b,String label) {
        if(a instanceof Number x && b instanceof Number y) {
            if(Double.doubleToLongBits(x.doubleValue())!=Double.doubleToLongBits(y.doubleValue()))
                throw new AssertionError(label+": "+x+" != "+y);
            checks++;return;
        }
        if(a instanceof LogoList x && b instanceof LogoList y) {
            if(x.size()!=y.size())throw new AssertionError(label+" list size");
            for(int i=0;i<x.size();i++)same(x.apply(i),y.apply(i),label+"["+i+"]");
            return;
        }
        if(!a.equals(b))throw new AssertionError(label+": "+a+" != "+b);
        checks++;
    }
    static String settings(int population,boolean periodic,boolean relative) {
        return "set simulation-seed 42 set dt .02 set noise-strength .05 set show-trails? false "
            +"set run-checks? false set population "+population+" set periodic? "+periodic
            +" set relative-anticipation? "+relative+" set spawn-rate 1.8 setup";
    }
    static void parity(HeadlessWorkspace w,boolean periodic,boolean relative)throws Exception {
        w.command(settings(150,true,relative)+" set active-periodic? "+periodic
            +" ask pedestrians [set vx random-float 3 - 1.5 set vy random-float 2 - 1 set wx vx set wy vy] "
            +"ask pedestrian 0 [setxy .01 5] ask pedestrian 1 [setxy 49.99 5] "
            +"foreach sort pedestrians [p -> ask p [update-desired-direction]]");
        w.command(REFERENCE);Object expected=w.report(ACCEL);
        w.command(BATCH);same(expected,w.report(ACCEL),"force periodic="+periodic+" relative="+relative);
        w.command("foreach sort pedestrians [p -> ask p [calculate-acceleration]]");
        same(expected,w.report(ACCEL),"direct-call snapshot API");
        w.command("random-seed 19 "+REFERENCE);Object rng=w.report("n-values 16 [random-float 1]");
        w.command("random-seed 19 "+BATCH);same(rng,w.report("n-values 16 [random-float 1]"),"RNG");
        // Manual edits between calls must be visible; no stale global cache.
        w.command("ask pedestrian 2 [set vx .92 set xcor 20 set desired-speed .83] set view-half-angle 87 set anticipation-time 1.7");
        w.command(REFERENCE);expected=w.report(ACCEL);
        w.command(BATCH);same(expected,w.report(ACCEL),"manual updates");
        System.out.println("PASS: bit-identical accelerations and future RNG, periodic="+periodic+", relative="+relative);
    }
    static double timed(HeadlessWorkspace w,String command,int repeats)throws Exception {
        long start=System.nanoTime();w.command("repeat "+repeats+" ["+command+"]");
        return (System.nanoTime()-start)/1e9;
    }
    static void benchmark(HeadlessWorkspace w)throws Exception {
        w.command(settings(150,true,true));
        timed(w,REFERENCE,30);timed(w,BATCH,30);
        double[] old=new double[3],now=new double[3];
        for(int i=0;i<3;i++) {
            if(i%2==0){old[i]=timed(w,REFERENCE,100);now[i]=timed(w,BATCH,100);}
            else{now[i]=timed(w,BATCH,100);old[i]=timed(w,REFERENCE,100);}
        }
        Arrays.sort(old);Arrays.sort(now);
        System.out.printf(Locale.ROOT,"BENCH: 150 people,100 complete force passes, median original %.6f s, snapshot %.6f s, %.3fx.%n",old[1],now[1],old[1]/now[1]);
    }
    static void replay(HeadlessWorkspace current,String path)throws Exception {
        var baseline=HeadlessWorkspace.newInstance();
        try {
            baseline.open(Path.of(path).toAbsolutePath().toString());
            for(boolean periodic:new boolean[]{false,true})for(boolean relative:new boolean[]{false,true}) {
                String setup=settings(40,periodic,relative)+" set pair-contact-enabled? false";
                baseline.command(setup);current.command(setup);
                same(baseline.report(STATE),current.report(STATE),"initial state");
                for(int step=0;step<100;step++) {
                    baseline.command("go");current.command("go");
                    same(baseline.report(STATE),current.report(STATE),"whole-step replay "+step);
                }
                same(baseline.report("n-values 16 [random-float 1]"),current.report("n-values 16 [random-float 1]"),"replay future RNG");
                System.out.println("PASS: exact 100-step baseline replay, periodic="+periodic+", relative="+relative);
            }
        }finally{baseline.dispose();}
    }
    public static void main(String[] args)throws Exception {
        var w=HeadlessWorkspace.newInstance();
        try {
            w.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            for(boolean periodic:new boolean[]{false,true})for(boolean relative:new boolean[]{false,true})parity(w,periodic,relative);
            for(int i=0;i<args.length-1;i++)if(args[i].equals("--baseline"))replay(w,args[i+1]);
            benchmark(w);
            System.out.println("PASS: "+checks+" exact force optimization assertions.");
        }finally{w.dispose();}
    }
}
