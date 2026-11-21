import java.nio.file.*;
import java.io.*;
import java.util.*;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Matched local fixed-step refinement of an observed unconstrained crossing.
 * This historical experiment explicitly uses radius .25 m, independently of
 * the current scenario default. Imported worlds must have that same radius.
 * Replays the seed to a pre-overlap state, exports one world/RNG state, then
 * imports that exact state for every dt. Arrivals are disabled in every branch.
 * No distances are corrected and no step is rejected. This is a local numerical
 * comparison, not a new long-run Poisson scenario or a proof of convergence.
 * From the project root, using NetLogo's Java runtime and jar:
 * java -Djava.awt.headless=true -cp <netlogo-7.0.4.jar> tests/AVMFreeReplay.java 42 55.0797184477 work/avm-no-limit/validation/refinement
 * This exports the pre-overlap world and runs all four fixed timesteps.
 * To reuse that exact world without replaying its earlier arrivals, append:
 * --world work/avm-no-limit/validation/refinement/pre-overlap-world.csv
 */
public class AVMFreeReplay {
    static final double HISTORICAL_RADIUS=.25;
    static double n(LogoList a,int i){return ((Number)a.apply(i)).doubleValue();}
    static double number(HeadlessWorkspace w,String s)throws Exception{return ((Number)w.report(s)).doubleValue();}
    static void historicalRadius(HeadlessWorkspace w)throws Exception{if(number(w,"body-radius")!=HISTORICAL_RADIUS)throw new AssertionError("Historical replay requires body-radius .25 m");}
    static String quote(Path p){return "\""+p.toAbsolutePath().toString().replace('\\','/')+"\"";}
    static String state(HeadlessWorkspace w)throws Exception{return w.report("(list elapsed-seconds (map [p -> [(list who xcor ycor vx vy move-ex move-ey ex ey desired-speed goal-x goal-y waypoints)] of p] sort pedestrians))").toString();}
    static double[] closest(LogoList a,LogoList b){
        double x=n(a,1)-n(b,1),y=n(a,2)-n(b,2),dx=n(a,3)-n(a,1)-n(b,3)+n(b,1),dy=n(a,4)-n(a,2)-n(b,4)+n(b,2);
        double den=dx*dx+dy*dy,t=den==0?0:Math.max(0,Math.min(1,-(x*dx+y*dy)/den));
        return new double[]{Math.hypot(x+t*dx,y+t*dy),t};
    }
    public static void main(String[] args)throws Exception{
        Locale.setDefault(Locale.ROOT);
        int seed=Integer.parseInt(args.length>0?args[0]:"42");
        double observed=Double.parseDouble(args.length>1?args[1]:"55.0797184477");
        Path root=Path.of(".").toAbsolutePath(),out=Path.of(args.length>2?args[2]:"work/avm-no-limit/validation/refinement").toAbsolutePath();
        Files.createDirectories(out);Path world=out.resolve("pre-overlap-world.csv");
        int baseTicks=Math.max(0,(int)Math.floor((observed+1e-10)/.02)-1);
        HeadlessWorkspace w=HeadlessWorkspace.newInstance();
        try{
            w.open(root.resolve("scenarios/Crossing.nlogox").toString());
            if(args.length>4&&args[3].equals("--world")) {
                w.command("import-world "+quote(Path.of(args[4])));
            } else {
                w.command("set simulation-seed "+seed+" set dt .02 set mean-desired-speed 1.55 set desired-speed-sd .18 set noise-strength 0 set show-trails? false set run-checks? false set log-csv? false set horizontal-rate 1.8 set vertical-rate 1.8 set two-way? false set corridor-width 4 setup set body-radius "+HISTORICAL_RADIUS);
                w.command("repeat "+baseTicks+" [go]");
            }
            historicalRadius(w);
            double base=number(w,"elapsed-seconds");String initial=state(w);
            w.command("export-world "+quote(world));
            Files.writeString(out.resolve("initial-state.txt"),initial);
            System.out.printf(Locale.ROOT,"Exported matched pre-overlap state at %.12f s.%n",base);
            try(PrintWriter summary=new PrintWriter(Files.newBufferedWriter(out.resolve("summary.csv")))){
                summary.println("seed,base_time_s,dt_s,duration_s,min_distance_m,max_overlap_depth_m,first_overlap_s,overlap_intervals,pair_a,pair_b,rejected_steps,arrivals_frozen");
                String baselineRng=null;
                for(double dt:new double[]{.02,.01,.005,.0025}){
                    w.command("import-world "+quote(world));
                    historicalRadius(w);
                    if(!state(w).equals(initial))throw new AssertionError("Imported physical state differs");
                    String rng=w.report("n-values 32 [random-float 1]").toString();
                    if(baselineRng==null){baselineRng=rng;Files.writeString(out.resolve("initial-rng-probe.txt"),rng);}
                    else if(!rng.equals(baselineRng))throw new AssertionError("Imported random state differs");
                    // Undo the probe so each actual branch starts with identical RNG.
                    w.command("import-world "+quote(world));
                    historicalRadius(w);
                    if(!state(w).equals(initial))throw new AssertionError("State changed after RNG reset");
                    w.command("set horizontal-rate 0 set vertical-rate 0 set dt "+dt);
                    double minimum=Double.POSITIVE_INFINITY,first=Double.NaN;long overlaps=0;int idA=-1,idB=-1;
                    double startRejected=number(w,"avm-rejected-steps");
                    try(PrintWriter trace=new PrintWriter(Files.newBufferedWriter(out.resolve("trace-dt"+dt+".csv")))){
                        trace.println("start_s,h_s,min_distance_m,pair_a,pair_b");
                        for(int step=0;step<Math.round(1/dt);step++){
                            w.command("go");
                            LogoList history=(LogoList)w.report("last-internal-step-paths");
                            if(history.size()!=1)throw new AssertionError("Unexpected adaptive step");
                            LogoList entry=(LogoList)history.apply(0),paths=(LogoList)entry.apply(2);double start=n(entry,0),h=n(entry,1),sub=Double.POSITIVE_INFINITY;int sa=-1,sb=-1;boolean overlap=false;
                            if(Math.abs(h-dt)>1e-12)throw new AssertionError("Timestep altered");
                            for(int i=0;i<paths.size();i++)for(int j=i+1;j<paths.size();j++){
                                LogoList a=(LogoList)paths.apply(i),b=(LogoList)paths.apply(j);double[] approach=closest(a,b);
                                if(approach[0]<sub){sub=approach[0];sa=(int)n(a,0);sb=(int)n(b,0);}
                                if(approach[0]<minimum){minimum=approach[0];idA=(int)n(a,0);idB=(int)n(b,0);}
                                if(approach[0]<2*HISTORICAL_RADIUS-1e-8){overlap=true;if(Double.isNaN(first))first=start+h*approach[1];}
                            }
                            if(overlap)overlaps++;
                            trace.printf(Locale.ROOT,"%.12g,%.12g,%.12g,%d,%d%n",start,h,sub,sa,sb);
                        }
                    }
                    double rejected=number(w,"avm-rejected-steps")-startRejected;if(rejected!=0)throw new AssertionError("Rejected step");
                    summary.printf(Locale.ROOT,"%d,%.12g,%.12g,1,%.12g,%.12g,%.12g,%d,%d,%d,%.0f,true%n",seed,base,dt,minimum,Math.max(0,2*HISTORICAL_RADIUS-minimum),first,overlaps,idA,idB,rejected);summary.flush();
                    Files.writeString(out.resolve("final-dt"+dt+".txt"),state(w));
                    System.out.printf(Locale.ROOT,"dt %.4g: min %.12g m, overlap depth %.9g m, %d overlap intervals, pair%d/%d%n",dt,minimum,Math.max(0,2*HISTORICAL_RADIUS-minimum),overlaps,idA,idB);
                }
            }
        }finally{w.dispose();}
    }
}
