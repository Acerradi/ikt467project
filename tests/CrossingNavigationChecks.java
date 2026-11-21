import java.nio.file.Path;
import java.util.Locale;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Geometric navigation regressions; no pedestrian spacing constraint. */
public class CrossingNavigationChecks {
    static int assertions;
    static double n(HeadlessWorkspace w,String s)throws Exception{return ((Number)w.report(s)).doubleValue();}
    static void ok(boolean value,String message){assertions++;if(!value)throw new AssertionError(message);}
    static String person(double x,double y,double gx,double gy,boolean vertical){
        int direction=(vertical?gy-y:gx-x)>0?1:-1;
        return String.format(Locale.ROOT,"create-pedestrians 1 [initialize-pedestrian %.15g %.15g %.15g %.15g %d set vertical? %s set crossing-admitted? false set crossing-cleared? false set free-time (vector-length (goal-x - xcor) (goal-y - ycor)) / desired-speed] set spawned-count 1",x,y,gx,gy,direction,vertical);
    }
    static void setup(HeadlessWorkspace w)throws Exception{
        w.command("set crossing-yielding? false set horizontal-rate 0 set vertical-rate 0 set two-way? false set corridor-width 4 set dt .02 set mean-desired-speed 1.55 set desired-speed-sd 0 set show-trails? false set run-checks? false set log-csv? false set simulation-seed 42 setup");
    }
    public static void main(String[] args)throws Exception{
        Locale.setDefault(Locale.ROOT);
        Path root=Path.of(args.length>0?args[0]:".").toAbsolutePath();
        HeadlessWorkspace w=HeadlessWorkspace.newInstance();
        try {
            w.open(root.resolve("scenarios/Crossing.nlogox").toString());
            setup(w);
            ok(n(w,"body-radius")==.2,"Changed body radius");
            ok(Boolean.FALSE.equals(w.report("pair-contact-enabled?")),"Pair enforcement active");
            ok(Boolean.TRUE.equals(w.report("crossing-route-clear? 1 14 29 16")),"Clear corridor route rejected");
            ok(Boolean.FALSE.equals(w.report("crossing-route-clear? 15 23 29 16")),"Corner obstruction missed");
            w.command(person(5,16,29,14,false)+" ask pedestrians [reroute-if-displaced update-desired-direction]");
            ok(n(w,"[ex] of one-of pedestrians")==1&&n(w,"[ey] of one-of pedestrians")==0,"Crossing approach did not aim at the nearest outgoing portal point");
            ok(n(w,"[goal-y] of one-of pedestrians")==14,"Routing changed the random final destination");
            w.command("ask pedestrians [setxy 20 16 reroute-if-displaced update-desired-direction]");
            ok(n(w,"[length waypoints] of one-of pedestrians")==0&&n(w,"[ey] of one-of pedestrians")<0,"After the outgoing portal, route did not return to the original goal");
            setup(w);
            w.command(person(16.6,16.9,29,16.5,false)+" ask pedestrians [reroute-if-displaced update-desired-direction]");
            ok(n(w,"[length waypoints] of one-of pedestrians")==1,"Near-corner waypoint was prematurely removed");
            ok(n(w,"[item 0 first waypoints] of one-of pedestrians")>16.7,"Short waypoint was not extended along the same ray");
            double[][] fixtures={{15,23,29,16,0},{15,7,29,14,0},{15,23,1,16,0},{15,7,1,14,0},
                                 {7,15,14,29,1},{23,15,16,29,1},{7,15,14,1,1},{23,15,16,1,1}};
            for(double[] f:fixtures){
                setup(w);w.command(person(f[0],f[1],f[2],f[3],f[4]==1));
                int steps=0;
                while(n(w,"count pedestrians")>0&&steps<1500){
                    w.command("go check-invariants");steps++;
                }
                ok(n(w,"arrived-count")==1,"Displaced route failed to reach destination: "+java.util.Arrays.toString(f));
                ok(n(w,"returned-count")==0,"Route returned through entrance");
                ok(n(w,"avm-rejected-steps")==0&&n(w,"contact-step-count")==0,"Route used a pair safeguard");
                System.out.printf(Locale.ROOT,"Route %s cleared in %.2f s%n",java.util.Arrays.toString(f),steps*.02);
            }
            System.out.println("PASS "+assertions+" crossing navigation checks");
        } finally {w.dispose();}
    }
}
