import java.nio.file.*;
import java.util.*;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Scenario navigation checks; independent of the AVM pair equations.
 * java -cp netlogo.jar tests/DoorRoutingChecks.java [model-root] [output-dir]
 */
public class DoorRoutingChecks {
    static final String PEOPLE = "map [p -> [(list who xcor ycor vx vy move-ex move-ey ex ey goal-x goal-y desired-speed waypoints door-still-seconds door-recovery-until door-recovery-target door-approach-complete?)] of p] sort pedestrians";
    static final String STATE = "(list elapsed-seconds arrived-count returned-count door-recovery-count last-internal-step-paths ("+PEOPLE+"))";
    static double n(LogoList row,int index){return ((Number)row.apply(index)).doubleValue();}
    static void check(boolean value,String message){if(!value)throw new AssertionError(message);}
    static Object report(HeadlessWorkspace w,org.nlogo.nvm.Procedure p)throws Exception{
        Object value=w.runCompiledReporter(w.defaultOwner(),p);error(w);return value;
    }
    static void command(HeadlessWorkspace w,org.nlogo.nvm.Procedure p)throws Exception{w.runCompiledCommands(w.defaultOwner(),p);error(w);}
    static void error(HeadlessWorkspace w)throws Exception{if(w.lastLogoException()!=null){var e=w.lastLogoException();w.lastLogoException_$eq(null);throw e;}}
    static void setup(HeadlessWorkspace w,int population)throws Exception{
        w.command("set simulation-seed 42 set dt .02 set mean-desired-speed 1.55 set desired-speed-sd .18 set noise-strength 0 set show-trails? false set run-checks? false set log-csv? false set initial-per-side "+population+" set spawn-rate 0 set two-sided? true set room-width 6 set door-width 1.2 setup");
    }
    static Map<Long,List<Double>> goals(LogoList people){
        Map<Long,List<Double>> result=new HashMap<>();
        for(Object raw:people.javaIterable()){LogoList p=(LogoList)raw;result.put((long)n(p,0),List.of(n(p,9),n(p,10)));}return result;
    }
    static String goBody(Path model)throws Exception{
        String source=Files.readString(model).replace("\r\n","\n");int start=source.indexOf("to go\n")+6,end=source.indexOf("\nend",start);
        check(start>=6&&end>start,"Cannot locate scenario go procedure");return source.substring(start,end);
    }
    static void order(Path root,StringBuilder results)throws Exception{
        var a=HeadlessWorkspace.newInstance();var b=HeadlessWorkspace.newInstance();
        try{
            Path model=root.resolve("scenarios/Door.nlogox").toAbsolutePath();a.open(model.toString());b.open(model.toString());setup(a,20);setup(b,20);
            Map<Long,List<Double>> original=goals((LogoList)a.report(PEOPLE));
            b.command("set avm-reverse-calculation? true");
            String reverse=goBody(model).replace("let walkers sort pedestrians","let walkers reverse sort pedestrians");
            check(reverse.contains("reverse sort pedestrians"),"Reverse-route fixture did not reverse the actors");
            var ga=a.compileCommands("go");var gb=b.compileCommands(reverse);var sa=a.compileReporter(STATE);var sb=b.compileReporter(STATE);
            // Candidate evaluation must have no state mutation and consume no RNG.
            a.command("ask pedestrians [set door-still-seconds 4 set vx 0 set vy 0]");
            b.command("ask pedestrians [set door-still-seconds 4 set vx 0 set vy 0]");
            Object before=report(a,sa);
            Object forward=a.report("map [p -> [door-recovery-plan avm-snapshot] of p] sort pedestrians");
            Object backward=a.report("reverse map [p -> [door-recovery-plan avm-snapshot] of p] reverse sort pedestrians");
            check(forward.equals(backward),"Recovery plans depend on evaluation order");
            check(before.equals(report(a,sa)),"Recovery reporter changed state");
            check(a.report("n-values 32 [random-float 1]").equals(b.report("n-values 32 [random-float 1]")),"Recovery probes consumed random values");
            // Reinitialize so the comparison is the ordinary scenario trajectory.
            setup(a,20);setup(b,20);b.command("set avm-reverse-calculation? true");
            int steps=3000;
            for(int k=0;k<steps;k++){
                command(a,ga);command(b,gb);LogoList left=(LogoList)report(a,sa),right=(LogoList)report(b,sb);
                check(left.equals(right),"Reversed route/core order changed state at step "+k);
                for(var entry:goals((LogoList)left.apply(5)).entrySet())check(entry.getValue().equals(original.get(entry.getKey())),"Original final destination changed for pedestrian "+entry.getKey());
            }
            check(((Number)a.report("door-recovery-count")).doubleValue()>0,"No recovery was exercised");
            check(a.report("n-values 32 [random-float 1]").equals(b.report("n-values 32 [random-float 1]")),"Future RNG changed by actor order");
            check(Boolean.FALSE.equals(a.report("pair-contact-enabled?")),"Pair contact constraint was enabled");
            results.append("PASS: 60 s / 3000 steps with reversed recovery-plan, target-commit and AVM calculation order: exact states, paths and counters; original final destinations retained; 32 future RNG draws identical.\n");
            results.append("PASS: recovery candidate probes are pure, order independent and consume no RNG.\n");
        }finally{a.dispose();b.dispose();}
    }
    static void geometry(Path root,StringBuilder results)throws Exception{
        var w=HeadlessWorkspace.newInstance();try{
            w.open(root.resolve("scenarios/Door.nlogox").toAbsolutePath().toString());setup(w,0);w.command("add-pedestrian 9 4 1");
            double actual=((Number)w.report("[door-visible-portal-y 10.3 4.2] of first sort pedestrians")).doubleValue();
            // Independent line-circle discriminant: line slope m tangent to the
            // lower post (10,3.9), from (9,4), has distance R from that post.
            double radius=.2+.1+1e-8,qx=1,qy=-.1;
            double slope=(qx*qy+radius*Math.sqrt(qx*qx+qy*qy-radius*radius))/(qx*qx-radius*radius);
            double expected=4+1.3*slope;
            check(Math.abs(actual-expected)<3e-8,"Portal tangent has inconsistent envelope radius: "+actual+" vs "+expected);
            double distance=Math.abs(qx*(actual-4)-qy*1.3)/Math.hypot(1.3,actual-4);
            check(Math.abs(distance-radius)<3e-8,"Portal ray is not tangent to the wall-glide envelope");
            check(Boolean.TRUE.equals(w.report("[door-route-clear? 10.3 "+actual+"] of first sort pedestrians")),"Tangent target is not route-clear");
            // Near each exit, a recovery is allowed only to an interior point.
            for(String point:List.of("1.01 4.5 1","18.99 4.5 -1")){
                w.command("ask pedestrians [die] add-pedestrian "+point+" ask pedestrians [set door-still-seconds 10 set vx 0 set vy 0]");
                LogoList plan=(LogoList)w.report("[door-recovery-plan avm-snapshot] of first sort pedestrians");LogoList target=(LogoList)plan.apply(2);
                check(!target.isEmpty(),"Near-exit fixture did not exercise recovery");
                check(n(target,0)>1&&n(target,0)<19&&n(target,1)>1.7&&n(target,1)<7.3,"Recovery selected a target outside the corridor");
            }
            results.append("PASS: independently derived wall-envelope tangent and wall-clear ray; temporary recovery targets stay inside both entrance/exit boundaries.\n");
        }finally{w.dispose();}
    }
    public static void main(String[] args)throws Exception{
        System.setProperty("java.awt.headless","true");Locale.setDefault(Locale.ROOT);
        Path root=Path.of(args.length>0?args[0]:"."),out=Path.of(args.length>1?args[1]:"work/avm-flow-fix/door/routing-checks");Files.createDirectories(out);
        StringBuilder result=new StringBuilder();geometry(root,result);order(root,result);Files.writeString(out.resolve("routing-result.txt"),result);System.out.print(result);
    }
}
