import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Pattern;
import javax.xml.parsers.DocumentBuilderFactory;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;
import org.w3c.dom.Element;

/** Independent observation parity, event timing and accepted-substep bookkeeping. */
public class AVMLifecycleChecks {
    static long assertions, comparisons;
    static Path root, output;
    record Fixture(String name,String file,String settings,int steps,boolean hallway) {}
    record Contract(List<String> globals,String expression) {}
    record Replay(List<Object> states,Object futureRng) {}
    static void ok(boolean condition,String message) {assertions++;if(!condition)throw new AssertionError(message);}
    static double number(Object value) {return ((Number)value).doubleValue();}
    static double n(HeadlessWorkspace w,String expression)throws Exception{return number(w.report(expression));}
    static void near(double actual,double expected,double tolerance,String message) {
        ok(Math.abs(actual-expected)<=tolerance,message+": "+actual+" expected "+expected);
    }
    static String quote(Path p){return "\""+p.toAbsolutePath().toString().replace('\\','/')+"\"";}
    static String strip(String s){return s.replaceAll("(?m);[^\\r\\n]*","");}
    static void declarations(String source,Set<String> globals,Set<String> owned) {
        var m=Pattern.compile("(?i)\\b(globals|pedestrians-own|turtles-own)\\s*\\[([^]]*)]").matcher(strip(source));
        while(m.find())for(String name:m.group(2).trim().split("\\s+"))
            if(!name.isBlank())(m.group(1).equalsIgnoreCase("globals")?globals:owned).add(name);
    }
    static Contract contract(Path file)throws Exception {
        var factory=DocumentBuilderFactory.newInstance();
        factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl",true);
        var xml=factory.newDocumentBuilder().parse(file.toFile());
        String source=xml.getElementsByTagName("code").item(0).getTextContent();
        Set<String> globals=new LinkedHashSet<>(),owned=new LinkedHashSet<>();
        declarations(source,globals,owned);
        var include=Pattern.compile("__includes\\s*\\[([^]]*)]").matcher(strip(source));
        if(include.find()) {
            var names=Pattern.compile("\"([^\"]+)\"").matcher(include.group(1));
            while(names.find()) {
                String name=names.group(1);
                // These modules contain only observation state, deliberately allowed to differ.
                if(!name.endsWith("checks.nls")&&!name.endsWith("stats.nls"))
                    declarations(Files.readString(file.getParent().resolve(name)),globals,owned);
            }
        }
        for(String tag:List.of("slider","switch","chooser","input")) {
            var widgets=xml.getElementsByTagName(tag);
            for(int i=0;i<widgets.getLength();i++) {
                String name=((Element)widgets.item(i)).getAttribute("variable");
                if(!name.isBlank()&&!Set.of("run-checks?","log-csv?").contains(name))globals.add(name);
            }
        }
        String turtles="(map [p -> [(list who xcor ycor heading color label size shape hidden? "
            +String.join(" ",owned)+")] of p] sort pedestrians)";
        return new Contract(List.copyOf(globals),"(list (list "+String.join(" ",globals)+") "+turtles
            +" (list ticks min-pxcor max-pxcor min-pycor max-pycor))");
    }
    static void identical(Object a,Object b,String context) {
        comparisons++;
        if(a instanceof Number x && b instanceof Number y) {
            ok(Double.isFinite(x.doubleValue())&&Double.isFinite(y.doubleValue())
                &&Double.doubleToRawLongBits(x.doubleValue())==Double.doubleToRawLongBits(y.doubleValue()),context+": "+a+" != "+b);
        } else if(a instanceof LogoList x && b instanceof LogoList y) {
            ok(x.size()==y.size(),context+" list lengths");
            for(int i=0;i<x.size();i++)identical(x.apply(i),y.apply(i),context+"["+i+"]");
        } else ok(a.equals(b),context+": "+a+" != "+b);
    }
    static void trace(HeadlessWorkspace w,double start,double finish,boolean traffic,boolean checks)throws Exception {
        LogoList rows=(LogoList)w.report("last-internal-step-paths");
        ok(rows.size()>0,"Missing accepted step trace");
        double cursor=start;
        for(int i=0;i<rows.size();i++) {
            LogoList row=(LogoList)rows.apply(i);
            double time=number(row.apply(0)),h=number(row.apply(1));
            near(time,cursor,2e-10,"Trace gap/duplicate");ok(h>0,"Nonpositive accepted duration");
            if(traffic) {
                double nextEvent=(Math.floor((time+1e-10)/.11)+1)*.11;
                ok(time+h<=nextEvent+2e-10,"Accepted segment straddles light switch");
            }
            LogoList paths=(LogoList)row.apply(2),states=(LogoList)row.apply(3);
            ok(paths.size()==states.size(),"Missing endpoint state");
            for(int j=0;j<states.size();j++)ok(((LogoList)states.apply(j)).size()==8,"Missing extended endpoint state");
            cursor=time+h;
        }
        near(cursor,finish,2e-10,"Trace does not cover complete outer step");
        if(checks)near(n(w,"checks-segments-processed"),rows.size(),0,"Unprocessed/reprocessed trace");
    }
    static Replay replay(Fixture f,Contract c,boolean checks,boolean logging,Replay baseline)throws Exception {
        HeadlessWorkspace w=HeadlessWorkspace.newInstance();
        String variant="checks-"+checks+"-csv-"+logging;
        Path run=output.resolve(f.name()).resolve(variant);
        Files.createDirectories(run.resolve("scenarios"));Files.createDirectories(run.resolve("work"));
        List<Object> snapshots=new ArrayList<>();
        long begin=System.nanoTime();
        try {
            w.open(root.resolve(f.file()).toString());
            // Route default scenario CSV names to this fixture's isolated directory.
            w.command("set-current-directory "+quote(run.resolve("scenarios")));
            String switches="set run-checks? "+checks+(f.hallway()?"":" set log-csv? "+logging);
            w.command("set dt .07 set mean-desired-speed 1.55 set desired-speed-sd .18 set simulation-seed 42 set show-trails? false "
                +switches+" "+f.settings()+" setup");
            if(f.hallway()&&logging)w.command("log-start "+quote(run.resolve("work/hallway.csv")));
            snapshots.add(w.report(c.expression()));
            if(baseline!=null)identical(baseline.states().get(0),snapshots.get(0),f.name()+" "+variant+" setup");
            for(int step=1;step<=f.steps();step++) {
                double before=n(w,"elapsed-seconds");
                w.command("go"+(f.hallway()&&logging?" log-row":""));
                double elapsed=n(w,"elapsed-seconds");
                near(elapsed,step*.07,2e-10,"Outer simulation time");
                trace(w,before,elapsed,f.name().equals("traffic"),checks);
                if(f.name().equals("traffic")) {
                    long switchesExpected=(long)Math.floor((elapsed+1e-10)/.11);
                    near(n(w,"light-switches"),switchesExpected,0,"Light switch count");
                    near(n(w,"light-timer"),elapsed-switchesExpected*.11,2e-10,"Light phase drift");
                    ok(((Boolean)w.report("light-green?"))==(switchesExpected%2==1),"Wrong light colour");
                    near(n(w,"dt"),.07,0,"Outer dt not restored");
                }
                w.command("check-invariants");
                Object state=w.report(c.expression());snapshots.add(state);
                if(baseline!=null)identical(baseline.states().get(step),state,f.name()+" "+variant+" step "+step);
            }
            Object rng=w.report("n-values 32 [random-float 1]");
            if(baseline!=null)identical(baseline.futureRng(),rng,f.name()+" future RNG "+variant);
            if(logging) {
                w.command("log-stop");
                try(var files=Files.list(run.resolve("work"))) {
                    List<Path> csv=files.filter(p->p.toString().endsWith(".csv")).toList();
                    ok(csv.size()==1,"Expected one isolated CSV");
                    List<String> lines=Files.readAllLines(csv.get(0));
                    ok(lines.size()>f.steps()/3,"CSV did not sample the run");
                    double previous=-1;
                    for(int i=1;i<lines.size();i++) {
                        double time=Double.parseDouble(lines.get(i).split(",")[0]);
                        ok(time>previous,"Duplicate/out-of-order CSV timestamp");previous=time;
                    }
                }
            }
            System.out.printf(Locale.ROOT,"PASS observation %s %s: %d steps, %.2f simulated s, %.2f wall s%n",
                f.name(),variant,f.steps(),f.steps()*.07,(System.nanoTime()-begin)/1e9);
            return new Replay(snapshots,rng);
        } finally {w.dispose();}
    }
    static void lightPhysicalPhase()throws Exception {
        HeadlessWorkspace w=HeadlessWorkspace.newInstance();
        try {
            w.open(root.resolve("scenarios/TrafficLight.nlogox").toString());
            w.command("set log-csv? false set initial-per-side 0 set spawn-rate 0 set desired-speed-sd 0 "
                +"set mean-desired-speed 1.55 set dt .07 set switch-time .11 set start-green? false "
                +"set road-width 6 set walkway-width 6 set run-checks? true setup "
                +"add-pedestrian (road-left - body-radius - .25) 4.5 1 "
                +"foreach sort pedestrians [p -> ask p [set goal-y ycor update-desired-direction set move-ex ex set move-ey ey]]");
            double stop=n(w,"road-left - body-radius"),x=stop-.25;
            int rowsSeen=0;
            for(int outer=0;outer<4;outer++) {
                w.command("go check-invariants");
                LogoList trace=(LogoList)w.report("last-internal-step-paths");
                for(int j=0;j<trace.size();j++) {
                    LogoList row=(LogoList)trace.apply(j);
                    double t=number(row.apply(0)),h=number(row.apply(1));
                    boolean green=((int)Math.floor((t+1e-10)/.11))%2==1;
                    double speed=green?1.55:Math.min(1.55,Math.max(0,(stop-x)/.6));
                    LogoList state=(LogoList)((LogoList)row.apply(3)).apply(0);
                    near(number(state.apply(3)),speed,2e-12,"Wrong light phase used for movement speed");
                    x+=h*speed;
                    near(number(state.apply(1)),x,2e-12,"Wrong split-step displacement");
                    rowsSeen++;
                }
                near(n(w,"[xcor] of first sort pedestrians"),x,2e-12,"Committed light displacement");
                near(n(w,"max-stop-overshoot"),0,0,"Red stop overshot");
            }
            ok(rowsSeen==6,"Expected four outer steps plus two exact light splits");
            System.out.println("PASS exact red/green phase affects speed and displacement on both sides of .11 s and .22 s switches.");
        } finally {w.dispose();}
    }
    static void doorBookkeeping()throws Exception {
        HeadlessWorkspace w=HeadlessWorkspace.newInstance();
        try {
            w.open(root.resolve("scenarios/Door.nlogox").toString());
            w.command("set log-csv? false set initial-per-side 0 set spawn-rate 0 set desired-speed-sd 0 "
                +"set mean-desired-speed 1.55 set dt .2 set door-width 4 set room-width 6 set run-checks? true setup "
                +"add-pedestrian 9.99 3 1 add-pedestrian 9.8 4.5 1 add-pedestrian 10.1 6 -1 "
                +"foreach sort pedestrians [p -> ask p [set waypoints [] set goal-y ycor update-desired-direction "
                +"set move-ex ex set move-ey ey]] go check-invariants");
            ok(n(w,"length last-internal-step-paths")==1,"Door changed the requested fixed timestep");
            near(n(w,"avm-rejected-steps"),0,0,"A pedestrian-distance step was rejected");
            ok((Boolean)w.report("door-sequence = [1 -1 1]"),"Door passages not sorted by absolute substep time");
            w.command("go check-invariants");near(n(w,"door-passes"),3,0,"Repeated Door count on later step");
            // Synthetic departed paths exercise out/back, exact-plane ownership,
            // chronological ordering and tie-breaking without requiring unsafe motion.
            w.command("setup checks-begin-step set last-internal-step-paths (list "
                +"(list 0 .1 [[7 9 4.5 11 4.5] [2 11 4.5 9 4.5]] []) "
                +"(list .1 .1 [[7 11 4.5 9 4.5] [8 9 4.5 10 4.5]] []) "
                +"(list .2 .1 [[8 10 4.5 11 4.5] [9 10 4.5 10 5]] [])) track-door");
            ok((Boolean)w.report("door-sequence = [-1 1 -1 1]"),"Door departed/out-back/endpoint/tie bookkeeping");
            // Earlier invalid endpoint must remain visible after a valid endpoint,
            // including when its pedestrian no longer exists.
            for(String invalid:List.of("[20 5 4.5 2 0 1 0 1.55]","[20 5 4.5 1 0 2 0 1.55]")) {
                w.command("checks-begin-step set last-internal-step-paths (list (list 0 .01 [] (list "+invalid+")) "
                    +"(list .01 .01 [] [[20 5 4.5 1 0 1 0 1.55]]))");
                boolean failed=false;try{w.command("check-invariants");}catch(Exception e){failed=true;}
                ok(failed,"Missed invalid departed endpoint: "+invalid);
            }
            System.out.println("PASS Door fixed-step motion, chronological passages, deleted paths, out/back, plane endpoints, exact ties; departed-state speed/heading diagnostics.");
        } finally {w.dispose();}
    }
    public static void main(String[] args)throws Exception {
        root=Path.of(args.length>0?args[0]:".").toAbsolutePath();
        output=Path.of(args.length>1?args[1]:"work/avm/validation/lifecycle").toAbsolutePath();
        List<Fixture> fixtures=List.of(
            new Fixture("hallway","Pedestrians.nlogox","set periodic? true set population 24 set hall-width 10",70,true),
            new Fixture("door","scenarios/Door.nlogox","set initial-per-side 5 set spawn-rate .8 set two-sided? true set door-width 1.2 set room-width 6",70,false),
            new Fixture("traffic","scenarios/TrafficLight.nlogox","set initial-per-side 5 set spawn-rate .8 set switch-time .11 set start-green? false set road-width 6 set walkway-width 6",100,false),
            new Fixture("crossing","scenarios/Crossing.nlogox","set horizontal-rate 1.8 set vertical-rate 1.8 set two-way? false set corridor-width 4",100,false));
        for(Fixture f:args.length>2&&args[2].equals("events-only")?List.<Fixture>of():fixtures) {
            Contract c=contract(root.resolve(f.file()));
            Replay baseline=replay(f,c,false,false,null);
            replay(f,c,true,false,baseline);replay(f,c,false,true,baseline);replay(f,c,true,true,baseline);
        }
        lightPhysicalPhase();doorBookkeeping();
        System.out.println("AVM lifecycle passed: "+assertions+" assertions, "+comparisons+" raw-bit state comparisons.");
        System.exit(0);
    }
}
