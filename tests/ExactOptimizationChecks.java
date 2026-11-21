import java.io.PrintWriter;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Pattern;
import javax.xml.parsers.DocumentBuilderFactory;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;
import org.w3c.dom.Element;

/** Bit-for-bit baseline replay and separately timed benchmarks in actual NetLogo.
 * The baseline is a pristine checkout/snapshot containing the four models/core.
 * No epsilon is used: every compared numeric value must have identical IEEE bits.
 * Existing globals/owned variables are discovered from BASELINE source; new
 * implementation-only caches are deliberately excluded. Existing solver state,
 * paths, scenario records, widgets, monitors, CSV and future RNG are included.
 *
 * --baseline work/exact-optimization-baseline --current .
 * --mode parity|benchmark --quick [only a short harness smoke]
 * --cases hallway-periodic,hallway-original,hallway-open,door,traffic,crossing
 * --output work/exact-optimization/parity.csv --repeats 3
 */
public class ExactOptimizationChecks {
    static long comparisons;
    static final String DEFAULT_BASELINE="work/exact-optimization-baseline";
    record Fixture(String name,String model,String settings,int steps,boolean csv,int benchmarkWarm,int benchmarkSteps) {}
    record Contract(List<String> globals,List<String> owned,List<String> monitors,String expression) {}
    static String option(String[] args,String key,String fallback) {
        for(int i=0;i<args.length-1;i++)if(args[i].equals(key))return args[i+1];
        return fallback;
    }
    static String quoted(Path path) {return "\""+path.toAbsolutePath().toString().replace('\\','/')+"\"";}
    static String stripComments(String text) {return text.replaceAll("(?m);[^\\r\\n]*", "");}
    static void declarations(String text,Set<String> globals,Set<String> owned) {
        text=stripComments(text);
        var matcher=Pattern.compile("(?i)\\b(globals|pedestrians-own|turtles-own)\\s*\\[([^]]*)]").matcher(text);
        while(matcher.find()) {
            Set<String> target=matcher.group(1).equalsIgnoreCase("globals")?globals:owned;
            for(String name:matcher.group(2).trim().split("\\s+"))if(!name.isBlank())target.add(name);
        }
    }
    static Contract contract(Path model)throws Exception {
        var factory=DocumentBuilderFactory.newInstance();
        factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl",true);
        var document=factory.newDocumentBuilder().parse(model.toFile());
        String code=document.getElementsByTagName("code").item(0).getTextContent();
        Set<String> globals=new LinkedHashSet<>(),owned=new LinkedHashSet<>();
        declarations(code,globals,owned);
        var include=Pattern.compile("__includes\\s*\\[([^]]*)]").matcher(stripComments(code));
        if(include.find()) {
            var filenames=Pattern.compile("\"([^\"]+)\"").matcher(include.group(1));
            while(filenames.find())declarations(Files.readString(model.getParent().resolve(filenames.group(1))),globals,owned);
        }
        for(String tag:List.of("slider","switch","chooser","input")) {
            var widgets=document.getElementsByTagName(tag);
            for(int i=0;i<widgets.getLength();i++) {
                String name=((Element)widgets.item(i)).getAttribute("variable");
                if(!name.isBlank())globals.add(name);
            }
        }
        List<String> monitors=new ArrayList<>();
        var monitorNodes=document.getElementsByTagName("monitor");
        for(int i=0;i<monitorNodes.getLength();i++)monitors.add(monitorNodes.item(i).getTextContent().trim());
        List<String> globalExpressions=globals.stream().map(s->s.equals("log-file")?"(log-file != \"\")":s).toList();
        String turtle="(map [p -> [(list who xcor ycor heading color label label-color size shape hidden? pen-mode pen-size "
            +"(breed = pedestrians) "+String.join(" ",owned)+")] of p] sort pedestrians)";
        String monitor="(list "+String.join(" ",monitors.stream().map(s->"("+s+")").toList())+")";
        String expression="(list (list "+String.join(" ",globalExpressions)+") "+turtle+" "+monitor
            +" (list ticks min-pxcor max-pxcor min-pycor max-pycor world-width world-height))";
        return new Contract(List.copyOf(globals),List.copyOf(owned),monitors,expression);
    }
    static void identical(Object expected,Object actual,String label) {
        comparisons++;
        if(expected instanceof Number a && actual instanceof Number b) {
            double x=a.doubleValue(),y=b.doubleValue();
            if(!Double.isFinite(x)||!Double.isFinite(y)||Double.doubleToRawLongBits(x)!=Double.doubleToRawLongBits(y))
                throw new AssertionError(label+": expected "+x+" ["+Long.toHexString(Double.doubleToRawLongBits(x))
                    +"], got "+y+" ["+Long.toHexString(Double.doubleToRawLongBits(y))+"]");
        }else if(expected instanceof LogoList a && actual instanceof LogoList b) {
            if(a.size()!=b.size())throw new AssertionError(label+": list lengths "+a.size()+" vs "+b.size());
            for(int i=0;i<a.size();i++)identical(a.apply(i),b.apply(i),label+"["+i+"]");
        }else if(expected instanceof Boolean || expected instanceof String) {
            if(!expected.equals(actual))throw new AssertionError(label+": "+expected+" vs "+actual);
        }else {
            // NetLogo agent references in constraints are represented by their
            // breed/ID string, not Java object identity across workspaces.
            if(expected==null||actual==null||!expected.getClass().equals(actual.getClass())
                ||!expected.toString().equals(actual.toString()))
                throw new AssertionError(label+": agent/value "+expected+" vs "+actual);
        }
    }
    static void state(HeadlessWorkspace baseline,HeadlessWorkspace current,Contract c,String label)throws Exception {
        LogoList a=(LogoList)baseline.report(c.expression()),b=(LogoList)current.report(c.expression());
        LogoList ga=(LogoList)a.apply(0),gb=(LogoList)b.apply(0);
        for(int i=0;i<c.globals().size();i++)identical(ga.apply(i),gb.apply(i),label+" global "+c.globals().get(i));
        identical(a.apply(1),b.apply(1),label+" turtle builtins/owned");
        identical(a.apply(2),b.apply(2),label+" monitor");
        identical(a.apply(3),b.apply(3),label+" clock/world");
    }
    static List<Fixture> fixtures() {
        String common="set dt .02 set mean-desired-speed 1.34 set desired-speed-sd .26 "
            +"set simulation-seed 42 set relative-anticipation? true set run-checks? true set noise-strength 0 set show-trails? false ";
        return List.of(
            new Fixture("hallway-periodic","Pedestrians.nlogox",common+"set periodic? true set population 150 set hall-width 10 setup",250,false,50,100),
            new Fixture("hallway-original","Pedestrians.nlogox",common+"set relative-anticipation? false set periodic? true set population 40 set hall-width 10 setup",250,false,50,100),
            new Fixture("hallway-open","Pedestrians.nlogox",common+"set periodic? false set spawn-rate 2 set dt .03 set noise-strength .05 set show-trails? true setup",300,true,150,150),
            new Fixture("door","scenarios/Door.nlogox",common+"set log-csv? false set initial-per-side 20 set spawn-rate 0 set two-sided? true set room-width 6 set door-width 1.2 setup",500,true,250,250),
            new Fixture("traffic","scenarios/TrafficLight.nlogox",common+"set log-csv? false set initial-per-side 20 set spawn-rate .5 set dt .07 set switch-time 2 set start-green? false set road-width 6 set walkway-width 6 setup",200,true,100,100),
            new Fixture("crossing","scenarios/Crossing.nlogox",common+"set log-csv? false set horizontal-rate 1.8 set vertical-rate 1.8 set two-way? false set corridor-width 4 setup",1500,true,1250,250)
        );
    }
    static String rng(HeadlessWorkspace world)throws Exception{return world.report("n-values 32 [random-float 1]").toString();}
    static void parity(Path baselineRoot,Path currentRoot,Fixture f,boolean quick,Path outDir,PrintWriter csv)throws Exception {
        var baseline=HeadlessWorkspace.newInstance();var current=HeadlessWorkspace.newInstance();
        long started=System.nanoTime(),before=comparisons;
        Path beforeCsv=outDir.resolve(f.name()+"-baseline.csv"),afterCsv=outDir.resolve(f.name()+"-current.csv");
        try {
            baseline.open(baselineRoot.resolve(f.model()).toString());current.open(currentRoot.resolve(f.model()).toString());
            Contract c=contract(baselineRoot.resolve(f.model()));
            baseline.command(f.settings());current.command(f.settings());
            if(f.csv()) {
                baseline.command("log-start "+quoted(beforeCsv));current.command("log-start "+quoted(afterCsv));
            }
            state(baseline,current,c,f.name()+" setup");
            int steps=quick?Math.min(20,f.steps()):f.steps();
            for(int step=1;step<=steps;step++) {
                baseline.command("go");current.command("go");
                state(baseline,current,c,f.name()+" step "+step);
                if(step%250==0)System.out.println("PROGRESS exact "+f.name()+" "+step+"/"+steps);
            }
            if(!rng(baseline).equals(rng(current)))throw new AssertionError(f.name()+": future RNG diverged");
            if(f.csv()) {
                baseline.command("log-stop");current.command("log-stop");
                if(!Arrays.equals(Files.readAllBytes(beforeCsv),Files.readAllBytes(afterCsv)))
                    throw new AssertionError(f.name()+": CSV bytes differ");
            }
            double seconds=((Number)current.report("elapsed-seconds")).doubleValue();
            double wall=(System.nanoTime()-started)/1e9;
            csv.printf(Locale.ROOT,"%s,%d,%.12f,%d,true,%s,%.6f%n",f.name(),steps,seconds,comparisons-before,f.csv(),wall);csv.flush();
            System.out.printf(Locale.ROOT,"PASS: %s, %d steps / %.3f simulated s, %d comparisons; exact state, monitors, counters and future RNG%s.%n",
                f.name(),steps,seconds,comparisons-before,f.csv()?", identical CSV bytes":"");
        }finally{baseline.dispose();current.dispose();}
    }
    static double timed(HeadlessWorkspace w,Fixture f,int warm,int steps)throws Exception {
        w.command(f.settings());
        if(warm>0)w.command("repeat "+warm+" [go]");
        long started=System.nanoTime();w.command("repeat "+steps+" [go]");return(System.nanoTime()-started)/1e9;
    }
    static void benchmark(Path baselineRoot,Path currentRoot,Fixture f,boolean quick,int repeats,PrintWriter csv)throws Exception {
        var baseline=HeadlessWorkspace.newInstance();var current=HeadlessWorkspace.newInstance();
        try {
            baseline.open(baselineRoot.resolve(f.model()).toString());current.open(currentRoot.resolve(f.model()).toString());
            int warm=quick?5:f.benchmarkWarm(),steps=quick?10:f.benchmarkSteps();
            // Warm the JVM independently, then re-create identical initial runs.
            timed(baseline,f,warm,steps);timed(current,f,warm,steps);
            for(int rep=1;rep<=repeats;rep++) {
                double oldTime,newTime;
                if(rep%2==1){oldTime=timed(baseline,f,warm,steps);newTime=timed(current,f,warm,steps);}
                else{newTime=timed(current,f,warm,steps);oldTime=timed(baseline,f,warm,steps);}
                state(baseline,current,contract(baselineRoot.resolve(f.model())),f.name()+" benchmark "+rep);
                if(!rng(baseline).equals(rng(current)))throw new AssertionError("Benchmark RNG mismatch "+f.name());
                csv.printf(Locale.ROOT,"%s,%d,%d,%d,%.9f,%.9f,%.6f%n",f.name(),rep,warm,steps,oldTime,newTime,oldTime/newTime);csv.flush();
                System.out.printf(Locale.ROOT,"BENCH %s repeat %d: baseline %.4fs, optimized %.4fs, %.3fx (identical final state/RNG).%n",f.name(),rep,oldTime,newTime,oldTime/newTime);
            }
        }finally{baseline.dispose();current.dispose();}
    }
    public static void main(String[] args)throws Exception {
        System.setProperty("java.awt.headless","true");Locale.setDefault(Locale.ROOT);
        Path baseline=Path.of(option(args,"--baseline",DEFAULT_BASELINE)).toAbsolutePath();
        Path current=Path.of(option(args,"--current",".")).toAbsolutePath();
        boolean quick=Arrays.asList(args).contains("--quick");
        String mode=option(args,"--mode","parity");
        Set<String> selected=new LinkedHashSet<>(Arrays.asList(option(args,"--cases",String.join(",",fixtures().stream().map(Fixture::name).toList())).split(",")));
        Set<String> known=new LinkedHashSet<>(fixtures().stream().map(Fixture::name).toList());
        if(!known.containsAll(selected))throw new IllegalArgumentException("Unknown fixtures: "+selected);
        Path output=Path.of(option(args,"--output","work/exact-optimization/"+mode+".csv")).toAbsolutePath();Files.createDirectories(output.getParent());
        try(var csv=new PrintWriter(Files.newBufferedWriter(output))) {
            csv.println(mode.equals("benchmark")?"fixture,repeat,warmup_steps,timed_steps,baseline_wall_s,optimized_wall_s,speedup":"fixture,steps,simulated_seconds,scalar_and_list_comparisons,exact_state_and_rng,csv_equal,wall_s");
            for(Fixture f:fixtures())if(selected.contains(f.name())) {
                if(mode.equals("parity"))parity(baseline,current,f,quick,output.getParent(),csv);
                else if(mode.equals("benchmark"))benchmark(baseline,current,f,quick,Integer.parseInt(option(args,"--repeats","3")),csv);
                else throw new IllegalArgumentException("Mode must be parity or benchmark");
            }
        }
        System.out.println("PASS: "+comparisons+" exact scalar/list comparisons; "+mode+" output "+output);
    }
}
