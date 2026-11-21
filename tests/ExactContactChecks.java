import java.nio.file.*;
import java.util.*;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Bitwise contact-only comparison against the saved pre-optimization source.
 * Usage: ExactContactChecks [--baseline directory] [--benchmark].
 * The comparison model copies baseline sources and replaces only contact.nls.
 */
public class ExactContactChecks {
    static int checks;
    static final String STATE="(list contact-step-count contact-solver-fallbacks contact-last-iterations contact-last-constraints "
        +"(map [cut -> (list [who] of item 0 cut ifelse-value (item 1 cut = false) [false] [[who] of item 1 cut] "
        +"item 2 cut item 3 cut item 4 cut item 5 cut)] contact-constraints) "
        +"(map [p -> [(list who xcor ycor wx wy vx vy ax ay next-x next-y next-vx next-vy next-wx next-wy "
        +"step-end-x step-end-y contact-dx contact-dy contact-original-dx contact-original-dy "
        +"contact-disk-cx contact-disk-cy contact-iteration-dx contact-iteration-dy contact-active?)] of p] sort pedestrians))";
    static void equal(Object a,Object b,String label) {
        if(a instanceof LogoList x && b instanceof LogoList y) {
            if(x.size()!=y.size())throw new AssertionError(label+" list size");
            for(int i=0;i<x.size();i++)equal(x.apply(i),y.apply(i),label+"/"+i);
        }else if(a instanceof Double x && b instanceof Double y) {
            if(Double.doubleToLongBits(x)!=Double.doubleToLongBits(y))throw new AssertionError(label+": "+x+" != "+y);
        }else if(!Objects.equals(a,b))throw new AssertionError(label+": "+a+" != "+b);
        checks++;
    }
    static void both(HeadlessWorkspace a,HeadlessWorkspace b,String command)throws Exception{a.command(command);b.command(command);}
    static void compare(HeadlessWorkspace a,HeadlessWorkspace b,String label)throws Exception{equal(a.report(STATE),b.report(STATE),label);}
    static String setup(boolean periodic) {
        return "set periodic? "+periodic+" set population 0 set spawn-rate 0 set dt .04 set simulation-seed 42 "
            +"set mean-desired-speed 1.34 set desired-speed-sd 0 set noise-strength 0 set run-checks? false setup "
            +"set wall-segments [] set potential-strength 0 set wall-strength 0";
    }
    static String person(double x,double y,double dx,double dy) {
        return "create-pedestrians 1 [initialize-pedestrian "+x+" "+y+" 49 6.5 1 set contact-dx "+dx+" set contact-dy "+dy+"] ";
    }
    static void discovery(HeadlessWorkspace a,HeadlessWorkspace b)throws Exception {
        var random=new Random(468132);
        for(boolean periodic:new boolean[]{false,true})for(int trial=0;trial<36;trial++) {
            var commands=new StringBuilder(setup(periodic));
            int count=trial<3?trial:3+random.nextInt(45);
            for(int i=0;i<count;i++) {
                double x=2+(i%10)*.57,y=2+(i/10)*.57;
                if(periodic && i==0)x=.26;
                if(periodic && i==1){x=49.74;y=2;}
                double scale=trial%6==0?110:.2;
                commands.append(' ').append(person(x,y,(random.nextDouble()-.5)*scale,(random.nextDouble()-.5)*scale));
            }
            commands.append(" contact-add-pair-cuts ").append(trial%2==0?"sort pedestrians":"reverse sort pedestrians");
            both(a,b,commands.toString());compare(a,b,"discovery "+periodic+"/"+trial);
            equal(a.report("n-values 8 [random-float 1]"),b.report("n-values 8 [random-float 1]"),"discovery RNG");
        }
        for(boolean periodic:new boolean[]{false,true}) {
            both(a,b,setup(periodic)+" "+person(periodic?.1:3,3,0,0)+person(periodic?49.9:3.2,3,0,0)
                +" carefully [contact-add-pair-cuts sort pedestrians] [set log-file error-message]");
            equal(a.report("log-file"),b.report("log-file"),"invalid initial overlap error");
            if(!a.report("log-file").toString().contains("start closer"))throw new AssertionError("Missing initial overlap error");
        }
        System.out.println("PASS: cached discovery exactly matches all cuts, order, active flags and RNG; invalid initial states still fail.");
    }
    static void integration(HeadlessWorkspace a,HeadlessWorkspace b)throws Exception {
        String[] fixtures={
            "create-pedestrians 1 [initialize-pedestrian 24.74 6.5 40 6.5 1 set wx 1.34 set vx wx] "
                +"create-pedestrians 1 [initialize-pedestrian 25.26 6.5 10 6.5 -1 set wx -1.34 set vx wx]",
            "set wall-segments [[25.72 1.5 25.72 11.5]] "
                +"create-pedestrians 3 [initialize-pedestrian (24.48 + .52 * who) 6.5 49 6.5 1 set wx 1.34 set vx wx]",
            "create-pedestrians 3 [initialize-pedestrian (24.48 + .52 * who) 6.5 49 6.5 1 set wx 1.34 set vx wx] "
                +"ask last sort pedestrians [set stop-x 25.52 set wx 0 set vx 0]",
            "set active-periodic? true create-pedestrians 1 [initialize-pedestrian 49.74 6.5 50 6.5 1 set wx 1.34 set vx wx] "
                +"create-pedestrians 1 [initialize-pedestrian .26 6.5 0 6.5 -1 set wx -1.34 set vx wx]"
        };
        for(int k=0;k<fixtures.length;k++) {
            both(a,b,setup(false)+" "+fixtures[k]);
            for(int step=0;step<30;step++){both(a,b,"ped-step");compare(a,b,"integration "+k+"/"+step);}
            equal(a.report("n-values 16 [random-float 1]"),b.report("n-values 16 [random-float 1]"),"integration RNG");
        }
        System.out.println("PASS: full contact corrections, buffers, projection iterations and future RNG match bit-for-bit.");
    }
    static double timed(HeadlessWorkspace w,int repeats)throws Exception {
        long started=System.nanoTime();
        w.command("repeat "+repeats+" [set contact-constraints [] contact-add-pair-cuts sort pedestrians]");
        return (System.nanoTime()-started)/1e9;
    }
    static void benchmark(HeadlessWorkspace a,HeadlessWorkspace b)throws Exception {
        both(a,b,"set periodic? true set population 150 set spawn-rate 0 set simulation-seed 42 set run-checks? false setup "
            +"foreach sort pedestrians [p -> ask p [set contact-dx .02 * travel-direction set contact-dy 0]]");
        timed(a,20);timed(b,20);
        double[] old=new double[3],now=new double[3];
        for(int i=0;i<3;i++){if(i%2==0){old[i]=timed(a,100);now[i]=timed(b,100);}else{now[i]=timed(b,100);old[i]=timed(a,100);}}
        Arrays.sort(old);Arrays.sort(now);
        compare(a,b,"benchmark final state");
        System.out.printf(Locale.ROOT,"CONTACT_BENCH 150 people x100 discovery passes: baseline %.6fs optimized %.6fs speedup %.3fx%n",old[1],now[1],old[1]/now[1]);
    }
    public static void main(String[] args)throws Exception {
        Path baseline=Path.of("work/exact-optimization-baseline");
        for(int i=0;i<args.length-1;i++)if(args[i].equals("--baseline"))baseline=Path.of(args[i+1]);
        Path candidate=Path.of("work/exact-contact-candidate");
        Files.createDirectories(candidate.resolve("core"));
        Files.copy(baseline.resolve("Pedestrians.nlogox"),candidate.resolve("Pedestrians.nlogox"),StandardCopyOption.REPLACE_EXISTING);
        try(var paths=Files.list(baseline.resolve("core"))) {
            for(Path p:paths.toList())if(p.toString().endsWith(".nls"))Files.copy(p,candidate.resolve("core").resolve(p.getFileName()),StandardCopyOption.REPLACE_EXISTING);
        }
        Files.copy(Path.of("core/contact.nls"),candidate.resolve("core/contact.nls"),StandardCopyOption.REPLACE_EXISTING);
        var a=HeadlessWorkspace.newInstance();var b=HeadlessWorkspace.newInstance();
        try {
            a.open(baseline.resolve("Pedestrians.nlogox").toAbsolutePath().toString());
            b.open(candidate.resolve("Pedestrians.nlogox").toAbsolutePath().toString());
            discovery(a,b);integration(a,b);
            if(Arrays.asList(args).contains("--benchmark"))benchmark(a,b);
            System.out.println("PASS: "+checks+" exact-contact comparisons.");
        }finally{a.dispose();b.dispose();}
    }
}
