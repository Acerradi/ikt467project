import java.nio.file.Path;
import java.util.*;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Exact before/after diagnostic parity, including long paths and periodic seams.
 * Optional first argument: directory containing the pristine baseline models.
 */
public class ExactDiagnosticsChecks {
    static int comparisons;
    static final String REPORTERS="(list pass-through-pairs swept-min-pair-separation min-pair-separation overlap-pair-count "
        +"min-wall-clearance wall-penetration-count speed-cap-violation-count above-desired-speed-count "
        +"pass-through-events worst-min-separation worst-wall-clearance speed-cap-violations)";
    static void equal(Object a,Object b,String where) {
        if(a instanceof Number x && b instanceof Number y) {
            if(Double.doubleToLongBits(x.doubleValue())!=Double.doubleToLongBits(y.doubleValue()))
                throw new AssertionError(where+": "+a+" != "+b);
            comparisons++;
        } else if(a instanceof LogoList x && b instanceof LogoList y) {
            if(x.size()!=y.size())throw new AssertionError(where+": list lengths differ");
            for(int i=0;i<x.size();i++)equal(x.apply(i),y.apply(i),where+"["+i+"]");
        } else if(!Objects.equals(a,b))throw new AssertionError(where+": "+a+" != "+b);
    }
    static void command(HeadlessWorkspace a,HeadlessWorkspace b,String code)throws Exception {
        a.command(code);b.command(code);
    }
    static String paths(Random random,int count,int trial) {
        var rows=new ArrayList<String>();
        for(int i=0;i<count;i++) {
            double x=random.nextDouble()*50,y=random.nextDouble()*10;
            double scale=trial%7==0?180:.08;
            rows.add("["+(3*i)+" "+x+" "+y+" "+(x+(random.nextDouble()-.5)*scale)+" "+(y+(random.nextDouble()-.5)*scale)+"]");
        }
        Collections.shuffle(rows,random); // Manual callers need not supply sorted paths.
        return "["+String.join(" ",rows)+"]";
    }
    static double time(HeadlessWorkspace w,String code)throws Exception {
        long start=System.nanoTime();w.command(code);return (System.nanoTime()-start)/1e9;
    }
    public static void main(String[] args)throws Exception {
        String baseline=args.length>0?args[0]:"work/exact-optimization-baseline";
        var before=HeadlessWorkspace.newInstance();var after=HeadlessWorkspace.newInstance();
        try {
            before.open(Path.of(baseline,"Pedestrians.nlogox").toAbsolutePath().toString());
            after.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            var random=new Random(20261113);
            for(boolean periodic:new boolean[]{false,true}) {
                command(before,after,"set periodic? true set population 0 set hall-width 10 set dt .02 set run-checks? false "
                    +"set simulation-seed 42 setup set active-periodic? "+periodic);
                for(int trial=0;trial<60;trial++) {
                    int count=trial<3?trial:3+random.nextInt(68);
                    var make=new StringBuilder("clear-turtles ");
                    for(int i=0;i<count;i++) {
                        double x=.1+random.nextDouble()*49.8,y=2+random.nextDouble()*9;
                        make.append("create-pedestrians 1 [initialize-pedestrian ").append(x).append(' ').append(y)
                            .append(" 49 6 1] ");
                    }
                    command(before,after,make+"set last-step-paths "+paths(random,count,trial));
                    equal(before.report(REPORTERS),after.report(REPORTERS),"reporters periodic="+periodic+" trial="+trial);
                    command(before,after,"checks-reset checks-update checks-update");
                    equal(before.report(REPORTERS),after.report(REPORTERS),"accumulators "+trial);
                }
                equal(before.report("n-values 32 [random-float 1]"),after.report("n-values 32 [random-float 1]"),"future RNG");
            }
            System.out.println("PASS: "+comparisons+" bitwise diagnostic/RNG comparisons across 120 synthetic states and path sets.");
            if(Arrays.asList(args).contains("--benchmark")) {
                command(before,after,"set periodic? true set population 150 set run-checks? false setup go");
                command(before,after,"repeat 20 [checks-update]");
                var oldTimes=new ArrayList<Double>();var newTimes=new ArrayList<Double>();
                for(int i=0;i<3;i++) {
                    if(i%2==0) {oldTimes.add(time(before,"repeat 100 [checks-update]"));newTimes.add(time(after,"repeat 100 [checks-update]"));}
                    else {newTimes.add(time(after,"repeat 100 [checks-update]"));oldTimes.add(time(before,"repeat 100 [checks-update]"));}
                }
                Collections.sort(oldTimes);Collections.sort(newTimes);
                System.out.printf(Locale.ROOT,"Diagnostic benchmark medians: baseline %.4fs, optimized %.4fs, %.2fx.%n",
                    oldTimes.get(1),newTimes.get(1),oldTimes.get(1)/newTimes.get(1));
            }
        }finally{before.dispose();after.dispose();}
    }
}
