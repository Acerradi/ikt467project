import java.nio.file.Path;
import java.util.Locale;
import org.nlogo.headless.HeadlessWorkspace;

/** Admission/clearance checks for the optional crossing controller. */
public class CrossingControlChecks {
  static int assertions;
  static void ok(boolean test,String why){assertions++;if(!test)throw new AssertionError(why);}
  static double n(HeadlessWorkspace w,String expression)throws Exception{return ((Number)w.report(expression)).doubleValue();}
  static void setup(HeadlessWorkspace w)throws Exception{
    w.command("set crossing-yielding? true set horizontal-rate 0 set vertical-rate 0 set two-way? false set corridor-width 4 set dt .02 set mean-desired-speed 1.55 set desired-speed-sd 0 set show-trails? false set run-checks? false set log-csv? false set simulation-seed 42 setup");
  }
  static String person(boolean vertical,int sign,double along){
    double x=vertical?15:along,y=vertical?along:15,gx=vertical?15:sign>0?29:1,gy=vertical?sign>0?29:1:15;
    return String.format(Locale.ROOT,"create-pedestrians 1 [initialize-pedestrian %.12g %.12g %.12g %.12g %d set vertical? %s set crossing-admitted? false set crossing-cleared? false set free-time 20] set spawned-count spawned-count + 1 ",x,y,gx,gy,sign,vertical);
  }
  static void invariant(HeadlessWorkspace w)throws Exception{
    double h=n(w,"count pedestrians with [crossing-admitted? and not crossing-cleared? and not vertical?]"),v=n(w,"count pedestrians with [crossing-admitted? and not crossing-cleared? and vertical?]");
    ok(h==0||v==0,"Both conflicting streams admitted before clearance");
    ok(n(w,"count pedestrians with [(is-number? stop-x and travel-direction * (xcor - stop-x) > 1e-8) or (is-number? stop-y and travel-direction * (ycor - stop-y) > 1e-8)]")==0,"Closed gate crossed");
  }
  public static void main(String[] args)throws Exception{
    Locale.setDefault(Locale.ROOT);var w=HeadlessWorkspace.newInstance();
    try{
      w.open(Path.of(args.length>0?args[0]:".","scenarios/Crossing.nlogox").toAbsolutePath().toString());
      for(boolean vertical:new boolean[]{false,true})for(int sign:new int[]{1,-1}){
        setup(w);w.command("set crossing-green-vertical? "+!vertical);
        double gate=sign>0?12.2:17.8;
        w.command(person(vertical,sign,gate-sign*.05));
        for(int i=0;i<100;i++){w.command("go");invariant(w);}
        String axis=vertical?"ycor":"xcor";
        ok(n(w,"count pedestrians with [crossing-admitted?]")==0,"Holding at gate counted as admission");
        ok(sign*(n(w,"["+axis+"] of one-of pedestrians")-gate)<=1e-8,"Stopped pedestrian overshot gate");
        w.command("set crossing-green-vertical? "+vertical);
        for(int i=0;i<50;i++){w.command("go");invariant(w);}
        ok(n(w,"count pedestrians with [crossing-admitted?]")==1,"Open gate never admitted walker");
        w.command("set crossing-draining? true");
        boolean sawWaiting=false;
        for(int i=0;i<1000&&n(w,"crossing-phase-count")==0;i++){
          if(n(w,"crossing-unfinished-count")>0)sawWaiting=true;
          w.command("go");invariant(w);
          ok(n(w,"count pedestrians with [crossing-admitted? and (is-number? stop-x or is-number? stop-y)]")==0,"Closing gate froze an admitted pedestrian");
        }
        ok(sawWaiting,"Fixture did not exercise clearance waiting");
        ok(n(w,"crossing-phase-count")==1,"Drain never opened next stream");
        ok(n(w,"item 3 first crossing-switch-log")==0,"Phase switched with a body still in conflict region");
      }
      setup(w);w.command("set crossing-phase-duration .1 repeat 30 [go]");
      ok(n(w,"crossing-phase-count")>=4,"Finite empty phases did not alternate fairly");
      ok(Boolean.FALSE.equals(w.report("pair-contact-enabled?")),"Controller enabled pair enforcement");
      System.out.println("PASS "+assertions+" crossing control checks (both axes and travel signs).");
    }finally{w.dispose();}
  }
}
