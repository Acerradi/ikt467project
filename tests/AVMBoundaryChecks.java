import java.nio.file.Path;
import javax.xml.parsers.DocumentBuilderFactory;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.nlogo.headless.HeadlessWorkspace;

/** Boundary geometry and diagnostics tests; no crowd-performance assertions. */
public class AVMBoundaryChecks {
    static int count;
    static void ok(boolean condition, String message) {
        count++; if (!condition) throw new AssertionError(message);
    }
    static double n(HeadlessWorkspace w, String s) throws Exception {
        return ((Number) w.report(s)).doubleValue();
    }
    static void close(HeadlessWorkspace w, String s, double expected) throws Exception {
        double actual=n(w,s); ok(Math.abs(actual-expected)<2e-9,s+": "+actual+" expected "+expected);
    }
    static void fails(HeadlessWorkspace w, String command) throws Exception {
        boolean failed=false; try {w.command(command);} catch(Exception e){failed=true;}
        ok(failed,"Expected diagnostic error: "+command);
    }
    // NetLogo 7.0.4 SliderWidget.load uses display as setVarName's input;
    // headless compilation uses variable. A label-only rename can therefore
    // pass every headless test while leaving the GUI without the expected global.
    static void validateSliderNames(Document model, String source) {
        for(String kind:new String[]{"slider","switch"}) {
          var sliders=model.getElementsByTagName(kind);
          for(int i=0;i<sliders.getLength();i++) {
            Element slider=(Element)sliders.item(i);
            String variable=slider.getAttribute("variable");
            String display=slider.getAttribute("display");
            if(variable.isBlank()||!variable.equals(display)) {
                throw new AssertionError(source+": "+kind+" GUI name '"+display
                    +"' must match variable '"+variable+"'");
            }
          }
        }
    }
    static void guiSliderNames(Path path) throws Exception {
        var factory=DocumentBuilderFactory.newInstance();
        factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl",true);
        Document model=factory.newDocumentBuilder().parse(path.toFile());
        validateSliderNames(model,path.toString());
        count++;
        // Recreate the reported defect in memory only. Production XML is never
        // modified, and the guard must reject what headless compilation accepted.
        Element timestep=null;
        var sliders=model.getElementsByTagName("slider");
        for(int i=0;i<sliders.getLength();i++) {
            Element slider=(Element)sliders.item(i);
            if(slider.getAttribute("variable").equals("dt"))timestep=slider;
        }
        ok(timestep!=null,path+": missing dt slider");
        timestep.setAttribute("display","outer dt");
        boolean rejected=false;
        try {validateSliderNames(model,path.toString());}
        catch(AssertionError expected){rejected=expected.getMessage().contains("outer dt");}
        ok(rejected,path+": GUI-name guard missed the old outer dt / dt mismatch");
    }
    public static void main(String[] args) throws Exception {
        Path root=Path.of(args.length>0?args[0]:".").toAbsolutePath();
        for(String file:new String[]{"Pedestrians.nlogox","scenarios/Door.nlogox",
                "scenarios/TrafficLight.nlogox","scenarios/Crossing.nlogox"}) {
            guiSliderNames(root.resolve(file));
        }
        HeadlessWorkspace w=HeadlessWorkspace.newInstance();
        try {
            w.open(root.resolve("Pedestrians.nlogox").toString());
            w.command("set periodic? false set population 0 set spawn-rate 0 setup");
            close(w,"body-radius",.2);
            close(w,"avm-time-gap",.6);
            // Analytical geometry below deliberately keeps its original .25 m
            // fixture radius; this is independent of the scenario default above.
            w.command("set body-radius .25 set wall-segments [[5 0 5 10]]");
            close(w,"avm-boundary-speed 4 5 1 0 2 1 false 1",.75);
            close(w,"avm-boundary-speed 4.75 5 -1 0 2 1 false 1",2);
            close(w,"avm-boundary-speed 4.75 5 1 0 2 1 false 1",0);
            close(w,"item 0 (avm-wall-heading 4.7 5 1 0)",-1);
            close(w,"item 1 (avm-wall-heading 4.7 5 1 0)",0);
            close(w,"item 0 (avm-wall-heading 4.6 5 1 0)",1);
            close(w,"item 0 (avm-wall-heading 4.7 5 (sqrt .5) (sqrt .5))",-.3/Math.sqrt(.09+.5));
            close(w,"item 1 (avm-wall-heading 4.7 5 (sqrt .5) (sqrt .5))",Math.sqrt(.5)/Math.sqrt(.09+.5));
            w.command("set wall-segments [[5 5 5 5]]");
            close(w,"avm-boundary-speed 4 5 1 0 2 1 false 1",.75);
            close(w,"avm-boundary-speed 4 5.25 1 0 2 1 false 1",2);
            close(w,"avm-boundary-speed 4 4 (sqrt .5) (sqrt .5) 2 1 false 1",Math.sqrt(2)-.25);
            w.command("set wall-segments [[5 0 5 10] [0 5 10 5]]");
            close(w,"avm-boundary-speed 4 4 (sqrt .5) (sqrt .5) 2 1 false 1",.75/Math.sqrt(.5));
            w.command("set wall-segments []");
            close(w,"avm-boundary-speed 4 5 1 0 2 .02 4.6 1",1);
            close(w,"avm-boundary-speed 4 5 1 0 2 1 4.6 1",.6);
            close(w,"avm-boundary-speed 4.6 5 1 0 2 .02 4.6 1",0);
            close(w,"avm-boundary-speed 5 5 -1 0 2 1 4.4 -1",.6);
            // Vertical yielding gates use exactly the established horizontal
            // stop law, including negative travel and large-step crossings.
            for(int dir:new int[]{-1,1}) for(double gap:new double[]{0,.01,.6,2})
              for(double h:new double[]{.02,1}) for(double component:new double[]{-1,0,.6,1}) {
                double stop=5+dir*gap, heading=dir*component;
                double expected=Math.min(2,gap/.6);
                if(component>0)expected=Math.min(expected,gap/(h*component));
                String axis="avm-axis-stop-speed 5 "+heading+" 2 "+h+" "+stop+" "+dir;
                close(w,axis,expected);
                double old=n(w,"avm-boundary-speed 5 5 "+heading+" 0 2 "+h+" "+stop+" "+dir);
                ok(Double.doubleToLongBits(n(w,axis))==Double.doubleToLongBits(old),"Axis stop law differs from existing x rule");
              }
            w.command("create-pedestrians 1 [initialize-pedestrian 10 6.5 10 11.5 1 set stop-y 7] repeat 200 [ped-step]");
            ok(n(w,"[ycor] of one-of pedestrians")<=7,"Closed y gate overshot");
            close(w,"[xcor] of one-of pedestrians",10);
            w.command("ask pedestrians [set stop-y false] repeat 50 [ped-step]");
            ok(n(w,"[ycor] of one-of pedestrians")>7.5,"Opening y gate did not release pedestrian");
            w.command("ask pedestrians [die] create-pedestrians 1 [initialize-pedestrian 10 10 10 2 -1 set stop-y 8] repeat 200 [ped-step]");
            ok(n(w,"[ycor] of one-of pedestrians")>=8,"Negative-direction y gate overshot");
            w.command("ask pedestrians [die]");
            // A periodic seam does not turn a full-width horizontal wall into a post.
            w.command("set active-periodic? true set hall-length 50 set wall-segments [[0 1.5 50 1.5]]");
            close(w,"avm-boundary-speed 49.9 1.8 1 0 2 1 false 1",2);
            close(w,"item 0 (avm-wall-heading 49.9 1.8 1 0)",1);
            close(w,"avm-boundary-speed 49.9 1.8 0 -1 2 1 false 1",.05);
            close(w,"distance-to-segment 51 1.8 [0 1.5 50 1.5]",.3);
            // AVM diagnostics report unsafe traces; they do not change them.
            w.command("set active-periodic? false set wall-segments [] checks-begin-step "
                +"set last-internal-step-paths (list "
                +"(list 0 .01 [[0 10 6.5 12 6.5] [1 12 6.5 10 6.5]] [[0 12 6.5 1 0] [1 10 6.5 -1 0]]) "
                +"(list .01 .01 [[0 12 6.5 13 6.5] [1 10 6.5 9 6.5]] [[0 13 6.5 1 0] [1 9 6.5 -1 0]])) "
                +"set last-step-paths item 2 last last-internal-step-paths checks-update checks-update");
            close(w,"worst-min-separation",0);
            close(w,"pass-through-events",1);
            close(w,"checks-segments-processed",2);
            w.command("check-invariants");
            count++; // Body overlap is measured, never used to halt this experiment.
            close(w,"item 3 first item 2 first last-internal-step-paths",12);
            w.command("checks-begin-step create-pedestrians 1 [initialize-pedestrian 10 6.5 49 6.5 1 set vx desired-speed + .1]");
            close(w,"speed-cap-violation-count",1);
            fails(w,"check-invariants");
        } finally {w.dispose();}
        for(String file:new String[]{"Door","TrafficLight","Crossing"}) {
            w=HeadlessWorkspace.newInstance();
            try {
                w.open(root.resolve("scenarios/"+file+".nlogox").toString());
                String settings=file.equals("Crossing")?"set horizontal-rate 0 set vertical-rate 0"
                    :"set initial-per-side 0 set spawn-rate 0";
                w.command("set log-csv? false "+settings+" setup go check-invariants");
                count++;
                close(w,"body-radius",.2);
                if(file.equals("Door")) {
                    w.command("add-pedestrian 3 3 1");
                    close(w,"[move-ex - ex] of one-of pedestrians",0);
                    close(w,"[move-ey - ey] of one-of pedestrians",0);
                    w.command("ask pedestrians [die] checks-begin-step "
                        +"set last-internal-step-paths (list "
                        +"(list 0 .01 [[0 9 4.5 11 4.5]] []) "
                        +"(list .01 .01 [[0 11 4.5 9 4.5]] [])) track-door");
                    close(w,"door-passes",2);
                    close(w,"item 0 door-sequence",1);
                    close(w,"item 1 door-sequence",-1);
                }
            } finally {w.dispose();}
        }
        System.out.println("AVM GUI-name/boundary/diagnostics checks passed: "+count);
        System.exit(0);
    }
}
