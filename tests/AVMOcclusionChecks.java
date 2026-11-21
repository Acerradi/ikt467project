import java.awt.geom.Line2D;
import java.nio.file.Path;
import java.util.Random;
import javax.xml.parsers.DocumentBuilderFactory;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;
import org.w3c.dom.Element;

/** Literal-wall visibility checks, independent of body contact and AVM force formulas.
 * Run with NetLogo's Java/runtime jar, optionally passing the model root.
 * No production files are modified. Current scenario radius is .20 m.
 */
public class AVMOcclusionChecks {
    static int checks;
    static void ok(boolean condition,String message) {
        checks++;
        if(!condition) throw new AssertionError(message);
    }
    static void near(HeadlessWorkspace w,String expression,double expected)throws Exception {
        double actual=((Number)w.report(expression)).doubleValue();
        ok(Math.abs(actual-expected)<2e-10,expression+": "+actual+" expected "+expected);
    }
    static void truth(HeadlessWorkspace w,String expression,boolean expected)throws Exception {
        Object actual=w.report(expression);
        ok(Boolean.valueOf(expected).equals(actual),expression+": "+actual+" expected "+expected);
    }
    static void clear(HeadlessWorkspace w,double ax,double ay,double bx,double by,boolean expected)throws Exception {
        String coordinates=ax+" "+ay+" "+bx+" "+by;
        truth(w,"avm-clear-line? "+coordinates,expected);
        truth(w,"avm-clear-line? "+bx+" "+by+" "+ax+" "+ay,expected);
    }
    static void ids(HeadlessWorkspace w,String expression,int... expected)throws Exception {
        LogoList actual=(LogoList)w.report("map [row -> item 0 row] "+expression);
        ok(actual.size()==expected.length,"Unexpected visible-neighbour count: "+actual);
        for(int i=0;i<expected.length;i++)
            ok(((Number)actual.apply(i)).intValue()==expected[i],"Neighbour order/identity: "+actual);
    }
    static void guiNames(Path model)throws Exception {
        var factory=DocumentBuilderFactory.newInstance();
        factory.setFeature("http://apache.org/xml/features/disallow-doctype-decl",true);
        var xml=factory.newDocumentBuilder().parse(model.toFile());
        var sliders=xml.getElementsByTagName("slider");
        for(int i=0;i<sliders.getLength();i++) {
            Element e=(Element)sliders.item(i);
            ok(!e.getAttribute("variable").isBlank()&&e.getAttribute("variable").equals(e.getAttribute("display")),
                model+": GUI slider name differs from compiler variable: "+e.getAttribute("display"));
        }
    }
    static void geometry(HeadlessWorkspace w)throws Exception {
        w.command("set active-periodic? false set wall-segments []");
        clear(w,0,0,10,10,true);
        w.command("set wall-segments [[5 0 5 10]]");
        clear(w,0,5,10,5,false);       // crosses the wall interior
        clear(w,0,0,10,0,false);       // grazes an endpoint
        clear(w,0,-.00001,10,-.00001,true);
        clear(w,0,5,4,5,true);         // wall beyond the sight segment
        clear(w,6,5,10,5,true);        // wall behind the sight segment
        clear(w,4,1,4,9,true);         // parallel, separated lines
        clear(w,5,2,5,8,false);        // collinear overlap
        clear(w,5,-2,5,-1,true);       // collinear but disjoint
        clear(w,5,-2,5,0,false);       // collinear endpoint touch
        clear(w,5,-2,5,12,false);      // sight segment contains entire wall
        clear(w,0,5,5,5,false);        // sight ends on wall interior
        clear(w,4,5,4,5,true);         // stationary point away from wall
        clear(w,5,5,5,5,false);        // stationary point on wall
        w.command("set wall-segments [[5 5 5 5]]");
        clear(w,0,5,10,5,false);       // zero-length wall point
        clear(w,0,5.00001,10,5.00001,true);
        w.command("set wall-segments [[5 0 5 4] [5 6 5 10]]");
        clear(w,0,5,10,5,true);        // open doorway
        clear(w,0,4,10,4,false);       // closed door-post endpoint
        clear(w,0,4.00001,10,4.00001,true);
        clear(w,0,4.1,10,4.1,true);    // no radius/buffer inflation in visibility

        // JDK geometry is an independent oracle, not a copy of the production
        // intersection reporter. Integer inputs avoid near-zero ambiguity.
        Random random=new Random(4672026);
        for(int trial=0;trial<80;trial++) {
            int ax=random.nextInt(41)-20,ay=random.nextInt(41)-20;
            int bx=random.nextInt(41)-20,by=random.nextInt(41)-20;
            int cx=random.nextInt(41)-20,cy=random.nextInt(41)-20;
            int dx=random.nextInt(41)-20,dy=random.nextInt(41)-20;
            if((ax==bx&&ay==by)||(cx==dx&&cy==dy)) {trial--;continue;}
            w.command("set wall-segments [["+cx+" "+cy+" "+dx+" "+dy+"]]");
            clear(w,ax,ay,bx,by,!Line2D.linesIntersect(ax,ay,bx,by,cx,cy,dx,dy));
        }
    }
    static void filtering(HeadlessWorkspace w)throws Exception {
        // Rows: [who x y vx vy movement-x movement-y desired-x desired-y v0].
        String a="[1 4.7 5 1.55 0 1 0 1 0 1.55]";
        String b="[2 5.3 5 -1.55 0 -1 0 -1 0 1.55]";
        String behind="[3 3.7 5 0 0 1 0 1 0 1.55]";
        String side="[4 4.7 6 0 0 0 1 0 1 1.55]";
        String all="(list "+a+" "+b+" "+behind+" "+side+")";
        String visible="(avm-unoccluded-neighbors "+a+" "+all+")";
        w.command("set active-periodic? false set wall-segments [[5 0 5 10]]");
        ids(w,visible,1,3,4);          // self retained; no extra FOV filtering
        near(w,"avm-speed "+a+" "+all+" 1 0",(.6-.4)/.6);
        near(w,"avm-speed "+a+" "+visible+" 1 0",1.55);
        near(w,"length (avm-tie-neighbors "+a+" "+all+")",1);
        near(w,"length (avm-tie-neighbors "+a+" "+visible+")",0);
        near(w,"item 1 (avm-influence "+a+" "+b+" 1)",12);
        near(w,"item 0 (avm-steering "+a+" "+visible+" .02 [])",1);
        near(w,"item 1 (avm-steering "+a+" "+visible+" .02 [])",0);
        // Pure kernel is intentionally geometry-independent: only its input
        // neighbour list changes. Removing the wall restores the same formulas.
        w.command("set wall-segments [[5 0 5 4] [5 6 5 10]]");
        ids(w,visible,1,2,3,4);
        near(w,"avm-speed "+a+" "+visible+" 1 0",(.6-.4)/.6);
        near(w,"length (avm-tie-neighbors "+a+" "+visible+")",1);
        truth(w,"(avm-steering "+a+" "+visible+" .02 [[2 1]]) = (avm-steering "+a+" "+all+" .02 [[2 1]])",true);
    }
    static void periodic(HeadlessWorkspace w)throws Exception {
        w.command("set active-periodic? true set hall-length 50 set wall-segments [[0 0 0 10]]");
        clear(w,49,5,51,5,false);       // translated wall at x=50
        clear(w,-1,5,1,5,false);       // base wall at x=0
        clear(w,48,5,49,5,true);
        clear(w,49.9,5,49.99,5,true);   // no body-radius inflation
        String a="[1 49.7 5 1.55 0 1 0 1 0 1.55]";
        String b="[2 .3 5 -1.55 0 -1 0 -1 0 1.55]";
        String all="(list "+a+" "+b+")";
        ids(w,"(avm-unoccluded-neighbors "+a+" "+all+")",1);
        w.command("set wall-segments []");
        ids(w,"(avm-unoccluded-neighbors "+a+" "+all+")",1,2);
        near(w,"avm-speed "+a+" "+all+" 1 0",(.6-.4)/.6);
        w.command("set wall-segments [[0 1.5 50 1.5]]");
        clear(w,49,2,51,2,true);        // no artificial seam post
        clear(w,49,2,51,1,false);       // wall still crosses the seam
        clear(w,-1,2,1,1,false);
        String above="[3 .3 2 0 0 1 0 1 0 1.55]";
        String own="[1 49.7 2 0 0 1 0 1 0 1.55]";
        String below="[2 .3 1 0 0 1 0 1 0 1.55]";
        ids(w,"(avm-unoccluded-neighbors "+own+" (list "+own+" "+below+" "+above+"))",1,3);
    }
    public static void main(String[] args)throws Exception {
        Path root=Path.of(args.length>0?args[0]:".").toAbsolutePath();
        for(String name:new String[]{"Pedestrians.nlogox","scenarios/Door.nlogox","scenarios/TrafficLight.nlogox","scenarios/Crossing.nlogox"}) {
            guiNames(root.resolve(name));
            HeadlessWorkspace w=HeadlessWorkspace.newInstance();
            try {
                w.open(root.resolve(name).toString());
                String settings=name.equals("Pedestrians.nlogox")?"set periodic? false set population 0 set spawn-rate 0":
                    name.contains("Crossing")?"set log-csv? false set horizontal-rate 0 set vertical-rate 0":
                    "set log-csv? false set initial-per-side 0 set spawn-rate 0";
                w.command(settings+" setup go");
                near(w,"body-radius",.2);
                truth(w,"pair-contact-enabled?",false);
                if(name.equals("Pedestrians.nlogox")) {geometry(w);filtering(w);periodic(w);}
            } finally {w.dispose();}
        }
        System.out.println("PASS "+checks+" AVM wall-occlusion, kernel-input, periodic, and GUI/startup checks.");
    }
}
