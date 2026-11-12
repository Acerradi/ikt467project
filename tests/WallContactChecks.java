import java.nio.file.Path;
import org.nlogo.headless.HeadlessWorkspace;

/** Independent swept-geometry regressions for hard walls and stop lines. */
public class WallContactChecks {
    private static HeadlessWorkspace world;
    private static int checks;

    private static double number(String expression) throws Exception {
        return ((Number) world.report(expression)).doubleValue();
    }
    private static double own(String expression) throws Exception {
        return number("[" + expression + "] of one-of pedestrians");
    }
    private static void require(boolean condition, String message) {
        if (!condition) throw new AssertionError(message);
        checks++;
    }
    private static void close(double actual, double expected, String label) {
        require(Double.isFinite(actual) && Math.abs(actual - expected) < 1e-10,
            label + ": expected " + expected + ", got " + actual);
    }
    private static void capCheck() throws Exception {
        double wx=own("next-wx"), wy=own("next-wy");
        double factor=Math.min(1, 1.3*own("desired-speed")/Math.max(1e-300,Math.hypot(wx,wy)));
        close(own("next-vx"),wx*factor,"Capped x velocity");
        close(own("next-vy"),wy*factor,"Capped y velocity");
    }
    private static double pointSegment(double px,double py,double ax,double ay,double bx,double by) {
        double dx=bx-ax,dy=by-ay;
        double norm=dx*dx+dy*dy;
        double fraction=norm==0?0:Math.max(0,Math.min(1,((px-ax)*dx+(py-ay)*dy)/norm));
        return Math.hypot(px-ax-fraction*dx,py-ay-fraction*dy);
    }
    private static double sweptPointDistance(double px,double py) throws Exception {
        return pointSegment(px,py,own("previous-x"),own("previous-y"),own("step-end-x"),own("step-end-y"));
    }
    private static void fixture(String walls,String state,double dt) throws Exception {
        world.command("set initial-per-side 0 set spawn-rate 0 set dt "+dt+" setup "
            +"add-pedestrian 5 4.5 1 set wall-segments "+walls+" ask pedestrians ["
            +"set desired-speed 1.34 set ax 0 set ay 0 set stop-x false "+state
            +" set vx wx set vy wy calculate-next-state]");
    }
    private static void corner(String walls) throws Exception {
        fixture(walls,"setxy 0.5 0.21 set wx -1.7 set wy -0.34",.02);
        close(own("step-end-x"),.2+.2*Math.sqrt(2),"Joint corner x");
        close(own("step-end-y"),.2,"Joint corner y");
        require(own("step-end-y")>=.2-1e-10,"Re-entered an earlier horizontal wall");
        require((own("step-end-x")-own("step-end-y"))/Math.sqrt(2)>=.2-1e-10,"Entered oblique wall");
        capCheck();
    }
    public static void main(String[] args) throws Exception {
        world=HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("scenarios/Door.nlogox").toAbsolutePath().toString());
            // Both original endpoints were outside the post; the old straight
            // step grazed through its exclusion disk by 0.5 mm at default dt.
            fixture("[[10 0 10 4]]","setxy 9.983 4.1995 set wx 1.7 set wy 0",.02);
            require(sweptPointDistance(10,4)>=.2-1e-10,"Swept endpoint grazing penetration");
            capCheck();
            // A longer step crosses a post even though its final point is clear.
            fixture("[[10 4 10 4]]","setxy 9.7 4.1 set wx 1.7 set wy 0",.5);
            require(sweptPointDistance(10,4)>=.2-1e-10,"Entire post was crossed between ticks");
            capCheck();
            // Resolve the intersection jointly, independent of segment order
            // and orientation, with valid initially uncapped speed (1.734 m/s).
            corner("[[0 0 20 0] [0 0 20 20]]");
            corner("[[0 0 20 20] [0 0 20 0]]");
            corner("[[20 0 0 0] [20 20 0 0]]");
            // Stop-line clamping recaps preferred velocity and keeps tangential
            // movement for the whole step, just like straight-wall contact.
            fixture("[]","setxy 9.99 4.5 set wx 2 set wy 0.6 set travel-direction 1 set stop-x 10",.02);
            close(own("step-end-x"),10,"Right-moving stop line");
            close(own("next-wx"),0,"Forbidden preferred velocity at stop line");
            close(own("step-end-y"),4.512,"Tangential stop-line movement");
            capCheck();
            fixture("[]","setxy 10.01 4.5 set wx -2 set wy 0.6 set travel-direction -1 set stop-x 10",.02);
            close(own("step-end-x"),10,"Left-moving stop line");
            close(own("next-wx"),0,"Opposite forbidden preferred velocity");
            capCheck();
        } finally {world.dispose();}
        world=HeadlessWorkspace.newInstance();
        try {
            world.open(Path.of("Pedestrians.nlogox").toAbsolutePath().toString());
            world.command("set periodic? true set population 1 set dt 0.02 setup "
                +"ask pedestrians [setxy 49.99 1.71 set desired-speed 1.34 set wx 100 set wy -100 "
                +"set ax 0 set ay 0 calculate-next-state]");
            close(own("step-end-x"),50.02484,"Unwrapped seam endpoint");
            close(own("next-x"),.02484,"Wrapped seam endpoint");
            close(own("step-end-y"),1.7,"Continuous periodic wall");
            close(own("previous-x"),49.99,"Previous position snapshot");
            capCheck();
            world.command("ask pedestrians [setxy 25 1.71 set wx 100 set wy -100 set ax 0 set ay 0 calculate-next-state]");
            close(own("step-end-x"),25.03484,"Equivalent interior wall contact");
            close(own("step-end-y"),1.7,"Equivalent interior clearance");
        } finally {world.dispose();}
        System.out.println("PASS: "+checks+" swept posts, joint corner constraints, reversed wall order, stop lines, speed caps and periodic seam checks.");
    }
}
