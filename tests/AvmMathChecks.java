import java.nio.file.*;
import java.util.*;
import java.util.regex.*;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Independent scalar implementations of AVM Eqs. (1)--(10).
 * The generated minimal model includes the real candidate avm.nls, but no
 * scenario, wall, spawning or integration code. No production file is edited.
 * The .25 m radius is an explicit analytical fixture, not a scenario default.
 */
public class AvmMathChecks {
    static int checks;
    static final double R=.25,K=6,D=.01,T=.6,TA=.75,TAU=.3;
    static boolean periodic;
    static double norm(double x,double y){return Math.hypot(x,y);}
    static double[] unit(double x,double y){double n=norm(x,y);return new double[]{x/n,y/n};}
    static double[] q(double[] a,double[] b){
        double x=b[1]-a[1];
        if(periodic)x=x-50*Math.floor((x+25)/50);
        return new double[]{x,b[2]-a[2]};
    }
    static boolean visible(double[] a,double[] q){return a[5]*q[0]+a[6]*q[1]>0 || a[7]*q[0]+a[8]*q[1]>0;}
    static double side(double[] a,double[] b,double[] q){return -(q[0]+TA*b[3])*a[8]+(q[1]+TA*b[4])*a[7];}
    static double[] influence(double[] a,double[] b,int tie){
        if(a[0]==b[0])return new double[]{0,0};
        double[] p=q(a,b);if(!visible(a,p))return new double[]{0,0};
        double d=norm(p[0],p[1]);
        // Literal Eq. (2), independently written using both predicted positions.
        double separation=Math.max(2*R,((p[0]+TA*b[3]-TA*a[3])*p[0]+(p[1]+TA*b[4]-TA*a[4])*p[1])/d);
        double alpha=K*(1+(1-a[7]*b[5]-a[8]*b[6])/2);
        double magnitude=alpha*Math.exp((2*R-separation)/D);
        double z=side(a,b,p),sign=Math.abs(z)<=1e-12?tie:-Math.signum(z);
        return new double[]{magnitude*sign*(-a[8]),magnitude*sign*a[7]};
    }
    static double[] steering(double[] a,double[][] rows,double h){
        double x=a[7],y=a[8];
        for(double[] b:rows){double[] f=influence(a,b,tie(b));x+=f[0];y+=f[1];}
        double[] optimum=unit(x,y);
        return unit((1-h/TAU)*a[5]+h/TAU*optimum[0],(1-h/TAU)*a[6]+h/TAU*optimum[1]);
    }
    static double speed(double[] a,double[][] rows,double[] e){
        double gap=Double.POSITIVE_INFINITY;
        for(double[] b:rows)if(a[0]!=b[0]){
            double[] p=q(a,b);double d=norm(p[0],p[1]);
            double ux=p[0]/d,uy=p[1]/d;
            if(e[0]*ux+e[1]*uy>=0 && Math.abs(-e[1]*ux+e[0]*uy)<=2*R/d)gap=Math.min(gap,d-2*R);
        }
        return Math.min(a[9],Math.max(0,gap/T));
    }
    static int tie(double[] row){return ((int)row[0])%2==0?-1:1;}
    static String row(double[] a){StringJoiner s=new StringJoiner(" ","[","]");for(double d:a)s.add(Double.toString(d));return s.toString();}
    static String rows(double[][] aa){StringJoiner s=new StringJoiner(" ","[","]");for(double[] a:aa)s.add(row(a));return s.toString();}
    static String choices(double[] own,double[][] rows){
        StringJoiner s=new StringJoiner(" ","[","]");
        for(double[] b:rows)if(b[0]!=own[0] && visible(own,q(own,b)) && Math.abs(side(own,b,q(own,b)))<=1e-12)s.add("["+b[0]+" "+tie(b)+"]");
        return s.toString();
    }
    static void near(double expected,Object actual,String label){
        if(!(actual instanceof Number))throw new AssertionError(label+" nonnumber "+actual);
        double value=((Number)actual).doubleValue();
        if(!Double.isFinite(value)||Math.abs(expected-value)>3e-11*Math.max(1,Math.abs(expected)))throw new AssertionError(label+": "+expected+" != "+value);
        checks++;
    }
    static void vector(double[] expected,Object actual,String label){LogoList l=(LogoList)actual;near(expected[0],l.apply(0),label+" x");near(expected[1],l.apply(1),label+" y");}
    static void yes(boolean value,String label){if(!value)throw new AssertionError(label);checks++;}
    static double[] agent(int who,double x,double y,double speed,double heading,double goal){return new double[]{who,x,y,speed*Math.cos(heading),speed*Math.sin(heading),Math.cos(heading),Math.sin(heading),Math.cos(goal),Math.sin(goal),1.55};}
    static void set(HeadlessWorkspace w,double[] own,double[][] rows)throws Exception {w.command("set fixture-row "+row(own)+" set fixture-states "+rows(rows)+" set fixture-ties "+choices(own,rows));}
    static void fixtures(HeadlessWorkspace w)throws Exception {
        double[] a=agent(0,0,0,1,0,0),b=agent(1,.6,.4,1,Math.PI,Math.PI);double[][] rows={a,b};
        set(w,a,rows);
        near((norm(.6,.4)-.5)/T,w.report("avm-speed fixture-row fixture-states 1 0"),"radial rather than projected gap");
        near(1.55,w.report("avm-speed fixture-row fixture-states 0 1"),"new heading determines strip");
        b=agent(1,1,.2,.5,-Math.PI/2,0);a=agent(0,0,0,5,Math.PI/2,0);rows=new double[][]{a,b};set(w,a,rows);
        double[] f=influence(a,b,1);yes(f[1]>0,"future j relative to current i chooses positive y");
        vector(f,w.report("avm-influence fixture-row item 1 fixture-states 0"),"future j/current i side");
        a=agent(0,0,0,1,0,0);b=agent(1,.6,0,1,Math.PI,Math.PI);rows=new double[][]{a,b};set(w,a,rows);
        vector(new double[]{0,2*K},w.report("avm-influence fixture-row item 1 fixture-states 1"),"clamped anticipated gap and opposite heading strength");
        yes(((LogoList)w.report("avm-tie-neighbors fixture-row fixture-states")).size()==1,"headon requests a tie");
        boolean missing=false;try{w.report("avm-steering fixture-row fixture-states .02 []");}catch(Exception e){missing=e.getMessage().contains("preassigned");}yes(missing,"missing tie fails explicitly");
        b=agent(1,0,1,0,0,0);rows=new double[][]{a,b};set(w,a,rows);
        vector(new double[]{0,0},w.report("avm-influence fixture-row item 1 fixture-states 1"),"strict FOV excludes side boundary");
        a=agent(0,0,0,0,Math.PI/2,0);set(w,a,rows);
        yes(((Boolean)w.report("avm-visible? fixture-row 0 1")),"movement half-plane extends desired FOV");
        a=agent(0,0,0,0,0,0);b=agent(1,.5,0,0,0,0);rows=new double[][]{a,b};set(w,a,rows);
        near(0,w.report("avm-speed fixture-row fixture-states 1 0"),"contact gives zero speed, no creep");
        b[2]=5e-13;set(w,a,rows);yes(((LogoList)w.report("avm-tie-neighbors fixture-row fixture-states")).size()==1,"numerical alignment tolerance");
        b[2]=2e-12;set(w,a,rows);yes(((LogoList)w.report("avm-tie-neighbors fixture-row fixture-states")).size()==0,"alignment tolerance bounded");
        rows=new double[][]{a};set(w,a,rows);near(1.55,w.report("avm-speed fixture-row fixture-states 1 0"),"isolated free speed");
        w.command("random-seed 1907");Object expected=w.report("n-values 16 [random-float 1]");
        w.command("random-seed 1907 repeat 20 [let z avm-steering fixture-row fixture-states .02 fixture-ties let z2 avm-speed fixture-row fixture-states 1 0 let z3 avm-tie-neighbors fixture-row fixture-states]");
        yes(expected.equals(w.report("n-values 16 [random-float 1]")),"reporters do not consume RNG");
    }
    static void randomChecks(HeadlessWorkspace w)throws Exception {
        Random rng=new Random(687124);
        for(boolean p:new boolean[]{false,true}){
            periodic=p;w.command("set active-periodic? "+p);
            for(int trial=0;trial<150;trial++){
                double[][] rows=new double[5][];
                for(int j=0;j<rows.length;j++)rows[j]=agent(j,p?rng.nextDouble()*50:rng.nextDouble()*3,rng.nextDouble()*3,rng.nextDouble()*2,rng.nextDouble()*2*Math.PI,rng.nextDouble()*2*Math.PI);
                if(p && trial%5==0){rows[0][1]=49.95;rows[1][1]=.05;rows[1][2]=rows[0][2]+.01;}
                double[] a=rows[0];set(w,a,rows);
                for(int j=1;j<rows.length;j++)vector(influence(a,rows[j],tie(rows[j])),w.report("avm-influence fixture-row item "+j+" fixture-states "+tie(rows[j])),"pair fixture "+trial);
                double h=.005+rng.nextDouble()*.045;double[] e=steering(a,rows,h);
                vector(e,w.report("avm-steering fixture-row fixture-states "+h+" fixture-ties"),"steering fixture "+trial);
                near(1,norm(e[0],e[1]),"unit heading");
                near(speed(a,rows,e),w.report("avm-speed fixture-row fixture-states "+e[0]+" "+e[1]),"speed fixture "+trial);
            }
        }
    }
    public static void main(String[] args)throws Exception {
        Path modelRoot=Path.of(".").toAbsolutePath().normalize();
        for(int i=0;i<args.length;i++){
            if(args[i].equals("--model-root") && i+1<args.length)modelRoot=Path.of(args[++i]).toAbsolutePath().normalize();
            else throw new IllegalArgumentException("Usage: AvmMathChecks.java [--model-root path]");
        }
        Path dir=Path.of("work/avm/math-checks").toAbsolutePath().normalize();Files.createDirectories(dir);
        String xml=Files.readString(modelRoot.resolve("Pedestrians.nlogox"));
        String include=dir.relativize(modelRoot.resolve("core/avm.nls")).toString().replace('\\','/');
        String code="__includes [\""+include+"\"]\n"
            +"globals [avm-strength avm-range avm-time-gap anticipation-time tau body-radius active-periodic? hall-length fixture-row fixture-states fixture-ties]\n"
            +"to-report periodic-delta-x [x] report ((x + hall-length / 2) mod hall-length) - hall-length / 2 end\n"
            +"to-report vector-length [x y] report sqrt (x * x + y * y) end\n";
        xml=xml.replaceFirst("(?s)<code>.*?</code>",Matcher.quoteReplacement("<code><![CDATA["+code+"]]></code>"));
        Matcher view=Pattern.compile("<view\\s[^>]*?/>").matcher(xml);if(!view.find())throw new AssertionError("no view");
        xml=xml.replaceFirst("(?s)<widgets>.*?</widgets>",Matcher.quoteReplacement("<widgets>"+view.group()+"</widgets>"));
        xml=xml.replaceFirst("(?s)<experiments>.*?</experiments>","<experiments />");
        Path model=dir.resolve("AvmMath.nlogox");Files.writeString(model,xml);
        var w=HeadlessWorkspace.newInstance();
        try{
            w.open(model.toString());w.command("set avm-strength 6 set avm-range .01 set avm-time-gap .6 set anticipation-time .75 set tau .3 set body-radius .25 set active-periodic? false set hall-length 50");
            fixtures(w);randomChecks(w);
            System.out.println("PASS: "+checks+" independent AVM math, periodic-image, tie and RNG assertions.");
        }finally{w.dispose();}
    }
}
