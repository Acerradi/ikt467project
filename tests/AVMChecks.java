import java.io.PrintWriter;
import java.nio.file.*;
import java.util.*;
import org.nlogo.core.LogoList;
import org.nlogo.headless.HeadlessWorkspace;

/** Actual-NetLogo AVM checks. Every fixed Euler step is audited independently.
 * --mode pairs|crowds|order|math|api-parity --cases comma-separated --seeds 42,1,7
 * --dt .02 --seconds 30 --model-root . --output work/avm-radius020/validation
 * --string-api disables workspace-local compilation caching for parity checks.
 * Swept overlap is measured, never rejected or used to stop a run; execution errors and flow are separate.
 */
public class AVMChecks {
    static int failures=0;
    static final double EXPECTED_RADIUS=.2;
    record Fixture(String name,String model,String setup,boolean pair,double seconds) {}
    static final String EMPTY="set periodic? false set population 0 set hall-width 10 set spawn-rate 0";
    static final List<Fixture> FIXTURES=List.of(
        new Fixture("headon-moving","Pedestrians.nlogox",EMPTY,true,10),
        new Fixture("headon-rest","Pedestrians.nlogox",EMPTY,true,10),
        new Fixture("offset","Pedestrians.nlogox",EMPTY,true,10),
        new Fixture("overtake","Pedestrians.nlogox",EMPTY,true,12),
        new Fixture("rightangle","Pedestrians.nlogox",EMPTY,true,10),
        new Fixture("rightangle-symmetric","Pedestrians.nlogox",EMPTY,true,10),
        new Fixture("side-pinch","Pedestrians.nlogox",EMPTY,true,1),
        new Fixture("hallway","Pedestrians.nlogox","set periodic? true set population 150 set hall-width 10 set spawn-rate 0",false,30),
        new Fixture("hallway-open","Pedestrians.nlogox","set periodic? false set population 0 set hall-width 10 set spawn-rate 1",false,60),
        new Fixture("door","scenarios/Door.nlogox","set log-csv? false set initial-per-side 20 set spawn-rate 0 set two-sided? true set room-width 6 set door-width 1.2",false,30),
        new Fixture("door-oneway","scenarios/Door.nlogox","set log-csv? false set initial-per-side 20 set spawn-rate 0 set two-sided? false set room-width 6 set door-width 1.2",false,60),
        new Fixture("traffic","scenarios/TrafficLight.nlogox","set log-csv? false set initial-per-side 20 set spawn-rate .5 set switch-time 10 set start-green? false set road-width 6 set walkway-width 6",false,30),
        new Fixture("crossing","scenarios/Crossing.nlogox","set log-csv? false set horizontal-rate 1.8 set vertical-rate 1.8 set two-way? false set corridor-width 4",false,30)
    );
    static final String OBS="(list elapsed-seconds last-internal-step-paths "
        +"(map [p -> [(list who xcor ycor vx vy desired-speed goal-x goal-y move-ex move-ey)] of p] sort pedestrians) "
        +"spawned-count arrived-count returned-count blocked-spawn-count contact-step-count contact-solver-fallbacks wrap-count "
        +"(list active-periodic? hall-length body-radius pair-contact-enabled? min-pxcor max-pxcor min-pycor max-pycor avm-time-gap tau anticipation-time avm-strength avm-range) wall-segments (list avm-accepted-steps avm-rejected-steps avm-last-min-step))";
    static String opt(String[] args,String name,String fallback){for(int i=0;i<args.length-1;i++)if(args[i].equals(name))return args[i+1];return fallback;}
    static double n(LogoList list,int i){return ((Number)list.apply(i)).doubleValue();}
    static double number(HeadlessWorkspace w,String s)throws Exception{return ((Number)w.report(s)).doubleValue();}
    // The compiled API leaves Logo runtime errors on the workspace instead of
    // always throwing them. Propagate and clear them just as the string API does.
    static void compiledError(HeadlessWorkspace w)throws Exception{
        if(w.lastLogoException()!=null){var error=w.lastLogoException();w.lastLogoException_$eq(null);throw error;}
    }
    static Object compiledReport(HeadlessWorkspace w,org.nlogo.nvm.Procedure reporter)throws Exception{
        Object result=w.runCompiledReporter(w.defaultOwner(),reporter);compiledError(w);return result;
    }
    static void compiledCommand(HeadlessWorkspace w,org.nlogo.nvm.Procedure command)throws Exception{
        w.runCompiledCommands(w.defaultOwner(),command);compiledError(w);
    }
    static class LoopApi{
        final HeadlessWorkspace workspace;final boolean stringApi;
        final org.nlogo.nvm.Procedure go,observation,elapsed,stopOvershoot;
        LoopApi(HeadlessWorkspace w,boolean strings,boolean traffic)throws Exception{
            workspace=w;stringApi=strings;
            go=strings?null:w.compileCommands("go");
            observation=strings?null:w.compileReporter(OBS);
            elapsed=strings?null:w.compileReporter("elapsed-seconds");
            stopOvershoot=strings||!traffic?null:w.compileReporter("max-stop-overshoot");
        }
        void go()throws Exception{if(stringApi)workspace.command("go");else compiledCommand(workspace,go);}
        LogoList observe()throws Exception{return (LogoList)(stringApi?workspace.report(OBS):compiledReport(workspace,observation));}
        double elapsed()throws Exception{return ((Number)(stringApi?workspace.report("elapsed-seconds"):compiledReport(workspace,elapsed))).doubleValue();}
        double stopOvershoot()throws Exception{return ((Number)(stringApi?workspace.report("max-stop-overshoot"):compiledReport(workspace,stopOvershoot))).doubleValue();}
    }
    static String csv(String s){return "\""+s.replace("\"","\"\"").replace('\n',' ').replace('\r',' ')+"\"";}
    static double[][] rows(LogoList list,int size){double[][] result=new double[list.size()][size];for(int i=0;i<result.length;i++)for(int j=0;j<size;j++){result[i][j]=n((LogoList)list.apply(i),j);if(!Double.isFinite(result[i][j]))throw new AssertionError("Nonfinite state/path");}return result;}
    static double periodic(double x,double period){return period>0?x-period*Math.floor(x/period+.5):x;}
    record Approach(double distance,double fraction) {}
    static Approach closest(double[] a,double[] b,double period){
        double qx=a[1]-b[1],qy=a[2]-b[2],dx=(a[3]-a[1])-(b[3]-b[1]),dy=(a[4]-a[2])-(b[4]-b[2]);
        int lo=0,hi=0;if(period>0){lo=(int)Math.floor(-Math.max(qx,qx+dx)/period)-1;hi=(int)Math.ceil(-Math.min(qx,qx+dx)/period)+1;}
        double best=Double.POSITIVE_INFINITY,fraction=0,den=dx*dx+dy*dy;
        for(int k=lo;k<=hi;k++){double x=qx+k*period,t=den==0?0:Math.max(0,Math.min(1,-(x*dx+qy*dy)/den));double d=Math.hypot(x+t*dx,qy+t*dy);if(d<best){best=d;fraction=t;}}
        return new Approach(best,fraction);
    }
    record Overlap(double first,double duration) {}
    // Continuous fractions where relative linear motion is strictly inside the
    // diameter threshold. Sum is pair-seconds, not a union over different pairs.
    static Overlap overlap(double[] a,double[] b,double period,double threshold){
        double qx=a[1]-b[1],qy=a[2]-b[2],dx=(a[3]-a[1])-(b[3]-b[1]),dy=(a[4]-a[2])-(b[4]-b[2]);
        int lo=0,hi=0;if(period>0){lo=(int)Math.floor(-Math.max(qx,qx+dx)/period)-1;hi=(int)Math.ceil(-Math.min(qx,qx+dx)/period)+1;}
        double first=Double.NaN,duration=0,aa=dx*dx+dy*dy;
        for(int k=lo;k<=hi;k++){
            double x=qx+k*period,bb=2*(x*dx+qy*dy),cc=x*x+qy*qy-threshold*threshold,start=0,end=1;
            if(aa==0){if(cc>=0)continue;}
            else {double disc=bb*bb-4*aa*cc;if(disc<=0)continue;double root=Math.sqrt(disc);start=Math.max(0,(-bb-root)/(2*aa));end=Math.min(1,(-bb+root)/(2*aa));if(end<=start)continue;}
            if(Double.isNaN(first)||start<first)first=start;duration+=end-start;
        }
        return new Overlap(first,duration);
    }
    static void fixedIntervals(LogoList obs,double start,double end,boolean traffic){
        double expected=start;LogoList intervals=(LogoList)obs.apply(1);
        if(intervals.isEmpty())throw new AssertionError("Missing fixed Euler interval");
        for(Object raw:intervals.javaIterable()){
            LogoList interval=(LogoList)raw;double next=end;
            if(traffic){double event=(Math.floor((expected+1e-9)/10)+1)*10;if(event<end-1e-9)next=event;}
            if(Math.abs(n(interval,0)-expected)>1e-8||Math.abs(n(interval,1)-(next-expected))>1e-8)throw new AssertionError("Unexpected adaptive subdivision or wrong light event interval");
            expected=next;
        }
        if(Math.abs(expected-end)>1e-8)throw new AssertionError("Fixed interval ended early");
    }
    static double pointSegment(double x,double y,double ax,double ay,double bx,double by){double dx=bx-ax,dy=by-ay,den=dx*dx+dy*dy,f=den==0?0:Math.max(0,Math.min(1,((x-ax)*dx+(y-ay)*dy)/den));return Math.hypot(x-ax-f*dx,y-ay-f*dy);}
    static double cross(double ax,double ay,double bx,double by){return ax*by-ay*bx;}
    static double segments(double ax,double ay,double bx,double by,double cx,double cy,double dx,double dy){
        double rx=bx-ax,ry=by-ay,sx=dx-cx,sy=dy-cy,den=cross(rx,ry,sx,sy);
        if(den!=0){double t=cross(cx-ax,cy-ay,sx,sy)/den,u=cross(cx-ax,cy-ay,rx,ry)/den;if(t>=0&&t<=1&&u>=0&&u<=1)return 0;}
        return Math.min(Math.min(pointSegment(ax,ay,cx,cy,dx,dy),pointSegment(bx,by,cx,cy,dx,dy)),Math.min(pointSegment(cx,cy,ax,ay,bx,by),pointSegment(dx,dy,ax,ay,bx,by)));
    }
    static void geometry(){
        if(closest(new double[]{0,-1,0,1,0},new double[]{1,1,0,-1,0},0).distance!=0)throw new AssertionError("Midstep crossing missed");
        if(closest(new double[]{0,49.9,0,50.1,0},new double[]{1,.1,0,-.1,0},50).distance>1e-12)throw new AssertionError("Periodic crossing missed");
        if(segments(0,0,1,1,0,1,1,0)!=0)throw new AssertionError("Wall crossing missed");
        Overlap crossing=overlap(new double[]{0,-1,0,1,0},new double[]{1,1,0,-1,0},0,.5);
        if(Math.abs(crossing.first-.375)>1e-12||Math.abs(crossing.duration-.25)>1e-12)throw new AssertionError("Overlap interval calculation");
        Overlap standing=overlap(new double[]{0,0,0,0,0},new double[]{1,.4,0,.4,0},0,.5);
        if(standing.first!=0||standing.duration!=1)throw new AssertionError("Persistent overlap measurement");
    }
    static String person(String label,double x,double y,double gx,double gy,double speed,double vx,double vy,double ex,double ey){return String.format(Locale.ROOT,
        "create-pedestrians 1 [initialize-pedestrian %.17g %.17g %.17g %.17g %d set label \"%s\" set desired-speed %.17g set vx %.17g set vy %.17g set wx vx set wy vy set move-ex %.17g set move-ey %.17g] ",x,y,gx,gy,gx>=x?1:-1,label,speed,vx,vy,ex,ey);}
    static String pair(String name){
        if(name.equals("overtake"))return person("a",20,6.5,45,6.5,1.8,1.8,0,1,0)+person("b",22,6.5,45,6.5,1.2,1.2,0,1,0);
        if(name.startsWith("rightangle")){double vb=name.endsWith("symmetric")?1.55:1.54;return person("a",21,6.5,40,6.5,1.55,1.55,0,1,0)+person("b",25,2.5,25,10.5,vb,0,vb,0,1);}
        if(name.equals("side-pinch")){double a=.01,b=Math.sqrt(1-a*a),xa=25-EXPECTED_RADIUS-.00005,xb=25+EXPECTED_RADIUS+.00005;return "set tau 1e9 "+person("a",xa,6,xa,10.5,1.55,1.55*a,1.55*b,a,b)+person("b",xb,6,xb,10.5,1.55,-1.55*a,1.55*b,-a,b);}
        double ya=name.equals("offset")?6.45:6.5,yb=name.equals("offset")?6.55:6.5,v=name.equals("headon-rest")?0:1.55;
        return person("a",20,ya,40,ya,1.55,v,0,1,0)+person("b",30,yb,10,yb,1.55,-v,0,-1,0);
    }
    static void setup(HeadlessWorkspace w,Fixture f,int seed,double dt)throws Exception{
        w.command("set simulation-seed "+seed+" set dt "+dt+" set mean-desired-speed 1.55 set desired-speed-sd "+(f.pair?"0":".18")+" set noise-strength 0 set show-trails? false set run-checks? false "+f.setup+" setup "+(f.pair?"set wall-segments [] "+pair(f.name)+" set spawned-count 2":""));
        if(number(w,"body-radius")!=EXPECTED_RADIUS||Boolean.TRUE.equals(w.report("pair-contact-enabled?")))throw new AssertionError("Wrong radius or pair contact active at setup");
    }
    static boolean hasOcclusionHelper(Path root)throws Exception{
        // Resolve against --model-root: historical snapshots do not necessarily
        // contain this geometry helper. A present helper must compile/run normally;
        // runtime failures in the diagnostic are never hidden by a legacy fallback.
        var definition=java.util.regex.Pattern.compile("(?m)^\\s*to-report\\s+avm-unoccluded-neighbors\\b");
        for(String file:List.of("core/boundaries.nls","core/avm.nls")){
            Path source=root.resolve(file);if(Files.isRegularFile(source)&&definition.matcher(Files.readString(source)).find())return true;
        }
        return false;
    }
    static void doorDetails(HeadlessWorkspace w,Path root,Path out,String id,double elapsed)throws Exception{
        String state="(list elapsed-seconds door-x door-center-y door-sequence "
            +"(map [p -> [(list who xcor ycor vx vy ex ey move-ex move-ey goal-x goal-y waypoints travel-direction)] of p] sort pedestrians))";
        boolean occlusion=hasOcclusionHelper(root);
        String neighbors=occlusion?"(avm-unoccluded-neighbors row avm-snapshot)":"avm-snapshot";
        String free="(avm-speed row "+neighbors+" (item 5 row) (item 6 row))";
        String wall="(avm-boundary-speed (item 1 row) (item 2 row) (item 5 row) (item 6 row) "+free+" dt false 1)";
        String probe="map [row -> (list (item 0 row) "+free+" "+wall+")] avm-snapshot";
        Files.writeString(out.resolve(id+"-door-at"+Math.round(elapsed)+"s.txt"),
            "State [time,door-x,door-y,passage-directions,people]. People [who,x,y,vx,vy,ex,ey,move-ex,move-ey,goal-x,goal-y,waypoints,travel-direction].\n"
            +w.report(state)+"\nCurrent heading probe [who,pedestrian-speed-law,after-wall-speed-limit]. Read-only, no movement or RNG.\n"
            +(occlusion?"Neighbors are filtered by the model's wall occlusion helper.":"Legacy model without an occlusion helper: unfiltered neighbor probe.")+"\n"+w.report(probe)+"\n");
    }
    record Motion(double start,double h,double length) {}
    static class Audit{
        double minimum=Double.POSITIVE_INFINITY,minTime=Double.NaN,minWall=Double.POSITIVE_INFINITY,maxSpeed=0,maxSpeedExcess=0,minH=Double.POSITIVE_INFINITY;
        double firstOverlap=Double.NaN,maxOverlapDepth=0,overlapPairTime=0;
        long overlapSteps=0,overlapPairIntervals=0,maxOverlapPairs=0;
        long accepted=0;String worst="";Map<Long,double[]> previous=new HashMap<>();Map<Long,Double> desired=new HashMap<>(),born=new HashMap<>();Map<Long,Deque<Motion>> history=new HashMap<>();
        void remember(LogoList obs){for(double[] p:rows((LogoList)obs.apply(2),10)){desired.put((long)p[0],p[5]);born.putIfAbsent((long)p[0],n(obs,0));}}
        void inspect(LogoList obs,double from,double until,PrintWriter trace){
            remember(obs);LogoList config=(LogoList)obs.apply(10);double period=Boolean.TRUE.equals(config.apply(0))?n(config,1):0,r=n(config,2);
            if(r!=EXPECTED_RADIUS||Boolean.TRUE.equals(config.apply(3))||n(obs,7)!=0||n(obs,8)!=0||n((LogoList)obs.apply(12),1)!=0)throw new AssertionError("Radius/contact/rejection invariant");
            double[][] walls=rows((LogoList)obs.apply(11),4);LogoList internal=(LogoList)obs.apply(1);double covered=from;
            if(internal.size()==0&&((LogoList)obs.apply(2)).size()==0)return;
            for(Object raw:internal.javaIterable()){
                LogoList step=(LogoList)raw;double start=n(step,0),h=n(step,1);if(!Double.isFinite(h)||h<=0||Math.abs(start-covered)>1e-8)throw new AssertionError("Missing/repeated accepted interval");
                double[][] paths=rows((LogoList)step.apply(2),5);LogoList rawStates=(LogoList)step.apply(3);double[][] states=rows(rawStates,5);
                if(paths.length!=states.length)throw new AssertionError("Departing path/state omitted");
                Map<Long,double[]> byId=new HashMap<>();Map<Long,LogoList> fullById=new HashMap<>();for(int i=0;i<states.length;i++){byId.put((long)states[i][0],states[i]);fullById.put((long)states[i][0],(LogoList)rawStates.apply(i));}
                double subMin=Double.POSITIVE_INFINITY;long overlapPairs=0;
                for(double[] p:paths){
                    long id=(long)p[0];double[] state=byId.get(id);LogoList full=fullById.get(id);if(state==null)throw new AssertionError("Missing endpoint");
                    if(full.size()>=8){desired.put(id,n(full,7));if(Math.abs(Math.hypot(n(full,5),n(full,6))-1)>1e-9)throw new AssertionError("Non-unit movement heading");}
                    if(!desired.containsKey(id))throw new AssertionError("Missing desired speed for path including departure");
                    if(Math.hypot(p[3]-state[1],p[4]-state[2])>1e-8)throw new AssertionError("Path/endpoint mismatch");
                    if(Math.hypot(p[3]-p[1]-h*state[3],p[4]-p[2]-h*state[4])>1e-9)throw new AssertionError("Motion differs from Euler velocity; possible position correction");
                    double[] prior=previous.get(id);if(prior!=null&&Math.hypot(periodic(p[1]-prior[1],period),p[2]-prior[2])>1e-8)throw new AssertionError("Discontinuous path");previous.put(id,state);
                    double speed=Math.hypot(state[3],state[4]),pathSpeed=Math.hypot(p[3]-p[1],p[4]-p[2])/h;
                    maxSpeed=Math.max(maxSpeed,Math.max(speed,pathSpeed));maxSpeedExcess=Math.max(maxSpeedExcess,Math.max(speed,pathSpeed)-desired.get(id));
                    if(maxSpeedExcess>2e-8)throw new AssertionError("AVM free-speed limit exceeded: "+maxSpeedExcess);
                    for(double[] wall:walls)for(int image=period>0?-1:0;image<=(period>0?1:0);image++){
                        double d=segments(p[1],p[2],p[3],p[4],wall[0]+image*period,wall[1],wall[2]+image*period,wall[3]);minWall=Math.min(minWall,d);if(d<r-2e-7)throw new AssertionError("Wall clearance failed: "+d);
                    }
                    born.merge(id,start,Math::min);Deque<Motion> q=history.computeIfAbsent(id,k->new ArrayDeque<>());q.addLast(new Motion(start,h,pathSpeed*h));while(!q.isEmpty()&&q.peekFirst().start+q.peekFirst().h<until-10)q.removeFirst();
                }
                for(int i=0;i<paths.length;i++)for(int j=i+1;j<paths.length;j++){
                    Approach a=closest(paths[i],paths[j],period);subMin=Math.min(subMin,a.distance);if(a.distance<minimum){minimum=a.distance;minTime=start+h*a.fraction;worst=step.toString();}
                    if(a.distance<2*r-1e-8){
                        overlapPairs++;overlapPairIntervals++;maxOverlapDepth=Math.max(maxOverlapDepth,2*r-a.distance);
                        Overlap overlap=overlap(paths[i],paths[j],period,2*r-1e-8);
                        if(Double.isNaN(overlap.first))throw new AssertionError("Overlap minimum/interval disagreement");
                        double time=start+h*overlap.first;if(Double.isNaN(firstOverlap)||time<firstOverlap)firstOverlap=time;
                        overlapPairTime+=h*overlap.duration;
                    }
                }
                if(overlapPairs>0)overlapSteps++;maxOverlapPairs=Math.max(maxOverlapPairs,overlapPairs);
                accepted++;minH=Math.min(minH,h);covered=start+h;trace.printf(Locale.ROOT,"%.12g,%.12g,%d,%.12g,%.12g,%.12g,%d%n",start,h,paths.length,subMin,maxSpeed,minWall,overlapPairs);
            }
            if(Math.abs(covered-until)>1e-8)throw new AssertionError("Incomplete accepted-step coverage: "+covered+" vs "+until);
            for(double[] p:rows((LogoList)obs.apply(2),10)){if(p[1]<n(config,4)-.5||p[1]>n(config,5)+.5||p[2]<n(config,6)-.5||p[2]>n(config,7)+.5)throw new AssertionError("Left world");if(Math.abs(Math.hypot(p[8],p[9])-1)>1e-9)throw new AssertionError("Non-unit live heading");}
            if((int)n(obs,3)!=(int)n(obs,4)+(int)n(obs,5)+((LogoList)obs.apply(2)).size())throw new AssertionError("Population accounting failed");
        }
        int staticCount(LogoList obs){int count=0;double end=n(obs,0);for(double[] p:rows((LogoList)obs.apply(2),10)){long id=(long)p[0];if(end-born.getOrDefault(id,end)<10-1e-8)continue;double distance=0;for(Motion s:history.getOrDefault(id,new ArrayDeque<>())){double duration=Math.max(0,Math.min(end,s.start+s.h)-Math.max(end-10,s.start));distance+=s.length*duration/s.h;}if(distance/10<p[5]/100)count++;}return count;}
    }
    static void run(Path root,Path out,Fixture f,int seed,double dt,double seconds,PrintWriter summary,boolean stringApi)throws Exception{
        String id=f.name+"-seed"+seed+"-dt"+dt;var w=HeadlessWorkspace.newInstance();Audit audit=new Audit();LogoList last=null;int outer=0;double elapsed=0;String error="";long started=System.nanoTime();
        try(PrintWriter trace=new PrintWriter(Files.newBufferedWriter(out.resolve(id+"-accepted.csv")))){
            trace.println("start_s,h_s,path_count,swept_min_m,max_speed_mps,min_wall_m,overlapping_pairs");
            try{
                w.open(root.resolve(f.model).toAbsolutePath().toString());setup(w,f,seed,dt);LoopApi api=new LoopApi(w,stringApi,f.name.equals("traffic"));last=api.observe();audit.remember(last);Files.writeString(out.resolve(id+"-initial.txt"),last.toString());
                for(int k=0;k<Math.round(seconds/dt);k++){
                    double before=api.elapsed();
                    try{api.go();}catch(Exception failed){last=api.observe();elapsed=n(last,0);if(elapsed>before+1e-13)audit.inspect(last,before,elapsed,trace);throw failed;}
                    last=api.observe();elapsed=n(last,0);outer++;if(Math.abs(elapsed-before-dt)>1e-8)throw new AssertionError("Outer time contract");audit.inspect(last,before,elapsed,trace);fixedIntervals(last,before,elapsed,f.name.equals("traffic"));
                    if(f.name.equals("traffic")&&api.stopOvershoot()>2e-7)throw new AssertionError("Held stop line crossed");
                    if(f.name.startsWith("door")&&outer%Math.round(30/dt)==0)doorDetails(w,root,out,id,elapsed);
                    if(outer%250==0){trace.flush();System.out.printf(Locale.ROOT,"PROGRESS %s %.2fs min=%.9fm accepted=%d%n",id,elapsed,audit.minimum,audit.accepted);}
                }
            }catch(Exception|AssertionError fail){failures++;error=fail.toString();try{Files.writeString(out.resolve(id+"-error.txt"),error+"\n"+w.report(OBS));}catch(Exception ignored){}}
        }finally{if(last!=null)Files.writeString(out.resolve(id+"-final.txt"),last.toString());w.dispose();}
        Files.writeString(out.resolve(id+"-worst.txt"),audit.worst);
        int walking=last==null?0:((LogoList)last.apply(2)).size(),spawned=last==null?0:(int)n(last,3),arrived=last==null?0:(int)n(last,4),returned=last==null?0:(int)n(last,5),blocked=last==null?0:(int)n(last,6),wraps=last==null?0:(int)n(last,9),static10=last==null?0:audit.staticCount(last);
        double mean=0;if(last!=null)for(double[] p:rows((LogoList)last.apply(2),10))mean+=Math.hypot(p[3],p[4]);if(walking>0)mean/=walking;
        double wall=(System.nanoTime()-started)/1e9;
        LogoList counters=last==null?null:(LogoList)last.apply(12);
        double rejected=counters==null?Double.NaN:n(counters,1);
        String outcome="crowd";
        if(f.pair){double[] a=audit.previous.get(0L),b=audit.previous.get(1L);boolean passed=false;
            if(a!=null&&b!=null)passed=f.name.startsWith("rightangle")?a[1]>25&&b[2]>6.5:a[1]>b[1];
            outcome=passed?"passed":"did-not-pass";
        }
        Files.writeString(out.resolve(id+"-integration.txt"),"accepted="+audit.accepted+"\nrejected="+rejected+"\nminimum_h="+audit.minH+"\noutcome="+outcome+"\napi="+(stringApi?"string":"compiled")+"\n");
        if(f.name.equals("side-pinch")&&dt>=.01&&audit.overlapPairIntervals==0){failures++;error+=" side-pinch did not expose the expected unrestricted fixed-step overlap";}
        summary.printf(Locale.ROOT,"%s,%d,%.9g,%.9g,%d,%.12g,%d,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%d,%d,%d,%d,%d,%d,%.12g,%d,%.6f,%.12g,%.12g,%d,%d,%d,%.12g,%s,%s%n",f.name,seed,dt,seconds,outer,elapsed,audit.accepted,audit.minH,audit.minimum,audit.minTime,audit.maxSpeed,audit.maxSpeedExcess,audit.minWall,spawned,arrived,returned,walking,blocked,wraps,mean,static10,wall,audit.firstOverlap,audit.maxOverlapDepth,audit.overlapSteps,audit.overlapPairIntervals,audit.maxOverlapPairs,audit.overlapPairTime,audit.overlapPairIntervals==0?"no-overlap-observed":"overlap-observed",csv(error));summary.flush();
        System.out.printf(Locale.ROOT,"RESULT %s %.3fs min=%.9fm arrived=%d wraps=%d walking=%d static10=%d wall=%.2fs %s%n",id,elapsed,audit.minimum,arrived,wraps,walking,static10,wall,error.isEmpty()?"PASS execution/finite/speed/walls; overlaps="+audit.overlapPairIntervals+" (measured, not enforced); flow reported separately":error);
    }
    static final String PHYSICAL="map [p -> [(list who xcor ycor vx vy wx wy move-ex move-ey goal-x goal-y desired-speed)] of p] sort pedestrians";
    static String physical(HeadlessWorkspace w)throws Exception{return w.report(PHYSICAL).toString();}
    static void order(Path root,Path out,double dt)throws Exception{
        Fixture f=FIXTURES.get(0);var a=HeadlessWorkspace.newInstance();var b=HeadlessWorkspace.newInstance();
        try{a.open(root.resolve(f.model).toAbsolutePath().toString());b.open(root.resolve(f.model).toAbsolutePath().toString());setup(a,f,42,dt);setup(b,f,42,dt);b.command("set avm-reverse-calculation? true");
            for(int i=0;i<Math.round(6/dt);i++){a.command("go");b.command("go");if(!physical(a).equals(physical(b)))throw new AssertionError("Reversed calculation changed physical state at step "+i);if(!a.report(OBS).toString().equals(b.report(OBS).toString()))throw new AssertionError("Reversed calculation changed trace/counter state at step "+i);}
            if(!a.report("n-values 32 [random-float 1]").toString().equals(b.report("n-values 32 [random-float 1]").toString()))throw new AssertionError("Canonical tie RNG differs by calculation order");
            String result="PASS same IDs with reversed actor calculation: physical state identical at every step over6s, including exact head-on ties;32 future RNG draws identical.\n";Files.writeString(out.resolve("order-result.txt"),result);System.out.print(result);
        }finally{a.dispose();b.dispose();}
    }
    @FunctionalInterface interface FailingCall{void run()throws Exception;}
    static String expectLogoFailure(FailingCall call,String expected)throws Exception{
        try{call.run();}catch(Exception error){
            if(!String.valueOf(error.getMessage()).contains(expected))throw new AssertionError("Unexpected API error: "+error,error);
            return error.getClass().getName()+": "+error.getMessage();
        }
        throw new AssertionError("API swallowed runtime error: "+expected);
    }
    static void apiErrors(HeadlessWorkspace strings,HeadlessWorkspace compiled)throws Exception{
        String failCommand="error \"AVM API parity intentional command failure\"";
        var command=compiled.compileCommands(failCommand);
        String a=expectLogoFailure(()->strings.command(failCommand),"AVM API parity intentional command failure");
        String b=expectLogoFailure(()->compiledCommand(compiled,command),"AVM API parity intentional command failure");
        if(!a.equals(b))throw new AssertionError("Command error propagation differs: "+a+" vs "+b);
        // Exercise an actual model reporter error. Its explicit message also
        // avoids localized primitive errors consulting Windows registry prefs.
        String failReporter="avm-boundary-speed 0 0 1 0 1 0 false 1";var reporter=compiled.compileReporter(failReporter);
        a=expectLogoFailure(()->strings.report(failReporter),"AVM boundary step must be positive");
        b=expectLogoFailure(()->compiledReport(compiled,reporter),"AVM boundary step must be positive");
        if(!a.equals(b))throw new AssertionError("Reporter error propagation differs: "+a+" vs "+b);
        if(!strings.report("42").equals(compiledReport(compiled,compiled.compileReporter("42"))))throw new AssertionError("Failed API recovery");
    }
    static void apiParity(Path root,Path out,double dt)throws Exception{
        if(!(dt>0&&Double.isFinite(dt)))throw new IllegalArgumentException("Positive finite dt required");
        int steps=(int)Math.round(6/dt);if(steps<1||Math.abs(steps*dt-6)>1e-9)throw new IllegalArgumentException("API parity requires dt to divide 6 s");
        StringBuilder result=new StringBuilder("Model root: "+root.toAbsolutePath()+"\n");
        for(Fixture f:FIXTURES)if(Set.of("hallway","door","traffic","crossing").contains(f.name)){
            var a=HeadlessWorkspace.newInstance();var b=HeadlessWorkspace.newInstance();
            Path aTrace=out.resolve(f.name+"-string-audit.csv"),bTrace=out.resolve(f.name+"-compiled-audit.csv");
            try(PrintWriter ta=new PrintWriter(Files.newBufferedWriter(aTrace));PrintWriter tb=new PrintWriter(Files.newBufferedWriter(bTrace))){
                a.open(root.resolve(f.model).toAbsolutePath().toString());b.open(root.resolve(f.model).toAbsolutePath().toString());setup(a,f,42,dt);setup(b,f,42,dt);
                boolean traffic=f.name.equals("traffic");LoopApi sa=new LoopApi(a,true,traffic),cb=new LoopApi(b,false,traffic);var compiledPhysical=b.compileReporter(PHYSICAL);
                Audit aa=new Audit(),ab=new Audit();LogoList oa=sa.observe(),ob=cb.observe();
                if(!oa.equals(ob)||!a.report(PHYSICAL).equals(compiledReport(b,compiledPhysical)))throw new AssertionError(f.name+" initial API state differs");
                aa.remember(oa);ab.remember(ob);
                for(int k=0;k<steps;k++){
                    double beforeA=sa.elapsed(),beforeB=cb.elapsed();if(beforeA!=beforeB)throw new AssertionError(f.name+" elapsed reporter differs at "+k);
                    sa.go();cb.go();oa=sa.observe();ob=cb.observe();
                    if(!oa.equals(ob)||!a.report(PHYSICAL).equals(compiledReport(b,compiledPhysical)))throw new AssertionError(f.name+" physical/path/counter API state differs at "+k);
                    double end=n(oa,0);if(Math.abs(end-beforeA-dt)>1e-8)throw new AssertionError("API parity outer time contract");
                    aa.inspect(oa,beforeA,end,ta);ab.inspect(ob,beforeB,end,tb);fixedIntervals(oa,beforeA,end,traffic);fixedIntervals(ob,beforeB,end,traffic);
                    if(traffic){double left=sa.stopOvershoot(),right=cb.stopOvershoot();if(left!=right||left>2e-7)throw new AssertionError("API stop-line check differs/fails");}
                }
                // Exercise the real diagnostic on only one workspace. It must
                // preserve physical observations and the future RNG sequence.
                if(f.name.equals("door"))doorDetails(b,root,out,"api-parity-door",6);
                if(!sa.observe().equals(cb.observe()))throw new AssertionError("Door diagnostic changed observations");
                if(!a.report("n-values 32 [random-float 1]").equals(compiledReport(b,b.compileReporter("n-values 32 [random-float 1]"))))throw new AssertionError(f.name+" future RNG differs by API");
                apiErrors(a,b);ta.flush();tb.flush();
                if(Files.mismatch(aTrace,bTrace)!=-1)throw new AssertionError(f.name+" full audit trace differs by API");
                String line=String.format(Locale.ROOT,"PASS %s: exact physical/OBS state at %d steps over 6 s; full audit CSV and 32 future RNG draws identical; command/reporter runtime errors propagated and recovered.%n",f.name,steps);
                result.append(line);System.out.print(line);
            }finally{a.dispose();b.dispose();}
        }
        Files.writeString(out.resolve("api-parity-result.txt"),result.toString());
    }
    static void boundCounterexample(Path out)throws Exception{
        double l=2*EXPECTED_RADIUS,eps=.0001,a=.01,b=Math.sqrt(1-a*a),v=1.55,T=.6,h=Math.min(T/2,l*(Math.sqrt(2)-1)/(v*Math.sqrt(2)));
        double d=l+eps,side=b*d,end=d-2*v*a*h,contact=eps/(2*v*a);
        if(side<=l||end>=l)throw new AssertionError("Side-pinch counterexample construction");
        String result=String.format(Locale.ROOT,"Global speed-law cap is insufficient for arbitrary admissible directions.\nl=%.12g eps=%.12g a=%.12g b=%.12g v0=%.12g T=%.12g\nBoth strips exclude: transverse=%.12g > l.\ncap_h=%.12g contact_time=%.12g end_distance=%.12g < l.\nUnconstrained AVM tests measure swept paths and timestep sensitivity without retry subdivision.\n",l,eps,a,b,v,T,side,h,contact,end);
        Files.writeString(out.resolve("global-bound-counterexample.txt"),result);System.out.print(result);
    }
    public static void main(String[] args)throws Exception{
        Locale.setDefault(Locale.ROOT);System.setProperty("java.awt.headless","true");geometry();String mode=opt(args,"--mode","pairs");Path root=Path.of(opt(args,"--model-root",".")),out=Path.of(opt(args,"--output","work/avm-radius020/validation"));Files.createDirectories(out);double dt=Double.parseDouble(opt(args,"--dt",".02"));
        if(mode.equals("order")){order(root,out,dt);return;}if(mode.equals("math")){boundCounterexample(out);return;}if(mode.equals("api-parity")){apiParity(root,out,dt);return;}
        boolean stringApi=Arrays.asList(args).contains("--string-api");System.out.println("Loop API: "+(stringApi?"string":"workspace-local compiled commands/reporters"));
        Set<String> selected=new LinkedHashSet<>(Arrays.asList(opt(args,"--cases",mode.equals("pairs")?"headon-moving,headon-rest,offset,overtake,rightangle,rightangle-symmetric":"hallway,door,traffic,crossing").split(",")));
        String[] seeds=opt(args,"--seeds",mode.equals("pairs")?"42":"42,1,7").split(",");
        try(PrintWriter summary=new PrintWriter(Files.newBufferedWriter(out.resolve("summary-"+mode+"-dt"+dt+".csv")))){
            summary.println("case,seed,outer_dt,requested_s,outer_steps,completed_s,accepted_steps,min_h_s,swept_min_m,min_time_s,max_speed_mps,max_speed_excess_mps,min_wall_m,spawned,arrived,returned,walking,blocked,wraps,final_mean_speed_mps,static_agents_last10s,wall_seconds,first_overlap_s,max_overlap_depth_m,overlap_steps,overlap_pair_intervals,max_overlapping_pairs_in_step,overlap_pair_time_s,clearance_result,error");
            for(String seed:seeds)for(Fixture f:FIXTURES)if(selected.contains(f.name))run(root,out,f,Integer.parseInt(seed),dt,Double.parseDouble(opt(args,"--seconds",Double.toString(f.seconds))),summary,stringApi);
        }
        if(failures>0)throw new AssertionError(failures+" AVM fixture(s) failed; inspect summary/error artifacts");
    }
}
