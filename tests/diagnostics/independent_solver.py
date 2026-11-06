"""Independent NumPy solver for the saved periodic Helbing--Molnar encounter.

Integrates r and w; v is cap(w), never a separately integrated state.
No NetLogo code executes here. RK4 and the production's first-order method
are compared from exactly the same snapshot. The default runs preserve the original force. --variants runs deliberate
model changes for diagnosis, not implementations of the 1995 equations.
Requires NumPy. Input is a fixed seed-42 snapshot at 20.52 seconds; generated
reports and trajectories go to ignored work/. No production files are edited.
"""
from pathlib import Path
import csv, json, time, argparse
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
(ROOT / 'work').mkdir(exist_ok=True)
with (ROOT / 'tests/fixtures/collision-state.csv').open(newline='') as file:
    rows = [{key: float(value) for key, value in row.items()}
            for row in csv.DictReader(file)]
assert [row['who'] for row in rows] == list(range(len(rows)))
N=len(rows)
pos0=np.array([[r['xcor'],r['ycor']] for r in rows])
w0=np.array([[r['wx'],r['wy']] for r in rows])
v0=np.array([[r['vx'],r['vy']] for r in rows])
speed=np.array([r['desired-speed'] for r in rows])
direction=np.array([r['travel-direction'] for r in rows])
desired=np.column_stack((speed*direction,np.zeros(N)))
diagonal=np.eye(N,dtype=bool)
cosphi=np.cos(np.deg2rad(100.))

def cap(w):
    norm=np.linalg.norm(w,axis=1)
    factor=np.minimum(1.,1.3*speed/np.maximum(norm,1e-300))
    return w*factor[:,None]

def rhs(state,eps,prediction='desired',perception='force'):
    r,w=state[:N],state[N:]
    v=cap(w)
    q=r[:,None,:]-r[None,:,:]
    q[:,:,0]=(q[:,:,0]+25.)%50.-25.
    if prediction=='desired':
        future=2.*np.column_stack((np.linalg.norm(v,axis=1)*direction,np.zeros(N)))
        displacement=future[None,:,:]
    elif prediction=='actual':
        displacement=2.*v[None,:,:]
    elif prediction=='relative':
        displacement=2.*(v[None,:,:]-v[:,None,:])
    else: raise ValueError(prediction)
    second=q-displacement
    d1=np.sqrt(np.sum(q*q,axis=2)+eps*eps)
    d2=np.sqrt(np.sum(second*second,axis=2)+eps*eps)
    d1[diagonal]=1.;d2[diagonal]=1.
    total=d1+d2
    length=np.linalg.norm(displacement,axis=2)
    rad=(total-length)*(total+length)
    b=.5*np.sqrt(np.maximum(4*eps*eps,np.maximum(rad,4e-24)))
    scale=7.*np.exp(-b/.3)*total/(4.*b)
    f=scale[:,:,None]*(q/d1[:,:,None]+second/d2[:,:,None])
    f[diagonal]=0.
    norm=np.linalg.norm(f,axis=2)
    if perception=='force':
        weights=np.where(-direction[:,None]*f[:,:,0]>=norm*cosphi,1.,.5)
    elif perception=='full':
        weights=np.ones_like(norm)
    else: raise ValueError(perception)
    acc=(desired-v)/.5 + (f*weights[:,:,None]).sum(axis=1)
    for wall_y in [1.5,11.5]:
        dy=r[:,1]-wall_y
        acc[:,1]+=50.*np.exp(-np.abs(dy)/.2)*np.sign(dy)
    return np.concatenate((v,acc))

def contacts(state):
    # Same added hard-wall condition, only when actually needed.
    hit_bottom=state[:N,1]<1.7
    hit_top=state[:N,1]>11.3
    hits=int(np.count_nonzero(hit_bottom|hit_top))
    state[:N,1]=np.clip(state[:N,1],1.7,11.3)
    state[N:,1][hit_bottom]=np.maximum(state[N:,1][hit_bottom],0)
    state[N:,1][hit_top]=np.minimum(state[N:,1][hit_top],0)
    return hits

def run(method,dt,eps,duration=.8,prediction='desired',perception='force'):
    state=np.concatenate((pos0.copy(),w0.copy()))
    minsep=100.;min_t=0.;contacts_n=0;passed=False
    started=time.perf_counter()
    trajectory=[]
    lateral=0.;pair_lateral=0.
    for k in range(round(duration/dt)):
        if method=='production':
            rate=rhs(state,eps,prediction,perception)
            state[N:]+=dt*rate[N:]
            state[:N]+=dt*cap(state[N:])
        else:
            k1=rhs(state,eps,prediction,perception)
            k2=rhs(state+.5*dt*k1,eps,prediction,perception)
            k3=rhs(state+.5*dt*k2,eps,prediction,perception)
            k4=rhs(state+dt*k3,eps,prediction,perception)
            state+=dt*(k1+2*k2+2*k3+k4)/6.
        hits = contacts(state)
        if hits:
            raise RuntimeError('Unexpected wall contact in the saved interior encounter')
        contacts_n += hits
        q=state[3]-state[52]
        q[0]=(q[0]+25)%50-25
        d=np.linalg.norm(q)
        if d<minsep:minsep=d;min_t=(k+1)*dt
        passed=passed or q[0]<0
        actual=cap(state[N:])
        lateral+=dt*np.mean(np.abs(actual[:,1]))
        pair_lateral+=dt*np.mean(np.abs(actual[[3,52],1]))
        if k%max(1,round(.005/dt))==0:
            trajectory.append([(k+1)*dt,*state[3],*state[52],d])
    label=f'{method}-{dt}-{eps}-{prediction}-{perception}'
    result=dict(method=method,dt=dt,epsilon=eps,min_separation=minsep,time_of_min=min_t,
                passed=bool(passed),wall_contact_steps=contacts_n,
                prediction=prediction,perception=perception,
                mean_lateral_speed=lateral/duration,pair_mean_lateral_speed=pair_lateral/duration,
                position3=state[3].tolist(),position52=state[52].tolist(),
                runtime=time.perf_counter()-started)
    np.savetxt(ROOT/f'work/independent-{label}.csv',trajectory,delimiter=',',header='time,x3,y3,x52,y52,distance')
    print(json.dumps(result),flush=True)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser()
    group=parser.add_mutually_exclusive_group()
    group.add_argument('--variants',action='store_true')
    group.add_argument('--quick',action='store_true')
    options=parser.parse_args()
    print('Initial cap mismatch:',np.max(np.abs(cap(w0)-v0)),flush=True)
    results=[]
    if options.variants:
        for prediction,perception in [('desired','force'),('actual','force'),('relative','force'),('desired','full')]:
            results.append(run('rk4',.001,1e-6,prediction=prediction,perception=perception))
        filename='independent-variants.json'
    else:
        cases=[('production',.02,1e-6),('rk4',.001,1e-6)]
        if not options.quick:
            cases += [('rk4',.0002,1e-6),('rk4',.0002,0.)]
        for method,dt,eps in cases:
            results.append(run(method,dt,eps))
        filename='independent-quick.json' if options.quick else 'independent-results.json'
    (ROOT/f'work/{filename}').write_text(json.dumps(results,indent=2))
