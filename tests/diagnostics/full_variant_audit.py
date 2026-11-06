"""Matched single-seed diagnostic; alternative predictions are NOT paper fidelity fixes."""
import csv
import json
import time
import numpy as np
import independent_solver as solver

initial = []
with (solver.ROOT/'tests/fixtures/initial-state.csv').open(newline='') as file:
    for row in csv.DictReader(file):
        if float(row['time']) != .02:
            break
        initial.append({key:float(value) for key,value in row.items()})
initial.sort(key=lambda row:row['who'])
assert len(initial) == solver.N == 150
assert np.allclose([row['desired'] for row in initial],solver.speed)
assert np.allclose([row['direction'] for row in initial],solver.direction)
initial_state = np.array([[row['x'],row['y']] for row in initial]
                       +[[row['wx'],row['wy']] for row in initial])
upper = np.triu_indices(solver.N,k=1)


def run(prediction):
    state=initial_state.copy()
    n=solver.N
    dt=.02
    steps=2999  # Saved first state t=.02 through t=60.00.
    minsep=float('inf')
    minpair=None
    overlap=near=0
    overlap_steps=near_steps=0
    min_t=0
    lateral=forward=total_speed=0.
    post_steps=0
    peakw=0
    peakwtime=0
    peakwid=0
    contacts=0
    sideways=0
    current_sideways=np.zeros(n,dtype=int)
    longest_sideways=np.zeros(n,dtype=int)
    started=time.perf_counter()
    metrics=[]
    for k in range(steps):
        rate=solver.rhs(state,1e-6,prediction,'force')
        state[n:]+=dt*rate[n:]
        freepos=state[:n]+dt*solver.cap(state[n:])
        bottom=freepos[:,1]<1.7
        top=freepos[:,1]>11.3
        hit=bottom|top
        contacts+=np.count_nonzero(hit)
        state[n:,1][bottom]=np.maximum(state[n:,1][bottom],0)
        state[n:,1][top]=np.minimum(state[n:,1][top],0)
        actual=solver.cap(state[n:])
        freepos[hit,0]=state[:n][hit,0]+dt*actual[hit,0]
        freepos[:,1]=np.clip(freepos[:,1],1.7,11.3)
        freepos[:,0]%=50
        state[:n]=freepos
        elapsed=.02+(k+1)*dt
        wnorm=np.linalg.norm(state[n:],axis=1)
        if wnorm.max()>peakw:
            peakw=float(wnorm.max())
            peakwtime=elapsed
            peakwid=int(wnorm.argmax())
        q=state[:n,None,:]-state[None,:n,:]
        q[:,:,0]=(q[:,:,0]+25)%50-25
        distances=np.linalg.norm(q,axis=2)[upper]
        stepmin=float(distances.min())
        if stepmin<minsep:
            minsep=stepmin
            j=int(distances.argmin())
            minpair=[int(upper[0][j]),int(upper[1][j])]
            min_t=elapsed
        overlap_count=int(np.count_nonzero(distances<.4))
        near_count=int(np.count_nonzero(distances<.1))
        overlap+=overlap_count
        near+=near_count
        overlap_steps+=overlap_count>0
        near_steps+=near_count>0
        lateral_dominant=np.abs(actual[:,1])>np.abs(actual[:,0])
        current_sideways=np.where(lateral_dominant,current_sideways+1,0)
        longest_sideways=np.maximum(longest_sideways,current_sideways)
        if elapsed>=10:
            lateral+=np.abs(actual[:,1]).mean()
            forward+=(actual[:,0]*solver.direction).mean()
            total_speed+=np.linalg.norm(actual,axis=1).mean()
            sideways+=int(lateral_dominant.sum())
            post_steps+=1
        if k%50==0:
            metrics.append([elapsed,stepmin,overlap_count,near_count,
                np.abs(actual[:,1]).mean(),(actual[:,0]*solver.direction).mean(),wnorm.max()])
        if (k+1)%1000==0:
            print(f'{prediction}: {elapsed:.2f}s simulated ({time.perf_counter()-started:.1f}s wall)',flush=True)
    pair_steps=steps*len(upper[0])
    result=dict(prediction=prediction,seed=42,start=.02,end=60,dt=dt,n=n,
        min_separation=minsep,min_pair=minpair,time_of_min=min_t,
        overlap_pair_time_fraction=overlap/pair_steps,
        nearcoincidence_pair_time_fraction=near/pair_steps,
        mean_overlapping_pairs=overlap/steps,
        mean_nearcoincident_pairs=near/steps,
        fraction_steps_with_overlap=overlap_steps/steps,
        fraction_steps_with_nearcoincidence=near_steps/steps,
        mean_abs_lateral_after10=lateral/post_steps,
        mean_forward_after10=forward/post_steps,
        mean_speed_after10=total_speed/post_steps,
        lateral_dominant_pedestrian_time_fraction_after10=sideways/(post_steps*n),
        longest_lateral_dominance_seconds=float(longest_sideways.max()*dt),
        longest_lateral_dominance_id=int(longest_sideways.argmax()),
        peak_preferred_speed=peakw,peak_preferred_id=peakwid,peak_preferred_time=peakwtime,
        wall_contact_agent_steps=int(contacts),runtime=time.perf_counter()-started)
    np.savetxt(solver.ROOT/f'work/full-variant-{prediction}.csv',metrics,delimiter=',',
        header='time,minsep,overlap_pairs,near_pairs,mean_abs_lateral,mean_forward,max_preferred')
    print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    results=[]
    for variant in ['desired','actual','relative']:
        results.append(run(variant))
        (solver.ROOT/'work/full-variant-results.json').write_text(json.dumps(results,indent=2))
