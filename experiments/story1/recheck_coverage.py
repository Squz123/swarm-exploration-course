"""Replay recorded segments through a separate port of region_vis.cpp's FOV
sample/rasterize logic. This is still NOT execution of ROS/region_visualizer.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from benchmark import Segment, STARTS, trajectory_positions, generate_map


def raster_footprint(pos):
    r=pos[2]*.5
    xmin,ymin=np.maximum(-25,pos[:2]-r)
    xmax,ymax=np.minimum(25,pos[:2]+r)
    # Preserve C++'s repeated x += 0.1, not xmin + k*0.1. The arithmetic order
    # matters at grid boundaries: floor/truncation otherwise skips columns.
    # numpy.cumsum performs sequential additions for this floating dtype.
    xx=np.cumsum(np.r_[xmin,np.full(max(0,int((xmax-xmin)/.1)+3),.1)])
    yy=np.cumsum(np.r_[ymin,np.full(max(0,int((ymax-ymin)/.1)+3),.1)])
    xx=xx[xx<=xmax];yy=yy[yy<=ymax]
    mask=(xx[None,:]-pos[0])**2+(yy[:,None]-pos[1])**2<=r*r
    iy,ix=np.nonzero(mask)
    gx=np.trunc((xx[ix]+25)/.1).astype(int)
    gy=np.trunc((yy[iy]+25)/.1).astype(int)
    good=(gx>=0)&(gx<500)&(gy>=0)&(gy<500)
    return gy[good]*500+gx[good]


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('replay',type=Path)
    ap.add_argument('--dt',type=float,default=.01)
    args=ap.parse_args()
    data=json.loads(args.replay.read_text(encoding='utf-8'))
    _,obstacle,_=generate_map(data['seed'])
    free=~obstacle.ravel();seen=np.zeros(250000,bool)
    histories=[[Segment(float(st),float(T),np.array(a),np.array(b)) for st,T,a,b in h] for h in data['segments']]
    finish=data['result']['elapsed_sim_s']
    ts=np.arange(0,finish+args.dt*.5,args.dt)
    poses=np.stack([trajectory_positions(h,ts,STARTS[i]) for i,h in enumerate(histories)],axis=1)
    start=time.perf_counter();t95=None;t100=None
    for t,frame in zip(ts,poses):
        for pos in frame:seen[raster_footprint(pos)]=True
        count=int(np.count_nonzero(seen&free))
        if t95 is None and count/free.sum()>=.95:t95=float(t)
        if count==free.sum():t100=float(t);break
    result={'seed':data['seed'],'variant':data['variant'],'scope':'Separate source-style rasterization replay, no ROS execution',
            'replay_dt_s':args.dt,'coverage':float(np.count_nonzero(seen&free)/free.sum()),
            'uncovered_cells':int(np.count_nonzero(free&~seen)),'source_style_t95_s':t95,'source_style_t100_s':t100,
            'benchmark_conservative_t100_s':data['result']['t100_s'],
            'recheck_wall_s':round(time.perf_counter()-start,3)}
    out=args.replay.with_name(args.replay.stem.replace('_replay','_coverage_recheck')+'.json')
    out.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
