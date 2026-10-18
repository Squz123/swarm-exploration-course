"""Focused regression checks for the two consequential validation paths."""
import json
import math
from pathlib import Path
import numpy as np
from recheck_coverage import raster_footprint
from benchmark import self_check


def literal_source_raster(pos):
    # Deliberately scalar reference: direct while-loop translation of
    # region_vis.cpp, independent of the vectorized cumsum implementation.
    px,py,z=pos;r=.5*z
    x=max(-25,px-r);xmax=min(25,px+r);ymin=max(-25,py-r);ymax=min(25,py+r)
    cells=set()
    while x<=xmax:
        y=ymin
        while y<=ymax:
            dx=x-px;dy=y-py
            if dx*dx+dy*dy<=r*r:
                gx=int((x+25)/.1);gy=int((y+25)/.1)
                if 0<=gx<500 and 0<=gy<500:cells.add(gy*500+gx)
            y+=.1
        x+=.1
    return cells


def main():
    self_check()
    for point in [[-2.,-1.,1.],[1.23,2.34,7.5],[-22.5,-22.5,7.5],[24.7,-24.2,7.5]]:
        assert set(raster_footprint(np.array(point)))==literal_source_raster(point),point
    root=Path(__file__).resolve().parent
    summary=json.loads((root/'results/summary.json').read_text(encoding='utf-8'))
    assert summary['predeclared_local_stop_condition_pass']
    assert summary['source_style_100hz_C_coverage_recheck_all_pass']
    assert summary['improved_maps']==10
    for seed in range(514,524):
        for variant in 'ABC':
            row=json.loads((root/f'results/holdout/{seed}_{variant}.json').read_text(encoding='utf-8'))
            assert row['coverage']==1 and row['uncovered_cells']==0
            assert row['safety']['pass']
            assert row['safety']['min_pair_certified_lower_bound_m']>=.4
            assert row['safety']['min_obstacle_certified_lower_bound_m']>=.2
    print('Result checks passed: scalar-source raster fixtures, 30 holdout runs, 10 independent 100 Hz coverage replays, predeclared stop condition.')


if __name__=='__main__':main()
