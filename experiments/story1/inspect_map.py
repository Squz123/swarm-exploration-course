import benchmark as b
import numpy as np
original=b.World.make_graph
b.World.make_graph=lambda w:None
w=b.World(513)
print('start3d',b.box_dist(b.STARTS,w.boxes))
print('cruise',b.box_dist(np.c_[b.STARTS[:,:2],np.full(6,b.Z)],w.boxes))
print('clear',w.clear(b.STARTS[:,:2]))
print('near',w.boxes[np.linalg.norm(w.boxes[:,:2],axis=1)<6])
print('targets',len(w.targets))
try:
    original(w)
except RuntimeError:
    print('bad sources',np.flatnonzero(~np.isfinite(w.cost).all(axis=1)))
    print('bad from 0',np.flatnonzero(~np.isfinite(w.cost[0])))
