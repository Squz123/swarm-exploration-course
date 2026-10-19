"""Pure message-based allocation and trajectory checks for six ROS peers.

No teacher coverage input. A round is committed only after all six peers vote
for the same task allocation AND the same safe subset of proposed trajectories.
This is an all-to-all, reliable-connected-network prototype, not fault-tolerant
consensus under arbitrary partitions. Missing messages cause a round to abort.
"""
import base64
import hashlib
import json
import zlib
import numpy as np
import benchmark as b

N = 6

def pack_seen(seen):
    return base64.b64encode(zlib.compress(np.packbits(seen).tobytes())).decode('ascii')

def unpack_seen(value):
    data = zlib.decompress(base64.b64decode(value))
    if len(data) != 31250:
        raise ValueError('Wrong coverage bitset size')
    return np.unpackbits(np.frombuffer(data, dtype=np.uint8)).astype(bool)

def serialize(history):
    return [s.serial() for s in history]

def deserialize(values):
    return [b.Segment(float(t), float(T), np.array(a), np.array(c)) for t,T,a,c in values]

def allocation(bids, ignore_pause=False):
    """Identical deterministic greedy matching at every peer; bids come from peers."""
    edges = []
    for i in range(N):
        message = bids[i]
        if not message['idle'] or (not message['enabled'] and not ignore_pause):
            continue
        for task, cost in message['offers']:
            edges.append((round(float(cost), 6), int(task), i))
    used = set(); result = {}
    for cost, task, i in sorted(edges):
        if i not in result and task not in used:
            result[i] = task; used.add(task)
    return result

def trajectory_safe(candidate, histories, drone, vmax):
    if not candidate:
        return False
    begin = candidate[0].start
    end = max([candidate[-1].end]+[h[-1].end for h in histories])
    dt = .025
    ts = np.arange(begin, end+dt, dt)
    own = b.trajectory_positions(candidate, ts, candidate[0].a)
    # Relative speed <= 2*vmax, distance to nearest sample <= dt/2.
    margin = .80 + vmax*dt
    for j,h in enumerate(histories):
        if j == drone:
            continue
        other = b.trajectory_positions(h, ts, b.STARTS[j])
        if np.linalg.norm(own-other, axis=1).min() < margin:
            return False
    return True

def decision(states, bids, proposals, vmax):
    chosen = allocation(bids)
    histories = [deserialize(states[i]['history']) for i in range(N)]
    accepted = {}
    # Rotating priority is explicitly carried in each proposal, avoiding a
    # permanent preference for the lowest UAV id at intersecting routes.
    round_id = proposals[0]['round']
    order = [(round_id+k)%N for k in range(N)]
    for i in order:
        p = proposals[i]
        if i not in chosen or p['task'] != chosen[i] or not p['segments']:
            continue
        segments = deserialize(p['segments'])
        current = histories[i][-1]
        if segments[0].start < current.end or np.linalg.norm(segments[0].a-current.b) > 1e-6:
            continue
        if trajectory_safe(segments, histories, i, vmax):
            accepted[i] = chosen[i]
            histories[i] = segments
    # Bind the vote to exact state/bid/proposal content, not just the winners.
    content = {'states':states,'bids':bids,'proposals':proposals,'accepted':accepted}
    digest = hashlib.sha256(json.dumps(content, sort_keys=True, separators=(',',':')).encode()).hexdigest()
    return accepted, digest

def unanimous(votes):
    return len(votes) == N and len({v['digest'] for v in votes.values()}) == 1

def self_test():
    bids = {i:{'idle':True,'enabled':True,'offers':[[6, i+1],[7, 8-i]]} for i in range(N)}
    assert allocation(bids) == {0:6, 5:7}
    bids[0]['enabled'] = False
    assert allocation(bids) == {1:6, 5:7}
    assert allocation(bids, True) == {0:6, 5:7}
    votes = {i:{'digest':'a'} for i in range(N)}
    assert unanimous(votes)
    assert not unanimous({i:v for i,v in votes.items() if i != 2})
    votes[2]['digest'] = 'b'; assert not unanimous(votes)
    seen = np.arange(250000)%7 == 0
    assert np.array_equal(seen, unpack_seen(pack_seen(seen)))
    h = [[b.Segment(0,1,np.array([i*3.,0,7.5]),np.array([i*3.,0,7.5]))] for i in range(N)]
    crossing = [b.Segment(2,4,h[0][-1].b,h[1][-1].b)]
    assert not trajectory_safe(crossing,h,0,4.5)
    print('ONLINE_PROTOCOL_CHECKS_PASS: deterministic allocation, pause, missing/conflicting votes, coverage, collision')

if __name__ == '__main__':
    self_test()
