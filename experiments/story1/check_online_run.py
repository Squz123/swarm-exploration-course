"""Independent reconstruction of actual online message decisions and acceptance."""
import argparse
import json
import math
from pathlib import Path
import numpy as np
import online_protocol as op

def check(path):
    result=json.loads((path/'result.json').read_text())
    events=json.loads((path/'messages.json').read_text())
    rounds=json.loads((path/'rounds.json').read_text())
    assert result['status']=='complete' and result['coverage']==1 and result['teacher_success_log_seen']
    assert result['actual_limits_pass'] and not result['teacher_rule_warnings']
    assert not result.get('assignment_acknowledgements_missing',[])
    box={}
    for e in events:
        box.setdefault((e['round'],e['kind']),{})[e['sender']]=e
    assignments=[e for e in events if e['kind']=='assignment']
    assert len(assignments)>6
    first_epoch=min(e['epoch'] for e in events if e['kind']=='takeoff_ready')
    assert sum(e['begins']>first_epoch+15 for e in assignments)>6
    shared_benefit=0; paused_rounds=0; paused_other_assignments=0; changed_by_pause=0; accepted_segments=0
    for r in rounds:
        k=r['round']
        current=[e for e in assignments if e['round']==k]
        if r['status']=='aborted':
            assert not current, 'Assignment from aborted round'
            continue
        votes=box[(k,'vote')]
        assert op.unanimous(votes) and votes[0]['digest']==r['digest']
        # Drop examiner receipt timestamps to reproduce exactly what peers hashed.
        def messages(kind):
            return {i:{a:b for a,b in v.items() if a!='received_ros_time'} for i,v in box[(k,kind)].items()}
        accepted,digest=op.decision(messages('state'),messages('bid'),messages('proposal'),result['vmax'])
        assert digest==r['digest']
        assert {int(i):task for i,task in r['accepted'].items()}==accepted
        tasks=[e['task'] for e in current]; assert len(tasks)==len(set(tasks))
        assert all(accepted[e['sender']]==e['task'] for e in current)
        for i in accepted:
            for s in op.deserialize(box[(k,'proposal')][i]['segments']):
                distance=float(np.linalg.norm(s.b-s.a))
                assert 1.875*distance/s.T<=result['vmax']+1e-6
                assert (10/math.sqrt(3))*distance/s.T**2<=result['amax']+1e-6
                accepted_segments+=1
        bids=box[(k,'bid')];states=box[(k,'state')]
        if op.allocation(bids)!=op.allocation(bids,ignore_pause=True):changed_by_pause+=1
        shared_benefit+=sum(e['shared_seen']>e['local_seen'] for e in bids.values())
        for i,state in states.items():
            if not state['enabled']:
                paused_rounds+=1
                assert i not in accepted
                paused_other_assignments+=sum(e['sender']!=i for e in current)
    assert shared_benefit>0
    injected=[e for e in events if e['kind']=='fault_injected']
    for e in injected:
        assert next(r for r in rounds if r['round']==e['round'])['status']=='aborted'
    if result['pause_uav']>=0:
        assert paused_rounds>0 and paused_other_assignments>0 and changed_by_pause>0
    report={'run':path.name,'pass':True,'teacher_seconds':result['teacher_mission_duration_wall_s'],
            'actual_assignment_messages':len(assignments),'six_peer_agreement_rounds':sum(r['status']=='committed' for r in rounds),
            'peer_messages_benefiting_from_shared_coverage':shared_benefit,
            'fault_injections_safely_aborted':len(injected),'paused_states':paused_rounds,
            'other_assignments_during_pause':paused_other_assignments,'allocation_rounds_changed_by_pause_message':changed_by_pause,
            'accepted_segments_with_analytic_dynamic_check':accepted_segments}
    (path/'independent_check.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('path',type=Path)
    check(parser.parse_args().path)
