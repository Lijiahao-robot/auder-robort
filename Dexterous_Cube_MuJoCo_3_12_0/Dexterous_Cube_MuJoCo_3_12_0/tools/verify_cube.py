from collections import Counter
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

import numpy as np
from dexcube.cube import CubeState, FACE_NORMALS, inverse_move, parse_move, scramble_moves
from dexcube.simulation import CubeTask


def logic():
    solved=CubeState().signature()
    checks={}
    for face in FACE_NORMALS:
        for suffix in ('',"'",'2'):
            s=CubeState();token=face+suffix;s.apply(token)
            assert not s.solved
            assert len(s.selected(face))==9
            assert sorted(Counter(color for colors in s.facelets().values() for color in colors).values())==[9]*6
            s.apply(inverse_move(token));assert s.signature()==solved
        s=CubeState()
        for _ in range(4):s.apply(face)
        assert s.signature()==solved
    checks['18_moves_inverse_and_four_turn_identity']=True
    a,b=CubeState(),CubeState()
    for t in ['R','U']:a.apply(t)
    for t in ['U','R']:b.apply(t)
    assert a.signature()!=b.signature()
    checks['noncommuting_layers']=True
    s=CubeState()
    for _ in range(6):
        for t in ['R','U',"R'","U'"]:s.apply(t)
    assert s.solved
    checks['six_commutators_identity']=True
    s=CubeState()
    for t in scramble_moves(30,2026):s.apply(t)
    assert len({tuple(p.position) for p in s.cubies})==26
    for p in s.cubies:
        assert np.array_equal(p.orientation.T@p.orientation,np.eye(3,dtype=int))
        assert round(np.linalg.det(p.orientation))==1
    for t in s.inverse_history():s.apply(t)
    assert s.solved
    checks['long_history_inverse_and_cubie_orientations']=True
    for bad in ['M','R3',"R'2",'u','',None]:
        try:parse_move(bad);raise AssertionError('bad token accepted')
        except ValueError:pass
    checks['bad_moves_rejected']=True
    return {'passed':True,'checks':checks}


def step_until_ready(task,limit=30):
    deadline=float(task.data.time)+limit
    while task.data.time<deadline and not task.error:
        task.step()
        if not task.current and not task.moves and task.phase=='READY':return
    raise AssertionError('Task did not settle: '+str(task.error))


def physical_layers():
    s=CubeTask()
    for _ in range(400):s.step()
    assert s.grasped and len(s.contacts)>=3
    s.enqueue(['D',"L'",'B2']);step_until_ready(s)
    assert not s.state.solved and s.completed==3
    s.restore();step_until_ready(s)
    assert s.state.solved and s.completed==6
    assert s.max_layer_error<.008 and s.max_hand_limit_violation<.025 and s.max_topology_position_jump<.0005
    return {'passed':True,'report':s.report()}


def negative_no_contact():
    s=CubeTask()
    # Disable every hand collision before the first physics step.
    mask=s.model.geom_contype==2
    s.model.geom_contype[mask]=0;s.model.geom_conaffinity[mask]=0
    s.enqueue(['R'])
    for _ in range(850):s.step()
    assert not s.grasped and not s.data.eq_active[s.weld] and s.completed==0 and s.max_contacts==0
    assert s.data.xpos[s.cube_body,2]<.12
    return {'passed':True,'report':s.report()}


def release():
    s=CubeTask()
    for _ in range(350):s.step()
    initial=float(s.data.xpos[s.cube_body,2])
    assert s.grasped
    s.release()
    for _ in range(850):s.step()
    final=float(s.data.xpos[s.cube_body,2])
    assert not s.grasped and not s.data.eq_active[s.weld] and final<initial-.08
    return {'passed':True,'initial_height_m':initial,'final_height_m':final,'report':s.report()}


def main():
    results={'cube_logic':logic(),'remaining_faces_and_half_turns':physical_layers(),
             'negative_no_contact':negative_no_contact(),'release_and_gravity':release()}
    results['success']=all(case['passed'] for case in results.values())
    (ROOT/'results/validation_cube.json').write_text(json.dumps(results,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v['passed'] for k,v in results.items() if isinstance(v,dict)},indent=2))


if __name__=='__main__':main()
