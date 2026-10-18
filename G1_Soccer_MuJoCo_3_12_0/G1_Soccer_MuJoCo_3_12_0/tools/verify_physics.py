"""Check both feet, absence of injected ball motion, and whole-ball scoring."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import mujoco
import numpy as np

from g1soccer.simulation import SoccerTask

def trial(left=False, foot_contacts=True):
    task = SoccerTask()
    for _ in range(650):
        task.step(now=0)
    task.reset_ball()
    if left:
        # Explicit test-scene setup before the kick, not an impact impulse.
        task.data.qpos[task.ball_qa + 1] = .12
        mujoco.mj_forward(task.model, task.data)
        task._previous_ball = task.ball_position.copy()
    if not foot_contacts:
        for gid in task.pads.values():
            task.model.geom_contype[gid] = 0
            task.model.geom_conaffinity[gid] = 0
    origin = task.ball_position.copy()
    task.kick()
    for _ in range(3700):
        task.step(now=0)
    report = task.report()
    moved = float(np.linalg.norm(task.ball_position[:2] - origin[:2]))
    if foot_contacts:
        passed = task.kick_contacts > 0 and task.max_ball_speed > 1 and task.goals == 1 and task.max_joint_violation < .02 and task.error is None
    else:
        passed = task.kick_contacts == 0 and moved < .001 and task.goals == 0 and task.error is None
    return {'passed': passed, 'left_foot': left, 'foot_contacts_enabled': foot_contacts, 'ball_horizontal_displacement_m': moved, 'report': report}


def scoring():
    task = SoccerTask()
    for a, b in [([4, 1.0, .2], [4.2, 1.0, .2]), ([4, 0, 1.0], [4.2, 0, 1.0]),
                 ([4, 2, .2], [4.2, 0, .2]), ([4, 0, -.3], [4.2, 0, -.3])]:
        task.check_goal(np.array(a, dtype=float), np.array(b, dtype=float))
        assert task.goals == 0
    task.check_goal(np.array([4, 0, .2]), np.array([4.2, 0, .2]))
    assert task.goals == 1
    task.check_goal(np.array([4, 0, .2]), np.array([4.2, 0, .2]))
    assert task.goals == 1
    task.goal_latched, task.goals = False, 0
    task.check_goal(np.array([-4, 0, .2]), np.array([-4.2, 0, .2]))
    assert task.goals == 1
    return {'passed': True, 'checks': ['outside posts rejected', 'above crossbar rejected', 'crossing-point interpolation',
                                     'below floor rejected', 'whole ball crosses line', 'no repeated scoring', 'both goal directions']}


def main():
    reports = {'right_foot': trial(), 'left_foot': trial(left=True),
               'negative_no_foot_contact': trial(foot_contacts=False), 'scoring': scoring()}
    reports['success'] = all(case['passed'] for case in reports.values())
    (ROOT / 'results/validation_physics.json').write_text(json.dumps(reports, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({key: value['passed'] for key, value in reports.items() if isinstance(value, dict)}, indent=2))
    if not reports['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
