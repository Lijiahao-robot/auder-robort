"""Deterministic physics checks, executable without a display or pytest."""
import os
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

import json
import math
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

from atlas_sim.simulation import AtlasSimulation, ROOT


def exercise(mode, duration, velocity=None, push=False, stop=True):
    sim = AtlasSimulation()
    for _ in range(1500):
        sim.step()
    start = sim.data.qpos[:3].copy()
    if velocity is not None:
        sim.set_velocity(*velocity)
    else:
        sim.set_mode(mode)
    if push:
        sim.push()
    minimum_height, max_external = 10., 0.
    sole_max = np.zeros(2)
    off_ground = np.zeros(2, dtype=int)
    com_heights, hand_positions, forces, normal_forces = [], [], [], []
    for i in range(round(duration/sim.dt)):
        sim.step()
        assert not sim.fallen, f'{mode} fell at {sim.data.time:.3f}s'
        assert np.isfinite(sim.data.qpos).all() and np.isfinite(sim.data.qvel).all()
        minimum_height = min(minimum_height, float(sim.data.xpos[sim.pelvis, 2]))
        max_external = max(max_external, float(np.max(np.abs(sim.data.xfrc_applied))))
        sole_max = np.maximum(sole_max, sim.data.site_xpos[sim.site_ids, 2])
        if i % 25 == 0:
            force = sim.contact_forces()
            off_ground += np.array(force) < 2.
            com_heights.append(float(sim.data.subtree_com[sim.pelvis, 2]))
            hand_positions.append(sim.data.body('r_lfarm').xpos.copy())
            normal_forces.append(sum(force))
    running = sim.status()
    if mode == 'stand':
        assert abs(np.mean(normal_forces[-100:])-sim.mass*9.81) < .03*sim.mass*9.81
    if mode == 'wave':
        assert np.max(np.ptp(np.array(hand_positions), axis=0)) > .12, 'hand must actually move'
        assert np.max(np.array(hand_positions)[:, 2]) > 1.35, 'greeting hand must lift near shoulder height'
    if mode == 'squat':
        assert np.ptp(com_heights) > .06, 'COM must actually descend and rise'
    if mode == 'walk':
        assert sim.step_number >= 5
        assert np.all(sole_max > .035), 'both feet must lift'
        assert np.all(off_ground > 10), 'both feet must lose contact during alternating steps'
        if velocity[0] > 0:
            assert sim.data.qpos[0]-start[0] > .5
        elif velocity[0] < 0:
            assert sim.data.qpos[0]-start[0] < -.35
        elif velocity[1] > 0:
            assert running['pelvis_yaw_rad'] > .5
        elif velocity[1] < 0:
            assert running['pelvis_yaw_rad'] < -.5
        if stop:
            sim.set_mode('stand')
            for _ in range(4000):
                sim.step()
                assert not sim.fallen, 'walk-to-stand transition must remain stable'
            assert sim.mode == 'stand' and sim.phase == 'stand'
    assert sim.max_torque_ratio <= 1.+1e-9
    if not push:
        assert max_external == 0., 'no external base support is allowed'
    for warning in sim.data.warning:
        assert warning.number == 0, 'MuJoCo numerical/capacity warning'
    return {'passed': True, 'requested_seconds': duration, 'running_state': running,
            'final_state': sim.status(), 'net_pelvis_displacement_m': (sim.data.qpos[:3]-start).tolist(),
            'minimum_pelvis_height_m': minimum_height,
            'max_sole_height_m': sole_max.tolist(), 'off_ground_samples': off_ground.tolist(),
            'max_external_wrench_component': max_external}


def main():
    if mujoco.__version__ != '3.12.0':
        raise RuntimeError(f'Requires MuJoCo 3.12.0, got {mujoco.__version__}')
    begun = time.perf_counter()
    cases = [
        ('stand_45s', dict(mode='stand', duration=45)),
        ('wave_15s', dict(mode='wave', duration=15)),
        ('squat_20s', dict(mode='squat', duration=20)),
        ('dance_20s', dict(mode='dance', duration=20)),
        ('forward_40s_then_stop', dict(mode='walk', duration=40, velocity=(.03, 0))),
        ('backward_25s_then_stop', dict(mode='walk', duration=25, velocity=(-.025, 0))),
        ('left_turn_40s_then_stop', dict(mode='walk', duration=40, velocity=(0, .04))),
        ('right_turn_40s_then_stop', dict(mode='walk', duration=40, velocity=(0, -.04))),
        ('35N_0p12s_push_then_balance', dict(mode='stand', duration=15, push=True)),
    ]
    results = []
    target = ROOT/'results'/'verification_report.json'

    def save():
        report = {'mujoco_version': mujoco.__version__, 'python': sys.version.split()[0],
                  'platform': sys.platform, 'all_passed': all(r['passed'] for r in results),
                  'case_count': len(results), 'wall_seconds': round(time.perf_counter()-begun, 3),
                  'note': 'Headless physics tests. Real MuJoCo contacts and torque actuators. No RL policy.',
                  'checks': results}
        target.parent.mkdir(exist_ok=True, parents=True)
        target.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf8')

    for name, kwargs in cases:
        try:
            result = {'name': name, **exercise(**kwargs)}
            print(f'PASS {name}', flush=True)
        except Exception as exc:
            result = {'name': name, 'passed': False, 'error': f'{type(exc).__name__}: {exc}'}
            print(f'FAIL {name}: {exc}', flush=True)
        results.append(result)
        save()
    try:
        sim = AtlasSimulation()
        for _ in range(1000):
            sim.step()
        before, t = sim.data.qpos.copy(), float(sim.data.time)
        sim.paused = True
        for _ in range(100):
            sim.step()
        assert np.array_equal(before, sim.data.qpos) and sim.data.time == t
        sim.reset()
        assert sim.data.time == 0 and not sim.fallen and not sim.paused
        assert sim.mode == 'stand' and np.array_equal(sim.data.qvel, np.zeros(sim.model.nv))
        assert sim.model.nq == 34 and sim.model.nv == 33 and sim.model.nu == 27
        assert int(sim.model.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        assert sim.model.neq == 0, 'no base weld or equality support'
        results.append({'name': 'pause_reset_floating_base_contract', 'passed': True})
        print('PASS pause_reset_floating_base_contract', flush=True)
    except Exception as exc:
        results.append({'name': 'pause_reset_floating_base_contract', 'passed': False, 'error': str(exc)})
    save()
    print(f'Report: {target}')
    return 0 if all(r['passed'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
