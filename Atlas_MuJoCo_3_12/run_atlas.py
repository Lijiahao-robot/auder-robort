"""Native MuJoCo viewer, deterministic headless runs and run reports."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import queue
import sys
import time
from pathlib import Path

# Keep small whole-body least-squares solves responsive on desktop BLAS builds.
os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

ROOT = Path(__file__).resolve().parent


def arguments():
    p = argparse.ArgumentParser(description='Atlas legacy / MuJoCo 3.12.0 tutorial')
    p.add_argument('--mode', choices=['stand', 'wave', 'squat', 'dance', 'walk'], default='stand')
    p.add_argument('--headless', action='store_true', help='physics only; no graphics window required')
    p.add_argument('--seconds', type=float, default=None, help='simulated duration; native viewer defaults to unlimited')
    p.add_argument('--demo', action='store_true', help='48-second automatic motion demonstration')
    p.add_argument('--snapshot', type=Path, help='save the final actual MuJoCo frame as PNG')
    p.add_argument('--report', type=Path, default=ROOT/'results'/'run_report.json')
    p.add_argument('--csv', type=Path, default=ROOT/'results'/'trajectory.csv')
    p.add_argument('--render-backend', choices=['egl', 'osmesa', 'glfw'], help='optional offscreen rendering backend')
    p.add_argument('--speed', type=float, default=.03, help='walk command in m/s; limited to +/-0.035')
    return p.parse_args()


def save_frame(sim, path):
    import mujoco
    from PIL import Image
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with mujoco.Renderer(sim.model, height=720, width=1080) as renderer:
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = 125, -14, 3.4
        cam.lookat[:] = [sim.data.qpos[0], sim.data.qpos[1], 1.0]
        opt = mujoco.MjvOption()
        opt.geomgroup[1] = 0
        opt.sitegroup[:] = 0
        renderer.update_scene(sim.data, camera=cam, scene_option=opt)
        Image.fromarray(renderer.render()).save(path)


def run(args):
    if args.render_backend:
        os.environ['MUJOCO_GL'] = args.render_backend
    try:
        import mujoco
        from atlas_sim.simulation import AtlasSimulation
    except ImportError as exc:
        print(f'Missing dependency: {exc}\nRun: python -m pip install -r requirements.txt', file=sys.stderr)
        return 2
    if mujoco.__version__ != '3.12.0':
        print(f'This project targets MuJoCo 3.12.0; found {mujoco.__version__}.\n'
              'Run: python -m pip install -r requirements.txt', file=sys.stderr)
        return 2
    try:
        sim = AtlasSimulation()
    except (OSError, ValueError) as exc:
        print(f'Model loading failed:\n{exc}', file=sys.stderr)
        print('Keep the complete models folder beside run_atlas.py; extract the full project archive.', file=sys.stderr)
        return 2
    events = queue.SimpleQueue()
    event_log = []
    samples = []
    started = time.perf_counter()
    failure = None
    demo_enabled = args.demo
    demo_index = 0
    schedule = [(0, 'stand'), (3, 'wave'), (10, 'squat'), (20, 'dance'), (30, 'walk'), (42, 'stand')]
    duration = args.seconds if args.seconds is not None else (49 if args.demo else 20 if args.headless else math.inf)
    if duration <= 0:
        raise ValueError('--seconds must be positive')
    if args.mode == 'walk':
        sim.set_velocity(args.speed, 0)
    elif args.mode != 'stand':
        sim.set_mode(args.mode)
    print('Atlas legacy / MuJoCo 3.12.0 | 27 motors | torque-only WBC')
    print('1 stand  2 wave  3 squat  4 dance  5 walk | W/S forward/back | A/D turn')
    print('X finish step and stand | Space pause | R reset | P small push | Esc close')
    print('This is the historical hydraulic Atlas model, not the new electric Atlas.')

    def record(message):
        event_log.append({'time': round(float(sim.data.time), 3), 'event': message})
        print(f'[{sim.data.time:6.2f}s] {message}', flush=True)

    def apply_key(code):
        nonlocal demo_enabled, demo_index
        demo_enabled = False
        if code == 32:
            sim.paused = not sim.paused
            record('paused' if sim.paused else 'resumed')
        elif code in (ord('R'), ord('r')):
            sim.reset()
            samples.clear()
            record('reset')
        elif code in (ord('P'), ord('p')):
            sim.push()
            record('35 N lateral push for 0.12 s')
        elif code in (ord('W'), ord('w'), 265):
            sim.set_velocity(.03, 0)
            record('slow forward walk requested')
        elif code in (ord('S'), ord('s'), 264):
            sim.set_velocity(-.025, 0)
            record('slow backward walk requested')
        elif code in (ord('A'), ord('a'), 263):
            sim.set_velocity(0, .04)
            record('slow left turn requested')
        elif code in (ord('D'), ord('d'), 262):
            sim.set_velocity(0, -.04)
            record('slow right turn requested')
        elif code in (ord('X'), ord('x')):
            sim.set_mode('stand')
            record('finish current step and stand')
        elif ord('1') <= code <= ord('5'):
            mode = ['stand', 'wave', 'squat', 'dance', 'walk'][code-ord('1')]
            sim.set_mode(mode)
            record(f'{mode} requested')

    def advance():
        nonlocal demo_index
        while not events.empty():
            apply_key(events.get())
        if demo_enabled and demo_index < len(schedule) and sim.data.time >= schedule[demo_index][0]:
            mode = schedule[demo_index][1]
            sim.set_mode(mode)
            record(f'demo: {mode}')
            demo_index += 1
        sim.step()
        if sim.tick % 25 == 0 and not sim.paused:
            s = sim.status()
            samples.append([s['time'], s['mode'], s['phase'], *s['pelvis_xyz'], *s['com_xyz'],
                            *s['foot_normal_N'], s['max_tilt_deg'], s['step_count']])

    try:
        if args.headless:
            while sim.data.time < duration-1e-10 and not sim.fallen:
                advance()
        else:
            import mujoco.viewer
            with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=events.put,
                                              show_left_ui=False, show_right_ui=False) as viewer:
                viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 125, -14, 3.4
                viewer.opt.geomgroup[0] = 1
                viewer.opt.geomgroup[1] = 0
                viewer.opt.sitegroup[:] = 0
                while viewer.is_running() and sim.data.time < duration-1e-10:
                    begin = time.perf_counter()
                    for _ in range(8):
                        advance()
                        if sim.fallen:
                            sim.paused = True
                            break
                    viewer.cam.lookat[:] = [sim.data.qpos[0], sim.data.qpos[1], 1.0]
                    s = sim.status()
                    status = 'FALL DETECTED - R to reset' if s['fallen'] else 'PAUSED' if s['paused'] else s['mode'].upper()
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                      'ATLAS LEGACY / MuJoCo 3.12.0\n'
                                      f'{status}  |  {s["phase"]}  |  {s["time"]:.1f} s\n'
                                      f'27 motors / no base support / steps {s["step_count"]}\n'
                                      '1 Stand  2 Wave  3 Squat  4 Dance  5 Walk\n'
                                      'W/S Move  A/D Turn  X Stop\nSpace Pause  R Reset  P Push', ''))
                    viewer.sync()
                    remaining = .016-(time.perf_counter()-begin)
                    if remaining > 0:
                        time.sleep(remaining)
    except KeyboardInterrupt:
        record('user interrupted')
    except Exception as exc:
        failure = f'{type(exc).__name__}: {exc}'
        print(failure, file=sys.stderr)
    if args.snapshot:
        try:
            save_frame(sim, args.snapshot)
        except Exception as exc:
            failure = (failure+'; ' if failure else '')+f'snapshot failed: {exc}'
            print(failure, file=sys.stderr)
    warnings = {}
    for name, value in mujoco.mjtWarning.__members__.items():
        index = int(value)
        if index < len(sim.data.warning) and sim.data.warning[index].number:
            warnings[name] = int(sim.data.warning[index].number)
    report = {'success': not sim.fallen and failure is None and not warnings, 'error': failure,
              'wall_seconds': round(time.perf_counter()-started, 3),
              'model_sha256': hashlib.sha256((ROOT/'models'/'atlas.xml').read_bytes()).hexdigest(),
              'physics_step_seconds': sim.dt, 'control_step_seconds': sim.control_dt,
              'warnings': warnings, 'model_generation': 'legacy hydraulic Atlas derivative',
              'controller': 'tutorial weighted inverse dynamics / ground CoP load allocation',
              'visual_replay': False, 'external_base_support': False,
              'state': sim.status(), 'events': event_log}
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf8')
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open('w', newline='', encoding='utf8') as f:
        writer = csv.writer(f)
        writer.writerow(['t', 'mode', 'phase', 'pelvis_x', 'pelvis_y', 'pelvis_z',
                         'com_x', 'com_y', 'com_z', 'left_foot_normal_N', 'right_foot_normal_N',
                         'max_tilt_deg', 'steps'])
        writer.writerows(samples)
    print(f'Report: {args.report.resolve()}')
    print(f'Simulation time: {sim.data.time:.3f} s | fallen: {sim.fallen} | steps: {sim.step_number}')
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(run(arguments()))
