#!/usr/bin/env python3
"""Run a contact-driven G1 soccer demo and optionally record its actual MuJoCo frames."""
import argparse
import json
import os
from pathlib import Path
import sys
import time

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
    os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np

from g1soccer.demo import Demo
from g1soccer.simulation import SoccerTask

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo', action='store_true', help='Walk, settle, then kick once (default behavior)')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--hold', action='store_true', help='Keep the final viewer pose until closed')
    parser.add_argument('--video', type=Path)
    parser.add_argument('--log', type=Path, default=ROOT / 'results/run_report.json')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    task, renderer, writer = SoccerTask(), None, None
    demo = Demo(task)
    next_frame = 0.0
    if args.video:
        import imageio.v2 as imageio
        from g1soccer.rendering import SceneRenderer
        renderer = SceneRenderer(task, 960, 540)
        args.video.parent.mkdir(parents=True, exist_ok=True)
        writer = imageio.get_writer(str(args.video), fps=25, codec='libx264', quality=7, macro_block_size=2)

    def step():
        nonlocal next_frame
        demo.update(task, now=0.0)
        task.step(now=0.0)
        if writer is not None and task.data.time >= next_frame:
            from PIL import Image, ImageDraw
            image = Image.fromarray(renderer.frame('field'))
            draw = ImageDraw.Draw(image)
            draw.rectangle((0, 0, 960, 43), fill=(11, 24, 20))
            draw.text((14, 7), 'G1 FOOTBALL | MuJoCo 3.12.0 | PHYSICAL FOOT-BALL CONTACT', fill=(231, 246, 221))
            draw.text((14, 25), f'ASSISTED PELVIS / IK + PD    GOALS {task.goals}    BALL {task.ball_speed:.2f} m/s', fill=(194, 227, 167))
            writer.append_data(np.asarray(image))
            next_frame += 1 / 25

    try:
        if args.headless:
            while task.data.time < demo.duration and not task.error:
                step()
        else:
            from mujoco import viewer as mjviewer
            with mjviewer.launch_passive(task.model, task.data) as viewer:
                viewer.cam.lookat[:], viewer.cam.distance = [.35, 0, .35], 8.4
                viewer.cam.azimuth, viewer.cam.elevation = 140, -29
                viewer.opt.geomgroup[3] = 0
                print('演示自动走位并踢球。网页键盘控制请运行 start_web.py。')
                while viewer.is_running() and not task.error:
                    wall = time.perf_counter()
                    if task.data.time < demo.duration:
                        with viewer.lock():
                            for _ in range(10):
                                step()
                    elif not args.hold:
                        break
                    viewer.sync()
                    time.sleep(max(.001, .02 - (time.perf_counter() - wall)))
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()
    report = task.report()
    report['checks'] = {'finite_state': task.error is None, 'physical_foot_contact': task.kick_contacts > 0,
                        'ball_moved_from_impact': task.max_ball_speed > 1.0, 'goal_scored': task.goals == 1,
                        'walk_completed': task.travel > .4, 'joint_limits': task.max_joint_violation < .02,
                        'base_tracking': task.max_root_error < .05}
    report['success'] = all(report['checks'].values())
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.log.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.verify and not report['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
