#!/usr/bin/env python3
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

import numpy as np
from dexcube.simulation import CubeTask
from dexcube.rendering import Demo, SceneRenderer

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description='双灵巧手魔方：接触抓持、内部伺服转层、历史逆序还原')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--hold', action='store_true')
    parser.add_argument('--verify', action='store_true')
    parser.add_argument('--video', type=Path)
    parser.add_argument('--log', type=Path, default=ROOT/'results/run_report.json')
    args = parser.parse_args()
    if args.video and not args.headless:
        parser.error('录制视频请使用 --headless --video，避免多个 OpenGL 窗口上下文冲突')
    task, demo = CubeTask(), Demo()
    renderer = writer = None
    next_frame = 0.0
    if args.video:
        import imageio.v2 as imageio
        renderer = SceneRenderer(task)
        args.video.parent.mkdir(parents=True, exist_ok=True)
        writer = imageio.get_writer(str(args.video), fps=25, codec='libx264', quality=7, macro_block_size=2)

    def step():
        nonlocal next_frame
        demo.update(task)
        task.step()
        if writer is not None and task.data.time >= next_frame:
            from PIL import Image, ImageDraw
            frame = Image.fromarray(renderer.frame(task.model, task.data))
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, 960, 43), fill=(14, 20, 32))
            draw.text((14, 7), 'DEXTEROUS CUBE | MuJoCo 3.12.0 | '+task.phase, fill=(227, 240, 251))
            draw.text((14, 25), f'ASSISTED GRASP + INTERNAL LAYER SERVO | MOVES {task.completed} | SOLVED {task.state.solved}', fill=(137, 196, 240))
            writer.append_data(np.asarray(frame))
            next_frame += 1/25

    try:
        if args.headless:
            while not demo.finished and not task.error and task.data.time < 45:
                step()
        else:
            # Recompiling the cube's topology changes MjModel. Use the webpage
            # for interactive controls; this entry opens a native demo window
            # with a renderer-owned GLFW display that can accept new models.
            import glfw
            import mujoco
            if not glfw.init():
                raise RuntimeError('GLFW 初始化失败')
            window = glfw.create_window(960, 600, 'Dexterous Cube | MuJoCo 3.12.0', None, None)
            if window is None:
                raise RuntimeError('无法创建窗口')
            glfw.make_context_current(window)
            scene = context = model = None
            camera, option = mujoco.MjvCamera(), mujoco.MjvOption()
            camera.lookat[:], camera.distance, camera.azimuth, camera.elevation = [0, 0, .16], .51, 130, -28
            try:
                while not glfw.window_should_close(window):
                    started = time.perf_counter()
                    if not demo.finished and not task.error and task.data.time < 45:
                        for _ in range(10):
                            step()
                    elif not args.hold:
                        break
                    if model is not task.model:
                        if context is not None:
                            context.free()
                        model = task.model
                        scene = mujoco.MjvScene(model, maxgeom=1500)
                        context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
                    width, height = glfw.get_framebuffer_size(window)
                    mujoco.mjv_updateScene(model, task.data, option, None, camera, mujoco.mjtCatBit.mjCAT_ALL, scene)
                    scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
                    mujoco.mjr_render(mujoco.MjrRect(0,0,width,height), scene, context)
                    glfw.swap_buffers(window)
                    glfw.poll_events()
                    time.sleep(max(.001,.02-(time.perf_counter()-started)))
            finally:
                if context is not None:
                    context.free()
                glfw.destroy_window(window)
                glfw.terminate()
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()
    report = task.report()
    report['checks'] = {'finite_state': task.error is None, 'contact_grasp': task.max_contacts >= 3 and task.grasped,
                        'six_turns_completed': task.completed == 6, 'restored': task.state.solved,
                        'hand_joint_limits': task.max_hand_limit_violation < .025,
                        'settled_turn_endpoints': task.max_layer_error < .008,
                        'grasp_tracking': task.max_anchor_error < .005}
    report['success'] = all(report['checks'].values())
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.log.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.verify and not report['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
