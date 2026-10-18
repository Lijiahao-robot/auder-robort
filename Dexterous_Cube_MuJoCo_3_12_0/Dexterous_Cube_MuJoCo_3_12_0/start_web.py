#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path
import sys
import threading
import webbrowser

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')
if sys.platform.startswith('linux') and not os.environ.get('DISPLAY'):
    os.environ.setdefault('MUJOCO_GL', 'egl')

from dexcube.webserver import LiveSimulation, SimulationServer

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description='灵巧手魔方网页，本机 MuJoCo 3.12.0 实时渲染')
    parser.add_argument('--port', type=int, default=8899)
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--width', type=int, default=960)
    parser.add_argument('--height', type=int, default=540)
    parser.add_argument('--fps', type=int, default=20)
    parser.add_argument('--duration', type=float, help='Optional wall-time limit for automated checks')
    args = parser.parse_args()
    if not 0 <= args.port <= 65535 or not 320 <= args.width <= 1280 or not 240 <= args.height <= 720 or not 5 <= args.fps <= 30:
        parser.error('参数范围：port 0–65535，width 320–1280，height 240–720，fps 5–30')
    sim, server = None, None
    try:
        sim = LiveSimulation(args.width, args.height, args.fps)
        sim.render()
        server = SimulationServer(('127.0.0.1', args.port), sim)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        url = f'http://127.0.0.1:{server.server_port}'
        print('MuJoCo 3.12.0 DEXTEROUS CUBE', flush=True)
        print('网页：' + url, flush=True)
        print('U/R/F/D/L/B 转面，Shift 逆时针，空格暂停。Ctrl+C 结束。', flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        sim.loop(args.duration)
    except KeyboardInterrupt:
        pass
    finally:
        if sim is not None:
            (ROOT / 'results').mkdir(exist_ok=True)
            (ROOT / 'results/web_last_run.json').write_text(json.dumps(sim.task.report(), ensure_ascii=False, indent=2), encoding='utf-8')
            sim.close()
        if server is not None:
            server.shutdown()
            server.server_close()


if __name__ == '__main__':
    main()
