"""Loopback HTTP, immutable render snapshots and a command queue for the main GL thread."""
from __future__ import annotations
import io
import json
import queue
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

import mujoco
from PIL import Image

from .rendering import Demo, SceneRenderer
from .simulation import CubeTask
from .cube import parse_move

ROOT = Path(__file__).resolve().parents[1]


class LiveSimulation:
    def __init__(self, width=960, height=540, fps=20):
        self.task = CubeTask()
        self.renderer = SceneRenderer(self.task, width, height)
        self.render_data = mujoco.MjData(self.task.model)
        self.physics_lock = threading.Lock()
        self.fps = fps
        self.camera = 'overview'
        self.commands = queue.Queue(maxsize=128)
        self.condition = threading.Condition()
        self.jpeg = b''
        self.frame_id = 0
        self.latest = self.task.status()
        self.demo = None
        self.closed = False
        self.frame_count = 0
        self.frame_started = time.monotonic()
        self.render_fps = 0.0
        self.last_clients = {}

    def enqueue(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('请求必须为 JSON 对象')
        action = payload.get('action')
        if action not in {'turn', 'sequence', 'scramble', 'restore', 'undo', 'release', 'clear', 'pause', 'reset', 'demo', 'camera'}:
            raise ValueError('未知操作')
        if action == 'turn':
            parse_move(payload.get('move'))
        if action == 'sequence':
            sequence = payload.get('moves')
            if not isinstance(sequence, str) or len(sequence) > 150 or not sequence.strip():
                raise ValueError('请输入动作序列')
            for move in sequence.split():
                parse_move(move)
            if len(sequence.split()) > 40:
                raise ValueError('一次最多 40 步')
        if action == 'camera' and payload.get('mode') not in {'overview', 'front', 'top'}:
            raise ValueError('未知相机')
        client, seq = payload.get('client', ''), payload.get('seq', 0)
        if not isinstance(client, str) or len(client) > 100 or not isinstance(seq, int) or isinstance(seq, bool):
            raise ValueError('无效指令编号')
        now = time.monotonic()
        # Queue ordering alone cannot order independently arriving HTTP requests.
        # Store each client's sequence on the HTTP thread under the condition lock.
        with self.condition:
            previous = self.last_clients.get(client, (-1, now))[0]
            if seq <= previous:
                return False
            try:
                self.commands.put_nowait((payload, now))
            except queue.Full:
                raise ValueError('指令队列已满，请松开按键稍后重试')
            self.last_clients[client] = (seq, now)
            if len(self.last_clients) > 64:
                self.last_clients = {k: v for k, v in self.last_clients.items() if now - v[1] < 60}
        return True

    def apply_commands(self):
        while True:
            try:
                payload, received = self.commands.get_nowait()
            except queue.Empty:
                break
            action, task = payload['action'], self.task
            try:
                if action == 'turn':
                    self.demo = None
                    task.enqueue([payload['move']])
                elif action == 'sequence':
                    self.demo = None
                    task.enqueue(payload['moves'].split())
                elif action == 'scramble':
                    self.demo = None
                    task.scramble()
                elif action == 'restore':
                    self.demo = None
                    task.restore()
                elif action == 'undo':
                    self.demo = None
                    task.undo()
                elif action == 'release':
                    self.demo = None
                    task.release()
                elif action == 'clear':
                    self.demo = None
                    task.moves.clear()
                    task.message = '已清空待执行动作，当前转层会完成'
                elif action == 'pause':
                    task.paused = not task.paused
                elif action == 'reset':
                    self.demo = None
                    task.reset()
                elif action == 'demo':
                    task.reset()
                    self.demo = Demo()
                elif action == 'camera':
                    self.camera = payload['mode']
            except ValueError as exc:
                task.message = str(exc)

    def render(self):
        # Copy a snapshot briefly; slow OpenGL work must never hold up physics.
        with self.physics_lock:
            if self.render_data is None or self.renderer.model is not self.task.model:
                self.render_data = mujoco.MjData(self.task.model)
            model = self.task.model
            mujoco.mj_copyData(self.render_data, self.task.model, self.task.data)
            camera = self.camera
        output = io.BytesIO()
        Image.fromarray(self.renderer.frame(model, self.render_data, camera)).save(output, 'JPEG', quality=85)
        self.frame_count += 1
        elapsed = time.monotonic() - self.frame_started
        if elapsed >= 1.0:
            self.render_fps = self.frame_count / elapsed
            self.frame_started, self.frame_count = time.monotonic(), 0
        with self.condition:
            self.jpeg = output.getvalue()
            self.frame_id += 1
            self.latest.update({'frame_id': self.frame_id, 'render_fps': round(self.render_fps, 1)})
            self.condition.notify_all()

    def physics_loop(self):
        previous, accumulator, next_state = time.monotonic(), 0.0, 0.0
        while not self.closed:
            now = time.monotonic()
            elapsed = min(now - previous, .08)
            previous = now
            with self.physics_lock:
                self.apply_commands()
                accumulator = 0.0 if self.task.paused else min(.08, accumulator + elapsed)
                while accumulator >= .002:
                    if self.demo is not None:
                        self.demo.update(self.task)
                        if self.demo.finished:
                            self.demo = None
                    self.task.step()
                    accumulator -= .002
                if now >= next_state:
                    state = self.task.status()
                    state.update({'render_fps': round(self.render_fps, 1), 'demo': self.demo is not None,
                                  'camera': self.camera, 'frame_id': self.frame_id})
                    with self.condition:
                        self.latest = state
                    next_state = now + .04
            time.sleep(.001)

    def loop(self, duration=None):
        start = time.monotonic()
        worker = threading.Thread(target=self.physics_loop, daemon=True, name='mujoco-physics')
        worker.start()
        try:
            while not self.closed and (duration is None or time.monotonic() - start < duration):
                at = time.monotonic()
                self.render()
                time.sleep(max(.001, 1 / self.fps - (time.monotonic() - at)))
        finally:
            self.closed = True
            worker.join(timeout=5)

    def close(self):
        self.closed = True
        with self.condition:
            self.condition.notify_all()
        self.renderer.close()


class SimulationServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, simulation):
        self.simulation = simulation
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, *_):
        pass

    def send_bytes(self, body, mime, status=200):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.end_headers()
        self.wfile.write(body)

    def json(self, value, status=200):
        self.send_bytes(json.dumps(value, ensure_ascii=False).encode('utf-8'), 'application/json; charset=utf-8', status)

    def local_request(self):
        host = self.headers.get('Host', '')
        if host not in {f'127.0.0.1:{self.server.server_port}', f'localhost:{self.server.server_port}'}:
            return False
        origin = self.headers.get('Origin')
        return origin is None or origin in {f'http://127.0.0.1:{self.server.server_port}', f'http://localhost:{self.server.server_port}'}

    def do_GET(self):
        if not self.local_request():
            self.json({'error': 'Only same-origin loopback requests are accepted'}, 403)
            return
        path = urlsplit(self.path).path
        sim = self.server.simulation
        if path == '/api/state':
            with sim.condition:
                state = sim.latest.copy()
            self.json(state)
        elif path == '/frame.jpg':
            with sim.condition:
                frame = sim.jpeg
            self.send_bytes(frame, 'image/jpeg') if frame else self.json({'error': 'Frame not ready'}, 503)
        elif path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache, no-store')
            self.send_header('Connection', 'close')
            self.end_headers()
            frame_id = -1
            try:
                while not sim.closed:
                    with sim.condition:
                        sim.condition.wait_for(lambda: sim.closed or sim.frame_id != frame_id, timeout=2.0)
                        frame, frame_id = sim.jpeg, sim.frame_id
                    if frame:
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(frame)).encode() + b'\r\n\r\n' + frame + b'\r\n')
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        else:
            files = {'/': ('index.html', 'text/html; charset=utf-8'), '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                     '/controls.js': ('controls.js', 'text/javascript; charset=utf-8'), '/style.css': ('style.css', 'text/css; charset=utf-8')}
            if path not in files:
                self.json({'error': 'Not found'}, 404)
                return
            name, mime = files[path]
            self.send_bytes((ROOT / 'web' / name).read_bytes(), mime)

    def do_POST(self):
        if not self.local_request():
            self.json({'error': 'Origin is not allowed'}, 403)
            return
        if urlsplit(self.path).path != '/api/control':
            self.json({'error': 'Not found'}, 404)
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('Request body limit is 4096 bytes')
            payload = json.loads(self.rfile.read(length))
            accepted = self.server.simulation.enqueue(payload)
            self.json({'ok': True, 'accepted': accepted})
        except (ValueError, json.JSONDecodeError) as exc:
            self.json({'error': str(exc)}, 400)
