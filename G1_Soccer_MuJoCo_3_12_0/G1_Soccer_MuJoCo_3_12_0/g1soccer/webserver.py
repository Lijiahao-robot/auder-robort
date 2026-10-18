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

from .demo import Demo
from .rendering import SceneRenderer
from .simulation import ALLOWED_KEYS, SoccerTask

ROOT = Path(__file__).resolve().parents[1]


class LiveSimulation:
    def __init__(self, width=960, height=540, fps=20):
        self.task = SoccerTask()
        self.renderer = SceneRenderer(self.task, width, height)
        self.render_data = mujoco.MjData(self.task.model)
        self.physics_lock = threading.Lock()
        self.fps = fps
        self.camera = 'field'
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
        if action not in {'keys', 'kick', 'stop', 'pause', 'reset', 'ball', 'demo', 'camera'}:
            raise ValueError('未知操作')
        if action == 'keys':
            keys = payload.get('keys', [])
            if not isinstance(keys, list) or len(keys) > 8 or any(not isinstance(k, str) or k not in ALLOWED_KEYS for k in keys):
                raise ValueError('无效按键')
        if action == 'camera' and payload.get('mode') not in {'field', 'follow', 'behind'}:
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
            if action == 'keys':
                if self.demo is not None and payload.get('keys'):
                    self.demo = None
                if self.demo is None:
                    task.command(payload.get('keys', []), now=received)
            elif action == 'kick':
                self.demo = None
                task.kick()
            elif action == 'stop':
                self.demo = None
                task.stop()
            elif action == 'pause':
                task.paused = not task.paused
                task.stop()
            elif action == 'reset':
                self.demo = None
                task.reset()
            elif action == 'ball':
                if task.kick_started is None:
                    task.reset_ball()
                    task.last_event = '足球已放回脚前，按空格射门'
            elif action == 'demo':
                task.reset()
                self.demo = Demo(task)
            elif action == 'camera':
                self.camera = payload['mode']

    def render(self):
        # Copy a snapshot briefly; slow OpenGL work must never hold up physics.
        with self.physics_lock:
            mujoco.mj_copyData(self.render_data, self.task.model, self.task.data)
            pose = [*self.task.base_xy.tolist(), self.task.heading]
            camera = self.camera
        output = io.BytesIO()
        Image.fromarray(self.renderer.frame(camera, self.render_data, pose)).save(output, 'JPEG', quality=82)
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
                    if self.demo is not None and not self.demo.update(self.task, now):
                        self.demo = None
                    self.task.step(now=now)
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
                     '/input.js': ('input.js', 'text/javascript; charset=utf-8'), '/style.css': ('style.css', 'text/css; charset=utf-8')}
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
