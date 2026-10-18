from __future__ import annotations

import io
import json
import queue
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from PIL import Image
from .simulation import MODES,Renderer,Simulation,number


def validate_command(body):
    if not isinstance(body,dict):
        raise ValueError("请求必须是 JSON 对象")
    action=body.get("action")
    if action not in ("configure","velocity","pause","reset","camera"):
        raise ValueError("未知仿真操作")
    if action=="configure":
        if body.get("mode") not in MODES:
            raise ValueError("未知动作模式")
        number(body.get("speed"),"速度",0,0.20)
        if type(body.get("assist")) is not bool:
            raise ValueError("辅助稳定应为布尔值")
    elif action=="velocity":
        for field,low,high in (("vx",-0.16,0.20),("vy",-0.10,0.10),("yaw",-0.45,0.45)):
            number(body.get(field),field,low,high)
    elif action=="camera" and body.get("view") not in ("front","side","rear"):
        raise ValueError("未知视角")
    return dict(body)


class LiveSimulation:
    def __init__(self):
        self.lock=threading.Lock()
        self.commands=queue.Queue(maxsize=24)
        self.ready=threading.Event()
        self.done=threading.Event()
        self.error=""
        self.image=b""
        self.status={"loading":True,"running":False}
        self.thread=threading.Thread(target=self._work,name="centaur-physics-render",daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(30):
            self.done.set()
            raise RuntimeError("模型初始化超时")
        if self.error:
            raise RuntimeError(self.error)

    def command(self,body):
        body=validate_command(body)
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
        try:
            self.commands.put_nowait(body)
        except queue.Full:
            raise ValueError("操作过多，请稍后再试")
        return {"accepted":True}

    def snapshot(self):
        with self.lock:
            return dict(self.status)

    def frame(self):
        with self.lock:
            return self.image

    def _work(self):
        renderer=None
        try:
            sim=Simulation()
            renderer=Renderer(sim)
            running=False
            accumulator=0.0
            last=time.monotonic()
            last_render=0.0
            last_velocity=-1e9
            while not self.done.is_set():
                now=time.monotonic()
                elapsed=min(now-last,0.15)
                last=now
                try:
                    while True:
                        body=self.commands.get_nowait()
                        action=body["action"]
                        if action=="configure":
                            sim.configure(body["mode"],body["speed"],body["assist"])
                            running=True
                        elif action=="velocity":
                            sim.velocity(body["vx"],body["vy"],body["yaw"])
                            last_velocity=now
                            running=True
                        elif action=="pause":
                            sim.manual[:]=0
                            running=False
                            accumulator=0
                        elif action=="reset":
                            sim.reset()
                            running=False
                            accumulator=0
                        else:
                            renderer.camera.azimuth={"front":135,"side":90,"rear":-45}[body["view"]]
                except queue.Empty:
                    pass
                if sim.mode=="manual" and now-last_velocity>0.65:
                    sim.manual[:]=0
                if running:
                    accumulator+=elapsed
                    count=int(accumulator/sim.model.opt.timestep)
                    if count:
                        sim.step(count)
                        accumulator-=count*sim.model.opt.timestep
                    if sim.fallen:
                        running=False
                if now-last_render>=(0.08 if running else 0.25):
                    buffer=io.BytesIO()
                    Image.fromarray(renderer.frame()).save(buffer,format="JPEG",quality=87)
                    status=sim.snapshot()
                    status.update(running=running,loading=False,error="",keyboard_velocity=sim.manual.tolist())
                    with self.lock:
                        self.image=buffer.getvalue()
                        self.status=status
                    last_render=now
                    self.ready.set()
                self.done.wait(0.002 if running else 0.025)
        except Exception as exc:
            with self.lock:
                self.error="仿真或渲染失败："+str(exc)
                self.status.update(running=False,loading=False,error=self.error)
            self.ready.set()
        finally:
            if renderer:
                renderer.close()

    def close(self):
        self.done.set()
        self.thread.join(timeout=5)


class WebServer(ThreadingHTTPServer):
    daemon_threads=True
    def __init__(self,address,live):
        self.live=live
        self.key=secrets.token_urlsafe(32)
        super().__init__(address,Handler)


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):
        pass
    def setup(self):
        super().setup()
        self.connection.settimeout(4)
    def _send(self,code,data,kind="application/json; charset=utf-8",attachment=False):
        raw=data if isinstance(data,bytes) else json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
        self.send_response(code)
        self.send_header("Content-Type",kind)
        self.send_header("Content-Length",str(len(raw)))
        self.send_header("Cache-Control","no-store")
        self.send_header("X-Content-Type-Options","nosniff")
        self.send_header("Referrer-Policy","no-referrer")
        self.send_header("Content-Security-Policy","default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
        self.send_header("Connection","close")
        if attachment:
            self.send_header("Content-Disposition",'attachment; filename="centaur_report.json"')
        self.end_headers()
        self.close_connection=True
        try:
            self.wfile.write(raw)
        except (BrokenPipeError,ConnectionResetError):
            pass
    def _origin(self):
        host=self.headers.get("Host","")
        parsed=urlsplit("http://"+host)
        if parsed.hostname not in ("localhost","127.0.0.1") or parsed.port!=self.server.server_port:
            raise PermissionError("仅允许本机仿真页面访问")
        if self.headers.get("Origin") not in (None,"http://"+host):
            raise PermissionError("请求来源无效")
    def do_GET(self):
        try:
            self._origin()
            path=urlsplit(self.path).path
            assets={"/":("index.html","text/html; charset=utf-8"),"/app.js":("app.js","text/javascript; charset=utf-8"),
                    "/input.js":("input.js","text/javascript; charset=utf-8"),"/app.css":("app.css","text/css; charset=utf-8")}
            if path in assets:
                filename,kind=assets[path]
                self._send(200,(Path(__file__).resolve().parents[1]/"web"/filename).read_bytes(),kind)
            elif path=="/api/session":
                self._send(200,{"key":self.server.key})
            elif path in ("/api/state","/api/report"):
                self._send(200,self.server.live.snapshot(),attachment=path=="/api/report")
            elif path=="/api/frame":
                frame=self.server.live.frame()
                self._send(200 if frame else 503,frame,"image/jpeg")
            else:
                self._send(404,{"error":"未找到接口"})
        except PermissionError as exc:
            self._send(403,{"error":str(exc)})
        except (ValueError,TypeError):
            self._send(400,{"error":"请求无效"})
    def do_POST(self):
        try:
            self._origin()
            if self.path!="/api/control":
                self._send(404,{"error":"未找到接口"});return
            key=self.headers.get("X-Simulation-Key","")
            if not secrets.compare_digest(key.encode(),self.server.key.encode()):
                raise PermissionError("会话校验失败，请刷新页面")
            if self.headers.get("Content-Type","").split(";")[0]!="application/json":
                raise ValueError("请使用 JSON 请求")
            length=int(self.headers.get("Content-Length","0"))
            if not 1<=length<=4096:
                raise ValueError("请求大小无效")
            raw=self.rfile.read(length)
            if len(raw)!=length:
                raise ValueError("请求体不完整")
            body=json.loads(raw.decode())
            self._send(202,self.server.live.command(body))
        except PermissionError as exc:
            self._send(403,{"error":str(exc)})
        except (ValueError,TypeError,UnicodeError) as exc:
            self._send(400,{"error":str(exc)})
        except RuntimeError as exc:
            self._send(503,{"error":str(exc)})
