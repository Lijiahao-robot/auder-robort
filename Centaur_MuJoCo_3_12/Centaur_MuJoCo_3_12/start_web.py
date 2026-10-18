import argparse
import os
import signal
import sys
import threading
import webbrowser

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL","egl")

try:
    from centaur.web import LiveSimulation,WebServer
except ModuleNotFoundError as exc:
    raise SystemExit(f"缺少依赖 {exc.name}。请运行 python bootstrap.py 自动准备环境。")


def main():
    parser=argparse.ArgumentParser(description="半人马 MuJoCo 3.12.0 网页仿真")
    parser.add_argument("--port",type=int,default=8700)
    parser.add_argument("--no-browser",action="store_true")
    args=parser.parse_args()
    live=LiveSimulation()
    server=None
    try:
        print("正在加载半人马模型与 MuJoCo 3.12.0 渲染器…",flush=True)
        live.start()
        server=WebServer(("127.0.0.1",args.port),live)
        url=f"http://127.0.0.1:{server.server_port}/"
        print("网页仿真："+url,flush=True)
        print("默认启用可关闭的辅助稳定；Ctrl+C 退出。",flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        stopping=threading.Event()
        def stop(signum,frame):
            if not stopping.is_set():
                stopping.set()
                threading.Thread(target=server.shutdown,daemon=True).start()
        signal.signal(signal.SIGINT,stop)
        if hasattr(signal,"SIGTERM"):
            signal.signal(signal.SIGTERM,stop)
        server.serve_forever(poll_interval=0.2)
    finally:
        if server:
            server.server_close()
        live.close()


if __name__=="__main__":
    main()
