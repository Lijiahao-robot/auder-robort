"""Standard-library-only launcher: isolate and install the exact dependencies."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import venv

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description="半人马 MuJoCo 3.12.0 独立环境启动器")
    parser.add_argument("--prepare-only",action="store_true",help="只准备环境，不启动网页")
    parser.add_argument("--app",choices=("web","sim"),default="web")
    args,remaining=parser.parse_known_args()
    if sys.version_info<(3,10):
        print("需要 Python 3.10 或更新版本，建议 Python 3.11/3.12。")
        return 2
    environment=dict(os.environ)
    environment.pop("PYTHONPATH",None)
    environment.pop("PYTHONHOME",None)
    environment["PYTHONNOUSERSITE"]="1"
    directory=ROOT/".venv"
    python=directory/("Scripts/python.exe" if os.name=="nt" else "bin/python")
    if not python.exists():
        print("正在创建本工程的独立 Python 环境…",flush=True)
        venv.EnvBuilder(with_pip=True).create(directory)
    check="import mujoco,numpy,PIL,imageio,imageio_ffmpeg; assert mujoco.__version__=='3.12.0'"
    checked=subprocess.run([str(python),"-c",check],cwd=ROOT,env=environment,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    if checked.returncode:
        print("正在安装 MuJoCo 3.12.0、Pillow 等依赖，首次启动需要网络…",flush=True)
        result=subprocess.run([str(python),"-m","pip","install","--disable-pip-version-check","-r",str(ROOT/"requirements.txt")],cwd=ROOT,env=environment)
        if result.returncode:
            print("依赖安装未完成。请检查网络后重新运行启动器。")
            return result.returncode
        checked=subprocess.run([str(python),"-c",check],cwd=ROOT,env=environment)
        if checked.returncode:
            print("依赖检查失败，请保留上面的错误信息。")
            return checked.returncode
    print("环境就绪：MuJoCo 3.12.0。",flush=True)
    if args.prepare_only:
        return 0
    script=ROOT/("start_web.py" if args.app=="web" else "run_sim.py")
    return subprocess.call([str(python),str(script),*remaining],cwd=ROOT,env=environment)


if __name__=="__main__":
    raise SystemExit(main())
