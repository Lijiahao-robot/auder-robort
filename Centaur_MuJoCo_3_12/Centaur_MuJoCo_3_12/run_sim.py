import argparse
import json
import os
from pathlib import Path
import sys
import time

if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
    os.environ.setdefault("MUJOCO_GL","egl")

from centaur.simulation import MODES, Renderer, Simulation, number


def main():
    parser=argparse.ArgumentParser(description="原创半人马机器人 MuJoCo 3.12.0 仿真")
    parser.add_argument("--mode",choices=MODES,default="walk")
    parser.add_argument("--speed",type=float,default=0.12)
    parser.add_argument("--seconds",type=float,default=20)
    parser.add_argument("--no-assist",action="store_true",help="关闭外部辅助稳定（实验）")
    parser.add_argument("--headless",action="store_true")
    parser.add_argument("--render",action="store_true")
    parser.add_argument("--demo",action="store_true",help="依次演示站立、慢走、挥手、律动")
    parser.add_argument("--output",default="results")
    args=parser.parse_args()
    seconds=number(args.seconds,"仿真时长",0.1,600)
    sim=Simulation(args.mode,args.speed,not args.no_assist)
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=True)
    def step(count=1):
        for _ in range(count):
            if args.demo:
                mode="stand" if sim.data.time<2 else "walk" if sim.data.time<7 else "wave" if sim.data.time<10 else "dance"
                if mode!=sim.mode:
                    sim.configure(mode,args.speed,not args.no_assist)
            sim.step()
            if sim.fallen:
                break
    if args.render:
        import imageio.v2 as imageio
        import numpy as np
        from PIL import Image, ImageDraw
        renderer=Renderer(sim)
        writer=imageio.get_writer(str(output/"centaur_demo.mp4"),fps=30,codec="libx264",quality=8,macro_block_size=16,pixelformat="yuv420p")
        try:
            count=max(1,int(seconds*30))
            for index in range(count):
                while sim.data.time<(index+1)/30-sim.model.opt.timestep/2 and not sim.fallen:
                    step()
                frame=Image.fromarray(renderer.frame())
                draw=ImageDraw.Draw(frame)
                draw.rounded_rectangle((16,16,420,82),radius=9,fill=(11,21,31))
                draw.text((30,29),"CENTAUR | MUJOCO 3.12.0",fill=(250,171,89))
                label="External stabilizer ON" if sim.assist else "External stabilizer OFF"
                draw.text((30,53),f"{sim.mode.upper()} | {label}",fill=(203,218,228))
                writer.append_data(np.asarray(frame))
                if index==min(count-1,120):
                    frame.save(output/"centaur_preview.png")
                if sim.fallen:
                    break
        finally:
            writer.close();renderer.close()
    elif args.headless:
        step(round(seconds/sim.model.opt.timestep))
    else:
        import mujoco.viewer
        with mujoco.viewer.launch_passive(sim.model,sim.data) as viewer:
            viewer.cam.distance,viewer.cam.azimuth,viewer.cam.elevation=3.1,135,-14
            started=time.monotonic()
            while viewer.is_running() and sim.data.time<seconds and not sim.fallen:
                with viewer.lock():
                    step(10)
                    viewer.cam.lookat[:]=sim.data.qpos[:3]+[0,0,0.18]
                viewer.sync()
                delay=sim.data.time-(time.monotonic()-started)
                if delay>0:
                    time.sleep(min(delay,0.02))
    report=sim.snapshot()
    (output/"run_report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding="utf-8")
    print(("检测到倒地，仿真已停止；报告：" if sim.fallen else "仿真完成；报告：")+str((output/"run_report.json").resolve()))
    return 2 if sim.fallen else 0


if __name__=="__main__":
    raise SystemExit(main())
