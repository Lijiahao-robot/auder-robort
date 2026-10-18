from __future__ import annotations

import math
from pathlib import Path
import mujoco
import numpy as np

from .model import LEGS, UPPER_LEG, LOWER_LEG, ROOT

MODES={"stand":"四足站立","walk":"四足慢走","wave":"挥手互动","dance":"原地律动","manual":"键盘控制"}
PHASES={"FL":0.0,"FR":0.5,"RL":0.75,"RR":0.25}


def number(value,name,low,high):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or not low<=value<=high:
        raise ValueError(f"{name} 应为 {low}–{high} 之间的有限数值")
    return float(value)


def leg_ik(xyz):
    x,y,z=xyz
    vertical=math.hypot(y,z)
    r2=x*x+vertical*vertical
    cosk=np.clip((r2-UPPER_LEG**2-LOWER_LEG**2)/(2*UPPER_LEG*LOWER_LEG),-0.98,0.98)
    knee=math.acos(cosk)
    hip=math.atan2(-x,vertical)-math.atan2(LOWER_LEG*math.sin(knee),UPPER_LEG+LOWER_LEG*math.cos(knee))
    return np.array([math.atan2(y,-z),hip,knee])


class Simulation:
    def __init__(self,mode="stand",speed=0.12,assist=True):
        if mujoco.__version__!="3.12.0":
            raise RuntimeError("本工程要求 MuJoCo 3.12.0；请运行 python bootstrap.py 安装独立环境")
        self.model=mujoco.MjModel.from_xml_path(str(ROOT/"assets/centaur.xml"))
        self.data=mujoco.MjData(self.model)
        self.ids=self.model.actuator_trnid[:,0].copy()
        self.names=[mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_JOINT,int(i)) for i in self.ids]
        self.index={name:i for i,name in enumerate(self.names)}
        self.qadr=self.model.jnt_qposadr[self.ids]
        self.vadr=self.model.jnt_dofadr[self.ids]
        self.limits=self.model.jnt_range[self.ids]
        self.root_id=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,"trunk")
        self.foot_ids={leg:mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,leg+"_pad") for leg in LEGS}
        self.mass=float(np.sum(self.model.body_mass))
        self.neutral=np.zeros(self.model.nu)
        neutral_leg=leg_ik([0,0,-0.54])
        for leg in LEGS:
            for name,q in zip(("abduction","hip","knee"),neutral_leg):
                self.neutral[self.index[leg+"_"+name+"_joint"]]=q
        for side,sgn in (("left",1),("right",-1)):
            for name,q in (("shoulder_pitch",-0.12),("shoulder_roll",0.13*sgn),("elbow",-0.55)):
                self.neutral[self.index[side+"_"+name+"_joint"]]=q
        self.kp=np.array([240 if n[:2] in LEGS else 100 if "waist" in n else 40 for n in self.names])
        self.kd=np.array([8 if n[:2] in LEGS else 7 if "waist" in n else 2 for n in self.names])
        self.mode=mode
        self.speed=speed
        self.assist=assist
        self.configure(mode,speed,assist)
        self.reset()

    def configure(self,mode,speed,assist):
        if mode not in MODES:
            raise ValueError("未知动作模式")
        speed=number(speed,"速度",0,0.20)
        if type(assist) is not bool:
            raise ValueError("辅助稳定应为布尔值")
        self.mode,self.speed,self.assist=mode,speed,assist
        self.manual=np.zeros(3)

    def velocity(self,vx,vy,yaw):
        self.manual=np.array([number(vx,"前后速度",-0.16,0.20),number(vy,"横移速度",-0.10,0.10),number(yaw,"转向角速度",-0.45,0.45)])
        self.mode="manual"

    def reset(self):
        mujoco.mj_resetData(self.model,self.data)
        self.data.qpos[self.qadr]=self.neutral
        self.phase=0.0
        self.filtered_velocity=np.zeros(3)
        self.gait_gain=0.0
        self.yaw_ref=0.0
        self.previous_ref=self.neutral.copy()
        self.last_ref=self.neutral.copy()
        self.steps=0
        self.error2=0.0
        self.max_limit_violation=0.0
        self.max_torque_ratio=0.0
        self.max_assist_force=0.0
        self.max_assist_torque=0.0
        self.min_height=float(self.data.qpos[2])
        self.max_tilt=0.0
        self.fallen=False
        self.contacts={leg:0.0 for leg in LEGS}
        self.contact_frames=0
        self.manual=np.zeros(3)
        mujoco.mj_forward(self.model,self.data)
        self.initial_position=self.data.qpos[:3].copy()
        self.position_ref=self.data.qpos[:2].copy()

    def _target(self):
        dt=self.model.opt.timestep
        velocity=np.array([self.speed,0,0]) if self.mode=="walk" else self.manual if self.mode=="manual" else np.zeros(3)
        self.filtered_velocity+=(velocity-self.filtered_velocity)*min(1,dt/0.35)
        gait=1.0 if np.linalg.norm(velocity)>0.005 or self.mode=="dance" else 0.0
        self.gait_gain+=(gait-self.gait_gain)*min(1,dt/0.4)
        period=2.4
        self.phase=(self.phase+dt/period)%1
        target=self.neutral.copy()
        vx,vy,yaw=self.filtered_velocity
        for leg,(hx,hy) in LEGS.items():
            phase=(self.phase+PHASES[leg])%1
            duty=0.8
            if phase<duty:
                advance=0.5-phase/duty
                lift=0.0
            else:
                s=(phase-duty)/(1-duty)
                advance=-0.5+s*s*(3-2*s)
                lift=0.045*math.sin(math.pi*s)**2
            stride=np.array([vx-yaw*hy,vy+yaw*hx])*period*duty
            foot=np.array([advance*stride[0],advance*stride[1],-0.54+lift*self.gait_gain])
            q=leg_ik(foot)
            for n,value in zip(("abduction","hip","knee"),q):
                target[self.index[leg+"_"+n+"_joint"]]=value
        t=float(self.data.time)
        blend=min(1,t/2)
        blend=blend*blend*(3-2*blend)
        s=math.sin(2*math.pi*t/3.2)
        if self.mode in ("wave","dance"):
            target[self.index["right_shoulder_pitch_joint"]]=-1.0+0.15*s
            target[self.index["right_shoulder_roll_joint"]]=-0.65
            target[self.index["right_elbow_joint"]]=-0.65+0.22*math.sin(4*math.pi*t/3.2)
            target[self.index["right_wrist_joint"]]=0.45*math.sin(4*math.pi*t/3.2)
            target[self.index["head_yaw_joint"]]=0.16*s
        if self.mode=="dance":
            target[self.index["left_shoulder_pitch_joint"]]=-0.8-0.30*s
            target[self.index["left_shoulder_roll_joint"]]=0.48
            target[self.index["waist_yaw_joint"]]=0.14*s
            target[self.index["head_pitch_joint"]]=0.08*math.sin(4*math.pi*t/3.2)
        return np.clip(self.neutral+(target-self.neutral)*blend,self.limits[:,0]+0.025,self.limits[:,1]-0.025)

    def _assist(self):
        self.data.xfrc_applied[:]=0
        dt=self.model.opt.timestep
        self.yaw_ref+=self.filtered_velocity[2]*dt
        c,s=math.cos(self.yaw_ref),math.sin(self.yaw_ref)
        desired=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        world_target=desired@np.array([*self.filtered_velocity[:2],0])
        self.position_ref+=world_target[:2]*dt
        if not self.assist:
            return
        rot=self.data.xmat[self.root_id].reshape(3,3)
        orientation_error=sum((np.cross(rot[:,i],desired[:,i]) for i in range(3)),np.zeros(3))*0.5
        velocity=np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,self.root_id,velocity,0)
        weight=self.mass*9.81
        force=np.zeros(3)
        position_error=self.position_ref-self.data.qpos[:2]
        if np.linalg.norm(self.filtered_velocity[:2])>0.01:
            self.position_ref[:]=self.data.qpos[:2]
            position_force=np.zeros(2)
        else:
            position_force=450*position_error
        force[:2]=np.clip(position_force+110*(world_target[:2]-velocity[3:5]),-0.25*weight,0.25*weight)
        force[2]=np.clip(0.10*weight+1800*(0.61-self.data.qpos[2])-110*velocity[5],-0.3*weight,0.7*weight)
        torque=np.clip(220*orientation_error-35*velocity[:3],-90,90)
        self.data.xfrc_applied[self.root_id,:3]=force
        self.data.xfrc_applied[self.root_id,3:]=torque
        self.max_assist_force=max(self.max_assist_force,float(np.linalg.norm(force)))
        self.max_assist_torque=max(self.max_assist_torque,float(np.linalg.norm(torque)))

    def step(self,count=1):
        dt=self.model.opt.timestep
        for _ in range(count):
            target=self._target()
            # Rate-limit references when switching gait/upper-body modes.
            target=np.clip(target,self.last_ref-2.8*dt,self.last_ref+2.8*dt)
            dq=(target-self.previous_ref)/dt
            self.previous_ref=target.copy()
            self.last_ref=target.copy()
            error=target-self.data.qpos[self.qadr]
            torque=self.kp*error+self.kd*(dq-self.data.qvel[self.vadr])+self.data.qfrc_bias[self.vadr]
            low,high=self.model.actuator_ctrlrange.T
            self.data.ctrl[:]=np.clip(torque,low,high)
            self._assist()
            mujoco.mj_step(self.model,self.data)
            if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
                raise RuntimeError("仿真状态异常，已停止")
            self.steps+=1
            self.error2+=float(np.mean(error**2))
            self.max_torque_ratio=max(self.max_torque_ratio,float(np.max(np.abs(self.data.ctrl)/np.maximum(np.abs(low),np.abs(high)))))
            violation=np.maximum(self.limits[:,0]-self.data.qpos[self.qadr],self.data.qpos[self.qadr]-self.limits[:,1])
            self.max_limit_violation=max(self.max_limit_violation,float(np.max(violation)),0)
            self.min_height=min(self.min_height,float(self.data.qpos[2]))
            rot=self.data.xmat[self.root_id].reshape(3,3)
            tilt=math.acos(float(np.clip(rot[2,2],-1,1)))
            self.max_tilt=max(self.max_tilt,tilt)
            self.contacts={leg:0.0 for leg in LEGS}
            for i in range(self.data.ncon):
                contact=self.data.contact[i]
                for leg,gid in self.foot_ids.items():
                    if gid in (contact.geom1,contact.geom2):
                        wrench=np.zeros(6)
                        mujoco.mj_contactForce(self.model,self.data,i,wrench)
                        self.contacts[leg]+=max(0,float(wrench[0]))
            if any(f>1 for f in self.contacts.values()):
                self.contact_frames+=1
            if self.data.qpos[2]<0.32 or tilt>math.radians(65):
                self.fallen=True

    def snapshot(self):
        return {"robot":"原创半人马教学机器人","mujoco_version":mujoco.__version__,"actuated_joints":int(self.model.nu),
                "mode":self.mode,"speed_m_s":self.speed,"assist_enabled":self.assist,
                "controller":"joint PD + optional external base wrench","simulation_time_s":float(self.data.time),
                "base_position_m":self.data.qpos[:3].tolist(),"displacement_m":(self.data.qpos[:3]-self.initial_position).tolist(),
                "base_height_m":float(self.data.qpos[2]),"min_base_height_m":self.min_height,
                "max_tilt_deg":math.degrees(self.max_tilt),"physics_steps":self.steps,
                "tracking_rms_rad":math.sqrt(self.error2/max(1,self.steps)),"max_joint_limit_violation_rad":self.max_limit_violation,
                "peak_torque_limit_fraction":self.max_torque_ratio,"peak_external_assist_force_N":self.max_assist_force,
                "peak_external_assist_torque_Nm":self.max_assist_torque,"foot_normal_forces_N":dict(self.contacts),
                "contact_frame_fraction":self.contact_frames/max(1,self.steps),"fallen":self.fallen,
                "self_collision_enabled":False,"hardware_connected":False,"unassisted_locomotion_verified":False,
                "joint_positions_rad":{n:float(q) for n,q in zip(self.names,self.data.qpos[self.qadr])}}


class Renderer:
    def __init__(self,sim,width=960,height=640):
        self.sim=sim
        self.renderer=mujoco.Renderer(sim.model,width=width,height=height)
        self.camera=mujoco.MjvCamera()
        self.camera.azimuth=135
        self.camera.elevation=-14
        self.camera.distance=3.1

    def frame(self):
        self.camera.lookat[:]=self.sim.data.qpos[:3]+np.array([0,0,0.18])
        self.renderer.update_scene(self.sim.data,camera=self.camera)
        return self.renderer.render().copy()

    def close(self):
        self.renderer.close()
