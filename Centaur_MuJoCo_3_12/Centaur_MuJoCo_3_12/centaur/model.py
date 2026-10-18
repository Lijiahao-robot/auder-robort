"""Generate an original parametric MJCF; no third-party robot assets."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
LEGS = {"FL": (0.31, 0.225), "FR": (0.31, -0.225),
        "RL": (-0.31, 0.225), "RR": (-0.31, -0.225)}
UPPER_LEG = 0.30
LOWER_LEG = 0.30


def build_xml():
    r = ET.Element("mujoco", model="centaur_teaching_robot")
    ET.SubElement(r, "compiler", angle="radian", autolimits="true")
    ET.SubElement(r, "option", timestep="0.002", integrator="implicitfast", gravity="0 0 -9.81", cone="elliptic")
    defaults = ET.SubElement(r, "default")
    ET.SubElement(defaults, "joint", damping="0.8", armature="0.035", frictionloss="0.12")
    ET.SubElement(defaults, "geom", contype="1", conaffinity="2", friction="1.0 0.03 0.003", condim="3", group="1", solref="0.008 1")
    ET.SubElement(defaults, "motor", ctrllimited="true", ctrlrange="-90 90")
    visual = ET.SubElement(r, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(visual, "quality", shadowsize="2048")
    ET.SubElement(visual, "headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3")
    assets = ET.SubElement(r, "asset")
    ET.SubElement(assets, "texture", type="skybox", builtin="gradient", rgb1="0.035 0.05 0.075", rgb2="0.15 0.19 0.23", width="512", height="512")
    ET.SubElement(assets, "texture", name="grid", type="2d", builtin="checker", rgb1="0.12 0.16 0.19", rgb2="0.09 0.12 0.15", width="512", height="512")
    for name, color in [("shell", "0.73 0.79 0.82 1"), ("dark", "0.07 0.10 0.14 1"),
                        ("amber", "0.96 0.49 0.15 1"), ("visor", "0.03 0.10 0.15 1")]:
        ET.SubElement(assets, "material", name=name, rgba=color, specular="0.5", shininess="0.55")
    ET.SubElement(assets, "material", name="floor_mat", texture="grid", texrepeat="30 30", reflectance="0.08")
    world = ET.SubElement(r, "worldbody")
    ET.SubElement(world, "light", pos="2 -3 5", dir="-0.4 0.6 -1", diffuse="0.8 0.8 0.8", castshadow="true")
    ET.SubElement(world, "light", pos="-2 2 3", dir="0.5 -0.5 -1", diffuse="0.25 0.35 0.45", castshadow="false")
    ET.SubElement(world, "geom", name="ground", type="plane", size="20 20 0.1", material="floor_mat", contype="2", conaffinity="1", group="2")
    trunk = ET.SubElement(world, "body", name="trunk", pos="0 0 0.62")
    ET.SubElement(trunk, "freejoint", name="floating_base")
    ET.SubElement(trunk, "geom", name="chassis", type="box", size="0.37 0.16 0.10", material="dark", mass="14")
    ET.SubElement(trunk, "geom", type="capsule", fromto="-0.25 0 0.04 0.25 0 0.04", size="0.15", material="shell", mass="0.8")
    ET.SubElement(trunk, "geom", type="box", size="0.17 0.13 0.07", pos="-0.20 0 0.14", material="dark", mass="4")
    ET.SubElement(trunk, "geom", type="box", size="0.32 0.165 0.012", pos="0 0 0.06", material="amber", mass="0.1", contype="0", conaffinity="0")
    actuator = ET.SubElement(r, "actuator")

    def hinge(body, name, axis, limits, force=90):
        ET.SubElement(body, "joint", name=name, axis=axis, range=limits)
        ET.SubElement(actuator, "motor", name=name.removesuffix("_joint"), joint=name, ctrlrange=f"{-force} {force}")

    for leg, (x,y) in LEGS.items():
        hip = ET.SubElement(trunk, "body", name=leg+"_hip", pos=f"{x} {y} -0.025")
        hinge(hip,leg+"_abduction_joint","1 0 0","-0.55 0.55")
        ET.SubElement(hip,"geom",type="sphere",size="0.065",material="amber",mass="0.35")
        thigh = ET.SubElement(hip,"body",name=leg+"_thigh")
        hinge(thigh,leg+"_hip_joint","0 1 0","-1.4 1.0")
        ET.SubElement(thigh,"geom",type="capsule",fromto=f"0 0 -0.035 0 0 {-UPPER_LEG+0.035}",size="0.043",material="shell",mass="1.4")
        shin = ET.SubElement(thigh,"body",name=leg+"_shin",pos=f"0 0 {-UPPER_LEG}")
        hinge(shin,leg+"_knee_joint","0 1 0","0.05 2.3")
        ET.SubElement(shin,"geom",type="sphere",size="0.057",material="dark",mass="0.15")
        ET.SubElement(shin,"geom",type="capsule",fromto=f"0 0 -0.025 0 0 {-LOWER_LEG+0.025}",size="0.032",material="shell",mass="0.95")
        foot = ET.SubElement(shin,"body",name=leg+"_foot",pos=f"0 0 {-LOWER_LEG}")
        ET.SubElement(foot,"geom",name=leg+"_pad",type="sphere",size="0.042",material="dark",mass="0.15")
        ET.SubElement(foot,"site",name=leg+"_toe",size="0.01",rgba="1 0.5 0.1 0")

    yaw=ET.SubElement(trunk,"body",name="waist_yaw",pos="0.22 0 0.13")
    hinge(yaw,"waist_yaw_joint","0 0 1","-0.7 0.7",50)
    ET.SubElement(yaw,"geom",type="cylinder",size="0.075 0.04",material="dark",mass="0.6")
    roll=ET.SubElement(yaw,"body",name="waist_roll",pos="0 0 0.04")
    hinge(roll,"waist_roll_joint","1 0 0","-0.35 0.35",60)
    ET.SubElement(roll,"geom",type="sphere",size="0.068",material="amber",mass="0.4")
    chest=ET.SubElement(roll,"body",name="torso",pos="0 0 0.04")
    hinge(chest,"waist_pitch_joint","0 1 0","-0.35 0.35",60)
    ET.SubElement(chest,"geom",type="box",size="0.105 0.155 0.17",pos="0 0 0.19",material="shell",mass="6")
    ET.SubElement(chest,"geom",type="box",size="0.025 0.09 0.075",pos="0.115 0 0.23",material="dark",mass="0.15")
    ET.SubElement(chest,"geom",type="box",size="0.008 0.07 0.012",pos="0.145 0 0.25",material="amber",mass="0.02",contype="0",conaffinity="0")
    neck=ET.SubElement(chest,"body",name="neck",pos="0 0 0.41")
    hinge(neck,"head_yaw_joint","0 0 1","-1.1 1.1",15)
    ET.SubElement(neck,"geom",type="cylinder",size="0.04 0.035",material="dark",mass="0.2")
    head=ET.SubElement(neck,"body",name="head",pos="0 0 0.065")
    hinge(head,"head_pitch_joint","0 1 0","-0.45 0.45",15)
    ET.SubElement(head,"geom",type="box",size="0.075 0.083 0.075",pos="0 0 0.04",material="shell",mass="1.0")
    ET.SubElement(head,"geom",type="box",size="0.013 0.072 0.027",pos="0.079 0 0.055",material="visor",mass="0.05")
    ET.SubElement(head,"geom",type="box",size="0.006 0.055 0.006",pos="0.094 0 0.055",material="amber",mass="0.01",contype="0",conaffinity="0")
    for side,sgn in (("left",1),("right",-1)):
        shoulder=ET.SubElement(chest,"body",name=side+"_shoulder",pos=f"0 {sgn*0.205} 0.34")
        hinge(shoulder,side+"_shoulder_pitch_joint","0 1 0","-2.1 1.0",40)
        ET.SubElement(shoulder,"geom",type="sphere",size="0.055",material="dark",mass="0.35")
        arm=ET.SubElement(shoulder,"body",name=side+"_upper_arm")
        hinge(arm,side+"_shoulder_roll_joint","1 0 0","-1.6 1.6",30)
        ET.SubElement(arm,"geom",type="capsule",fromto="0 0 -0.03 0 0 -0.21",size="0.035",material="shell",mass="0.9")
        forearm=ET.SubElement(arm,"body",name=side+"_forearm",pos="0 0 -0.24")
        hinge(forearm,side+"_elbow_joint","0 1 0","-2.1 0.15",25)
        ET.SubElement(forearm,"geom",type="sphere",size="0.042",material="amber",mass="0.15")
        ET.SubElement(forearm,"geom",type="capsule",fromto="0 0 -0.02 0 0 -0.19",size="0.03",material="shell",mass="0.65")
        hand=ET.SubElement(forearm,"body",name=side+"_hand",pos="0 0 -0.22")
        hinge(hand,side+"_wrist_joint","0 0 1","-1.3 1.3",12)
        ET.SubElement(hand,"geom",type="box",size="0.035 0.045 0.055",pos="0 0 -0.03",material="dark",mass="0.3")
        ET.SubElement(hand,"geom",type="box",size="0.008 0.047 0.025",pos="0.035 0 -0.03",material="amber",mass="0.02",contype="0",conaffinity="0")
    ET.indent(r)
    return ET.tostring(r,encoding="unicode")


if __name__ == "__main__":
    path=ROOT/"assets/centaur.xml"
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(build_xml(),encoding="utf-8")
    print(path)
