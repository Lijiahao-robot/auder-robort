from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))

import mujoco
from dexcube.scene import scene_xml,vec
from dexcube.simulation import CubeTask

task=CubeTask()
tree=ET.fromstring(scene_xml(task.state))
keys=ET.SubElement(tree,'keyframe')
ET.SubElement(keys,'key',name='grasp_ready',qpos=vec(task.data.qpos),ctrl=vec(task.data.ctrl),
              mpos=vec(task.data.mocap_pos.ravel()),mquat=vec(task.data.mocap_quat.ravel()))
ET.indent(tree)
path=ROOT/'scene.xml'
path.write_text(ET.tostring(tree,encoding='unicode'),encoding='utf-8')
model=mujoco.MjModel.from_xml_path(str(path))
assert model.nu==43 and model.nmocap==1 and model.neq==1
print('Scene loaded: 40 finger actuators + 2 wrist slides + 1 internal layer servo; 26 cubies.')
