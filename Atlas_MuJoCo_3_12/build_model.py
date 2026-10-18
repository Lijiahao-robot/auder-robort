"""Reproducibly adapt the attributed legacy Atlas model to MuJoCo 3.12."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def build():
    original = (ROOT / 'models' / 'atlas_source.xml').read_text()
    header = original[:original.index('<mujoco')]
    root = ET.fromstring(original)
    root.set('model', 'Atlas legacy — 27 actuated joints')
    compiler = root.find('compiler')
    compiler.attrib.pop('coordinate', None)
    compiler.set('inertiafromgeom', 'false')
    root.find('size').attrib.clear()
    option = root.find('option')
    option.attrib.update(timestep='0.002', integrator='implicitfast', iterations='80',
                         solver='Newton', cone='elliptic', gravity='0 0 -9.81')
    default = root.find('default')
    default.find('joint').attrib.update(damping='1.0', armature='0.03')
    default.find('geom').attrib.update(contype='1', conaffinity='2',
                                      friction='1.1 0.005 0.0001',
                                      solref='0.008 1', solimp='0.95 0.99 0.001',
                                      rgba='0.35 0.55 0.75 0.15')
    default.find('motor').attrib.update(gear='1', ctrlrange='-200 200')
    world = root.find('worldbody')
    pelvis = world.find("body[@name='pelvis']")
    for joint in list(pelvis.findall('joint')):
        pelvis.remove(joint)
    pelvis.insert(0, ET.Element('freejoint', name='floating_base'))
    for joint in root.findall('.//body/joint'):
        joint.set('damping', '1.0')
        joint.set('stiffness', '0')
    for light in list(pelvis.findall('light')):
        pelvis.remove(light)
    motors = root.find('actuator')
    for motor in motors:
        limit = float(motor.get('gear', '1')) * 0.95
        motor.set('gear', '1')
        motor.set('ctrlrange', f'{-limit:g} {limit:g}')
    for side, word in [('l', 'left'), ('r', 'right')]:
        foot = root.find(f".//body[@name='{side}_foot']")
        foot.insert(1, ET.Element('joint', name=f'{side}_ankle_roll', axis='1 0 0',
                                  range='-0.8 0.8', damping='1', armature='0.03'))
        ET.SubElement(motors, 'motor', name=f'{side}_ankle_roll_actuator',
                      joint=f'{side}_ankle_roll', gear='1', ctrlrange='-342 342')
        sole = foot.find(f"geom[@name='{word}_foot_back']")
        sole.set('size', '0.14 0.071 0.025')
        sole.set('pos', '0.04 0 -0.049119')
        sole.set('condim', '6')
        sole.set('friction', '1.2 0.015 0.001')
        front = foot.find(f"geom[@name='{word}_foot_front']")
        foot.remove(front)
        ET.SubElement(foot, 'site', name=f'{side}_sole', pos='0.04 0 -0.074119',
                      size='0.006', rgba='0.2 0.8 0.9 0.5', group='3')
    for body in root.findall('.//body'):
        name = body.get('name', '')
        color = '0.72 0.77 0.82 1'
        if name in ('utorso', 'mtorso', 'ltorso'):
            color = '0.24 0.43 0.67 1'
        elif name == 'pelvis' or 'glut' in name:
            color = '0.25 0.32 0.42 1'
        elif 'foot' in name:
            color = '0.20 0.24 0.30 1'
        for geom in body.findall("geom[@class='viz']"):
            geom.set('rgba', color)
    floor = world.find("geom[@name='floor']")
    floor.attrib.update(contype='2', conaffinity='1', size='30 30 0.1',
                        rgba='1 1 1 1', friction='1.2 0.015 0.001')
    for texture in root.find('asset').findall('texture'):
        if texture.get('type') == 'skybox':
            texture.attrib.update(rgb1='0.13 0.17 0.23', rgb2='0.025 0.035 0.055')
        else:
            texture.attrib.update(rgb1='0.11 0.14 0.18', rgb2='0.15 0.18 0.23',
                                  width='512', height='512')
    root.find("asset/material[@name='MatPlane']").attrib.update(texrepeat='4 4', reflectance='0.1')
    for light in list(world.findall('light')):
        world.remove(light)
    ET.SubElement(world, 'light', name='key', pos='2 -3 5', dir='-0.4 0.5 -1',
                  diffuse='0.9 0.9 0.9', specular='0.25 0.25 0.25', castshadow='true')
    ET.SubElement(world, 'light', name='fill', pos='-3 2 4', dir='0.5 -0.3 -1',
                  diffuse='0.5 0.65 0.85', castshadow='false')
    visual = root.find('visual')
    visual.find('map').attrib.update(fogstart='30', fogend='80', znear='0.01')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='800')
    ET.SubElement(visual, 'quality', shadowsize='2048')
    ET.SubElement(root, 'statistic', center='0 0 1', extent='3')
    ET.indent(root, space='  ')
    target = ROOT / 'models' / 'atlas.xml'
    target.write_text(header + '<!-- Adaptation: free base, restored ankle rolls, SI torque motors, updated contact/visual settings. -->\n' + ET.tostring(root, encoding='unicode'), encoding='utf8')
    return target


if __name__ == '__main__':
    print(build())
