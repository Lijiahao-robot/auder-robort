from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def scene_xml(mesh_dir=None):
    tree = ET.parse(ROOT / 'assets/g1/g1_29dof.xml').getroot()
    tree.set('model', 'G1_assisted_soccer')
    tree.find('compiler').set('meshdir', str(mesh_dir or ROOT / 'assets/g1/meshes'))
    ET.SubElement(tree, 'option', timestep='.002', integrator='implicitfast', gravity='0 0 -9.81', iterations='80')
    visual = ET.SubElement(tree, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='720')
    ET.SubElement(visual, 'headlight', ambient='.55 .55 .55', diffuse='.7 .7 .7')
    ET.SubElement(tree, 'statistic', center='0 0 .35', extent='6')
    asset, world = tree.find('asset'), tree.find('worldbody')
    ET.SubElement(asset, 'texture', name='sky', type='skybox', builtin='gradient', rgb1='.10 .17 .22', rgb2='.55 .66 .69', width='512', height='3072')
    ET.SubElement(asset, 'texture', name='grass_tex', type='2d', builtin='checker', rgb1='.10 .29 .15', rgb2='.13 .34 .18', width='512', height='512')
    ET.SubElement(asset, 'material', name='grass', texture='grass_tex', texrepeat='8 5', reflectance='.02')
    ET.SubElement(asset, 'texture', name='football_tex', type='2d', file=str(ROOT / 'assets/football.png'))
    ET.SubElement(asset, 'material', name='football_mat', texture='football_tex', texuniform='true', specular='.3', shininess='.3')
    pelvis = world.find("body[@name='pelvis']")
    for geom in pelvis.iter('geom'):
        visible = geom.get('contype') == '0' and geom.get('conaffinity') == '0'
        geom.set('contype', '0')
        geom.set('conaffinity', '0')
        geom.set('group', '1' if visible else '3')
    for side in ('left', 'right'):
        ankle = pelvis.find(f".//body[@name='{side}_ankle_roll_link']")
        ET.SubElement(ankle, 'geom', name=side + '_sole_pad', type='box', pos='.035 0 -.035', size='.073 .038 .006', mass='0', rgba='.06 .11 .14 1', friction='1.0 .002 .0005', solref='.008 1')
        ET.SubElement(ankle, 'site', name=side + '_sole', pos='.035 0 -.041', size='.004', rgba='0 0 0 0')
    ET.SubElement(world, 'light', pos='0 -2 5', dir='.1 .2 -1', directional='true')
    ET.SubElement(world, 'light', pos='2 3 4', dir='-.3 -.5 -1', diffuse='.4 .4 .4', castshadow='false')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='7 5 .1', material='grass', condim='6', friction='.8 .002 .0005', solref='.008 1')

    def line(a, b, width=.012, color='.92 .97 .91 1'):
        ET.SubElement(world, 'geom', type='capsule', fromto=' '.join(str(v) for v in (*a, .003, *b, .003)), size=str(width), rgba=color, contype='0', conaffinity='0', group='1')

    for a, b in [((-4, -2.4), (4, -2.4)), ((-4, 2.4), (4, 2.4)), ((-4, -2.4), (-4, 2.4)), ((4, -2.4), (4, 2.4)), ((0, -2.4), (0, 2.4))]:
        line(a, b)
    for i in range(64):
        line((.6 * math.cos(i * math.tau / 64), .6 * math.sin(i * math.tau / 64)),
             (.6 * math.cos((i + 1) * math.tau / 64), .6 * math.sin((i + 1) * math.tau / 64)), .009)
    for sign in (-1, 1):
        x = sign * 4.0
        for y in (-.9, .9):
            ET.SubElement(world, 'geom', name=f'post_{sign}_{y}', type='capsule', fromto=f'{x} {y} 0 {x} {y} .9', size='.035', rgba='.92 .97 .96 1', friction='.5 .001 .0002')
        ET.SubElement(world, 'geom', name=f'bar_{sign}', type='capsule', fromto=f'{x} -.9 .9 {x} .9 .9', size='.035', rgba='.92 .97 .96 1')
        ET.SubElement(world, 'geom', name=f'goal_back_{sign}', type='box', pos=f'{x + sign * .6} 0 .45', size='.018 .9 .45', rgba='.55 .76 .67 .20', friction='.6 .002 .001')
        for i in range(13):
            z = i * .9 / 12
            ET.SubElement(world, 'geom', type='capsule', fromto=f'{x + sign * .59} -.9 {z} {x + sign * .59} .9 {z}', size='.003', rgba='.63 .8 .72 .65', contype='0', conaffinity='0')
        for i in range(19):
            y = -.9 + i * .1
            ET.SubElement(world, 'geom', type='capsule', fromto=f'{x + sign * .59} {y} 0 {x + sign * .59} {y} .9', size='.003', rgba='.63 .8 .72 .65', contype='0', conaffinity='0')
        line((sign * 3.0, -1.4), (sign * 3.0, 1.4), .009)
        line((sign * 3.0, -1.4), (x, -1.4), .009)
        line((sign * 3.0, 1.4), (x, 1.4), .009)
    ball = ET.SubElement(world, 'body', name='football', pos='-.65 -.12 .105')
    ET.SubElement(ball, 'freejoint', name='football_free')
    ET.SubElement(ball, 'geom', name='football_geom', type='sphere', size='.105', mass='.43', material='football_mat', condim='6', friction='.55 .001 .0002', solref='.006 .9', solimp='.95 .99 .001')
    target = ET.SubElement(world, 'body', name='virtual_base', mocap='true', pos=pelvis.get('pos'))
    ET.SubElement(target, 'site', name='virtual_base_site', size='.001', rgba='0 0 0 0')
    ET.SubElement(pelvis, 'site', name='pelvis_support', size='.001', rgba='0 0 0 0')
    equality = ET.SubElement(tree, 'equality')
    ET.SubElement(equality, 'weld', name='assisted_base_support', site1='virtual_base_site', site2='pelvis_support', solref='.008 1')
    return ET.tostring(tree, encoding='unicode')
