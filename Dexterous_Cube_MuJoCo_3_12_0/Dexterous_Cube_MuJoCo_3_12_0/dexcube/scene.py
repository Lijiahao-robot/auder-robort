from __future__ import annotations
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .cube import FACE_NORMALS, FACE_COLORS, HALF, PITCH

CUBE_POSITION = np.array([0., 0., .205])
FINGERS = ('index', 'middle', 'ring', 'little', 'thumb')
HAND_JOINTS = [f'{side}_{finger}_{i}' for side in ('left', 'right') for finger in FINGERS for i in range(4)]


def vec(values):
    return ' '.join(f'{float(v):.10g}' for v in values)


def scene_xml(state, active=None):
    tree = ET.Element('mujoco', model='dual_dexterous_cube_assisted')
    ET.SubElement(tree, 'compiler', angle='radian', autolimits='true')
    ET.SubElement(tree, 'option', timestep='.002', integrator='implicitfast', gravity='0 0 -9.81', iterations='80', cone='elliptic')
    default = ET.SubElement(tree, 'default')
    ET.SubElement(default, 'joint', damping='.015', armature='.000025')
    ET.SubElement(default, 'geom', solref='.008 1', solimp='.95 .99 .001', friction='1.0 .004 .0001', margin='.00015')
    ET.SubElement(default, 'position', kp='1.6', kv='.035', forcerange='-.35 .35', ctrllimited='true')
    visual = ET.SubElement(tree, 'visual')
    ET.SubElement(visual, 'global', offwidth='1280', offheight='720')
    ET.SubElement(visual, 'headlight', ambient='.5 .5 .5', diffuse='.7 .7 .7')
    ET.SubElement(visual, 'quality', shadowsize='2048')
    ET.SubElement(tree, 'statistic', center='0 0 .15', extent='.32')
    asset = ET.SubElement(tree, 'asset')
    ET.SubElement(asset, 'texture', name='sky', type='skybox', builtin='gradient', rgb1='.08 .12 .19', rgb2='.43 .49 .56', width='256', height='1536')
    ET.SubElement(asset, 'material', name='table_mat', rgba='.12 .16 .22 1', specular='.35', shininess='.45')
    world = ET.SubElement(tree, 'worldbody')
    ET.SubElement(world, 'light', pos='-.2 -.3 .8', dir='.2 .3 -1', directional='true')
    ET.SubElement(world, 'light', pos='.3 .3 .6', dir='-.3 -.3 -1', diffuse='.4 .4 .4', castshadow='false')
    ET.SubElement(world, 'geom', name='table', type='box', pos='0 0 -.012', size='.32 .25 .012', material='table_mat', contype='4', conaffinity='3')
    # A low tray catches a released cube; it never supports the grasped cube.
    ET.SubElement(world, 'geom', name='catch_tray', type='cylinder', pos='0 0 .018', size='.063 .018', rgba='.17 .22 .29 1', contype='4', conaffinity='3')
    actuators = ET.SubElement(tree, 'actuator')
    for side, sign, color in [('left', -1, '.38 .62 .85 1'), ('right', 1, '.90 .57 .31 1')]:
        root = ET.SubElement(world, 'body', name=side + '_hand', pos=vec([sign * .078, 0, .159]), quat=('1 0 0 0' if sign == -1 else '0 0 0 1'))
        ET.SubElement(root, 'joint', name=side + '_retract', type='slide', axis='-1 0 0', range='0 .032', damping='2', armature='.001')
        ET.SubElement(actuators, 'position', name=side + '_retract', joint=side + '_retract', kp='300', kv='8', ctrlrange='0 .03', forcerange='-12 12')
        ET.SubElement(root, 'geom', name=side + '_palm', type='box', size='.009 .040 .034', mass='.17', rgba=color, contype='2', conaffinity='1')
        ET.SubElement(root, 'geom', type='cylinder', pos='0 0 -.058', size='.018 .024', rgba='.22 .28 .35 1', contype='0', conaffinity='0')
        ET.SubElement(world, 'geom', type='box', pos=vec([sign * .078, 0, .040]), size='.025 .030 .040', rgba='.17 .22 .29 1', contype='0', conaffinity='0')
        for finger, y in zip(FINGERS, (-.028, -.009, .010, .029, -.048)):
            thumb = finger == 'thumb'
            body = ET.SubElement(root, 'body', name=f'{side}_{finger}_spread', pos=vec([.008, y, -.016 if thumb else .034]),
                                 **({'quat': '.9004471 0 .4349655 0'} if thumb else {}))
            lower, upper = (-.8, .8) if thumb else (-.45, .45)
            joint_name = f'{side}_{finger}_0'
            ET.SubElement(body, 'joint', name=joint_name, axis='1 0 0', range=vec([lower, upper]))
            ET.SubElement(body, 'geom', type='sphere', size='.005', mass='.004', rgba='.10 .15 .21 1', contype='0', conaffinity='0')
            ET.SubElement(actuators, 'position', name=joint_name, joint=joint_name, ctrlrange=vec([lower, upper]))
            lengths = [.029, .022, .017] if thumb else [.025, .020, .016]
            for i, length in enumerate(lengths, 1):
                segment = ET.SubElement(body, 'body', name=f'{side}_{finger}_link_{i}')
                joint_name = f'{side}_{finger}_{i}'
                ET.SubElement(segment, 'joint', name=joint_name, axis='0 1 0', range='-.40 1.8')
                ET.SubElement(segment, 'geom', type='capsule', fromto=f'0 0 0 0 0 {length}', size='.0042', mass='.009', rgba='.72 .77 .79 1', contype='2', conaffinity='1')
                ET.SubElement(segment, 'geom', type='sphere', size='.0048', mass='.001', rgba='.08 .13 .19 1', contype='0', conaffinity='0')
                ET.SubElement(actuators, 'position', name=joint_name, joint=joint_name, ctrlrange='-.40 1.8')
                body = ET.SubElement(segment, 'body', name=f'{side}_{finger}_end_{i}', pos=f'0 0 {length}')
            ET.SubElement(body, 'geom', name=f'{side}_{finger}_pad', type='sphere', size='.006', mass='.003', rgba=color, contype='2', conaffinity='1', friction='1.3 .004 .0001')
            ET.SubElement(body, 'site', name=f'{side}_{finger}_tip', size='.0015', rgba='0 0 0 0')
    anchor = ET.SubElement(world, 'body', name='grasp_anchor', mocap='true', pos=vec(CUBE_POSITION))
    ET.SubElement(anchor, 'site', name='anchor_frame', size='.001', rgba='0 0 0 0')
    cube = ET.SubElement(world, 'body', name='cube_core', pos=vec(CUBE_POSITION))
    ET.SubElement(cube, 'freejoint', name='cube_free')
    ET.SubElement(cube, 'inertial', pos='0 0 0', mass='.015', diaginertia='.000004 .000004 .000004')
    ET.SubElement(cube, 'site', name='cube_frame', size='.001', rgba='0 0 0 0')
    normal = FACE_NORMALS[active[0]] if active else (0, 0, 1)
    pivot = ET.SubElement(cube, 'body', name='layer_pivot')
    ET.SubElement(pivot, 'joint', name='layer_turn', type='hinge', axis=vec(normal), limited='false', damping='.003', armature='.00002')
    ET.SubElement(pivot, 'inertial', pos='0 0 0', mass='.001', diaginertia='.0000001 .0000001 .0000001')
    selected = {p.name for p in state.selected(active[0])} if active else set()
    for piece in state.cubies:
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, piece.orientation.astype(float).ravel())
        body = ET.SubElement(pivot if piece.name in selected else cube, 'body', name=piece.name, pos=vec(piece.position * PITCH), quat=vec(quat))
        ET.SubElement(body, 'geom', name=piece.name + '_plastic', type='box', size=vec([HALF] * 3), mass='.006', rgba='.055 .065 .075 1', contype='1', conaffinity='6')
        for face, n in FACE_NORMALS.items():
            n = np.array(n)
            if int(piece.home @ n) != 1:
                continue
            axis = int(np.argmax(np.abs(n)))
            half = np.full(3, .0090)
            half[axis] = .00025
            color = FACE_COLORS[face].lstrip('#')
            rgba = [int(color[i:i+2], 16) / 255 for i in (0, 2, 4)] + [1]
            ET.SubElement(body, 'geom', name=piece.name + '_' + face, type='box', size=vec(half), pos=vec(n * (HALF + .00027)), mass='0', rgba=vec(rgba), contype='0', conaffinity='0')
    ET.SubElement(actuators, 'position', name='layer_motor', joint='layer_turn', kp='.8', kv='.025', ctrllimited='false', forcerange='-.6 .6')
    eq = ET.SubElement(tree, 'equality')
    ET.SubElement(eq, 'weld', name='assisted_grasp', site1='anchor_frame', site2='cube_frame', active='false', solref='.006 1')
    return ET.tostring(tree, encoding='unicode')
