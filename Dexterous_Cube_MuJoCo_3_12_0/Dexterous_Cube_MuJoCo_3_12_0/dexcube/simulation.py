from __future__ import annotations
from collections import deque
import math

import mujoco
import numpy as np

from .cube import CubeState, FACE_NORMALS, inverse_move, parse_move, scramble_moves
from .scene import CUBE_POSITION, FINGERS, HAND_JOINTS, scene_xml


def smooth(s):
    s = float(np.clip(s, 0, 1))
    return s**3 * (10 - 15*s + 6*s*s)


def object_id(model, kind, name):
    result = mujoco.mj_name2id(model, kind, name)
    if result < 0:
        raise ValueError('Missing model object: ' + name)
    return result


class CubeTask:
    """Contact-gated assisted grasp and internal-servo layer turning."""
    def __init__(self):
        self.state = CubeState()
        self.revision = 0
        self._build()
        self.grasp_q = self._calibrate()
        self.open_q = np.zeros(len(HAND_JOINTS))
        self.open_q[1::4] = .12
        self.open_q[2::4] = .15
        self.open_q[3::4] = .10
        self.open_q[16] = self.open_q[36] = .40
        self.reset()

    def _build(self, active=None, preserve=False):
        saved = None
        if preserve:
            saved = {'q': self.data.qpos[self.qa].copy(), 'v': self.data.qvel[self.da].copy(),
                     'cube_q': self.data.qpos[self.cube_qa:self.cube_qa+7].copy(),
                     'cube_v': self.data.qvel[self.cube_da:self.cube_da+6].copy(),
                     'time': float(self.data.time), 'mpos': self.data.mocap_pos.copy(),
                     'mquat': self.data.mocap_quat.copy(), 'eq': bool(self.data.eq_active[self.weld]),
                     'stages_q': self.data.qpos[self.stage_qa].copy(), 'stages_v': self.data.qvel[self.stage_da].copy(),
                     'pieces': {p.name: self.data.xpos[object_id(self.model,mujoco.mjtObj.mjOBJ_BODY,p.name)].copy() for p in self.state.cubies}}
        self.model = m = mujoco.MjModel.from_xml_string(scene_xml(self.state, active))
        self.data = d = mujoco.MjData(m)
        self.ik = mujoco.MjData(m)
        self.jids = np.array([object_id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in HAND_JOINTS])
        self.qa, self.da = m.jnt_qposadr[self.jids], m.jnt_dofadr[self.jids]
        self.motors = np.array([object_id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in HAND_JOINTS])
        stages = np.array([object_id(m, mujoco.mjtObj.mjOBJ_JOINT, side+'_retract') for side in ('left', 'right')])
        self.stage_qa, self.stage_da = m.jnt_qposadr[stages], m.jnt_dofadr[stages]
        self.stage_motors = np.array([object_id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, side+'_retract') for side in ('left', 'right')])
        joint = object_id(m, mujoco.mjtObj.mjOBJ_JOINT, 'cube_free')
        self.cube_qa, self.cube_da = int(m.jnt_qposadr[joint]), int(m.jnt_dofadr[joint])
        self.cube_body = object_id(m, mujoco.mjtObj.mjOBJ_BODY, 'cube_core')
        turn = object_id(m, mujoco.mjtObj.mjOBJ_JOINT, 'layer_turn')
        self.turn_qa, self.turn_da = int(m.jnt_qposadr[turn]), int(m.jnt_dofadr[turn])
        self.turn_motor = object_id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, 'layer_motor')
        self.weld = object_id(m, mujoco.mjtObj.mjOBJ_EQUALITY, 'assisted_grasp')
        self.tip_sites, self.pad_fingers = {}, {}
        for side in ('left', 'right'):
            for finger in FINGERS:
                name = side + '_' + finger
                self.tip_sites[name] = object_id(m, mujoco.mjtObj.mjOBJ_SITE, name + '_tip')
                self.pad_fingers[object_id(m, mujoco.mjtObj.mjOBJ_GEOM, name + '_pad')] = name
        self.cube_geoms = {i for i in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or '').endswith('_plastic')}
        if saved:
            d.qpos[self.qa], d.qvel[self.da] = saved['q'], saved['v']
            d.qpos[self.cube_qa:self.cube_qa+7] = saved['cube_q']
            d.qvel[self.cube_da:self.cube_da+6] = saved['cube_v']
            d.time, d.mocap_pos[:], d.mocap_quat[:] = saved['time'], saved['mpos'], saved['mquat']
            d.eq_active[self.weld] = saved['eq']
            d.qpos[self.stage_qa], d.qvel[self.stage_da] = saved['stages_q'], saved['stages_v']
            d.ctrl[self.motors] = self.grasp_q
        self.revision += 1
        mujoco.mj_forward(m, d)
        if saved:
            jump = max(float(np.linalg.norm(d.xpos[object_id(m,mujoco.mjtObj.mjOBJ_BODY,name)]-position)) for name,position in saved['pieces'].items())
            self.max_topology_position_jump = max(self.max_topology_position_jump, jump)

    def _calibrate(self):
        m, d = self.model, self.ik
        q = np.zeros(len(HAND_JOINTS))
        for i in range(0, len(q), 4):
            q[i:i+4] = [0, .10, 1.2, .85]
            if (i // 4) % 5 == 4:
                q[i:i+4] = [-.17, .05, -.10, -.10]
        d.qpos[:] = self.data.qpos
        d.qpos[self.qa] = q
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for side, sign in [('left', 1), ('right', -1)]:
            palm_x = -.078 * sign
            for f, finger in enumerate(FINGERS):
                group = np.array([HAND_JOINTS.index(f'{side}_{finger}_{i}') for i in range(4)])
                target_local = [ .063, -.037, .021] if finger == 'thumb' else [.0405, [-.027, -.009, .010, .027][f], [.056, .052, .048, .044][f]]
                target = np.array([palm_x + sign * target_local[0], sign * target_local[1], .159 + target_local[2]])
                qa, da = self.qa[group], self.da[group]
                limits = m.jnt_range[self.jids[group]]
                site = self.tip_sites[f'{side}_{finger}']
                for _ in range(110):
                    mujoco.mj_kinematics(m, d)
                    mujoco.mj_comPos(m, d)
                    error = target - d.site_xpos[site]
                    if np.linalg.norm(error) < .00015:
                        break
                    mujoco.mj_jacSite(m, d, jp, jr, site)
                    j = jp[:, da]
                    delta = j.T @ np.linalg.solve(j @ j.T + .001**2 * np.eye(3), error)
                    d.qpos[qa] = np.clip(d.qpos[qa] + np.clip(delta, -.10, .10), limits[:, 0]+.01, limits[:, 1]-.01)
        return d.qpos[self.qa].copy()

    def reset(self):
        self.state = CubeState()
        self._build()
        self.data.qpos[self.qa] = self.grasp_q
        self.data.ctrl[self.motors] = self.grasp_q
        self.data.mocap_pos[0] = CUBE_POSITION
        self.gripping, self.grasped, self.paused = True, False, False
        self.moves = deque()
        self.current = None
        self.turn_started = None
        self.completed = 0
        self.contacts = set()
        self.grasp_stable_steps = 0
        self.max_contacts = 0
        self.max_hand_limit_violation = 0.0
        self.max_anchor_error = 0.0
        self.max_layer_error = 0.0
        self.max_topology_position_jump = 0.0
        self.motor_work = 0.0
        self.grasp_failures = 0
        self.turn_records = []
        self.message = '双手正在建立接触抓持'
        self.error = None
        self.restore_mode = False
        self.release_at = None
        self.peak_height = float(CUBE_POSITION[2])
        self.phase = 'GRASP'
        mujoco.mj_forward(self.model, self.data)

    def enqueue(self, moves):
        moves = list(moves)
        for token in moves:
            parse_move(token)
        if self.error:
            raise ValueError('请先复位场景')
        if not self.gripping:
            raise ValueError('请复位并重新建立抓持')
        if len(self.moves) + len(moves) > 40:
            raise ValueError('待执行动作最多 40 步')
        self.moves.extend(moves)

    def scramble(self, count=6, seed=None):
        if self.current or self.moves:
            raise ValueError('请等待当前动作完成')
        self.enqueue(scramble_moves(count, seed))

    def restore(self):
        if self.current or self.moves:
            raise ValueError('请等待当前动作完成')
        moves = self.state.inverse_history()
        if not moves:
            self.message = '魔方已处于还原状态'
            return
        if not self.gripping:
            raise ValueError('请先复位抓持')
        if len(moves) > 512:
            raise ValueError('操作历史过长，请复位后重新开始')
        self.moves.extend(moves)
        self.restore_mode = True
        self.message = '按已知操作历史逆序还原'

    def undo(self):
        if self.current or self.moves:
            raise ValueError('请等待当前动作完成')
        if self.state.history:
            self.enqueue([inverse_move(self.state.history[-1])])

    def release(self):
        if self.current or self.moves:
            raise ValueError('请先完成或清空待执行动作')
        self.gripping, self.grasped = False, False
        self.data.eq_active[self.weld] = False
        self.release_at = float(self.data.time)
        self.message, self.phase = '已解除辅助抓持，魔方自由下落', 'RELEASED'

    def _contact_fingers(self):
        fingers = set()
        for c in self.data.contact[:self.data.ncon]:
            a, b = int(c.geom1), int(c.geom2)
            if a in self.pad_fingers and b in self.cube_geoms:
                fingers.add(self.pad_fingers[a])
            if b in self.pad_fingers and a in self.cube_geoms:
                fingers.add(self.pad_fingers[b])
        return fingers

    def _start_move(self):
        token = self.moves.popleft()
        self.current = token
        self._build(active=token, preserve=True)
        self.turn_started = float(self.data.time)
        _, turns = parse_move(token)
        self.turn_target = -turns * math.pi / 2
        self.phase = 'RELAX'
        self.message = '配合转层 ' + token

    def step(self):
        if self.paused or self.error:
            return
        m, d = self.model, self.data
        if self.gripping and not self.grasped:
            if len(self.contacts) >= 3 and len({s.split('_')[0] for s in self.contacts}) == 2:
                self.grasp_stable_steps += 1
            else:
                self.grasp_stable_steps = 0
            if self.grasp_stable_steps >= 8:
                d.mocap_pos[0] = d.xpos[self.cube_body]
                d.mocap_quat[0] = d.xquat[self.cube_body]
                d.eq_active[self.weld] = True
                self.grasped, self.phase = True, 'READY'
                self.message = '抓持成立，可选择魔方面转动'
            if d.time > 1.0 and not self.grasped:
                self.message = '尚未建立双手接触，请复位'
        if self.grasped and self.current is None and self.moves and self.phase == 'READY':
            self._start_move()
            m, d = self.model, self.data
        grip_amount = 1.0 if self.gripping else 1 - smooth((d.time - (self.release_at or 0))/.35)
        if self.current:
            elapsed = float(d.time) - self.turn_started
            duration = .95 if not self.current.endswith('2') else 1.30
            if elapsed < .32:
                self.phase = 'RELAX'
                grip_amount = 1 - smooth(elapsed/.32)
            elif elapsed < .32 + duration:
                self.phase = 'TURN'
                grip_amount = 0.0
                d.ctrl[self.turn_motor] = self.turn_target*smooth((elapsed-.32)/duration)
            else:
                self.phase = 'SETTLE'
                d.ctrl[self.turn_motor] = self.turn_target
                grip_amount = 0.0
                angle_error = abs(float(d.qpos[self.turn_qa]) - self.turn_target)
                velocity = abs(float(d.qvel[self.turn_da]))
                if angle_error < .008 and velocity < .055:
                    self.state.apply(self.current)
                    self.max_layer_error = max(self.max_layer_error, angle_error)
                    self.turn_records.append({'move': self.current, 'time_s': float(d.time), 'angle_error_rad': angle_error, 'angular_velocity_rad_s': velocity})
                    self.completed += 1
                    self.current = None
                    self._build(preserve=True)
                    m, d = self.model, self.data
                    self.phase = 'REGRASP'
                    self.regrasp_at = float(d.time)
                    self.message = '转层完成，重新抓持'
                elif elapsed > duration + 2.8:
                    self.error = '转层未达到端点，请复位'
        elif self.phase == 'REGRASP':
            grip_amount = smooth((d.time-self.regrasp_at)/.45)
            if d.time-self.regrasp_at > .55:
                self.phase = 'READY'
                self.message = '魔方已还原' if self.state.solved else '动作完成，等待下一步'
                if self.restore_mode and not self.moves:
                    if self.state.solved:
                        self.state.history.clear()
                    self.restore_mode = False
        d.ctrl[self.motors] = self.open_q + grip_amount*(self.grasp_q-self.open_q)
        d.ctrl[self.stage_motors] = .026*(1-grip_amount)
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        self.contacts = self._contact_fingers()
        self.max_contacts = max(self.max_contacts, len(self.contacts))
        self.peak_height = max(self.peak_height, float(d.xpos[self.cube_body, 2]))
        self.motor_work += abs(float(d.actuator_force[self.turn_motor])*float(d.qvel[self.turn_da]))*.002
        limits, q = m.jnt_range[self.jids], d.qpos[self.qa]
        self.max_hand_limit_violation = max(self.max_hand_limit_violation, float(np.max(np.maximum(limits[:,0]-q, q-limits[:,1]))))
        if self.grasped:
            self.max_anchor_error = max(self.max_anchor_error, float(np.linalg.norm(d.xpos[self.cube_body]-d.mocap_pos[0])))
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            self.error = '仿真出现非有限状态，请复位'

    def status(self):
        return {'time': round(float(self.data.time), 2), 'phase': self.phase, 'message': self.message,
                'error': self.error, 'paused': self.paused, 'grasped': self.grasped,
                'contacts': len(self.contacts), 'finger_contacts': sorted(self.contacts),
                'solved': self.state.solved, 'completed': self.completed, 'current': self.current,
                'queue': list(self.moves), 'history': self.state.history[-32:], 'faces': self.state.facelets(),
                'cube_position': self.data.xpos[self.cube_body].tolist(),
                'turn_angle': round(float(self.data.qpos[self.turn_qa])*180/math.pi, 1),
                'mujoco_version': mujoco.__version__, 'assisted': True, 'revision': self.revision}

    def report(self):
        return {'mujoco_version': mujoco.__version__, 'simulation_seconds': float(self.data.time),
                'solved': self.state.solved, 'completed_moves': self.completed, 'history': self.state.history,
                'max_finger_contacts': self.max_contacts, 'grasped': self.grasped,
                'cube_position_m': self.data.xpos[self.cube_body].tolist(),
                'max_hand_joint_violation_rad': self.max_hand_limit_violation,
                'max_assisted_grasp_position_error_m': self.max_anchor_error,
                'max_layer_endpoint_error_rad': self.max_layer_error, 'layer_motor_abs_work_j': self.motor_work,
                'max_cubie_endpoint_rebuild_jump_m': self.max_topology_position_jump,
                'turn_records': self.turn_records, 'error': self.error,
                'limitations': ['Original primitive dual hands, 20 finger DOF each and 2 assisted wrist slides.',
                    'Contact-gated mocap weld assists cube grasp.', 'An internal layer position servo drives face rotation.',
                    'Cubie topology rebuilt at settled endpoints; cube pose/velocity and hand states preserved.',
                    'History inversion only; no vision, arbitrary-state solver or learned manipulation policy.']}
