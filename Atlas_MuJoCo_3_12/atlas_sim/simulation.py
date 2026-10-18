"""Torque-only Atlas tutorial WBC. No base weld, pose replay or support force."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .model_loader import load_atlas_model

ROOT = Path(__file__).resolve().parents[1]


def smooth(s: float):
    s = float(np.clip(s, 0.0, 1.0))
    return 10*s**3 - 15*s**4 + 6*s**5, 30*s*s*(s-1)**2, 60*s*(2*s*s-3*s+1)


def rz(angle: float):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def rotation_error(current, target):
    return 0.5 * sum((np.cross(current[:, i], target[:, i]) for i in range(3)), np.zeros(3))


@dataclass
class FootReference:
    p: np.ndarray
    R: np.ndarray
    v: np.ndarray
    a: np.ndarray
    contact: bool = True


class AtlasSimulation:
    """Educational inverse-dynamics controller with slow quasi-static gait."""

    def __init__(self, model_dir=None):
        self.model = load_atlas_model(model_dir)
        self.data = mujoco.MjData(self.model)
        self.pelvis = self.model.body('pelvis').id
        self.site_ids = [self.model.site('l_sole').id, self.model.site('r_sole').id]
        self.motor_joints = self.model.actuator_trnid[:, 0].copy()
        self.dofs = self.model.jnt_dofadr[self.motor_joints]
        self.qaddrs = self.model.jnt_qposadr[self.motor_joints]
        self.nv = self.model.nv
        self.mass = float(self.model.body_subtreemass[self.pelvis])
        self.dt = float(self.model.opt.timestep)
        self.control_steps = 4
        self.control_dt = self.dt * self.control_steps
        self.paused = False
        self.mode = 'stand'
        self.pending_mode = None
        self.command = np.zeros(2)  # forward velocity, yaw velocity (tutorial limits)
        self.last_wrench = np.zeros(12)
        self.control_ms = 0.0
        self.reset()

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        d, m = self.data, self.model
        for side in ('l', 'r'):
            for name, angle in [('hip_flexion', .32), ('knee_angle', -.64), ('ankle_angle', .32)]:
                d.joint(f'{name}_{side}').qpos[0] = angle
            d.joint(f'{side}_arm_ely').qpos[0] = .12
            d.joint(f'{side}_arm_elx').qpos[0] = .25 if side == 'l' else -.25
        mujoco.mj_forward(m, d)
        d.qpos[2] -= float(d.site_xpos[self.site_ids[0], 2]) + .0003
        mujoco.mj_forward(m, d)
        self.home = d.qpos.copy()
        self.feet = [d.site_xpos[s].copy() for s in self.site_ids]
        self.foot_rotations = [d.site_xmat[s].reshape(3, 3).copy() for s in self.site_ids]
        self.initial_com = d.subtree_com[self.pelvis].copy()
        self.nominal_com_z = float(self.initial_com[2])
        self.com_offset = np.array([-.015, 0., self.nominal_com_z])
        self.heading = 0.
        self.phase = 'stand'
        self.phase_t = 0.
        self.mode_time = 0.
        self.mode = 'stand'
        self.pending_mode = None
        self.command[:] = 0
        self.support = 1
        self.step_number = 0
        self.tick = 0
        self.fallen = False
        self.previous_jcom = None
        self.previous_jfeet = None
        self.last_com_ref = self.initial_com.copy()
        self.last_tau = np.zeros(m.nu)
        self.last_wrench = np.zeros(12)
        self.max_torque_ratio = 0.
        self.max_tilt = 0.
        self.distance = 0.
        self.paused = False
        self.push_until = -1.
        self.push_force = np.zeros(3)

    def set_mode(self, mode: str):
        if mode not in ('stand', 'wave', 'squat', 'dance', 'walk'):
            raise ValueError('Unknown mode')
        if self.fallen:
            return
        if self.phase != 'stand' and mode != 'walk':
            self.command[:] = 0
            self.pending_mode = mode
            return
        self.mode = mode
        self.initial_com = self.data.subtree_com[self.pelvis].copy()
        self.mode_time = 0.
        self.pending_mode = None
        if mode == 'walk':
            if abs(self.command[0]) + abs(self.command[1]) < 1e-6:
                self.command[0] = .03
            if self.phase == 'stand':
                self._begin_shift()

    def set_velocity(self, forward: float, yaw: float):
        self.command[:] = [np.clip(forward, -.035, .035), np.clip(yaw, -.045, .045)]
        if np.linalg.norm(self.command) > .001:
            if self.mode != 'walk':
                self.set_mode('walk')
        elif self.mode == 'walk':
            self.pending_mode = 'stand'

    def push(self, force=35., seconds=.12):
        self.push_force[:] = [0., force, 0.]
        self.push_until = self.data.time + seconds

    def _begin_shift(self):
        self.phase = 'shift'
        self.phase_t = 0.
        self.shift_start = self.last_com_ref.copy()
        self.shift_goal = self.feet[self.support].copy()
        self.shift_goal += rz(self.heading) @ np.array([-.015, 0., 0.])
        self.shift_goal[2] = self.nominal_com_z

    def _begin_swing(self):
        self.phase = 'swing'
        self.phase_t = 0.
        swing = 1 - self.support
        self.swing_start = self.feet[swing].copy()
        self.swing_rotation = self.foot_rotations[swing].copy()
        self.next_heading = self.heading + float(self.command[1]) * 3.3
        stride = float(np.clip(self.command[0] * 3.3, -.12, .12))
        lateral = .223 if swing == 0 else -.223
        self.swing_goal = self.feet[self.support] + rz(self.next_heading) @ np.array([stride, lateral, 0.])
        self.swing_goal[2] = -.0003

    def references(self, dt):
        self.mode_time += dt
        self.phase_t += dt
        zero = np.zeros(3)
        refs = [FootReference(p.copy(), R.copy(), zero.copy(), zero.copy()) for p, R in zip(self.feet, self.foot_rotations)]
        center = (self.feet[0] + self.feet[1]) * .5
        com = center + rz(self.heading) @ np.array([-.015, 0., 0.])
        com[2] = self.nominal_com_z
        com_v, com_a = zero.copy(), zero.copy()
        if self.mode != 'walk' and self.phase == 'stand':
            blend, _, _ = smooth(min(self.mode_time / 2., 1.))
            com = self.initial_com*(1-blend) + com*blend
            if self.mode == 'squat':
                a = .5*(1-math.cos(2*math.pi*self.mode_time/5.))
                com[2] -= .09*a
                com_v[2] = -.09*math.pi/5*math.sin(2*math.pi*self.mode_time/5.)
                com_a[2] = -.09*2*math.pi**2/25*math.cos(2*math.pi*self.mode_time/5.)
            if self.mode == 'dance':
                fade = smooth(self.mode_time/2)[0]
                com[1] += fade*.018*math.sin(2*math.pi*self.mode_time/4)
                com[2] -= fade*.025*(1-math.cos(2*math.pi*self.mode_time/3))
        if self.phase == 'shift':
            duration = 1.6
            f, fd, fdd = smooth(self.phase_t/duration)
            delta = self.shift_goal-self.shift_start
            com = self.shift_start + f*delta
            com_v = fd/duration*delta
            com_a = fdd/duration**2*delta
            if self.phase_t >= duration:
                if self.pending_mode:
                    self.phase = 'stand'
                    self.mode, self.pending_mode = self.pending_mode, None
                    self.initial_com = self.last_com_ref.copy()
                    self.mode_time = 0.
                else:
                    self._begin_swing()
        elif self.phase == 'swing':
            duration = 1.4
            s = min(self.phase_t/duration, 1.)
            f, fd, fdd = smooth(s)
            swing = 1-self.support
            delta = self.swing_goal-self.swing_start
            p = self.swing_start+f*delta
            v = fd/duration*delta
            a = fdd/duration**2*delta
            p[2] += .055*math.sin(math.pi*s)**2
            v[2] += .055*math.pi/duration*math.sin(2*math.pi*s)
            a[2] += .11*math.pi**2/duration**2*math.cos(2*math.pi*s)
            current_yaw = math.atan2(self.swing_rotation[1, 0], self.swing_rotation[0, 0])
            R = rz(current_yaw+f*(self.next_heading-current_yaw))
            refs[swing] = FootReference(p, R, v, a, False)
            com = self.shift_goal.copy()
            if s >= 1.:
                self.feet[swing] = self.swing_goal.copy()
                self.foot_rotations[swing] = rz(self.next_heading)
                self.heading = self.next_heading
                self.phase = 'land'
                self.phase_t = 0.
                refs[swing].contact = True
                self.step_number += 1
        elif self.phase == 'land':
            com = self.shift_goal.copy()
            if self.phase_t > .3:
                self.support = 1-self.support
                if self.pending_mode:
                    self.mode, self.pending_mode = self.pending_mode, None
                    self.phase = 'stand'
                    self.mode_time = 0.
                    self.initial_com = self.last_com_ref.copy()
                else:
                    self._begin_shift()
        self.last_com_ref = com.copy()
        target = self.home.copy()
        fade = smooth(self.mode_time/1.8)[0]
        if self.mode in ('wave', 'dance'):
            def put(name, angle):
                joint = self.model.joint(name)
                target[joint.qposadr[0]] = angle
            put('r_arm_shx', -fade*1.35)
            put('r_arm_ely', .12+fade*.5)
            put('r_arm_elx', -.25-fade*(1.75+.15*math.sin(self.mode_time*2.4)))
            put('r_arm_wrx', fade*.28*math.sin(self.mode_time*3.2))
            if self.mode == 'dance':
                put('l_arm_shx', -fade*(.75+.15*math.sin(self.mode_time*2)))
                put('l_arm_elx', .25+fade*1.2)
                put('back_bkz', fade*.12*math.sin(self.mode_time*1.6))
        return refs, com, com_v, com_a, target

    def control(self):
        m, d = self.model, self.data
        refs, com_ref, com_v, com_a, posture = self.references(self.control_dt)
        jac_com = np.zeros((3, self.nv))
        mujoco.mj_jacSubtreeCom(m, d, jac_com, self.pelvis)
        com_bias = np.zeros(3) if self.previous_jcom is None else ((jac_com-self.previous_jcom)/self.control_dt)@d.qvel
        self.previous_jcom = jac_com.copy()
        com_acc = com_a + 65*(com_ref-d.subtree_com[self.pelvis]) + 17*(com_v-jac_com@d.qvel) - com_bias
        jp, jr = np.zeros((3, self.nv)), np.zeros((3, self.nv))
        mujoco.mj_jacBody(m, d, jp, jr, self.pelvis)
        R = d.xmat[self.pelvis].reshape(3, 3)
        foot_yaws = [math.atan2(ref.R[1, 0], ref.R[0, 0]) for ref in refs]
        torso_yaw = math.atan2(sum(math.sin(y) for y in foot_yaws), sum(math.cos(y) for y in foot_yaws))
        ori_acc = 100*rotation_error(R, rz(torso_yaw)) - 22*jr@d.qvel
        matrices, targets = [], []
        jac_feet, contact_jacs, contact_positions = [], [], []
        for i, (sid, ref) in enumerate(zip(self.site_ids, refs)):
            jp, jr = np.zeros((3, self.nv)), np.zeros((3, self.nv))
            mujoco.mj_jacSite(m, d, jp, jr, sid)
            J = np.vstack((jp, jr))
            jac_feet.append(J)
            bias = np.zeros(6) if self.previous_jfeet is None else ((J-self.previous_jfeet[i])/self.control_dt)@d.qvel
            error = np.r_[ref.p-d.site_xpos[sid], rotation_error(d.site_xmat[sid].reshape(3, 3), ref.R)]
            foot_acc = np.r_[ref.a, np.zeros(3)] + 260*error + 32*(np.r_[ref.v, np.zeros(3)]-J@d.qvel) - bias
            weight = 18. if ref.contact else 7.
            matrices.append(weight*J)
            targets.append(weight*np.clip(foot_acc, -90, 90))
            if ref.contact:
                contact_jacs.append(J)
                contact_positions.append(d.site_xpos[sid].copy())
        self.previous_jfeet = [J.copy() for J in jac_feet]
        jp, pelvis_jr = np.zeros((3, self.nv)), np.zeros((3, self.nv))
        mujoco.mj_jacBody(m, d, jp, pelvis_jr, self.pelvis)
        matrices += [5*jac_com, 3*pelvis_jr]
        targets += [5*np.clip(com_acc, -20, 20), 3*np.clip(ori_acc, -35, 35)]
        selector = np.zeros((m.nu, self.nv))
        selector[np.arange(m.nu), self.dofs] = 1
        qdd_posture = 65*(posture[self.qaddrs]-d.qpos[self.qaddrs])-14*d.qvel[self.dofs]
        weights = np.array([.35 if '_arm_' in m.joint(j).name else .12 for j in self.motor_joints])
        matrices.append(selector*weights[:, None])
        targets.append(qdd_posture*weights)
        matrices.append(.025*np.eye(self.nv))
        targets.append(np.zeros(self.nv))
        qdd = np.linalg.lstsq(np.vstack(matrices), np.concatenate(targets), rcond=1e-5)[0]
        qdd = np.clip(qdd, -120, 120)
        M = np.zeros((self.nv, self.nv))
        mujoco.mj_fullM(m, d, M)
        required = M@qdd + d.qfrc_bias - d.qfrc_passive
        Jc = np.vstack(contact_jacs)
        # Allocate vertical load from the desired ground center of pressure.
        # Independent clipping of an unconstrained min-norm wrench would break
        # the unactuated base equations during a double-to-single support shift.
        total_force = required[:3].copy()
        root_moment = R @ required[3:6]
        origin_moment = root_moment + np.cross(d.xpos[self.pelvis], total_force)
        fz = max(float(total_force[2]), 1.)
        cop = np.array([-origin_moment[1]/fz, origin_moment[0]/fz])
        positions = np.array(contact_positions)
        if len(positions) == 2:
            delta = positions[0, :2]-positions[1, :2]
            left = float(np.clip(np.dot(cop-positions[1, :2], delta)/(np.dot(delta, delta)+1e-12), 0., 1.))
            loads = np.array([left, 1.-left])
        else:
            loads = np.ones(1)
        forces = loads[:, None]*total_force
        moment_from_forces = sum((np.cross(p-d.xpos[self.pelvis], f) for p, f in zip(positions, forces)), np.zeros(3))
        remaining_moment = root_moment-moment_from_forces
        wrench = np.concatenate([np.r_[force, load*remaining_moment] for force, load in zip(forces, loads)])
        for i in range(len(contact_jacs)):
            f = wrench[6*i:6*i+6]
            f[2] = np.clip(f[2], 0., self.mass*9.81*1.7)
            f[:2] = np.clip(f[:2], -.7*f[2], .7*f[2])
            f[3] = np.clip(f[3], -.065*f[2], .065*f[2])
            f[4] = np.clip(f[4], -.13*f[2], .13*f[2])
            f[5] = np.clip(f[5], -.015*f[2], .015*f[2])
        tau = (required-Jc.T@wrench)[self.dofs]
        limits = m.actuator_ctrlrange
        tau = np.clip(tau, limits[:, 0], limits[:, 1])
        d.ctrl[:] = tau
        self.last_tau = tau.copy()
        self.last_wrench = wrench.copy()
        self.max_torque_ratio = max(self.max_torque_ratio, float(np.max(np.abs(tau)/limits[:, 1])))

    def step(self):
        if self.paused or self.fallen:
            return
        old_xy = self.data.qpos[:2].copy()
        if self.tick % self.control_steps == 0:
            self.control()
        self.data.xfrc_applied[:] = 0
        if self.data.time < self.push_until:
            self.data.xfrc_applied[self.pelvis, :3] = self.push_force
        mujoco.mj_step(self.model, self.data)
        self.tick += 1
        self.distance += float(np.linalg.norm(self.data.qpos[:2]-old_xy))
        tilt = math.acos(float(np.clip(self.data.xmat[self.pelvis].reshape(3, 3)[2, 2], -1, 1)))
        self.max_tilt = max(self.max_tilt, tilt)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            self.fallen = True
            self.data.ctrl[:] = 0
            raise RuntimeError('Non-finite physics state; simulation stopped.')
        if self.data.xpos[self.pelvis, 2] < .5 or tilt > math.radians(58):
            self.fallen = True
            self.data.ctrl[:] = 0

    def contact_forces(self):
        normal = [0., 0.]
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            ids = [int(contact.geom1), int(contact.geom2)]
            foot = None
            for g in ids:
                name = self.model.geom(g).name or ''
                if 'left_foot' in name:
                    foot = 0
                if 'right_foot' in name:
                    foot = 1
            if foot is not None:
                wrench = np.zeros(6)
                mujoco.mj_contactForce(self.model, self.data, i, wrench)
                normal[foot] += max(float(wrench[0]), 0.)
        return normal

    def status(self):
        return {'mujoco_version': mujoco.__version__, 'time': round(float(self.data.time), 3),
                'mode': self.mode, 'phase': self.phase, 'paused': self.paused, 'fallen': self.fallen,
                'pelvis_xyz': self.data.xpos[self.pelvis].tolist(),
                'com_xyz': self.data.subtree_com[self.pelvis].tolist(),
                'max_tilt_deg': math.degrees(self.max_tilt), 'step_count': self.step_number,
                'distance_m': self.distance, 'foot_normal_N': self.contact_forces(),
                'max_torque_ratio': self.max_torque_ratio, 'actuators': int(self.model.nu),
                'mass_kg': self.mass, 'external_support': False,
                'forward_command': float(self.command[0]), 'yaw_command': float(self.command[1]),
                'pelvis_yaw_rad': math.atan2(self.data.xmat[self.pelvis].reshape(3, 3)[1, 0], self.data.xmat[self.pelvis].reshape(3, 3)[0, 0])}
