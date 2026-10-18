from __future__ import annotations
import math
import time

import mujoco
import numpy as np

from .mathutils import ident, rotation, rotation_error, smooth
from .scene import scene_xml

LEG_PARTS = ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll')
ARM_PARTS = ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw')
ALLOWED_KEYS = {'forward', 'back', 'left', 'right', 'turn_left', 'turn_right', 'boost'}


class SoccerTask:
    """Torque-controlled limbs, assisted floating pelvis, contact-driven free ball."""
    def __init__(self):
        self.model = m = mujoco.MjModel.from_xml_string(scene_xml())
        self.data, self.ik = mujoco.MjData(m), mujoco.MjData(m)
        self.jids = m.actuator_trnid[:, 0].copy()
        self.qadr, self.dadr = m.jnt_qposadr[self.jids], m.jnt_dofadr[self.jids]
        self.ref, self.ref_vel = np.zeros(m.nu), np.zeros(m.nu)
        self.kp, self.kd = np.full(m.nu, 100.), np.full(m.nu, 7.)
        self.legs, self.arms, self.soles, self.pads = {}, {}, {}, {}
        self.heading = 0.0
        for side, sign in (('left', 1), ('right', -1)):
            ids = np.array([self.index(side + '_' + part) for part in LEG_PARTS])
            self.legs[side] = ids
            self.ref[ids] = [-.20, 0., 0., .40, -.20, 0.]
            self.kp[ids], self.kd[ids] = [360, 300, 180, 320, 180, 150], [24, 20, 14, 22, 12, 10]
            ids = np.array([self.index(side + '_' + part) for part in ARM_PARTS])
            self.arms[side] = ids
            self.ref[ids] = [.05, sign * .18, 0., .50, 0., 0., 0.]
            self.kp[ids], self.kd[ids] = [100, 100, 80, 80, 35, 35, 35], [7, 7, 6, 6, 3, 3, 3]
            self.soles[side] = ident(m, mujoco.mjtObj.mjOBJ_SITE, side + '_sole')
            self.pads[side] = ident(m, mujoco.mjtObj.mjOBJ_GEOM, side + '_sole_pad')
        self.home = self.ref.copy()
        self.upper = np.setdiff1d(np.arange(m.nu), np.concatenate(list(self.legs.values())))
        self.base_joint = ident(m, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base_joint')
        self.base_qa = int(m.jnt_qposadr[self.base_joint])
        self.pelvis = ident(m, mujoco.mjtObj.mjOBJ_BODY, 'pelvis')
        self.mocap = int(m.body_mocapid[ident(m, mujoco.mjtObj.mjOBJ_BODY, 'virtual_base')])
        self.ball_body = ident(m, mujoco.mjtObj.mjOBJ_BODY, 'football')
        self.ball_geom = ident(m, mujoco.mjtObj.mjOBJ_GEOM, 'football_geom')
        ball_joint = ident(m, mujoco.mjtObj.mjOBJ_JOINT, 'football_free')
        self.ball_qa, self.ball_da = int(m.jnt_qposadr[ball_joint]), int(m.jnt_dofadr[ball_joint])
        self.data.qpos[self.qadr] = self.ref
        mujoco.mj_forward(m, self.data)
        foot_z = np.mean([self.data.site_xpos[s][2] for s in self.soles.values()])
        self.stand_height = float(self.data.xpos[self.pelvis, 2] - foot_z - .001)
        self.data.qpos[self.base_qa:self.base_qa + 3] = [0., 0., self.stand_height]
        self.data.mocap_pos[self.mocap] = [0., 0., self.stand_height]
        mujoco.mj_forward(m, self.data)
        self.foot_home = {side: self.data.site_xpos[site].copy() for side, site in self.soles.items()}
        for p in self.foot_home.values():
            p[2] = -.001
        self.targets = {s: p.copy() for s, p in self.foot_home.items()}
        for _ in range(10):
            self.solve_legs(False)
        self.home = self.ref.copy()
        self.reset()

    def index(self, name):
        return ident(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def reset(self):
        mujoco.mj_resetData(self.model, self.data)
        self.ref, self.ref_vel = self.home.copy(), np.zeros(self.model.nu)
        self.data.qpos[self.qadr] = self.ref
        self.base_xy, self.heading = np.array([-1.6, 0.0]), 0.0
        self.data.qpos[self.base_qa:self.base_qa + 7] = [-1.6, 0, self.stand_height, 1, 0, 0, 0]
        self.data.mocap_pos[self.mocap] = [-1.6, 0, self.stand_height]
        self.data.mocap_quat[self.mocap] = [1, 0, 0, 0]
        self.keys = set()
        self.command_at = -math.inf
        self.velocity = np.zeros(3)
        self.phase = 0.0
        self.amplitude = 0.0
        self.kick_started = None
        self.kick_side = 'right'
        self.kick_contacts = 0
        self.contact_episodes = 0
        self._last_contact = False
        self.shots, self.goals = 0, 0
        self.goal_latched = False
        self.last_event = '准备就绪，WASD 走位，空格踢球'
        self.last_control = -1.0
        self.max_ball_speed = 0.0
        self.max_joint_violation = 0.0
        self.max_foot_error = 0.0
        self.max_root_error = 0.0
        self.travel = 0.0
        self.error = None
        self.paused = False
        self.control_timed_out = True
        self.reset_ball(in_front=False)
        self._previous_ball = self.ball_position.copy()
        self.update_control()

    @property
    def ball_position(self):
        return self.data.xpos[self.ball_body].copy()

    @property
    def ball_speed(self):
        return float(np.linalg.norm(self.data.qvel[self.ball_da:self.ball_da + 3]))

    def local_ball(self):
        return rotation(z=self.heading).T @ (self.ball_position - np.r_[self.base_xy, 0.])

    def reset_ball(self, in_front=True):
        # Ball state is written only by this explicit reset, never by a kick.
        xy = self.base_xy + (rotation(z=self.heading) @ [.37, -.12, 0])[:2] if in_front else np.array([-.65, -.12])
        self.data.qpos[self.ball_qa:self.ball_qa + 7] = [*xy, .105, 1, 0, 0, 0]
        self.data.qvel[self.ball_da:self.ball_da + 6] = 0
        self.goal_latched = False
        mujoco.mj_forward(self.model, self.data)
        self._previous_ball = self.ball_position.copy()

    def command(self, keys, now=None):
        self.keys = set(keys) & ALLOWED_KEYS
        self.command_at = time.monotonic() if now is None else now

    def stop(self):
        self.keys.clear()
        self.velocity[:] = 0
        self.command_at = -math.inf

    def kick(self):
        if self.paused or self.kick_started is not None:
            return False
        self.stop()
        self.kick_started = float(self.data.time)
        self.kick_side = 'left' if self.local_ball()[1] > 0 else 'right'
        self.shots += 1
        self.last_event = '左脚射门' if self.kick_side == 'left' else '右脚射门'
        return True

    def solve_legs(self, rate_limit=True):
        m, d = self.model, self.ik
        d.qpos[:] = self.data.qpos
        d.qpos[self.qadr] = self.ref
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        orientation = rotation(z=self.heading)
        for side, target in self.targets.items():
            ids = self.legs[side]
            qa, da = self.qadr[ids], self.dadr[ids]
            bounds = m.jnt_range[self.jids[ids]]
            for _ in range(14):
                mujoco.mj_kinematics(m, d)
                mujoco.mj_comPos(m, d)
                site = self.soles[side]
                ep = target - d.site_xpos[site]
                er = rotation_error(d.site_xmat[site].reshape(3, 3), orientation)
                if np.linalg.norm(ep) < .0005 and np.linalg.norm(er) < .004:
                    break
                mujoco.mj_jacSite(m, d, jp, jr, site)
                jac = np.vstack([jp[:, da], .16 * jr[:, da]])
                dq = jac.T @ np.linalg.solve(jac @ jac.T + .012**2 * np.eye(6), np.r_[ep, .16 * er])
                d.qpos[qa] = np.clip(d.qpos[qa] + np.clip(dq, -.10, .10), bounds[:, 0] + .015, bounds[:, 1] - .015)
            delta = d.qpos[qa] - self.ref[ids]
            self.ref[ids] += np.clip(delta, -.10, .10) if rate_limit else delta

    def update_control(self):
        if self.paused:
            return
        step = .01
        keys = self.keys
        speed = .36 if 'boost' in keys else .24
        desired = np.array([('forward' in keys) - ('back' in keys), ('left' in keys) - ('right' in keys),
                            .6 * (('turn_left' in keys) - ('turn_right' in keys))], dtype=float)
        length = np.linalg.norm(desired[:2])
        desired[:2] *= speed / max(1.0, length)
        kicking = self.kick_started is not None
        if kicking:
            desired[:] = 0
        self.velocity += np.clip(desired - self.velocity, -np.array([.55, .55, 1.2]) * step, np.array([.55, .55, 1.2]) * step)
        before_xy = self.base_xy.copy()
        self.heading += self.velocity[2] * step
        world_v = rotation(z=self.heading) @ np.r_[self.velocity[:2], 0]
        self.base_xy = np.clip(self.base_xy + world_v[:2] * step, [-3.55, -1.95], [3.45, 1.95])
        self.travel += float(np.linalg.norm(self.base_xy - before_xy))
        activity = min(1., np.linalg.norm(self.velocity[:2]) / .18 + abs(self.velocity[2]) / .45)
        self.amplitude += np.clip(activity - self.amplitude, -.035, .035)
        if self.amplitude > .005:
            self.phase += step * activity / 1.10
        if self.amplitude < .005:
            self.phase = 0.0
        amplitude = 0.0 if kicking else self.amplitude
        kick_envelope = 0.0
        if kicking:
            kick_time = float(self.data.time) - self.kick_started
            kick_envelope = smooth(kick_time / .25) * smooth((1.30 - kick_time) / .30)
        root = np.r_[self.base_xy, self.stand_height - .035 * max(amplitude, kick_envelope)]
        orient = rotation(z=self.heading)
        self.data.mocap_pos[self.mocap] = root
        mujoco.mju_mat2Quat(self.data.mocap_quat[self.mocap], orient.ravel())
        prior = self.ref.copy()
        self.ref[self.upper] = self.home[self.upper]
        stride = min(.28, np.linalg.norm(self.velocity[:2]) * 1.1)
        direction = self.velocity[:2] / max(np.linalg.norm(self.velocity[:2]), 1e-8)
        for side, sign, shift in (('left', 1, .31), ('right', -1, .81)):
            local = self.foot_home[side].copy()
            phase = (self.phase + shift) % 1.
            if phase >= .62:
                t = (phase - .62) / .38
                offset = stride * (-.31 + smooth(t) - (phase - .62))
                height = .055 * amplitude * math.sin(math.pi * t)**2
            else:
                offset, height = stride * (.31 - phase), 0.
            local[:2] += amplitude * offset * direction
            local[2] += height
            if kicking and side == self.kick_side:
                t = float(self.data.time) - self.kick_started
                # Prescribed foot path: preload, forward strike, recovery.
                if t < .45:
                    blend = smooth(t / .45)
                    local += np.array([-.13, 0, .07]) * blend
                elif t < .67:
                    blend = smooth((t - .45) / .22)
                    local += [-.13 + .48 * blend, 0, .07 + .015 * math.sin(math.pi * blend)]
                elif t < .95:
                    blend = smooth((t - .67) / .28)
                    local += [.35 * (1 - blend), 0, .07 + .035 * math.sin(math.pi * blend)]
                elif t < 1.3:
                    local += [0, 0, .07 * (1 - smooth((t - .95) / .35))]
                else:
                    self.kick_started = None
                    self.last_event = '射门动作完成'
            self.targets[side] = orient @ local + np.r_[self.base_xy, 0]
            self.ref[self.index(side + '_shoulder_pitch')] = .05 + .23 * amplitude * sign * math.sin(math.tau * self.phase)
        self.solve_legs()
        bounds = self.model.jnt_range[self.jids]
        self.ref = np.clip(self.ref, bounds[:, 0] + .015, bounds[:, 1] - .015)
        self.ref_vel = np.clip((self.ref - prior) / step, -10, 10)

    def step(self, now=None):
        if self.paused or self.error:
            return
        now = time.monotonic() if now is None else now
        self.control_timed_out = now - self.command_at > .45
        if self.control_timed_out:
            self.keys.clear()
        m, d = self.model, self.data
        if d.time - self.last_control >= .01 - 1e-9:
            self.update_control()
            self.last_control = float(d.time)
        torque = self.kp * (self.ref - d.qpos[self.qadr]) + self.kd * (self.ref_vel - d.qvel[self.dadr]) + d.qfrc_bias[self.dadr]
        d.ctrl[:] = np.clip(torque, m.actuator_ctrlrange[:, 0], m.actuator_ctrlrange[:, 1])
        mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        if not np.all(np.isfinite(d.qpos)) or not np.all(np.isfinite(d.qvel)):
            self.error = '仿真出现非有限状态，请复位'
            return
        contact = any(self.ball_geom in (int(c.geom1), int(c.geom2)) and
                      any(p in (int(c.geom1), int(c.geom2)) for p in self.pads.values()) for c in d.contact[:d.ncon])
        if contact:
            self.kick_contacts += 1
            if not self._last_contact:
                self.contact_episodes += 1
        self._last_contact = contact
        current = self.ball_position
        self.check_goal(self._previous_ball, current)
        self._previous_ball = current
        self.max_ball_speed = max(self.max_ball_speed, self.ball_speed)
        q, limits = d.qpos[self.qadr], m.jnt_range[self.jids]
        self.max_joint_violation = max(self.max_joint_violation, float(np.max(np.maximum(limits[:, 0] - q, q - limits[:, 1]))))
        self.max_root_error = max(self.max_root_error, float(np.linalg.norm(d.xpos[self.pelvis] - d.mocap_pos[self.mocap])))
        if self.kick_started is not None:
            self.max_foot_error = max(self.max_foot_error, max(float(np.linalg.norm(d.site_xpos[self.soles[s]] - p)) for s, p in self.targets.items()))

    def check_goal(self, previous, current):
        if self.goal_latched:
            return
        radius = .105
        for sign in (1, -1):
            plane = sign * (4.0 + radius)
            if sign * previous[0] <= sign * plane < sign * current[0]:
                t = (plane - previous[0]) / (current[0] - previous[0])
                crossing = previous + t * (current - previous)
                if abs(crossing[1]) < .9 - .035 - radius and radius - .015 <= crossing[2] < .9 - .035 - radius:
                    self.goals += 1
                    self.goal_latched = True
                    self.last_event = '进球！按 B 把球放回脚前'

    def status(self):
        local = self.local_ball()
        ready = .15 < local[0] < .52 and abs(local[1]) < .27 and self.ball_speed < .8
        phase = '踢球中' if self.kick_started is not None else ('行走中' if np.linalg.norm(self.velocity) > .02 else '站立')
        return {'time': round(float(self.data.time), 2), 'robot': [*self.base_xy.tolist(), self.heading],
                'ball': self.ball_position.tolist(), 'ball_speed': round(self.ball_speed, 2),
                'speed': round(float(np.linalg.norm(self.velocity[:2])), 2), 'goals': self.goals,
                'shots': self.shots, 'phase': phase, 'paused': self.paused, 'ready_to_kick': bool(ready),
                'contact_count': self.contact_episodes, 'event': self.last_event, 'error': self.error,
                'mujoco_version': mujoco.__version__, 'assisted_base': True, 'command_timeout': self.control_timed_out}

    def report(self):
        return {'mujoco_version': mujoco.__version__, 'simulation_seconds': float(self.data.time),
                'goals': self.goals, 'shots': self.shots, 'foot_ball_contact_steps': self.kick_contacts,
                'contact_episodes': self.contact_episodes, 'max_ball_speed_m_s': self.max_ball_speed,
                'ball_position_m': self.ball_position.tolist(), 'robot_travel_m': self.travel,
                'max_joint_limit_violation_rad': self.max_joint_violation,
                'max_root_tracking_error_m': self.max_root_error, 'max_kick_foot_tracking_error_m': self.max_foot_error,
                'error': self.error, 'assisted_pelvis': True, 'physical_ball_contact': True,
                'limitations': ['Mocap/weld assisted pelvis; no autonomous balance.',
                                'Prescribed foot trajectories, IK, torque-limited PD.',
                                'Only sole pads collide on the robot; no full self-collision model.',
                                'No SDK/DDS/ROS or hardware commands.']}
