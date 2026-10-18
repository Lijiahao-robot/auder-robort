from __future__ import annotations
import math
import mujoco
import numpy as np

def vec(values):
    return " ".join(f"{float(v):.9g}" for v in values)


def smooth(s):
    s = float(np.clip(s, 0, 1))
    return s**3*(10-15*s+6*s*s)


def integral(s):
    s = float(np.clip(s, 0, 1))
    return 2.5*s**4-3*s**5+s**6


def envelope(t, duration, ramp):
    return smooth(t/ramp)*smooth((duration-t)/ramp)


def rotation(x=0., y=0., z=0.):
    sx, cx, sy, cy, sz, cz = math.sin(x), math.cos(x), math.sin(y), math.cos(y), math.sin(z), math.cos(z)
    return np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1.]]) @ np.array(
        [[cy, 0, sy], [0, 1., 0], [-sy, 0, cy]]) @ np.array([[1., 0, 0], [0, cx, -sx], [0, sx, cx]])


def rotation_error(current, desired):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, (desired@current.T).ravel())
    if q[0] < 0:
        q = -q
    n = np.linalg.norm(q[1:])
    return q[1:]*(2*math.atan2(n, q[0])/n) if n > 1e-10 else np.zeros(3)


def ident(model, kind, name):
    i = mujoco.mj_name2id(model, kind, name)
    if i < 0:
        raise ValueError(f"缺少官方模型对象：{name}")
    return i


