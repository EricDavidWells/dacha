"""MHR body + procedural motions.

World frame (same as the mocap manager): meters, Z up. The body stands at the origin, feet on
z=0, facing -Y. MHR native frame is cm, Y up, facing +Z.
"""
from pathlib import Path

import numpy as np
import torch
from mhr.mhr import MHR
from scipy.spatial.transform import Rotation

ASSETS = Path(__file__).parent / "assets"
FPS = 30

# MHR (x, y, z) cm  ->  world (x, -z, y) m
MHR_TO_WORLD = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=np.float64) / 100.0

# Both sides take the same sign (MHR mirrors r_* internally).
T_POSE = {"l_uparm_ry": 0.8, "r_uparm_ry": 0.8}


def both(**kw):
    """{'uparm_ry': v} -> {'l_uparm_ry': v, 'r_uparm_ry': v}"""
    return {f"{s}_{k}": v for k, v in kw.items() for s in "lr"}


BEHIND = both(uparm_ry=-0.6, uparm_rz=-0.4, uparm_twist=-2.2, elbow_bend=1.2)

# name -> list of (time_s, {param: value}). Params not listed are 0 (A-pose).
# Values: radians, except root_t* (MHR units, 1 = 10 cm).
MOTIONS = {
    "t_pose": [(0, T_POSE), (1.0, T_POSE)],
    "arm_swing": [
        (0, {}),
        (0.5, {"l_uparm_rz": 0.6, "r_uparm_rz": -0.6}),
        (1.0, {"l_uparm_rz": -0.6, "r_uparm_rz": 0.6}),
        (1.5, {"l_uparm_rz": 0.6, "r_uparm_rz": -0.6}),
        (2.0, {}),
    ],
    "reach_overhead": [(0, {}), (1.0, both(uparm_rz=2.3, uparm_ry=0.5)), (1.5, both(uparm_rz=2.3, uparm_ry=0.5)), (2.5, {})],
    "reach_across": [
        (0, {}),
        (1.0, {"l_uparm_rz": 1.4, "l_uparm_ry": -0.6, "spine_twist0": -0.4}),
        (1.5, {"l_uparm_rz": 1.4, "l_uparm_ry": -0.6, "spine_twist0": -0.4}),
        (2.5, {}),
        (3.5, {"r_uparm_rz": 1.4, "r_uparm_ry": -0.6, "spine_twist0": 0.4}),
        (4.0, {"r_uparm_rz": 1.4, "r_uparm_ry": -0.6, "spine_twist0": 0.4}),
        (5.0, {}),
    ],
    "hands_behind_back": [(0, {}), (1.0, BEHIND), (2.0, BEHIND), (3.0, {})],
    "squat": [
        (0, {}),
        (1.2, {**both(upleg_rz=-1.3, knee_bend=1.8, foot_bend=0.5, uparm_rz=1.4), "spine_bend0": 0.3}),
        (1.8, {**both(upleg_rz=-1.3, knee_bend=1.8, foot_bend=0.5, uparm_rz=1.4), "spine_bend0": 0.3}),
        (3.0, {}),
    ],
    "turn_360": [(0, T_POSE), (4.0, {**T_POSE, "root_ry": 2 * np.pi})],
    "walk_in_place": [
        (0, {}),
        (0.5, {"l_upleg_rz": -0.8, "l_knee_bend": 1.2, "l_uparm_rz": -0.5, "r_uparm_rz": 0.5}),
        (1.0, {}),
        (1.5, {"r_upleg_rz": -0.8, "r_knee_bend": 1.2, "r_uparm_rz": -0.5, "l_uparm_rz": 0.5}),
        (2.0, {}),
        (2.5, {"l_upleg_rz": -0.8, "l_knee_bend": 1.2, "l_uparm_rz": -0.5, "r_uparm_rz": 0.5}),
        (3.0, {}),
    ],
}


class Body:
    def __init__(self, lod=4):
        self.model = MHR.from_files(folder=ASSETS, device=torch.device("cpu"), lod=lod)
        c = self.model.character
        self.param_names = list(c.parameter_transform.names[:204])
        self.joint_names = list(c.skeleton.joint_names)
        self.faces = np.asarray(c.mesh.faces, dtype=np.uint32)

    def params(self, motion):
        """Keyframes -> [frames, 204] model params (smoothstep between keys)."""
        keys = MOTIONS[motion]
        times = np.array([t for t, _ in keys])
        vals = np.zeros((len(keys), 204), dtype=np.float32)
        for i, (_, d) in enumerate(keys):
            for k, v in d.items():
                vals[i, self.param_names.index(k)] = v
        out = []
        for t in np.arange(0, times[-1] + 1e-9, 1.0 / FPS):
            i = min(np.searchsorted(times, t, side="right") - 1, len(keys) - 2)
            u = np.clip((t - times[i]) / (times[i + 1] - times[i]), 0, 1)
            u = u * u * (3 - 2 * u)
            out.append(vals[i] * (1 - u) + vals[i + 1] * u)
        return np.stack(out).astype(np.float32)

    def evaluate(self, params):
        """[F, 204] -> verts [F, V, 3] world m, joints [F, J, 4, 4] world (rotation + translation, m)."""
        mp = torch.from_numpy(params)
        with torch.no_grad():
            v, s = self.model(torch.zeros(1, 45), mp, None)
        v, s = v.numpy().astype(np.float64), s.numpy().astype(np.float64)
        verts = v @ MHR_TO_WORLD.T
        # feet on the floor every frame
        dz = verts[:, :, 2].min(axis=1, keepdims=True)
        verts[:, :, 2] -= dz
        F, J = s.shape[:2]
        rot = Rotation.from_quat(s[:, :, 3:7].reshape(-1, 4)).as_matrix().reshape(F, J, 3, 3)
        joints = np.tile(np.eye(4), (F, J, 1, 1))
        R = MHR_TO_WORLD * 100.0  # pure rotation part
        joints[:, :, :3, :3] = R @ rot
        joints[:, :, :3, 3] = s[:, :, :3] @ MHR_TO_WORLD.T
        joints[:, :, 2, 3] -= dz
        return verts, joints
