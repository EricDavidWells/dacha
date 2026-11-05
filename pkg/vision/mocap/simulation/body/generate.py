"""Make a simulation scene: MHR body + wrist marker plates + cameras.

Writes to --out:
  sequence.pb         SimulationSequenceProto, played by the Rust simulator ('sequence_path' scene)
  manager_sim.txtpb   manager config: dummy cameras, per_camera, rigid body definitions
  truth.npz           ground truth for record.py / make_viewer.py

Example:
  .venv/bin/python generate.py --motions all --cameras ring:6:2.1:3.0 --out out/ring6
"""
import argparse
import re
import sys
from pathlib import Path

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

HERE = Path(__file__).parent
MOCAP = HERE.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "gen"))
sys.path.insert(0, str(MOCAP / "parts" / "wrist_mount"))
import wrist_mount as wm  # noqa: E402
from body import FPS, MOTIONS, Body  # noqa: E402
from pkg.vision.mocap.proto import simulation_pb2  # noqa: E402

MARKER_D = 0.014          # marker sphere diameter (m), doc/markers.md "human body"
STRAP_FROM_WRIST = 0.06   # plate center distance from the wrist joint toward the elbow (m)
PIXEL_PITCH = 3.0e-6      # AR0234
WIDTH, HEIGHT = 1920, 1200
GAP_S = 0.5               # blend time between motions

# (col, row, standoff mm) on the plate grid. Different patterns so the tracker can tell them apart.
LAYOUTS = {
    "l": [(0, 0, 10), (6, 1, 20), (2, 3, 15), (4, 0, 5), (5, 3, 25)],
    "r": [(0, 3, 15), (6, 0, 10), (1, 0, 25), (4, 3, 20), (2, 1, 5)],
}


def unit(v):
    return v / np.linalg.norm(v)


def hole_marker(col, row, standoff_mm):
    """Marker center in plate coords (mm, base bottom = z 0)."""
    a = (col - (wm.N_COLS - 1) / 2) * wm.COL_PITCH / wm.R_OUT
    y = (row - (wm.N_ROWS - 1) / 2) * wm.ROW_PITCH
    assert any(abs(a - ha) < 1e-9 and abs(y - hy) < 1e-9 for ha, hy in wm.holes()), f"no hole at {col},{row}"
    x, z = wm.arc_pt(wm.R_OUT + standoff_mm + MARKER_D * 1e3 / 2, a)
    return np.array([x, y, z + wm.BASE_T + wm.RISE])


def mount_hull():
    """Convex hull of base + seated plate, base coords (mm)."""
    pts = []
    for solid, dz in ((wm.build_base().val(), 0), (wm.build_plate().val(), wm.BASE_T + wm.RISE)):
        v, _ = solid.tessellate(2.0, 0.5)
        pts += [(p.x, p.y, p.z + dz) for p in v]
    return trimesh.Trimesh(np.array(pts)).convex_hull


def wrist_rig(body, rest_verts, rest_joints, side):
    """Rest-pose world transform of the plate frame (base bottom center) and its parent bone."""
    J = body.joint_names
    wrist = rest_joints[J.index(f"{side}_wrist"), :3, 3]
    elbow = rest_joints[J.index(f"{side}_lowarm"), :3, 3]
    axis = unit(elbow - wrist)
    c = wrist + STRAP_FROM_WRIST * axis
    lateral = np.array([1.0 if side == "l" else -1.0, 0, 0])
    n = unit(lateral - axis * (lateral @ axis))           # back of the forearm in A-pose
    d = rest_verts - c
    slab = np.abs(d @ axis) < 0.01
    skin = (d[slab] @ n).max()
    T = np.eye(4)
    T[:3, 0] = np.cross(axis, n)    # plate X (along strap, around the limb)
    T[:3, 1] = axis                 # plate Y (across strap, along forearm)
    T[:3, 2] = n                    # plate Z (away from skin)
    T[:3, 3] = c + n * (skin + wm.WEB_T / 1e3)
    return T, J.index(f"{side}_wrist_twist")


def ring_cameras(n, height, radius, aim_z, lens_mm):
    f = lens_mm * 1e-3 / PIXEL_PITCH
    cams = []
    for i in range(n):
        th = 2 * np.pi * i / n
        C = np.array([radius * np.cos(th), radius * np.sin(th), height])
        z = unit(np.array([0, 0, aim_z]) - C)
        x = unit(np.cross(z, [0, 0, 1]))
        y = np.cross(z, x)                  # camera +y is down
        R = np.stack([x, y, z])             # world -> camera
        cams.append(dict(id=i + 1, f=f, rotation=Rotation.from_matrix(R).as_rotvec(), translation=-R @ C))
    return cams


def manager_config(cams, bodies):
    base = (MOCAP / "config" / "manager.txtpb").read_text()
    base = re.sub(r'^camera_service:.*$', "", base, flags=re.M)
    base = re.sub(r'^# make_dummy_cameras: true', "make_dummy_cameras: true", base, flags=re.M)
    base = re.sub(r'(initial_camera_config \{\s*\n\s*)frame_rate: 0', r'\1frame_rate: 30', base)
    fmt = lambda v: ", ".join(f"{x:.9g}" for x in v)
    body_txt = "".join(
        "    bodies {\n        id: %d\n%s    }\n" % (bid, "".join(f"        points {{ values: [{fmt(p)}] }}\n" for p in pts))
        for bid, pts in bodies)
    base = base.replace("rigid_body_tracker {\n", "rigid_body_tracker {\n" + body_txt, 1)
    base = base.replace("aux_rpc_server {\n", "aux_rpc_server {\n    autostart: true\n", 1)
    cam_txt = "".join(f"""
per_camera {{
    camera_id: {c['id']}
    enabled: true
    intrinsics {{ focal_length: [{c['f']:.6g}, {c['f']:.6g}] center: [{WIDTH / 2}, {HEIGHT / 2}] k1: 0 k2: 0 }}
    extrinsics {{ rotation: [{fmt(c['rotation'])}] translation: [{fmt(c['translation'])}] }}
}}
""" for c in cams)
    return base + cam_txt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motions", default="all")
    ap.add_argument("--cameras", default="ring:6:2.1:3.0", help="ring:N:height_m:radius_m[:aim_z_m]")
    ap.add_argument("--lens", type=float, default=4.35, help="focal length mm")
    ap.add_argument("--rigs", default="l,r", help="wrist plates: l, r")
    ap.add_argument("--out", default="out/default")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    motions = list(MOTIONS) if a.motions == "all" else a.motions.split(",")
    body = Body()

    # one keyframe track over all motions, GAP_S blend between them
    keys, clip_starts, t0 = [], [], 0.0
    for m in motions:
        clip_starts.append((m, int(round(t0 * FPS))))
        keys += [(t0 + t, d) for t, d in MOTIONS[m]]
        t0 = keys[-1][0] + GAP_S
    MOTIONS["_all"] = keys
    params = body.params("_all")
    rest_v, rest_j = body.evaluate(np.zeros((1, 204), np.float32))
    verts, joints = body.evaluate(params)
    F = len(params)

    hull = mount_hull()
    hull_v = np.c_[hull.vertices / 1e3, np.ones(len(hull.vertices))]
    all_faces = [body.faces]
    n_body = verts.shape[1]
    occ_v = [verts]
    markers, rig_poses, bodies = [], [], []
    for bid, side in enumerate(a.rigs.split(","), start=1):
        T_rest, bone = wrist_rig(body, rest_v[0], rest_j[0], side)
        # plate pose per frame: bone motion applied to the rest plate frame
        T = joints[:, bone] @ np.linalg.inv(rest_j[0, bone]) @ T_rest
        rig_poses.append(T)
        local = np.array([hole_marker(*h) for h in LAYOUTS[side]]) / 1e3
        bodies.append((bid, local))
        markers.append(np.einsum("fij,mj->fmi", T[:, :3, :3], local) + T[:, None, :3, 3])
        all_faces.append(hull.faces + n_body + sum(h.shape[1] for h in occ_v[1:]))
        occ_v.append(np.einsum("fij,vj->fvi", T, hull_v)[:, :, :3])
    occ_v = np.concatenate(occ_v, axis=1).astype(np.float32)
    markers = np.concatenate(markers, axis=1).astype(np.float32)
    faces = np.concatenate([f.astype(np.uint32) for f in all_faces])

    seq = simulation_pb2.SimulationSequenceProto(fps=FPS, marker_radius=MARKER_D / 2, faces=faces.tobytes())
    for f in range(F):
        seq.frames.add(vertices=occ_v[f].tobytes(), markers=markers[f].tobytes())
    (out / "sequence.pb").write_bytes(seq.SerializeToString())

    _, n, h, r, *rest = a.cameras.split(":")
    cams = ring_cameras(int(n), float(h), float(r), float(rest[0]) if rest else 1.0, a.lens)
    (out / "manager_sim.txtpb").write_text(manager_config(cams, bodies))

    np.savez_compressed(
        out / "truth.npz", faces=faces, vertices=occ_v, markers=markers, rig_poses=np.stack(rig_poses, 1),
        rig_local=np.stack([b[1] for b in bodies]), marker_rig=np.repeat(np.arange(len(bodies)), [len(b[1]) for b in bodies]),
        clip_names=np.array([c[0] for c in clip_starts]), clip_starts=np.array([c[1] for c in clip_starts]),
        cam_ids=np.array([c["id"] for c in cams]), cam_f=np.array([c["f"] for c in cams]),
        cam_rot=np.array([c["rotation"] for c in cams]), cam_t=np.array([c["translation"] for c in cams]),
        width=WIDTH, height=HEIGHT, marker_radius=MARKER_D / 2, fps=FPS)

    print(f"frames={F} ({F / FPS:.1f} s) occluder verts={occ_v.shape[1]} tris={len(faces)} "
          f"markers={markers.shape[1]} cameras={len(cams)} -> {out}/")


if __name__ == "__main__":
    main()
