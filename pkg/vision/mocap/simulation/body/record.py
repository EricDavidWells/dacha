"""Play a generated scene through the running mocap stack and record occlusion.

Needs mocap_app built with --features simulation and started with the scene config:
  cargo build --release -p mocap_app --features simulation
  ./target/release/mocap_app --config=<scene>/manager_sim.txtpb --data_dir=<scene>/data

Usage:
  .venv/bin/python record.py out/ring6               # record -> out/ring6/results.npz + table
  .venv/bin/python record.py out/ring6 --goto 420    # pause simulator at frame 420 (look in the UI)
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE / "gen"))
import grpc  # noqa: E402
from pkg.vision.mocap.proto import manager_pb2, manager_pb2_grpc, simulation_pb2  # noqa: E402

SEQUENCE_TIMESTAMP_BLOCK = 100000  # simulator.rs
MATCH_DIST = 0.010                 # tracked point to true marker (m)
MIN_REMATCH_POINTS = 3             # config/manager.txtpb rigid_body_tracker


def configure(stub, scene, **kw):
    cfg = simulation_pb2.SimulationAnimationConfig(sequence_path=str((scene / "sequence.pb").resolve()), **kw)
    stub.Execute(manager_pb2.ExecuteRequest(configure_simulation=cfg), timeout=30)


def record(stub, scene, truth, unthrottled):
    fps, F = float(truth["fps"]), truth["markers"].shape[0]
    frames = {}
    configure(stub, scene, playing=True, current_frame=0, unthrottled=unthrottled)
    started = False
    for r in stub.ReadTrackedPoints(manager_pb2.ReadTrackedPointsRequest()):
        f = int(round(r.frame_timestamp * fps / 1e9)) % SEQUENCE_TIMESTAMP_BLOCK
        if not started:
            started = f == 0          # skip output from before our restart
            if not started:
                continue
        if f >= F or (frames and f < max(frames)):   # wrapped around: done
            break
        frames[f] = r                 # a paused frame 0 may repeat: keep the last
        if len(frames) % 100 == 0:
            print(f"  {len(frames)}/{F}", flush=True)
    configure(stub, scene, playing=False, current_frame=0)
    return frames


def analyze(truth, frames):
    M, F = truth["markers"], truth["markers"].shape[0]
    rig_of, rig_local = truth["marker_rig"], truth["rig_local"]
    n_rigs = rig_local.shape[0]
    received = np.zeros(F, bool)
    cams = np.zeros((F, M.shape[1]), int)       # cameras that saw each true marker (0 = lost)
    found = np.zeros((F, n_rigs), bool)
    pose_err = np.full((F, n_rigs), np.nan)     # mean marker error of the tracked pose (m)
    for f, r in frames.items():
        received[f] = True
        pts = np.array([p.position.values for p in r.points]).reshape(-1, 3)
        ncam = np.array([len(p.camera_ids) for p in r.points])
        for m in range(M.shape[1]):
            if len(pts):
                d = np.linalg.norm(pts - M[f, m], axis=1)
                i = d.argmin()
                if d[i] < MATCH_DIST:
                    cams[f, m] = ncam[i]
        for b in r.rigid_bodies:
            k = b.id - 1
            found[f, k] = b.found
            if b.found:
                R = Rotation.from_rotvec(list(b.rotation.values)).as_matrix()
                pred = rig_local[k] @ R.T + np.array(b.translation.values)
                pose_err[f, k] = np.linalg.norm(pred - M[f, rig_of == k], axis=1).mean()
    observed = np.stack([(cams[:, rig_of == k] > 0).sum(1) for k in range(n_rigs)], 1)
    lost = received[:, None] & (~found | (observed < MIN_REMATCH_POINTS))
    return dict(received=received, cams=cams, found=found, observed=observed, lost=lost, pose_err=pose_err)


def events(lost_col, received):
    """[(start, end_exclusive)] runs of lost frames."""
    out, start = [], None
    for f, x in enumerate(lost_col):
        if x and start is None:
            start = f
        if not x and start is not None:
            out.append((start, f))
            start = None
    if start is not None:
        out.append((start, len(lost_col)))
    return out


def clip_at(truth, f):
    i = np.searchsorted(truth["clip_starts"], f, side="right") - 1
    return str(truth["clip_names"][i])


def report(truth, res):
    fps = float(truth["fps"])
    F = len(res["received"])
    print(f"\nframes received {res['received'].sum()}/{F}")
    print(f"{'rig':>4} {'tracked %':>9} {'events':>6} {'pose err mm':>11}  worst events")
    for k in range(res["lost"].shape[1]):
        ev = sorted(events(res["lost"][:, k], res["received"]), key=lambda e: e[0] - e[1])
        pct = 100 * (1 - res["lost"][res["received"], k].mean())
        err = np.nanmedian(res["pose_err"][:, k]) * 1e3 if np.isfinite(res["pose_err"][:, k]).any() else float("nan")
        worst = ", ".join(f"{clip_at(truth, s)} f{s}-{e} ({(e - s) / fps:.2f}s)" for s, e in ev[:3])
        print(f"{k + 1:>4} {pct:9.1f} {len(ev):6d} {err:11.2f}  {worst}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scene", type=Path)
    ap.add_argument("--addr", default="localhost:50051")
    ap.add_argument("--goto", type=int, help="pause the simulator at this frame and exit")
    ap.add_argument("--unthrottled", action="store_true", help="render as fast as possible (check for dropped frames)")
    a = ap.parse_args()

    stub = manager_pb2_grpc.ManagerStub(grpc.insecure_channel(a.addr))
    if a.goto is not None:
        configure(stub, a.scene, playing=False, current_frame=a.goto)
        return

    truth = np.load(a.scene / "truth.npz")
    frames = record(stub, a.scene, truth, a.unthrottled)
    res = analyze(truth, frames)
    np.savez_compressed(a.scene / "results.npz", **res)
    report(truth, res)


if __name__ == "__main__":
    main()
