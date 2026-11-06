"""Minimum visible marker size vs distance, using the camera blob filter (frame_processor.rs).

A marker of diameter D at distance Z images to d = f * D / Z pixels (f = lens_mm / pixel_pitch).
We render ideal disks of diameter d at random sub-pixel offsets (pixel value = 255 * brightness *
coverage), run the same blob filter as the camera, and find the smallest d that passes.
Not modeled: lens blur / defocus, motion blur, LED falloff with distance, sensor noise.
Finding: the limit is the elliptical area check (elliptical_area_error), not min_area. Pixels just
above threshold count fully in 'area' but barely in the intensity-weighted moments, so small blobs
have area >> pi*ra*rb. Brighter (and blurrier) markers have more such edge pixels.

  .venv/bin/python marker_size.py  ->  out/marker_size.png
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PIXEL_PITCH_UM = 3.0                   # AR0234 (doc/camera_sensors.md)
LENSES_MM = [2.7, 3.6, 3.9, 4.35]      # doc/camera_lens.md, 4.35 recommended
MARKERS_MM = [9, 14, 19]               # doc/markers.md
THRESHOLD = 80                         # config/manager.txtpb pixel_threshold
F = dict(min_area=6, max_aspect=1.5, max_radius_ratio=1.5, max_area_error=0.25, skew_abs=2.0, skew_rel=0.05)
SS = 16                                # supersampling per pixel axis for coverage
TRIALS = 800


def render(d, cx, cy, brightness):
    n = int(np.ceil(d)) + 4
    s = (np.arange(n * SS) + 0.5) / SS
    X, Y = np.meshgrid(s, s)
    inside = (X - cx) ** 2 + (Y - cy) ** 2 <= (d / 2) ** 2
    cov = inside.reshape(n, SS, n, SS).mean(axis=(1, 3))
    return np.round(255 * brightness * cov)


def passes(img):
    m = img > THRESHOLD
    area = m.sum()
    if area < F["min_area"]:
        return False
    ys, xs = np.nonzero(m)
    w, h = xs.max() - xs.min() + 1, ys.max() - ys.min() + 1
    if max(w, h) / min(w, h) > F["max_aspect"]:
        return False
    wgt = img[m] - THRESHOLD
    mx, my = (wgt * xs).sum() / wgt.sum(), (wgt * ys).sum() / wgt.sum()
    vx, vy = (wgt * (xs - mx) ** 2).sum() / wgt.sum(), (wgt * (ys - my) ** 2).sum() / wgt.sum()
    cxy = (wgt * (xs - mx) * (ys - my)).sum() / wgt.sum()
    mean, diff = (vx + vy) / 2, (vx - vy) / 2
    r = np.hypot(diff, cxy)
    ra, rb = 2 * np.sqrt(mean + r), 2 * np.sqrt(max(mean - r, 1e-12))
    if ra / rb > F["max_radius_ratio"]:
        return False
    exp = np.pi * ra * rb
    if abs(exp - area) / exp > F["max_area_error"]:
        return False
    bbox_c = np.array([xs.min() + w / 2, ys.min() + h / 2])
    err = max(F["skew_abs"], F["skew_rel"] * ra)
    return np.sum((np.array([mx + 0.5, my + 0.5]) - bbox_c) ** 2) <= err * err


def pass_rate(d, brightness, rng):
    ok = 0
    for _ in range(TRIALS):
        cx, cy = 2 + d / 2 + rng.random(2)
        ok += passes(render(d, cx, cy, brightness))
    return ok / TRIALS


def main():
    rng = np.random.default_rng(0)
    ds = np.arange(1.5, 8.01, 0.1)
    curves = {b: np.array([pass_rate(d, b, rng) for d in ds]) for b in (1.0, 0.6)}
    # smallest d where this and every larger d pass >= 95%
    d_min = {b: ds[len(c) - np.argmax(c[::-1] < 0.95)] for b, c in curves.items()}
    for b, c in curves.items():
        print(f"brightness {b:.1f}: 50% pass at d={ds[np.argmax(c >= 0.5)]:.1f} px, >=95% from d={d_min[b]:.1f} px")

    fig, (a0, a1) = plt.subplots(1, 2, figsize=(14, 5.5))
    for b, c in curves.items():
        a0.plot(ds, 100 * c, label=f"marker brightness {b:.0%} of full scale")
    a0.axhline(95, ls=":", c="gray")
    a0.set(xlabel="marker image diameter (px)", ylabel="blobs accepted (%)",
           title=f"Blob filter pass rate (threshold {THRESHOLD}, min_area {F['min_area']}, area err ≤ {F['max_area_error']})")
    a0.legend(); a0.grid(alpha=0.3)

    Z = np.linspace(0.3, 10, 300)
    dpx = d_min[1.0]
    for lens in LENSES_MM:
        f = lens * 1e3 / PIXEL_PITCH_UM
        a1.plot(Z, dpx * Z / f * 1e3, label=f"{lens} mm lens (f={f:.0f} px)", lw=2.5 if lens == 4.35 else 1.2)
    for D in MARKERS_MM:
        a1.axhline(D, ls="--", c="gray", lw=0.8)
        f = 4.35e3 / PIXEL_PITCH_UM
        zmax = D * 1e-3 * f / dpx
        a1.annotate(f"{D} mm marker: max {zmax:.1f} m (4.35 mm lens)", (6.2, D + 0.3), fontsize=9)
    a1.set(xlabel="distance from camera (m)", ylabel="minimum marker diameter (mm)", ylim=(0, 25),
           title=f"Minimum visible marker (≥95% accepted, d ≥ {dpx:.1f} px, full brightness)")
    a1.legend(loc="upper left"); a1.grid(alpha=0.3)
    fig.tight_layout()
    out = Path(__file__).parent / "out" / "marker_size.png"
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=110)
    print(out)


if __name__ == "__main__":
    main()
