"""Blob filter lab: synthetic marker images through the camera blob filter.

Compares the CURRENT filter (camera/core/src/frame_processor.rs, config/manager.txtpb values) with a
PROPOSED radial-symmetry filter:
  - theta spread: pixels binned in 0.5 px rings around the weighted centroid; RMS deviation from each
    ring's mean, / peak. A marker image depends on r only, so this should be ~0 (pixelation + noise).
  - outward rise: largest increase of ring-mean brightness moving outward, / peak. A marker is
    brightest in the middle, so this should be ~0. Catches rings (which have no theta dependence).

Outputs (out/):
  blob_gallery.png   one sample per shape x size: pixels, threshold outline, centroid, moment ellipse,
                     and the verdict of each pipeline (accepted, or the stage that rejected it)
  blob_funnel.png    % of blobs rejected at each stage (first failing stage), Monte Carlo over random
                     sub-pixel offsets and rotations

  .venv/bin/python blob_lab.py [--blur 0.5] [--noise 2] [--brightness 1.0] [--trials 300]
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse
from scipy import ndimage

# --- current camera config (config/manager.txtpb) ---
T = 80
MIN_AREA, MAX_AREA = 6, 10000
MAX_ASPECT, MAX_RADIUS_RATIO, MAX_AREA_ERROR = 1.5, 1.5, 0.25
SKEW_ABS, SKEW_REL = 2.0, 0.05
# --- proposed ---
MAX_THETA_SPREAD = 0.13
MAX_OUTWARD_RISE = 0.30

GOOD, BAD = "#0ca30c", "#d03b3b"   # status colors, always paired with a check/cross + stage text
SS = 16                             # supersampling for pixel coverage
SIZES = [4, 6, 8, 12]               # marker image diameter (px)

# name -> (expected verdict, shape(x, y, R) -> inside). Coordinates are in pixels from the center.
SHAPES = {
    "disk": ("accept", lambda x, y, R: x * x + y * y <= R * R),
    "cut, 95% visible": ("?", lambda x, y, R: (x * x + y * y <= R * R) & (x < 0.8 * R)),
    "cut, 80% visible": ("reject", lambda x, y, R: (x * x + y * y <= R * R) & (x < 0.5 * R)),
    "cut, 50% visible": ("reject", lambda x, y, R: (x * x + y * y <= R * R) & (x < 0)),
    "crescent": ("reject", lambda x, y, R: (x * x + y * y <= R * R) & ~((x - 0.5 * R) ** 2 + y * y <= R * R)),
    "ring": ("reject", lambda x, y, R: (x * x + y * y <= R * R) & ~(x * x + y * y <= (0.6 * R) ** 2)),
    "plus": ("reject", lambda x, y, R: ((abs(x) < 0.25 * R) & (abs(y) < R)) | ((abs(y) < 0.25 * R) & (abs(x) < R))),
    "two merged": ("reject", lambda x, y, R: ((x - 0.9 * R) ** 2 + y * y <= R * R) | ((x + 0.9 * R) ** 2 + y * y <= R * R)),
    "motion streak": ("?", lambda x, y, R: (np.hypot(np.clip(x, -R, R) - x, y) <= R)),
}


def render(shape, d, rng, brightness, blur, noise):
    """8-bit image of one blob at a random sub-pixel offset and rotation."""
    n = int(np.ceil(2 * d)) + 8
    c = n / 2 + rng.random(2)
    a = rng.random() * 2 * np.pi
    s = (np.arange(n * SS) + 0.5) / SS
    X, Y = np.meshgrid(s - c[0], s - c[1])
    xr, yr = X * np.cos(a) + Y * np.sin(a), -X * np.sin(a) + Y * np.cos(a)
    cov = shape(xr, yr, d / 2).reshape(n, SS, n, SS).mean(axis=(1, 3))
    img = 255 * brightness * cov
    if blur > 0:
        img = ndimage.gaussian_filter(img, blur)
    img = img + rng.normal(0, noise, img.shape) if noise > 0 else img
    return np.clip(np.round(img), 0, 255), c - 0.5   # image, true center in pixel-index coords


def analyze(img):
    """Stats of the brightest connected component (8-way, like the camera)."""
    lab, k = ndimage.label(img > T, structure=np.ones((3, 3)))
    if k == 0:
        return None
    best = 1 + int(np.argmax(ndimage.sum(img - T, lab, range(1, k + 1))))
    m = lab == best
    ys, xs = np.nonzero(m)
    w = img[m] - T
    M = w.sum()
    mx, my = (w * xs).sum() / M, (w * ys).sum() / M
    vx, vy = (w * (xs - mx) ** 2).sum() / M, (w * (ys - my) ** 2).sum() / M
    cxy = (w * (xs - mx) * (ys - my)).sum() / M
    mean, diff = (vx + vy) / 2, (vx - vy) / 2
    r = np.hypot(diff, cxy)
    ra, rb = 2 * np.sqrt(mean + r), 2 * np.sqrt(max(mean - r, 1e-12))
    bw, bh = xs.max() - xs.min() + 1, ys.max() - ys.min() + 1
    st = dict(mask=m, area=m.sum(), cx=mx, cy=my, ra=ra, rb=rb, angle=0.5 * np.arctan2(cxy, diff),
              aspect=max(bw, bh) / min(bw, bh),
              edge=xs.min() == 0 or ys.min() == 0 or xs.max() == img.shape[1] - 1 or ys.max() == img.shape[0] - 1,
              skew=np.hypot(mx + 0.5 - (xs.min() + bw / 2), my + 0.5 - (ys.min() + bh / 2)))
    st["area_error"] = (np.pi * ra * rb - st["area"]) / (np.pi * ra * rb)
    # radial profile around the weighted centroid, using all pixels (also those below threshold)
    Yi, Xi = np.mgrid[0:img.shape[0], 0:img.shape[1]]
    rr = np.hypot(Xi - mx, Yi - my)
    sel = rr <= np.hypot(xs - mx, ys - my).max() + 1
    v, bins = img[sel], np.floor(rr[sel] / 0.5).astype(int)
    peak = max(img[m].max(), 1)
    res, prof = 0.0, []
    for b in np.unique(bins):
        vb = v[bins == b]
        res += ((vb - vb.mean()) ** 2).sum()
        prof.append(vb.mean())
    prof = np.array(prof)
    st["theta_spread"] = np.sqrt(res / len(v)) / peak
    st["outward_rise"] = float(np.max(np.maximum.accumulate(prof[::-1])[::-1] - prof)) / peak
    return st


COMMON = [
    ("min area", lambda s: s["area"] >= MIN_AREA),
    ("max area", lambda s: s["area"] <= MAX_AREA),
    ("edge", lambda s: not s["edge"]),
]
CURRENT = COMMON + [
    ("aspect", lambda s: s["aspect"] <= MAX_ASPECT),
    ("radius ratio", lambda s: s["ra"] / s["rb"] <= MAX_RADIUS_RATIO),
    ("area error", lambda s: abs(s["area_error"]) <= MAX_AREA_ERROR),
    ("skew", lambda s: s["skew"] <= max(SKEW_ABS, SKEW_REL * s["ra"])),
]
PROPOSED = COMMON + [
    ("theta spread", lambda s: s["theta_spread"] <= MAX_THETA_SPREAD),
    ("outward rise", lambda s: s["outward_rise"] <= MAX_OUTWARD_RISE),
]


def verdict(st, stages):
    if st is None:
        return "no blob"
    for name, ok in stages:
        if not ok(st):
            return name
    return "accepted"


def gallery(args, rng, out):
    fig, axes = plt.subplots(len(SHAPES), len(SIZES), figsize=(3.1 * len(SIZES), 2.9 * len(SHAPES)))
    for i, (name, (expect, shape)) in enumerate(SHAPES.items()):
        for j, d in enumerate(SIZES):
            ax = axes[i, j]
            img, center = render(shape, d, rng, args.brightness, args.blur, args.noise)
            st = analyze(img)
            ax.imshow(img, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
            ax.set_xticks([]); ax.set_yticks([])
            if st is not None:
                ax.contour(st["mask"], levels=[0.5], colors=["#3987e5"], linewidths=1)
                ax.add_patch(Ellipse((st["cx"], st["cy"]), 2 * st["ra"], 2 * st["rb"], angle=np.degrees(st["angle"]),
                                     fill=False, ls="--", lw=1, ec="#fab219"))
                ax.plot(st["cx"], st["cy"], "+", c="#fab219", ms=8)
            for k, (label, stages) in enumerate((("now", CURRENT), ("new", PROPOSED))):
                v = verdict(st, stages)
                ok = v == "accepted"
                ax.text(0.02, 0.98 - 0.13 * k, f"{label}: {'✓ accepted' if ok else '✗ ' + v}", transform=ax.transAxes,
                        va="top", fontsize=8, color="white", bbox=dict(fc=GOOD if ok else BAD, ec="none", pad=1.5))
            if st is not None:
                bias = np.hypot(st["cx"] - center[0], st["cy"] - center[1])
                ax.text(0.02, 0.02, f"centroid error {bias:.2f} px\n"
                        f"A/E {st['area'] / (np.pi * st['ra'] * st['rb']):.2f}  ra/rb {st['ra'] / st['rb']:.2f}\n"
                        f"θ {st['theta_spread']:.3f}  rise {st['outward_rise']:.2f}", transform=ax.transAxes,
                        fontsize=7, color="white", va="bottom", family="monospace")
            if i == 0:
                ax.set_title(f"{d} px", fontsize=11)
            if j == 0:
                ax.set_ylabel(f"{name}\n(should {expect})" if expect != "?" else f"{name}\n(either)", fontsize=10)
    fig.suptitle(f"Blob filter gallery  (blur σ={args.blur} px, noise σ={args.noise}, brightness {args.brightness:.0%})\n"
                 "blue = threshold outline, yellow + = weighted centroid, dashed = moment ellipse", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(out / "blob_gallery.png", dpi=90)
    plt.close(fig)


def funnel(args, rng, out):
    rows = [(name, d) for name in SHAPES for d in SIZES]
    fig, axes = plt.subplots(1, 2, figsize=(17, 0.32 * len(rows) + 2.5))
    for ax, (title, stages) in zip(axes, (("current filter", CURRENT), ("proposed filter", PROPOSED))):
        cols = [n for n, _ in stages] + ["accepted"]
        mat = np.zeros((len(rows), len(cols)))
        for r, (name, d) in enumerate(rows):
            for _ in range(args.trials):
                v = verdict(analyze(render(SHAPES[name][1], d, rng, args.brightness, args.blur, args.noise)[0]), stages)
                mat[r, cols.index(v) if v in cols else 0] += 1
        mat *= 100 / args.trials
        ax.imshow(mat, cmap=matplotlib.colors.LinearSegmentedColormap.from_list("b", ["#ffffff", "#cde2fb", "#3987e5", "#104281"]),
                  vmin=0, vmax=100, aspect="auto")
        for r in range(len(rows)):
            for c in range(len(cols)):
                if mat[r, c] >= 0.5:
                    ax.text(c, r, f"{mat[r, c]:.0f}", ha="center", va="center", fontsize=7,
                            color="white" if mat[r, c] > 55 else "#0b0b0b")
            expect = SHAPES[rows[r][0]][0]
            acc = mat[r, -1]
            right = (expect == "accept" and acc >= 95) or (expect == "reject" and acc <= 5)
            if expect != "?":
                ax.text(len(cols) - 0.4, r, "✓" if right else "✗", color=GOOD if right else BAD, va="center", fontsize=10)
        ax.set_xticks(range(len(cols)), cols, rotation=30, ha="right")
        ax.set_yticks(range(len(rows)), [f"{n} · {d}px" for n, d in rows], fontsize=8)
        for r in range(len(SIZES), len(rows), len(SIZES)):
            ax.axhline(r - 0.5, c="#c3c2b7", lw=0.8)
        ax.set_title(f"{title}: % of blobs rejected at each stage (first failure)\n"
                     "✓/✗ = right/wrong overall (good shapes ≥95% accepted, bad ≤5%)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "blob_funnel.png", dpi=90)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--blur", type=float, default=0.5, help="lens blur gaussian sigma (px)")
    ap.add_argument("--noise", type=float, default=2.0, help="sensor noise sigma (counts)")
    ap.add_argument("--brightness", type=float, default=1.0, help="marker peak / 255")
    ap.add_argument("--trials", type=int, default=300)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    out = Path(__file__).parent / "out"
    out.mkdir(exist_ok=True)
    rng = np.random.default_rng(args.seed)
    gallery(args, rng, out)
    funnel(args, rng, out)
    print(out / "blob_gallery.png")
    print(out / "blob_funnel.png")


if __name__ == "__main__":
    main()
