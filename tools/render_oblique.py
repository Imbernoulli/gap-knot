"""An episode seen at an angle, so a knot reads as a knot: which strand is on top, and what is off the table.

    python tools/render_oblique.py EPISODE_DIR [--mp4] [--keys N]

The rope is drawn back to front (each segment's depth from the camera), outlined, so the strand on top hides the
one under it; its shadow on the table shows how high every part is. Hands are drawn where their tool centres are,
with a line down to the table. Writes oblique_keys.png and, with --mp4, oblique.mp4 (8x speed).
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from knotlab import diagram, task  # noqa: E402

W = np.array([0.6, 0.0, 0.75])
EVERY = 15
AZ, EL = math.radians(-20.0), math.radians(38.0)  # camera: from the robot's side, a little to its right, 38 deg up


def camera(p: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """World-ish (bench frame, metres) points to screen xy and depth (larger = farther)."""
    # Look along +x (away from the robot), rotated by AZ about z, tilted down by EL.
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    ca, sa = math.cos(AZ), math.sin(AZ)
    fwd = x * ca - y * sa            # along the view's ground direction
    side = x * sa + y * ca           # screen horizontal (robot's left is +y -> screen left)
    ce, se = math.cos(EL), math.sin(EL)
    up = z * ce + fwd * se           # screen vertical
    depth = fwd * ce - z * se
    return np.stack([-side, up], axis=1), depth


def draw(ax, rope: np.ndarray, hands, title: str, lim) -> None:
    ax.set_facecolor("#e9ebef")
    # Table: a grid patch around the rope.
    gx = np.linspace(lim[0][0], lim[0][1], 9)
    gy = np.linspace(lim[1][0], lim[1][1], 9)
    for v in gx:
        q, _ = camera(np.array([[v, lim[1][0], 0], [v, lim[1][1], 0]]))
        ax.plot(q[:, 0], q[:, 1], color="#d4d8df", lw=0.8, zorder=0)
    for v in gy:
        q, _ = camera(np.array([[lim[0][0], v, 0], [lim[0][1], v, 0]]))
        ax.plot(q[:, 0], q[:, 1], color="#d4d8df", lw=0.8, zorder=0)
    # Shadow.
    sh = rope.copy()
    sh[:, 2] = 0.0
    q, _ = camera(sh)
    ax.plot(q[:, 0], q[:, 1], color="#b9bec8", lw=5, solid_capstyle="round", zorder=1, alpha=0.8)
    # Rope, back to front.
    q, d = camera(rope)
    s = diagram.arc_length(rope)
    seg_depth = 0.5 * (d[:-1] + d[1:])
    order = np.argsort(-seg_depth)
    base = 10
    for rank, k in enumerate(order):
        col = plt.cm.cool(0.1 + 0.8 * s[k] / s[-1])
        ax.plot(q[k:k + 2, 0], q[k:k + 2, 1], color="#1b1d25", lw=9, solid_capstyle="round", zorder=base + 2 * rank)
        ax.plot(q[k:k + 2, 0], q[k:k + 2, 1], color=col, lw=6.5, solid_capstyle="round", zorder=base + 2 * rank + 1)
    top = base + 2 * len(order) + 10
    # Hands.
    if hands is not None and len(hands) >= 9:
        for i, (lab, c) in enumerate((("L", "#2457c5"), ("R", "#d4731a"))):
            x, y, z, w = hands[1 + 4 * i: 5 + 4 * i]
            if not np.isfinite(x):
                continue
            pt = np.array([[x, y, z], [x, y, 0.0]]) - np.array([0.6, 0.0, 0.75 + 0.0]) * np.array([1, 1, 1])
            pt[1, 2] = 0.0
            pq, _ = camera(pt)
            ax.plot(pq[:, 0], pq[:, 1], color=c, lw=1, ls=":", zorder=top)
            ax.plot(pq[0, 0], pq[0, 1], marker="v", ms=11, color=c, zorder=top + 1)
            ax.annotate(f"{lab} {1e3 * (z - 0.75):.0f} mm", (pq[0, 0], pq[0, 1]), xytext=(8, 4), textcoords="offset points",
                        fontsize=8, color=c, zorder=top + 1)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    ax.set_title(title, fontsize=9)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("ep")
    ap.add_argument("--mp4", action="store_true")
    ap.add_argument("--keys", type=int, default=8)
    a = ap.parse_args()
    ep = Path(a.ep)
    r = json.loads((ep / "result.json").read_text())
    ropes = np.load(ep / "rope_frames.npy").astype(np.float64) - W
    hands = np.load(ep / "hand_frames.npy") if (ep / "hand_frames.npy").exists() else None
    v = task.Variant(**r["variant"])
    allp = ropes.reshape(-1, 3)
    pad = 0.03
    lim = ((allp[:, 0].min() - pad, allp[:, 0].max() + pad), (allp[:, 1].min() - pad, allp[:, 1].max() + pad))
    corners = np.array([[lim[0][i], lim[1][j], zz] for i in (0, 1) for j in (0, 1) for zz in (0.0, max(0.05, allp[:, 2].max()))])
    cq, _ = camera(corners)
    xlim = (cq[:, 0].min() - 0.01, cq[:, 0].max() + 0.01)
    ylim = (cq[:, 1].min() - 0.01, cq[:, 1].max() + 0.01)

    def label(k: int) -> str:
        p = ropes[k]
        sc = task.score(p, knot=v.knot)
        step = int(hands[k][0]) if hands is not None else k * EVERY
        lifted = 1e3 * p[:, 2].max()
        return f"t={step / 60:.1f}s  {sc['crossings']} crossings  {sc['code']}  rope in knot {1e3 * task.diagram_knot_length(p):.0f} mm  highest {lifted:.0f} mm"

    idx = np.linspace(0, len(ropes) - 1, a.keys).round().astype(int)
    cols = 2
    rows = int(math.ceil(len(idx) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(7.2 * cols, 4.2 * rows))
    for ax, k in zip(np.ravel(axes), idx):
        draw(ax, ropes[k], hands[k] if hands is not None else None, label(k), lim)
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
    for ax in np.ravel(axes)[len(idx):]:
        ax.axis("off")
    plt.tight_layout()
    plt.savefig(ep / "oblique_keys.png", dpi=85)
    plt.close(fig)
    if a.mp4:
        tmp = Path(tempfile.mkdtemp())
        stride = max(1, len(ropes) // 400)
        for n, k in enumerate(range(0, len(ropes), stride)):
            fig, ax = plt.subplots(figsize=(8, 4.8))
            draw(ax, ropes[k], hands[k] if hands is not None else None, label(k), lim)
            ax.set_xlim(*xlim)
            ax.set_ylim(*ylim)
            plt.tight_layout()
            plt.savefig(tmp / f"f{n:05d}.png", dpi=100)
            plt.close(fig)
        fps = max(1, round(8 * 60 / (EVERY * stride)))
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(fps), "-i", str(tmp / "f%05d.png"),
                        "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2", "-pix_fmt", "yuv420p", str(ep / "oblique.mp4")], check=False)
    print("rendered", ep)


if __name__ == "__main__":
    main()
