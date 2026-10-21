"""Top-down pictures of an episode: the rope (upper strand on top), both hands, the pins, and the target as a ghost.

    python tools/render_episode.py EPISODE_DIR [--mp4] [--keys N]

Writes EPISODE_DIR/final.png (the end beside the target), EPISODE_DIR/keys.png (N moments in a grid,
labelled with the node running then), and with --mp4 EPISODE_DIR/episode.mp4 (8x speed).
"""
from __future__ import annotations

import argparse
import json
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
EVERY = 15  # steps between saved frames (knotlab.episode --frames)


def _rope(ax, p, *, ghost=False):
    segs = np.stack([p[:-1, :2], p[1:, :2]], axis=1)
    zc = 0.5 * (p[:-1, 2] + p[1:, 2])
    s = diagram.arc_length(p)
    if ghost:
        for k in range(len(segs)):
            a = segs[k][:, [1, 0]]
            ax.plot(-a[:, 0], a[:, 1], color="white", lw=6, alpha=0.55, solid_capstyle="round", zorder=1)
        return
    for k in np.argsort(zc):
        a = segs[k][:, [1, 0]]
        ax.plot(-a[:, 0], a[:, 1], color="#3a1020", lw=7.5, solid_capstyle="round", zorder=10 + 2 * k)
        ax.plot(-a[:, 0], a[:, 1], color=plt.cm.spring(0.15 + 0.7 * s[k] / s[-1]), lw=5.5, solid_capstyle="round", zorder=11 + 2 * k)


def frame(ax, rope_w, hands_row, lay, title):
    ax.set_facecolor("#c9ccd3")
    _rope(ax, np.asarray(lay["target"]) + W, ghost=True)
    _rope(ax, rope_w)
    for name, q in lay["pins"].items():
        q = q + W
        ax.plot(-q[1], q[0], "o", color="#0d7a6e", ms=7, zorder=10**6)
    if hands_row is not None and len(hands_row) >= 9:
        for k, (arm, c) in enumerate((("L", "#1f5fbf"), ("R", "#d4731a"))):
            x, y, z, w = hands_row[1 + 4 * k: 5 + 4 * k]
            if np.isfinite(x):
                ax.plot(-y, x, marker="s", ms=8 + 40 * max(0.0, min(1.0, (z - 0.75) / 0.1)) * 0.3, mfc="none", mec=c, mew=2.5, zorder=10**6)
                ax.annotate(f"{arm} {1e3 * (z - 0.75):.0f}mm{' shut' if w < 0.015 else ''}", (-y, x), xytext=(7, -10),
                            textcoords="offset points", fontsize=7, color=c, zorder=10**6)
    ax.set_xlim(-0.24, 0.24)
    ax.set_ylim(0.33, 0.66)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=8)


def node_at(spans, step):
    best = ""
    for sp in spans:
        if sp.get("start") is not None and sp["start"] <= step <= (sp.get("end") or step) and "." in sp["node"]:
            best = sp["node"].split(".")[-1]
    return best


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("ep")
    ap.add_argument("--mp4", action="store_true")
    ap.add_argument("--keys", type=int, default=12)
    a = ap.parse_args()
    ep = Path(a.ep)
    r = json.loads((ep / "result.json").read_text())
    v = task.Variant(**r["variant"])
    lay = task.layout(v)
    ropes = np.load(ep / "rope_frames.npy")
    hands = np.load(ep / "hand_frames.npy") if (ep / "hand_frames.npy").exists() else None
    try:
        from harness.analyze import diagnose, load_episode, target_crossings

        spans = diagnose(load_episode(ep), target_crossings())["spans"]
    except Exception:
        spans = []
    # The end beside the target.
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    last = ropes[-1].astype(np.float64)
    sc = task.score(last - W)
    frame(axes[0], last, hands[-1] if hands is not None else None, lay, f"end (t={r['sim_time_s']} s): code {sc['code']}  prefix {sc['prefix']}/16")
    frame(axes[1], np.asarray(lay["target"]) + W, None, lay, "target")
    plt.tight_layout()
    plt.savefig(ep / "final.png", dpi=110)
    plt.close(fig)
    # Key moments.
    idx = np.linspace(0, len(ropes) - 1, a.keys).round().astype(int)
    cols = 4
    rows = int(np.ceil(len(idx) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 2.9 * rows))
    for ax, k in zip(np.ravel(axes), idx):
        step = int(hands[k][0]) if hands is not None else k * EVERY
        frame(ax, ropes[k].astype(np.float64), hands[k] if hands is not None else None, lay,
              f"t={step / 60:.1f}s {node_at(spans, step)}  prefix {task.score(ropes[k].astype(np.float64) - W)['prefix']}")
    for ax in np.ravel(axes)[len(idx):]:
        ax.axis("off")
    plt.tight_layout()
    plt.savefig(ep / "keys.png", dpi=90)
    plt.close(fig)
    if a.mp4:
        tmp = Path(tempfile.mkdtemp())
        stride = max(1, len(ropes) // 600)
        for n, k in enumerate(range(0, len(ropes), stride)):
            fig, ax = plt.subplots(figsize=(6.4, 4.6))
            step = int(hands[k][0]) if hands is not None else k * EVERY
            frame(ax, ropes[k].astype(np.float64), hands[k] if hands is not None else None, lay, f"t={step / 60:5.1f}s  {node_at(spans, step)}")
            plt.tight_layout()
            plt.savefig(tmp / f"f{n:05d}.png", dpi=100)
            plt.close(fig)
        fps = max(1, round(8 * 60 / (EVERY * stride)))
        subprocess.run(["ffmpeg", "-v", "error", "-y", "-framerate", str(fps), "-i", str(tmp / "f%05d.png"), "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
                        "-pix_fmt", "yuv420p", str(ep / "episode.mp4")], check=False)
    print("rendered", ep)


if __name__ == "__main__":
    main()
