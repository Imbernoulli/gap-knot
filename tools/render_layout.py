"""Draw the start and target layouts (world frame, seen from above with the robot at the bottom) for the agents' docs."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from knotlab import task, diagram
from harness.analyze import target_crossings

W = np.array([0.6, 0.0, 0.75])


def draw(ax, pts, lay, title, label_cross=False):
    p = pts + W
    segs = np.stack([p[:-1, :2], p[1:, :2]], axis=1); zc = 0.5 * (p[:-1, 2] + p[1:, 2])
    s = diagram.arc_length(pts)
    for k in np.argsort(zc):
        a = segs[k][:, [1, 0]]
        ax.plot(-a[:, 0], a[:, 1], color="white", lw=10, solid_capstyle="round", zorder=2 * k)
        ax.plot(-a[:, 0], a[:, 1], color=plt.cm.viridis(s[k] / s[-1]), lw=7, solid_capstyle="round", zorder=2 * k + 1)
    for name, q in lay["pins"].items():
        q = q + W
        ax.plot(-q[1], q[0], "o", color="crimson", ms=9, zorder=10**6)
        ax.annotate(f"pin_{name}", (-q[1], q[0]), xytext=(6, 6), textcoords="offset points", zorder=10**6, color="crimson")
    ax.plot(-p[0, 1], p[0, 0], "^", color="k", ms=11, zorder=10**6)
    ax.annotate("node 0 (s=0)", (-p[0, 1], p[0, 0]), xytext=(5, -14), textcoords="offset points", zorder=10**6)
    ax.plot(-p[-1, 1], p[-1, 0], "v", color="k", ms=11, zorder=10**6)
    ax.annotate(f"last node (s={s[-1]:.2f})", (-p[-1, 1], p[-1, 0]), xytext=(5, -14), textcoords="offset points", zorder=10**6)
    for sv in np.arange(0.1, s[-1], 0.1):
        k = int(np.searchsorted(s, sv))
        ax.annotate(f"{sv:.1f}", (-p[k, 1], p[k, 0]), fontsize=7, color="navy", zorder=10**6, xytext=(3, 3), textcoords="offset points")
    if label_cross:
        for c in target_crossings():
            k = int(np.searchsorted(s, c["s_a"])); q = p[k]
            ax.annotate(str(c["id"]), (-q[1], q[0]), fontsize=12, weight="bold", color="darkorange", zorder=10**6, xytext=(-4, -4),
                        textcoords="offset points", bbox=dict(boxstyle="circle,pad=0.15", fc="white", ec="darkorange"))
    ax.add_patch(plt.Rectangle((-0.20, 0.35), 0.40, 0.20, fill=False, ls="--", ec="gray", zorder=10**6))
    ax.annotate("both hands reach (tool down)", (-0.2, 0.552), fontsize=8, color="gray", zorder=10**6)
    ax.set_aspect("equal"); ax.set_facecolor("#d8dbe2"); ax.set_title(title, fontsize=10)
    ax.set_xlabel("<- robot's left (world +y)        world y, mirrored        robot's right (world -y) ->")
    ax.set_ylabel("world x (away from the robot)")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "harness/docs/img/start_and_target.png"
    lay = task.layout(task.NOMINAL)
    fig, axes = plt.subplots(1, 2, figsize=(17, 8))
    draw(axes[0], lay["start"], lay, "START (nominal): the middle hung on pin_top; each half down, out, up the side and folded back, all inside reach")
    draw(axes[1], lay["target"], lay, "TARGET: the double coin knot. Orange numbers = crossing ids. The upper strand is drawn on top.\nColour and the small numbers = arc length s from node 0.", label_cross=True)
    for ax in axes:
        ax.set_xlim(-0.27, 0.27); ax.set_ylim(0.32, 0.72)
    plt.tight_layout(); plt.savefig(out, dpi=90)
