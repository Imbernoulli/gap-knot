"""The robot's rope after each phase of the demo graph beside the demonstration's key shapes.

    python tools/demo_compare.py EPISODE_DIR [--out PNG]

EPISODE_DIR holds the graph's states.jsonl (graphs/overhand_demo with params "log_states": true). Top row: the
demo's rope (video/overhand/demo_plan.json key shapes) mirrored across the rope's line and scaled to the sim knot
(knotlab.demo), centred on X1 when the shape has one; bottom row: the robot's rope when each phase ended, centred
on its X1 (else its centroid). Same scale in every panel; titles carry the Gauss codes from the working end.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from knotlab import demo, diagram  # noqa: E402

W = np.array([0.6, 0.0, 0.75])
PAIRS = [("loop", "after step 1 (X1)", 22.7), ("over", "end drawn out", 25.0), ("thread", "tucked (X2)", 29.3),
         ("tail", "drawn through (X3)", 32.0), ("pull", "pulled", 35.8)]


def _rope(ax, xy, colour_by=None, lw=5):
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))])
    for k in range(len(xy) - 1):
        a = xy[k:k + 2]
        ax.plot(a[:, 0], a[:, 1], color=plt.cm.viridis(s[k] / max(s[-1], 1e-9)), lw=lw, solid_capstyle="round")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("ep")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    ep = Path(a.ep)
    rows = [json.loads(l) for l in open(ep / "states.jsonl")]
    d = json.loads((ROOT / "video" / "overhand" / "demo_plan.json").read_text())
    shapes = {round(k["t"], 1): k for k in d["key_shapes"]}
    k_scale = 0.503 / demo.DEMO_KNOT_ROPE
    fig, axs = plt.subplots(2, len(PAIRS), figsize=(3.4 * len(PAIRS), 6.6))
    for j, (phase, label, t) in enumerate(PAIRS):
        ks = shapes.get(t)
        ax = axs[0, j]
        if ks is not None:
            c = np.asarray(ks.get("centreline_centred") or ks["centreline"], dtype=float)
            # Demo table (x away, y left) -> the robot's view (plot x = -y, plot y = x), mirrored across the line.
            xy = np.stack([-c[:, 1], -c[:, 0]], axis=1) * k_scale
            _rope(ax, xy)
            ax.set_title(f"demo {t:.1f} s: {ks['label']}\n{d['crossing_structure']['after_step1']['gauss_from_tip'] if t < 26 else ('1O 2U 3O 1U 2O 3U' if t > 30 else '')}", fontsize=8)
        ax = axs[1, j]
        mine = [r for r in rows if (r.get("step") or {}).get("phase") == phase]
        if mine:
            r = mine[-1]
            p = np.asarray(r["nodes"], dtype=float)
            cs = r.get("crossings") or []
            x1 = demo.find_x1({"crossings": cs}) if cs else None
            ctr = np.asarray(x1["at"]) if x1 else p[:, :2].mean(axis=0)
            xy = np.stack([-(p[:, 1] - ctr[1]), p[:, 0] - ctr[0]], axis=1)
            _rope(ax, xy)
            for c in diagram.crossings(p - W):
                q = np.array([c.xy[0] + W[0], c.xy[1]]) - ctr[::1][:2]
                ax.plot(-q[1], q[0], "o", mfc="none", mec="red", ms=10)
            ax.set_title(f"robot after '{phase}' ({label})\n{r['code']}", fontsize=8)
        for ax in axs[:, j]:
            ax.set_xlim(-0.25, 0.25)
            ax.set_ylim(-0.22, 0.18)
            ax.set_aspect("equal")
            ax.grid(alpha=0.3)
            ax.tick_params(labelsize=6)
    fig.suptitle("demo key shapes (mirrored across the line, x1.15) vs the robot's rope after each phase; robot's view: up = away, right = robot's right", fontsize=9)
    plt.tight_layout()
    out = a.out or str(ep / "demo_compare.png")
    plt.savefig(out, dpi=90)
    print(out)


if __name__ == "__main__":
    main()
