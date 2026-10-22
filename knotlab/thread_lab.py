"""The overhand knot's under-crossing rehearsed on its own: the rope as the thread step finds it, and how to score it.

GaP divide and conquer: the full graph never once made its one under-crossing, so this segment is
rehearsed in isolation, from a constructed start, with several primitives side by side.

``pre_thread(target)``: the reference knot up to s = ``cut`` (the loop and crossing 1, B over A at
s ≈ 0.41), then B's remaining tail laid flat down x ≈ 0.40 toward -y and back along y ≈ -0.19 toward +x,
inside reach and crossing nothing. Crossing 2 (B at s ≈ 0.56 UNDER A at s ≈ 0.19, at world ≈ (0.42, 0))
is what the segment must make.
"""

from __future__ import annotations

import numpy as np

from knotlab import diagram

CUT = 0.50  # arc length up to which the reference knot is kept [m]
TOL_S = 0.04  # a crossing within this much arc length of the target's is that crossing [m]


def pre_thread(target: np.ndarray, radius: float = 0.005, *, cut: float = CUT, tail_x: float = -0.13, tail_y: float = -0.19) -> np.ndarray:
    """``target`` (local frame, ``[n, 3]``) kept up to ``cut``, then the rest laid flat out of the way; same node count."""
    s = diagram.arc_length(target)
    keep = target[s <= cut]
    length = float(s[-1])
    rest = length - diagram.arc_length(keep)[-1]
    p0 = keep[-1, :2]
    r = 0.02
    # From the cut: to (tail_x, p0.y) then down to tail_y, round a corner, and along +x for what is left.
    pts = [p0, np.array([tail_x, p0[1] - 0.01]), np.array([tail_x, tail_y + r])]
    for a in np.linspace(0.0, 0.5 * np.pi, 8)[1:]:
        pts.append(np.array([tail_x + r - r * np.cos(a), tail_y + r - r * np.sin(a)]))
    used = sum(np.linalg.norm(np.diff(np.asarray(pts), axis=0), axis=1))
    pts.append(np.array([tail_x + r + max(rest - used, 0.0), tail_y]))
    tail = np.asarray(pts)
    tail3 = np.hstack([tail, np.full((len(tail), 1), radius)])
    tail3[0, 2] = keep[-1, 2]
    path = np.vstack([keep, tail3[1:]])
    sp = diagram.arc_length(path)
    s_new = np.linspace(0.0, sp[-1], len(target))
    out = np.stack([np.interp(s_new, sp, path[:, k]) for k in range(3)], axis=1)
    # Scale the tail so the whole rope keeps the reference's length.
    return out


def target_crossing(target: np.ndarray, which: int) -> dict:
    cs = sorted(diagram.crossings(target), key=lambda c: c.s_i)
    c = cs[which - 1]
    return {"s_a": c.s_i, "s_b": c.s_j, "a_over": c.i_over, "xy": c.xy}


def segment_score(points: np.ndarray, target: np.ndarray) -> dict:
    """Did the rope make the under-crossing (B, past the cut, UNDER A near crossing 2's A strand) and keep crossing 1?

    Topological, not positional: B may cross A at any of its arc lengths past the cut (a pulled-through end
    slides along), A within 5 cm of where the target's A strand is.
    """
    found = diagram.crossings(points)
    out = {"crossings": len(found), "code": diagram.code_str(diagram.gauss_code(points))}
    t1, t2 = target_crossing(target, 1), target_crossing(target, 2)
    hit1 = [c for c in found if abs(c.s_i - t1["s_a"]) < TOL_S and abs(c.s_j - t1["s_b"]) < TOL_S]
    out["c1"] = "missing" if not hit1 else ("ok" if hit1[0].i_over == t1["a_over"] else "flipped")
    hit2 = [c for c in found if abs(c.s_i - t2["s_a"]) < 0.05 and c.s_j > CUT - 0.01]
    out["c2"] = "missing" if not hit2 else ("ok" if hit2[0].i_over == t2["a_over"] else "flipped")
    out["c2_b_s"] = round(hit2[0].s_j, 3) if hit2 else None
    out["under_made"] = out["c2"] == "ok" and out["c1"] == "ok"
    return out
