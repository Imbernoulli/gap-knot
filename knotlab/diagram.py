"""The knot diagram of a rope seen from above: its crossings, which strand is over, and its Gauss code.

A rope is an ordered centreline ``[n, 3]``. Seen from above, two non-adjacent
segments whose projections cross make a crossing; the one higher at the
crossing point is over. Walking the rope from its first node, every crossing is
met twice, once over and once under; the sequence of (label, O/U) is the long
knot's Gauss code. Two ropes with the same Gauss code lie in the same diagram,
which is what "the knot is tied" means here: the double coin knot is the open
3-lead 4-bight Turk's head, eight crossings met alternately over and under.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Crossing:
    i: int  # the earlier segment (i < j)
    j: int
    s_i: float  # arc length of the crossing along the rope [m], on segment i
    s_j: float
    i_over: bool  # segment i passes over segment j
    gap: float  # height of the over strand's axis above the under strand's [m]
    xy: tuple[float, float]


def arc_length(points: np.ndarray) -> np.ndarray:
    legs = np.linalg.norm(np.diff(points, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(legs)])


def crossings(points: np.ndarray, *, skip: int = 2) -> list[Crossing]:
    """Every crossing of the rope's projection, with which strand is over, in no particular order."""
    p = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    s = arc_length(p)
    a, b = p[:-1], p[1:]
    n = len(a)
    out: list[Crossing] = []
    for i in range(n - skip):
        j = np.arange(i + skip, n)
        d1 = b[i, :2] - a[i, :2]
        d2 = b[j, :2] - a[j, :2]
        r = a[j, :2] - a[i, :2]
        denom = d1[0] * d2[:, 1] - d1[1] * d2[:, 0]
        ok = np.abs(denom) > 1e-18
        with np.errstate(divide="ignore", invalid="ignore"):
            u = (r[:, 0] * d2[:, 1] - r[:, 1] * d2[:, 0]) / denom
            v = (r[:, 0] * d1[1] - r[:, 1] * d1[0]) / denom
        hit = ok & (u >= 0.0) & (u < 1.0) & (v >= 0.0) & (v < 1.0)
        for k in np.nonzero(hit)[0]:
            jj = int(j[k])
            zi = a[i, 2] + u[k] * (b[i, 2] - a[i, 2])
            zj = a[jj, 2] + v[k] * (b[jj, 2] - a[jj, 2])
            xy = a[i, :2] + u[k] * d1
            out.append(
                Crossing(
                    i=i,
                    j=jj,
                    s_i=float(s[i] + u[k] * (s[i + 1] - s[i])),
                    s_j=float(s[jj] + v[k] * (s[jj + 1] - s[jj])),
                    i_over=bool(zi > zj),
                    gap=float(abs(zi - zj)),
                    xy=(float(xy[0]), float(xy[1])),
                )
            )
    return out


def gauss_code(points: np.ndarray) -> list[tuple[int, str]]:
    """The long knot's Gauss code: walking the rope, each crossing met as (label, 'O' or 'U').

    Labels number crossings in the order they are first met, so two ropes in the
    same diagram give the same code whatever their size or position.
    """
    events = []
    for c in crossings(points):
        events.append((c.s_i, id(c), "O" if c.i_over else "U"))
        events.append((c.s_j, id(c), "U" if c.i_over else "O"))
    events.sort()
    label: dict[int, int] = {}
    code = []
    for _, key, ou in events:
        if key not in label:
            label[key] = len(label) + 1
        code.append((label[key], ou))
    return code


def code_str(code: list[tuple[int, str]]) -> str:
    return " ".join(f"{k}{ou}" for k, ou in code) or "(none)"


def is_alternating(code: list[tuple[int, str]]) -> bool:
    return all(code[k][1] != code[k + 1][1] for k in range(len(code) - 1))


def prefix_match(code: list[tuple[int, str]], target: list[tuple[int, str]]) -> int:
    """How many entries of ``code`` agree with ``target`` from the start."""
    k = 0
    while k < min(len(code), len(target)) and code[k] == target[k]:
        k += 1
    return k
