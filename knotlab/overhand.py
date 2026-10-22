"""The overhand knot (单结): a loose trefoil lying on the bench, and the rope as it starts, straight.

``trefoil`` is RoboSimStudio's ``robosimstudio_materials.rod.knot.overhand`` (Apache-2.0,
copyright the RoboSimStudio authors), copied so the analysis side, which runs
without RoboSimStudio installed, can draw the same target. Its tails run along
-y and +y from the knot, which lies toward -x of ``at``; node 0 is the end of the
-y tail. Walking from node 0 its Gauss code is ``1U 2O 3U 1O 2U 3O``.
"""

from __future__ import annotations

import math

import numpy as np

from knotlab.diagram import arc_length


def trefoil(*, at=(0.0, 0.0), size=0.05, radius=0.005, tail=0.15, spacing=0.008, gap=0.2 * math.pi) -> np.ndarray:
    t = np.linspace(math.pi / 3 + float(gap), math.pi / 3 + 2.0 * math.pi - float(gap), 480)
    x = np.sin(t) + 2.0 * np.sin(2.0 * t)
    y = np.cos(t) - 2.0 * np.cos(2.0 * t)
    z = -np.sin(3.0 * t)
    tip = math.atan2(
        math.cos(math.pi / 3) - 2.0 * math.cos(2 * math.pi / 3), math.sin(math.pi / 3) + 2 * math.sin(2 * math.pi / 3)
    )
    c, s = math.cos(-tip), math.sin(-tip)
    x, y = c * x - s * y, s * x + c * y
    scale = size / 6.0
    lift = 2.0 * radius
    knot = np.stack([x * scale, y * scale, radius + lift * (z + 1.0)], axis=1)
    bend = max(2.5 * radius, 0.25 * size)

    def tail_from(end: np.ndarray, heading: np.ndarray, side: float) -> np.ndarray:
        d = heading / np.linalg.norm(heading)
        knee = end + bend * d
        out = knee + np.array([0.0, side * bend, 0.0])
        u = np.linspace(0.0, 1.0, 24)[:, None]
        arc = (1 - u) ** 2 * end + 2 * u * (1 - u) * knee + u**2 * out
        far = out + np.array([0.0, side * float(tail), 0.0])
        arc[:, 2] = np.linspace(end[2], radius, len(arc))
        far[2] = radius
        return np.vstack([arc, far])

    lower_first = knot[0, 1] < knot[-1, 1]
    head = tail_from(knot[0], knot[0] - knot[1], -1.0 if lower_first else 1.0)
    foot = tail_from(knot[-1], knot[-1] - knot[-2], 1.0 if lower_first else -1.0)
    path = np.vstack([head[::-1], knot[1:-1], foot])
    if not lower_first:
        path = path[::-1]
    path[:, :2] += np.asarray(at, dtype=np.float64) - 0.5 * (path[0, :2] + path[-1, :2])
    legs = np.linalg.norm(np.diff(path, axis=0), axis=1)
    s_ = np.concatenate([[0.0], np.cumsum(legs)])
    count = max(2, int(round(s_[-1] / float(spacing))) + 1)
    s_new = np.linspace(0.0, s_[-1], count)
    return np.stack([np.interp(s_new, s_, path[:, k]) for k in range(3)], axis=1)


SIZE, TAIL = 0.15, 0.05  # loops wide enough for the YAM's fingers to thread (RoboSimStudio's example ties 0.10)


def target(radius: float = 0.005, at=(-0.10, 0.0), size: float = 0.0, tail: float = 0.0) -> np.ndarray:
    """The tied knot in the bench frame: tails along y on the line x = at[0], the knot toward the robot of it."""
    return trefoil(at=at, size=size or SIZE, radius=radius, tail=tail or TAIL, spacing=0.008)


def start(radius: float = 0.005, at=(-0.10, 0.0), *, half_run: float = 0.20) -> np.ndarray:
    """The rope straight along y through ``at`` for ``half_run`` each side, both ends then bent toward the robot.

    Same length and node count as :func:`target`, so node k is the same point of rope in both; node 0 is the -y end.
    """
    goal = target(radius, at)
    length = float(arc_length(goal)[-1])
    rest = 0.5 * length - half_run
    r = 0.025
    x0, y0 = at
    # From the -y end (toward the robot), up to the line round a quarter turn, along it, and back down at +y.
    down = rest - 0.5 * math.pi * r + r  # the run toward the robot; the corners take a quarter turn each
    path = [np.array([x0 - r - down, y0 - half_run]), np.array([x0 - r, y0 - half_run])]
    for a in np.linspace(0.0, 0.5 * math.pi, 10)[1:]:
        path.append(np.array([x0 - r + r * math.sin(a), y0 - half_run + r - r * math.cos(a)]))
    path.append(np.array([x0, y0 + half_run - r]))
    for a in np.linspace(0.0, 0.5 * math.pi, 10)[1:]:
        path.append(np.array([x0 - r + r * math.cos(a), y0 + half_run - r + r * math.sin(a)]))
    path.append(np.array([x0 - r - down, y0 + half_run]))
    path = np.asarray(path)
    s = arc_length(np.hstack([path, np.zeros((len(path), 1))]))
    s_new = np.linspace(0.0, s[-1], len(goal))
    xy = np.stack([np.interp(s_new, s, path[:, k]) for k in range(2)], axis=1)
    return np.hstack([xy, np.full((len(xy), 1), radius)])


def is_overhand(code: list) -> bool:
    """Whether a long knot's Gauss code is a trefoil, of either handedness: three crossings met in the order 1 2 3 1 2 3, alternating."""
    if len(code) != 6:
        return False
    labels = [k for k, _ in code]
    return labels == [1, 2, 3, 1, 2, 3] and all(code[k][1] != code[k + 1][1] for k in range(5))
