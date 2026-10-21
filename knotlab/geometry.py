"""The double coin knot (双钱结) as a rope centreline, the board's pins, and the rope as it starts.

The double coin knot is the 3-lead 4-bight Turk's head (the Carrick mat) cut
open at one bight: r(t) = R0 + A cos(4t/3), theta = t, over t in [0, 6 pi), has
four bights (right, bottom, left, top) and eight crossings, met alternately over
and under. Cut at the bottom bight, its two ends become the tails and the rope's
middle sits at the top bight, as in the video (Art Sprouts, "How to tie a Double
Coin Knot", youtu.be/dg6XrRY_FzI), where the middle hangs on the top pin and the
two halves form the left and right lobes.

Frames: the board frame has X to the right as the robot sees it and Y away from
the robot; the bench (work-surface) frame of RoboSimStudio has +x away from the
robot and +y to the robot's left, so bench = (Y, -X).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from knotlab.diagram import arc_length, crossings


@dataclass(frozen=True)
class KnotSpec:
    span: float = 0.16  # lobe tip to lobe tip, left to right [m]: what the YAM pair reaches tool-down
    ratio: float = 0.5  # A / R0 of the Turk's head
    radius: float = 0.005  # the rope [m]: a 10 mm cord, the video's 4 mm cord scaled to the YAM's fingers
    tail: float = 0.10  # each tail's straight run beyond the knot [m]
    spacing: float = 0.008  # element length [m], under the rope's diameter
    cut: float = 0.55  # bottom bight removed, in parameter t around its apex [rad]
    tail_angle: float = 15.0  # each tail's heading below horizontal [deg]: out sideways, inside the reach
    pin_inset: float = 0.0  # each pin's offset from its lobe's eye centre toward the apex [m]
    lift: float = 2.0  # an over strand's axis this many radii over an under strand's
    centre: tuple[float, float] = (0.0, 0.0)  # knot centre in the board frame [m]
    extra: dict = field(default_factory=dict)


def _turks_head(spec: KnotSpec, t: np.ndarray) -> np.ndarray:
    # r = R0 + A cos(4t/3); outer reach R0 + A is half the span.
    r0 = 0.5 * spec.span / (1.0 + spec.ratio)
    r = r0 * (1.0 + spec.ratio * np.cos(4.0 * t / 3.0))
    return np.stack([r * np.cos(t), r * np.sin(t)], axis=1)


def _resample(path: np.ndarray, spacing: float) -> np.ndarray:
    s = arc_length(path)
    count = max(2, int(round(s[-1] / spacing)) + 1)
    s_new = np.linspace(0.0, s[-1], count)
    return np.stack([np.interp(s_new, s, path[:, k]) for k in range(path.shape[1])], axis=1)


def planar_path(spec: KnotSpec) -> np.ndarray:
    """The knot's centreline in the board plane, tail to tail, ``[n, 2]``, before heights."""
    t0 = 1.5 * math.pi + 0.5 * spec.cut  # just past the bottom bight's apex
    t1 = 1.5 * math.pi + 6.0 * math.pi - 0.5 * spec.cut
    t = np.linspace(t0, t1, 4000)
    body = _turks_head(spec, t)
    ang = math.radians(spec.tail_angle)
    u = np.linspace(0.0, 1.0, 200)[1:, None]

    def tail(end: np.ndarray, toward: np.ndarray) -> np.ndarray:
        # Leave the end along the curve's own heading, then bend onto ``toward``.
        heading = end[-1] - end[-2]
        heading /= np.linalg.norm(heading)
        knee = end[-1] + 0.04 * heading
        far = knee + spec.tail * toward
        bez = (1 - u) ** 2 * end[-1] + 2 * u * (1 - u) * knee + u**2 * (knee + 0.04 * toward)
        line = np.linspace(bez[-1], far, 60)[1:]
        return np.vstack([bez, line])

    # The start of t is the right leg of the cut bight (moving right/up), the end its left leg.
    start_tail = tail(body[:2][::-1] if False else np.vstack([body[1], body[0]]), np.array([math.cos(ang), -math.sin(ang)]))
    end_tail = tail(body[-2:], np.array([-math.cos(ang), -math.sin(ang)]))
    path = np.vstack([start_tail[::-1], body, end_tail])
    path += np.asarray(spec.centre)
    return _resample(path, spec.spacing)


def target(spec: KnotSpec = KnotSpec()) -> np.ndarray:
    """The tied knot lying on the board: ``[n, 3]`` in the board frame, z the rope axis height over the board."""
    xy = planar_path(spec)
    flat = np.hstack([xy, np.zeros((len(xy), 1))])
    s = arc_length(flat)
    found = crossings(flat)
    # Walk the rope: crossings met alternately over and under, the first met over.
    events = sorted([(c.s_i, k) for k, c in enumerate(found)] + [(c.s_j, k) for k, c in enumerate(found)])
    z = np.full(len(xy), spec.radius)
    width = 3.0 * spec.radius  # each over strand rises over this arc length either side
    for n, (s_e, _) in enumerate(events):
        if n % 2 == 0:  # over
            bump = spec.lift * spec.radius * np.clip(1.0 - np.abs(s - s_e) / width, 0.0, 1.0) ** 0.5
            z = np.maximum(z, spec.radius + bump)
    return np.hstack([xy, z[:, None]])


def pins(spec: KnotSpec = KnotSpec()) -> dict[str, np.ndarray]:
    """One pin in the eye of each of the top, left and right lobes (board frame).

    Along a bight's own ray the curve passes its apex at R0 + A and a crossing at
    R0 - A/2; the eye between them is centred at R0 + A/4.
    """
    r0 = 0.5 * spec.span / (1.0 + spec.ratio)
    eye = r0 * (1.0 + 0.25 * spec.ratio) + spec.pin_inset
    c = np.asarray(spec.centre)
    return {
        "top": c + np.array([0.0, eye]),
        "left": c + np.array([-eye, 0.0]),
        "right": c + np.array([eye, 0.0]),
    }


def _fillet(poly: np.ndarray, radius: float) -> np.ndarray:
    """Round every interior corner of a polyline with a circular-ish arc (quadratic Bezier) of about ``radius``."""
    out = [poly[0]]
    for k in range(1, len(poly) - 1):
        a, b, c = poly[k - 1], poly[k], poly[k + 1]
        da, dc = a - b, c - b
        r = min(radius, 0.5 * np.linalg.norm(da), 0.5 * np.linalg.norm(dc))
        p0, p2 = b + r * da / np.linalg.norm(da), b + r * dc / np.linalg.norm(dc)
        u = np.linspace(0.0, 1.0, 16)[:, None]
        out.extend(list((1 - u) ** 2 * p0 + 2 * u * (1 - u) * b + u**2 * p2))
    out.append(poly[-1])
    return np.asarray(out)


def start(
    spec: KnotSpec = KnotSpec(),
    *,
    splay: float = 0.05,
    drop: float = 0.14,
    side: float = 0.15,
    up: float = 0.16,
    fold: float = 0.07,
    turn: float = 0.025,
) -> np.ndarray:
    """The rope as the video starts it: its middle hung round the top pin, both halves down the board.

    The video's halves run straight down off the board; the YAM pair reaches a
    band only ~20 cm deep and 40 cm wide, so the whole cord is laid inside it,
    clear of where the knot goes: after ``drop`` each half turns out sideways for
    ``side``, runs back up the side of the board, folds in by ``fold`` and comes
    back down that corridor, stopping 4 cm above where it started out; the run up
    is solved so the half's length comes out exact (``up`` is the first guess).
    Corners are rounded on ``turn``.

    Same length and element count as :func:`target`, so node k is the same point of
    rope in both.
    """
    goal = target(spec)
    length = float(arc_length(goal)[-1])
    top = pins(spec)["top"]
    r = 0.5 * splay
    gap = 0.04  # the folded-back run stops this far above the bottom run

    def half(sign: float, up_: float) -> np.ndarray:
        phi = np.linspace(0.5 * math.pi, 0.5 * math.pi - sign * 0.5 * math.pi, 12)
        arc = top + np.stack([r * np.cos(phi), r * np.sin(phi)], axis=1)
        apex = top + np.array([0.0, r])
        corner1 = top + np.array([sign * r, -drop])
        corner2 = corner1 + np.array([sign * side, 0.0])
        corner3 = corner2 + np.array([0.0, up_])
        corner4 = corner3 + np.array([-sign * fold, 0.0])
        end = np.array([corner4[0], corner1[1] + gap])
        path = _fillet(np.vstack([arc[:-1], arc[-1], corner1, corner2, corner3, corner4, end]), turn)
        return np.vstack([apex[None], path[1:]])

    def plen(p: np.ndarray) -> float:
        return float(arc_length(np.hstack([p, np.zeros((len(p), 1))]))[-1])

    halves = []
    for sign in (+1.0, -1.0):
        # The run up the side is what is free: choose it so the half's length is exactly half the rope's
        # (its path length grows two for one with it, up and back down).
        u0, u1 = up, up + 0.05
        l0, l1 = plen(half(sign, u0)), plen(half(sign, u1))
        u = u0 + (0.5 * length - l0) * (u1 - u0) / (l1 - l0)
        for _ in range(2):
            lu = plen(half(sign, u))
            u += (0.5 * length - lu) / 2.0
        path = half(sign, u)
        s_ = arc_length(np.hstack([path, np.zeros((len(path), 1))]))
        keep = s_ <= 0.5 * length
        cut = int(np.argmax(~keep)) if (~keep).any() else len(path)
        if cut < len(path):
            end = path[cut - 1] + (0.5 * length - s_[cut - 1]) * (path[cut] - path[cut - 1]) / max(s_[cut] - s_[cut - 1], 1e-9)
            path = np.vstack([path[:cut], end[None]])
        halves.append(path)
    plus, minus = halves
    path = np.vstack([plus[::-1], minus[1:]])
    # Node 0 is the target's first tail; the target's first tail is at +X, so the +X half runs first, end to apex.
    plus, minus = halves
    path = np.vstack([plus[::-1], minus[1:]])
    if (goal[0, 0] > goal[-1, 0]) != (path[0, 0] > path[-1, 0]):
        path = path[::-1]
    xy = _resample(path, length / (len(goal) - 1))
    if len(xy) != len(goal):  # equal arc length, same count as the target
        s = arc_length(np.hstack([path, np.zeros((len(path), 1))]))
        s_new = np.linspace(0.0, s[-1], len(goal))
        xy = np.stack([np.interp(s_new, s, path[:, k]) for k in range(2)], axis=1)
    return np.hstack([xy, np.full((len(xy), 1), spec.radius)])


def to_bench(points: np.ndarray, board_origin=(0.0, 0.0, 0.0)) -> np.ndarray:
    """Board-frame points (X right, Y away, z up from the board top) to the bench frame (+x away, +y left)."""
    p = np.asarray(points, dtype=np.float64)
    out = np.stack([p[:, 1], -p[:, 0], p[:, 2] if p.shape[1] > 2 else np.zeros(len(p))], axis=1)
    return out + np.asarray(board_origin)
