"""The double coin knot as a RoboSimStudio task: the board, its pins, the rope, and when it counts as tied.

Real2Sim of youtu.be/dg6XrRY_FzI (Art Sprouts, "How to tie a Double Coin Knot"):
a foam board with pins, the cord's middle hung on the top pin and its halves
down the board, a lobe laid round each side pin and the tails woven through.
Scaled to the YAM pair: a 10 mm cord, the knot 16 cm across, the board in the
band both hands reach tool-down (bench x -0.25 .. -0.05).

A :class:`Variant` is one instance of the task (Variational Automation): the
rope's stiffness and friction, where the board and its pins sit, how the rope
was laid out. ``build(variant)`` registers the assets for it and returns the
``bench.Env``.
"""

from __future__ import annotations

import contextlib
import dataclasses
import json
import math
import os
from dataclasses import dataclass

import numpy as np

from knotlab import diagram, geometry, overhand

GRAPH_ROOT_DEFAULT = None


@dataclass(frozen=True)
class Variant:
    seed: int = 0
    knot: str = "coin"  # "coin": the double coin knot (双钱结) on the pinned board; "overhand": the overhand knot (单结)
    # The rope.
    radius: float = 0.005
    young_modulus: float = 1.0e5  # [Pa]: a soft cord (the knot example's)
    axial_stiffness: float = 300.0  # [N]
    mu_cable: float = 1.0  # friction coefficient squared (SuperDex: one friction for strands, pads and bench).
    # Real2Sim: in the video a cord laid on the foam board stays put; at 0.25 placements sprang back 2-6 cm, at 1.0 1-3 mm.
    # Where things are, in the bench frame.
    board_x: float = -0.14  # the knot's centre, bench x [m]
    board_y: float = 0.0
    board_yaw: float = 0.0  # [rad]
    pin_jitter: float = 0.0  # each pin moved up to this far from its nominal place [m]
    # How the rope starts.
    splay: float = 0.05  # the bight's width round the top pin [m]
    drop: float = 0.14  # how far each half runs down before turning out [m]
    side: float = 0.15  # how far it then runs out sideways [m]
    up: float = 0.16  # then back up the side of the board [m]
    fold: float = 0.07  # then in, and back down that corridor with the rest [m]
    lay_noise: float = 0.0  # lateral wiggle laid onto the start [m]
    start_shape: str = "omega"  # "omega": the video's start; "tied": the loose knot itself (to rehearse dressing)
    knot_size: float = 0.0  # overhand "loose_knot" start: the loose knot's size [m] (0: the reference's)

    def to_json(self) -> str:
        return json.dumps(dataclasses.asdict(self))

    @staticmethod
    def from_json(text: str) -> "Variant":
        return Variant(**json.loads(text))


NOMINAL = Variant()
BOARD_THICK = 0.001  # the foam board, drawn only: SuperDex's rope rests on the bench, not on a slab [m]
PIN_RADIUS, PIN_HEIGHT = 0.0025, 0.050  # the video's pins stand ~5 cord diameters; at 30 mm a cord carried at 35 mm slid over one


def knot_spec(v: Variant) -> geometry.KnotSpec:
    return geometry.KnotSpec(radius=v.radius)


def _place(v: Variant, board_pts: np.ndarray) -> np.ndarray:
    """Board-frame points to the bench frame for this variant (knot centre at (board_x, board_y), turned by yaw)."""
    p = np.asarray(board_pts, dtype=np.float64)
    bench = geometry.to_bench(p)  # (Y, -X, z)
    c, s = math.cos(v.board_yaw), math.sin(v.board_yaw)
    xy = bench[:, :2] @ np.array([[c, s], [-s, c]])
    out = np.hstack([xy + np.array([v.board_x, v.board_y]), bench[:, 2:3]])
    return out


def _layout_overhand(v: Variant) -> dict:
    """The overhand knot: no pins; the rope straight along y with its ends bent toward the robot, the knot to tie
    toward the robot of that line. ``board_x``/``board_y`` place the line's middle (bench frame), ``board_yaw`` turns it."""
    rng = np.random.default_rng(v.seed)
    at = (0.0, 0.0)
    start = overhand.start(v.radius, at)
    tgt = overhand.target(v.radius, at)
    if v.start_shape == "loose_knot":  # the clip's step 3: a loose overhand knot to pull tight (tails 12 cm to take)
        tgt = overhand.target(v.radius, at, size=v.knot_size or overhand.SIZE, tail=0.12)
        start = tgt.copy()
    if v.start_shape == "pre_thread":  # the under-crossing segment rehearsed alone (knotlab.thread_lab)
        from knotlab import thread_lab

        start = thread_lab.pre_thread(tgt, v.radius)
    if v.lay_noise > 0 and v.start_shape not in ("pre_thread", "loose_knot"):
        s = diagram.arc_length(start)
        wiggle = v.lay_noise * np.sin(2 * math.pi * s / rng.uniform(0.2, 0.4) + rng.uniform(0, 2 * math.pi))
        tangent = np.gradient(start[:, :2], axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
        start = start.copy()
        start[:, :2] += wiggle[:, None] * normal
        s2 = diagram.arc_length(start)
        s_new = np.linspace(0.0, s2[-1], len(start))
        start = np.stack([np.interp(s_new, s2, start[:, k]) for k in range(3)], axis=1)
    c, sn = math.cos(v.board_yaw), math.sin(v.board_yaw)
    R = np.array([[c, -sn], [sn, c]])

    def place(p: np.ndarray) -> np.ndarray:
        q = np.asarray(p, dtype=np.float64).copy()
        q[:, :2] = q[:, :2] @ R.T + np.array([v.board_x, v.board_y])
        return q

    return {"start": place(start), "target": place(tgt), "pins": {}, "board_centre": np.array([v.board_x, v.board_y, 0.0])}


def layout(v: Variant) -> dict:
    """Everything this variant places, in the bench frame: rope start nodes, target, pins."""
    if v.knot == "overhand":
        return _layout_overhand(v)
    rng = np.random.default_rng(v.seed)
    spec = knot_spec(v)
    start = geometry.start(spec, splay=v.splay, drop=v.drop, side=v.side, up=v.up, fold=v.fold)
    if v.start_shape == "tied":
        start = geometry.target(spec)
    if v.lay_noise > 0:
        s = diagram.arc_length(start)
        wiggle = v.lay_noise * np.sin(2 * math.pi * s / rng.uniform(0.3, 0.6) + rng.uniform(0, 2 * math.pi))
        tangent = np.gradient(start[:, :2], axis=0)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
        far = np.clip((s - 0.1) / 0.2, 0, 1) * np.clip((s[-1] - s - 0.1) / 0.2, 0, 1)  # not near the ends
        mid = np.abs(s - s[-1] / 2) > 0.12  # leave the bight on its pin
        start = start.copy()
        start[:, :2] += (wiggle * far * mid)[:, None] * normal
        # Even spacing again (the wiggle stretched some elements and squeezed others), same node count.
        s2 = diagram.arc_length(start)
        s_new = np.linspace(0.0, s2[-1], len(start))
        start = np.stack([np.interp(s_new, s2, start[:, k]) for k in range(3)], axis=1)
    pins = geometry.pins(spec)
    pins = {k: p + (rng.uniform(-1, 1, 2) * v.pin_jitter if v.pin_jitter else 0.0) for k, p in pins.items()}
    pin_bench = {k: _place(v, np.array([[*p, 0.0]]))[0] for k, p in pins.items()}
    return {
        "start": _place(v, start),
        "target": _place(v, geometry.target(spec)),
        "pins": pin_bench,
        "board_centre": np.array([v.board_x, v.board_y, 0.0]),
    }


TARGET_CODE = diagram.gauss_code(geometry.target(geometry.KnotSpec()))
TARGET_CODES = {"coin": TARGET_CODE, "overhand": diagram.gauss_code(overhand.target())}


def target_code(knot: str | None = None) -> list:
    knot = knot or (CURRENT.get("variant").knot if CURRENT.get("variant") is not None else "coin")
    return TARGET_CODES[knot]


def score(points: np.ndarray, radius: float = 0.005, knot: str | None = None) -> dict:
    """How far the rope is from the tied knot: its code against the target knot's, and the knot's size."""
    tc = target_code(knot)
    code = diagram.gauss_code(points)
    n = diagram.prefix_match(code, tc)
    exact = code == tc
    if (knot or (CURRENT.get("variant").knot if CURRENT.get("variant") is not None else "coin")) == "overhand":
        exact = overhand.is_overhand(code)  # either handedness is an overhand knot
        v_ = CURRENT.get("variant")
        if v_ is not None and v_.start_shape == "loose_knot" and CURRENT.get("L0"):
            # Pulling it tight: still an overhand knot, with its rope drawn up to a fraction of the loose knot's.
            exact = exact and diagram_knot_length(points) <= TIGHT_FRACTION * CURRENT["L0"]
    found = diagram.crossings(points)
    xy = np.array([c.xy for c in found]) if found else np.zeros((0, 2))
    return {
        "code": diagram.code_str(code),
        "crossings": len(found),
        "prefix": n,
        "progress": n / len(tc),
        "knot_length": diagram_knot_length(points),
        "tied": bool(exact),
        "alternating": diagram.is_alternating(code),
        "knot_extent": float(np.ptp(xy, axis=0).max()) if len(xy) > 1 else 0.0,
    }


def diagram_knot_length(points: np.ndarray) -> float:
    """The rope between its first crossing and its last, along it [m]: what the knot ties up (0 with no crossing)."""
    found = diagram.crossings(points)
    if not found:
        return 0.0
    segs = [k for c in found for k in (c.i, c.j)]
    p = np.asarray(points, dtype=np.float64)
    lo, hi = min(segs), max(segs)
    return float(np.linalg.norm(np.diff(p[lo : hi + 2], axis=0), axis=1).sum())


TIGHT_FRACTION = float(os.environ.get("KNOTLAB_TIGHT", "0.75"))
"""A pulled-tight overhand knot holds at most this fraction of the loose knot's rope between its crossings."""

_REGISTERED: set[str] = set()


def _register_measure() -> None:
    from robosimstudio.assetlib.semantics.atoms import ArgSpec, AtomSpec, register_atom
    from robosimstudio.assetlib.semantics.handlers import handler

    if "coin_progress" in _REGISTERED:
        return
    # coin_progress scores whichever knot the current variant ties (the name predates the overhand knot).

    def coin_progress(positions: np.ndarray) -> float:
        # 1.0 only when the knot counts as tied (for the overhand knot: either handedness, and drawn tight when the
        # episode starts from a loose knot); otherwise the code matched so far, kept below 1.
        sc = score(np.asarray(positions, dtype=np.float64))
        return 1.0 if sc["tied"] else min(sc["progress"], 0.99)

    with contextlib.suppress(ValueError):
        register_atom(
            AtomSpec(
                name="coin_progress",
                summary="How much of the double coin knot's Gauss code a rope shows, from its first tail: 1 is tied.",
                fn=coin_progress,
                args=(ArgSpec(name="body", substrates=("rod",)),),
                directions=("check", "observe"),
                returns=("coin_progress",),
                provenance="knotlab: the open 3-lead 4-bight Turk's head, eight crossings met alternately over and under.",
            )
        )

        @handler("coin_progress", substrates=("rod",), summary="The double coin knot's code matched so far, 0..1.")
        def _h(positions: np.ndarray) -> dict:
            return {"coin_progress": coin_progress(positions)}

    _REGISTERED.add("coin_progress")


def task_doc(max_steps: int = 6000, knot: str = "coin") -> str:
    # $KNOTLAB_GOAL_HOLD lengthens the hold, so a rehearsal that starts tied (a dressing probe) is not
    # scored done on its first second.
    hold = float(os.environ.get("KNOTLAB_GOAL_HOLD", "1.0"))
    if knot == "overhand":
        name = "overhand_knot"
        language = "tie an overhand knot in the cord lying on the board: make a loop, pass the end through it, and pull it tight"
        fixtures = "(:fixtures board - knot_board)"
    else:
        name = "double_coin_knot"
        language = ("tie a double coin knot: hang the cord's middle on the top pin, lay a lobe round each side pin, "
                    "weave the tails over and under, and pull it even")
        fixtures = "(:fixtures board - knot_board pin_top - knot_pin_top pin_left - knot_pin_left pin_right - knot_pin_right)"
    return f"""
(define (problem {name})
  (:language {language})
  (:staging (:scene bench_075) (:embodiment yam_bimanual) (:physics knot_superdex@v1) (:max-steps {max_steps}))
  {fixtures}
  (:objects rope - knot_rope)
  (:goal (once (hold-for {hold} (>= (coin_progress rope) 1.0)))))
"""


CURRENT: dict = {}
"""The variant the last :func:`build` made, and its layout: what the ``rope.*`` tools report."""


def build(v: Variant = NOMINAL, *, max_steps: int = 6000):
    """Register this variant's assets and return its ``bench.Env``."""
    import robosimstudio as rss
    import robosimstudio_bench as bench
    from robosimstudio.assetlib import register_asset
    from robosimstudio.domains.rigid import Body
    from robosimstudio_materials.rod import Rod

    _register_measure()
    engine = rss.backend("superdex", substeps=2, feedback="off")  # the fingers drive the rope by pose
    physics = rss.replace(rss.GENERIC, backends=(("rod", engine), *rss.GENERIC.backends))
    with contextlib.suppress(ValueError):
        bench.register_arrangement("knot_superdex@v1", physics)
    lay = layout(v)
    CURRENT.clear()
    CURRENT.update(variant=v, layout=lay, L0=diagram_knot_length(lay["start"]))
    bx, by = float(v.board_x), float(v.board_y)

    def reg(record) -> None:
        # A variant re-registers under the same names: the latest wins.
        with contextlib.suppress(ValueError):
            register_asset(record)
        try:
            register_asset(record, replace=True)  # type: ignore[call-arg]
        except TypeError:
            pass

    board = rss.slab(
        name="board", half=(0.12, 0.17, BOARD_THICK / 2), at=(bx, by, BOARD_THICK / 2), color=(0.80, 0.82, 0.86)
    )
    reg(Body.from_spec(board, name="knot_board"))
    for name, p in lay["pins"].items():
        pin = rss.post(
            name=f"pin_{name}",
            radius=PIN_RADIUS,
            half_height=PIN_HEIGHT / 2,
            at=(float(p[0]), float(p[1]), BOARD_THICK + PIN_HEIGHT / 2),
            color=(0.10, 0.55, 0.50),
        )
        reg(Body.from_spec(pin, name=f"knot_pin_{name}"))
    rope = rss.rod(
        name="rope",
        radius=v.radius,
        nodes=tuple(map(tuple, lay["start"].tolist())),
        self_contact=True,
        young_modulus=v.young_modulus,
        axial_stiffness=v.axial_stiffness,
        mu_cable=v.mu_cable,
        friction_falloff=1.0e-3,
        contact_penalty=4.0e8,
        skin_samples="P1Q6",
        color=(0.85, 0.15, 0.45),
    )
    reg(Rod.from_spec(rope, name="knot_rope"))
    return bench.Env(bench.task(task_doc(max_steps, v.knot)))
