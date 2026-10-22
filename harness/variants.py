"""Task instances for rehearsal (Variational Automation): the rope's physics, where the jig sits, how the rope was laid.

Train variants are what the self-learning loop rehearses and compares candidates on (paired: every
candidate in an iteration sees the same instances). Test variants are held out and only ever scored.
"""
from __future__ import annotations

import dataclasses

import numpy as np

from knotlab.task import Variant

RANGES = {
    "young_modulus": ("log", 0.5e5, 2.0e5),  # a softer or stiffer cord
    "mu_cable": ("lin", 0.6, 1.4),  # slipperier or grippier: friction 0.77-1.18
    "axial_stiffness": ("log", 200.0, 450.0),
    "board_x": ("lin", -0.150, -0.130),  # the jig shifted on the bench
    "board_y": ("lin", -0.015, 0.015),
    "board_yaw": ("lin", -0.08, 0.08),  # turned up to ~4.6 degrees
    "pin_jitter": ("lin", 0.0, 0.003),
    "splay": ("lin", 0.04, 0.06),
    "drop": ("lin", 0.12, 0.155),
    "side": ("lin", 0.135, 0.16),
    "up": ("lin", 0.13, 0.165),
    "fold": ("lin", 0.055, 0.08),
    "lay_noise": ("lin", 0.0, 0.006),
}


RANGES_OVERHAND = {
    # Real2Sim of the Howcast clip (video/overhand/REAL2SIM.md): E = 2e4 Pa matched the loose knot's settled
    # shape (silhouette IoU 0.71 vs 0.35 at 2e6); the clip cannot identify friction, so mu_cable keeps the range
    # the double coin's board set (a grip needs it: SuperDex has one friction for strands, pads and bench).
    "young_modulus": ("log", 1.0e4, 4.0e4),
    "mu_cable": ("lin", 0.6, 1.4),
    "axial_stiffness": ("log", 200.0, 450.0),
    "board_x": ("lin", -0.08, -0.06),  # the cord's line, bench x (world 0.52-0.54: the big knot lies toward the robot of it)
    "board_y": ("lin", -0.015, 0.015),
    "board_yaw": ("lin", -0.08, 0.08),
    "lay_noise": ("lin", 0.0, 0.006),
}
KNOT = "coin"  # which task the samplers draw instances of; the loop sets it


def sample(seed: int, knot: str | None = None) -> Variant:
    knot = knot or KNOT
    rng = np.random.default_rng(10_000 + seed)
    if knot == "overhand_tighten":  # the clip's step 3: pull a loose overhand knot tight
        v = sample(seed, "overhand")
        return dataclasses.replace(v, start_shape="loose_knot", knot_size=float(rng.uniform(0.13, 0.17)), lay_noise=0.0)
    kw = {"knot": knot}
    if knot == "overhand":
        kw["board_x"] = -0.07
    for k, (kind, lo, hi) in (RANGES_OVERHAND if knot == "overhand" else RANGES).items():
        kw[k] = float(np.exp(rng.uniform(np.log(lo), np.log(hi)))) if kind == "log" else float(rng.uniform(lo, hi))
    return Variant(seed=seed, **kw)


def train(n: int, offset: int = 0) -> list[Variant]:
    return [sample(offset + k) for k in range(n)]


def test(n: int) -> list[Variant]:
    return [sample(100_000 + k) for k in range(n)]


def nominal(knot: str | None = None) -> Variant:
    knot = knot or KNOT
    if knot == "overhand_tighten":
        return Variant(knot="overhand", board_x=-0.07, young_modulus=2.0e4, start_shape="loose_knot", knot_size=0.15)
    return Variant(knot=knot, board_x=-0.07, young_modulus=2.0e4) if knot == "overhand" else Variant()


if __name__ == "__main__":
    for v in train(3):
        print(v.to_json())
