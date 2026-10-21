"""Task instances for rehearsal (Variational Automation): the rope's physics, where the jig sits, how the rope was laid.

Train variants are what the self-learning loop rehearses and compares candidates on (paired: every
candidate in an iteration sees the same instances). Test variants are held out and only ever scored.
"""
from __future__ import annotations

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


def sample(seed: int) -> Variant:
    rng = np.random.default_rng(10_000 + seed)
    kw = {}
    for k, (kind, lo, hi) in RANGES.items():
        kw[k] = float(np.exp(rng.uniform(np.log(lo), np.log(hi)))) if kind == "log" else float(rng.uniform(lo, hi))
    return Variant(seed=seed, **kw)


def train(n: int, offset: int = 0) -> list[Variant]:
    return [sample(offset + k) for k in range(n)]


def test(n: int) -> list[Variant]:
    return [sample(100_000 + k) for k in range(n)]


if __name__ == "__main__":
    for v in train(3):
        print(v.to_json())
