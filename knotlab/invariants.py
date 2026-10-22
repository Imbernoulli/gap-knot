"""Knot invariants of a rope lying on the table, so "is it a knot" is not a matter of looking at a picture.

The rope is closed by joining its two ends outside the diagram (its tails run outward on opposite sides, so the
closing arc crosses nothing), and the knot determinant is computed from the Fox colouring matrix of that diagram:
1 for the unknot (any diagram where the rope only ever lies on top of what came before is the unknot), 3 for the
trefoil (the overhand knot), 5 for the figure-eight, and so on.
"""
from __future__ import annotations

import numpy as np

from knotlab import diagram


def determinant(points: np.ndarray) -> int:
    """|det| of the reduced Fox colouring matrix of the rope's diagram, ends joined outside: 1 = unknot, 3 = trefoil."""
    p = np.asarray(points, dtype=np.float64)
    found = diagram.crossings(p)
    n = len(found)
    if n == 0:
        return 1
    # Events along the rope: (arc length, crossing index, over?)
    events = sorted([(c.s_i, k, c.i_over) for k, c in enumerate(found)] + [(c.s_j, k, not c.i_over) for k, c in enumerate(found)])
    unders = [i for i, e in enumerate(events) if not e[2]]
    if not unders:  # the rope is never under anything: a descending diagram, the unknot
        return 1
    # Arcs run from one under-passage to the next (cyclically, the ends being joined); arc a starts after under #a.
    m = len(unders)
    arc_of_event = {}
    for i in range(len(events)):
        # the arc an event lies on: the last under-passage at or before it (cyclically)
        prev = [u for u in unders if u < i]
        a = (len(prev) - 1) % m if prev else m - 1
        arc_of_event[i] = a
    M = np.zeros((n, m), dtype=np.int64)
    for k in range(n):
        over_i = next(i for i, e in enumerate(events) if e[1] == k and e[2])
        under_i = next(i for i, e in enumerate(events) if e[1] == k and not e[2])
        u = unders.index(under_i)
        incoming = (u - 1) % m  # the arc arriving at this under-passage
        outgoing = u            # the arc leaving it
        M[k, arc_of_event[over_i]] += 2
        M[k, incoming] -= 1
        M[k, outgoing] -= 1
    if n < 2:
        return 1
    minor = M[1:, 1:].astype(float)
    return int(round(abs(np.linalg.det(minor)))) if minor.size else 1
