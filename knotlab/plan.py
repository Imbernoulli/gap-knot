"""A laying plan for the double coin knot: which point of the rope goes where, in what order, over or under.

The rope's middle hangs on the top pin; each half (A: node 0's, the robot's
right in the start; B: the other) is laid outward from it, point by point. A
crossing is made when the second of its two strands is laid. A tool-down hand
carrying cord over cord makes it cross OVER; it cannot pass cord UNDER a strand
anchored at both ends (its own column hits the strand), so every crossing whose
later-laid strand must be under is an ``under`` step, done by threading the free
end (``skills.thread_end``) with the other hand lifting the strand.

Which half is laid how far, and when, decides how many crossings are under
steps. :func:`schedule` searches every interleaving of the two halves' crossing
locations for the fewest; for the double coin knot it is 2 (one is a crossing of
half A with itself, which no order avoids).

Pure functions of ``rope.layout``'s answer; no ``ctx``.
"""

from __future__ import annotations

import itertools
from dataclasses import asdict, dataclass, field

import numpy as np

from knotlab import diagram

BENCH_ORIGIN = np.array([0.6, 0.0, 0.75])
REACH_X = (-0.25, -0.05)  # bench x both YAM hands reach tool-down [m]
REACH_Y = (-0.20, 0.20)


@dataclass(frozen=True)
class Location:
    """One of a crossing's two strands: where along the rope, which half, how far from the middle."""

    crossing: int  # index into the target's crossings
    s: float
    half: str  # "A" (s < L/2) or "B"
    d: float  # arc length from the rope's middle [m]
    over: bool  # this strand is the over one
    xy: tuple[float, float]  # world


@dataclass
class Step:
    k: int
    half: str
    s: float  # the rope point this step carries [m from node 0]
    d: float
    xy: tuple[float, float]  # where it goes (world)
    arm_id: int  # 0 left, 1 right
    makes: list = field(default_factory=list)  # crossings made by this step
    kind: str = "lay"  # "lay" (no crossing, or all over) or "thread" (an under crossing)
    helper_arm: int | None = None
    lift_s: float | None = None  # the strand the helper lifts, for a thread step
    reachable: bool = True

    def to_dict(self) -> dict:
        return asdict(self)


def target_locations(target_nodes, target_arc=None) -> tuple[list[Location], float]:
    """Every crossing location of the tied knot, from its world-frame nodes."""
    p = np.asarray(target_nodes, dtype=np.float64)
    arc = diagram.arc_length(p) if target_arc is None else np.asarray(target_arc, dtype=np.float64)
    length = float(arc[-1])
    mid = 0.5 * length
    locs: list[Location] = []
    for k, c in enumerate(sorted(diagram.crossings(p - BENCH_ORIGIN), key=lambda c: c.s_i)):
        xy = (float(c.xy[0] + BENCH_ORIGIN[0]), float(c.xy[1] + BENCH_ORIGIN[1]))
        for s, over in ((c.s_i, c.i_over), (c.s_j, not c.i_over)):
            locs.append(Location(k, float(s), "A" if s < mid else "B", abs(float(s) - mid), bool(over), xy))
    return locs, length


def _hard(order: list[Location]) -> list[int]:
    """Crossings whose later-laid location is the under one, for a laying order of locations."""
    seen: dict[int, Location] = {}
    hard = []
    for loc in order:
        if loc.crossing in seen and not loc.over:
            hard.append(loc.crossing)
        seen[loc.crossing] = loc
    return hard


def schedule(locs: list[Location]) -> tuple[list[Location], list[int]]:
    """The interleaving of the two halves' locations (each half outward from the middle) with fewest under steps.

    Ties go to the fewest switches between halves (each switch is a hand-over).
    """
    a = sorted([x for x in locs if x.half == "A"], key=lambda x: x.d)
    b = sorted([x for x in locs if x.half == "B"], key=lambda x: x.d)
    best = None
    for picks in itertools.combinations(range(len(a) + len(b)), len(a)):
        chosen, ia, ib = set(picks), 0, 0
        order = []
        for i in range(len(a) + len(b)):
            if i in chosen:
                order.append(a[ia])
                ia += 1
            else:
                order.append(b[ib])
                ib += 1
        hard = _hard(order)
        switches = sum(order[i].half != order[i + 1].half for i in range(len(order) - 1))
        key = (len(hard), switches)
        if best is None or key < best[0]:
            best = (key, order, hard)
    return best[1], best[2]


def _phases(order: list[Location], length: float) -> list[tuple[str, float]]:
    """Runs of one half in ``order``, as (half, lay up to distance d from the middle)."""
    phases: list[tuple[str, float]] = []
    for i, loc in enumerate(order):
        if phases and phases[-1][0] == loc.half:
            continue
        # This half runs until the next location of the other half; it lays up to midway to its own next location.
        j = i
        while j + 1 < len(order) and order[j + 1].half == loc.half:
            j += 1
        own_next = next((x.d for x in order[j + 1 :] if x.half == loc.half), None)
        upto = 0.5 * (order[j].d + own_next) if own_next is not None else 0.5 * length
        phases.append((loc.half, upto))
    return phases


def _arm_for(xy, centre_y: float, previous: int | None) -> int:
    off = xy[1] - centre_y
    if off > 0.02:
        return 0
    if off < -0.02:
        return 1
    return previous if previous is not None else (0 if off >= 0 else 1)


def _reachable(xy) -> bool:
    bx, by = xy[0] - BENCH_ORIGIN[0], xy[1] - BENCH_ORIGIN[1]
    return REACH_X[0] - 0.01 <= bx <= REACH_X[1] + 0.01 and REACH_Y[0] <= by <= REACH_Y[1]


def lay_plan(layout: dict, *, spacing: float = 0.05, after: float = 0.025, start_d: float = 0.05) -> dict:
    """The plan: ordered placement steps for both halves, each with the crossings it makes.

    Args:
        layout: ``rope.layout``'s answer (``target_nodes`` world, ``target_arc``, ``pins``, ``board_centre``).
        spacing: Arc length between placed points within a phase [m].
        after: A step is put this far past each crossing location, so a crossing is
            set down right after it is made [m].
        start_d: The first ``start_d`` of each half from the middle is the bight on
            the top pin, in place from the start [m].
    """
    tgt = np.asarray(layout["target_nodes"], dtype=np.float64)
    arc = np.asarray(layout.get("target_arc") or diagram.arc_length(tgt), dtype=np.float64)
    locs, length = target_locations(tgt, arc)
    mid = 0.5 * length
    order, hard = schedule(locs)
    phases = _phases(order, length)
    centre_y = float(layout.get("board_centre", [0.6, 0.0, 0.75])[1])

    def xy_at(s: float) -> tuple[float, float]:
        return (float(np.interp(s, arc, tgt[:, 0])), float(np.interp(s, arc, tgt[:, 1])))

    laid = {"A": start_d, "B": start_d}  # how far out each half is laid
    steps: list[Step] = []
    last_arm: dict[str, int | None] = {"A": None, "B": None}
    made: set[int] = set()
    for half, upto in phases:
        d0 = laid[half]
        ds = list(np.arange(d0 + spacing, upto + 1e-9, spacing))
        ds += [x.d + after for x in locs if x.half == half and d0 < x.d <= upto]
        ds.append(upto)
        ds = sorted(d for d in ds if d0 < d <= upto + after + 1e-9)
        pruned: list[float] = []
        for d in ds:
            if pruned and d - pruned[-1] < 0.6 * spacing and d != ds[-1]:
                continue
            pruned.append(d)
        prev = d0
        for d in pruned:
            s = mid - d if half == "A" else mid + d
            s = float(np.clip(s, 0.0, length))
            xy = xy_at(s)
            arm = _arm_for(xy, centre_y, last_arm[half])
            last_arm[half] = arm
            step = Step(k=len(steps), half=half, s=round(s, 4), d=round(d, 4), xy=(round(xy[0], 4), round(xy[1], 4)), arm_id=arm)
            step.reachable = _reachable(xy)
            for loc in locs:
                if loc.half != half or not (prev < loc.d <= d) or loc.crossing in made:
                    continue
                partner = next(x for x in locs if x.crossing == loc.crossing and x is not loc)
                if partner.d > laid[partner.half] + 1e-9:
                    continue  # the partner is not down yet: the crossing is made when it is
                made.add(loc.crossing)
                step.makes.append(
                    {"crossing": loc.crossing, "working_over": loc.over, "at": [round(v, 4) for v in loc.xy], "other_s": round(partner.s, 4)}
                )
                if not loc.over:
                    step.kind = "thread"
                    step.helper_arm = 1 - arm
                    step.lift_s = round(partner.s, 4)
            steps.append(step)
            laid[half] = max(laid[half], d)
            prev = d
    return {
        "length": round(length, 4),
        "phases": [(h, round(u, 4)) for h, u in phases],
        "under_crossings": hard,
        "steps": [s.to_dict() for s in steps],
    }
