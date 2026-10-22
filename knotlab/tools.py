"""``rope.*``: what a graph may ask about the rope and the board, registered as the task's own tools.

Privileged in sim (the rope's nodes come from the simulator); on a real cell the
same answers come from the top-down camera (``rope.*`` would be backed by a
centreline tracker) and from the jig's drawing (pins, the target layout).

Every position is in the WORLD frame, as ``robot.go_to_pose`` takes it. The
bench's work-surface origin is world (0.6, 0, 0.75).
"""

from __future__ import annotations

from typing import Any

import numpy as np

from knotlab import diagram, task

BENCH_ORIGIN = task.BENCH_ORIGIN


def _world(p) -> list[float]:
    return [float(x) for x in (np.asarray(p, dtype=np.float64) + BENCH_ORIGIN)]


class RopeTools:
    def __init__(self, env, bus) -> None:
        self.env, self.bus = env, bus

    def _nodes(self) -> np.ndarray:
        rope = self.env["rope"]
        return self.bus.call(lambda: np.asarray(rope.particle_q, dtype=np.float64).copy())

    def state(self) -> dict[str, Any]:
        """The rope now: its nodes (world), arc length, the knot diagram it lies in, and how much of the double coin it is."""
        p = self._nodes()
        bench = p - BENCH_ORIGIN
        s = diagram.arc_length(p)
        found = diagram.crossings(bench)
        sc = task.score(bench)
        return {
            "nodes": p.round(5).tolist(),
            "arc": s.round(5).tolist(),
            "length": float(s[-1]),
            "code": sc["code"],
            "target_code": diagram.code_str(task.target_code()),
            "progress": sc["progress"],
            "prefix": sc["prefix"],
            "tied": sc["tied"],
            "knot_length": sc["knot_length"],
            "max_height": sc["max_height"],
            "crossings": [
                {
                    "s_a": c.s_i,
                    "s_b": c.s_j,
                    "a_over": c.i_over,
                    "gap": c.gap,
                    "at": _world([*c.xy, 0.0])[:2],
                }
                for c in sorted(found, key=lambda c: c.s_i)
            ],
            "ends": {"first": p[0].round(5).tolist(), "last": p[-1].round(5).tolist()},
        }

    def layout(self) -> dict[str, Any]:
        """The jig and the plan: the pins (world), and where each node of the rope lies in the tied knot (world)."""
        lay = task.CURRENT["layout"]
        tgt = np.asarray(lay["target"])
        return {
            "pins": {k: _world(v) for k, v in lay["pins"].items()},
            "pin_radius": task.PIN_RADIUS,
            "pin_height": task.PIN_HEIGHT,
            "target_nodes": [_world(x) for x in tgt],
            "target_arc": diagram.arc_length(tgt).round(5).tolist(),
            "target_code": diagram.code_str(task.target_code()),
            "rope_radius": float(task.CURRENT["variant"].radius),
            "board_centre": _world(lay["board_centre"]),
        }

    def point(self, s: float) -> dict[str, Any]:
        """Where the rope is ``s`` metres from its first end now, and its heading there (deg, world xy)."""
        p = self._nodes()
        arc = diagram.arc_length(p)
        s = float(np.clip(s, 0.0, arc[-1]))
        k = int(np.clip(np.searchsorted(arc, s) - 1, 0, len(p) - 2))
        u = (s - arc[k]) / max(arc[k + 1] - arc[k], 1e-9)
        at = p[k] + u * (p[k + 1] - p[k])
        lo, hi = max(k - 2, 0), min(k + 3, len(p) - 1)
        t = p[hi] - p[lo]
        return {"position": at.round(5).tolist(), "heading_deg": float(np.degrees(np.arctan2(t[1], t[0]))), "node": k}


def register(registry, env, bus) -> None:
    tools = RopeTools(env, bus)
    registry.register_callable("rope.state", tools.state, summary=tools.state.__doc__.splitlines()[0], tags=("rope",))
    registry.register_callable("rope.layout", tools.layout, summary=tools.layout.__doc__.splitlines()[0], tags=("rope",))
    registry.register_callable("rope.point", tools.point, summary=tools.point.__doc__.splitlines()[0], tags=("rope",))
