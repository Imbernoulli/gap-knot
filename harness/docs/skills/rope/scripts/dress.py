"""Both hands draw the tails apart in small steps to dress the knot, stopping if a crossing starts to go."""
from typing import TypedDict

from gap import NodeContext

from knotlab import skills


class Output(TypedDict):
    outcome: str  # "dressed" | "not_held" | "progress_drop"
    progress: float
    report: dict


def run(ctx: NodeContext, layout: dict, distance: float = 0.04, step: float = 0.01, inset: float = 0.04) -> Output:
    import math
    st = ctx.tool("rope.state")
    length = st["length"]
    c = layout["board_centre"]
    def away(s):
        p = ctx.tool("rope.point", s=s)["position"]
        dx, dy = p[0] - c[0], p[1] - c[1]
        n = math.hypot(dx, dy) or 1.0
        return (dx / n, dy / n)
    r = skills.pull(ctx, (0, 1), s_left=length - inset, s_right=inset, direction_left=away(length - inset),
                    direction_right=away(inset), distance=distance, step=step)
    out = "dressed" if r.get("ok") else (r.get("stopped") or r.get("failure", "failed"))
    return {"outcome": out, "progress": float(r.get("after", {}).get("progress", 0.0)), "report": r}
