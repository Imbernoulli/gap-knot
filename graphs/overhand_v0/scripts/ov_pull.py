"""Both hands take the tails and draw them apart to tighten the knot."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import params, state
from knotlab import skills


def run(ctx) -> dict:
    p = params()["pull"]
    st = ctx.tool("rope.state")
    L = float(st["length"])
    first, last = st["nodes"][0], st["nodes"][-1]
    # Pull each tail away from the other, along the line between the two ends.
    dx, dy = last[0] - first[0], last[1] - first[1]
    n = (dx * dx + dy * dy) ** 0.5 or 1.0
    away_last, away_first = (dx / n, dy / n), (-dx / n, -dy / n)
    left_is_last = last[1] > first[1]  # arm 0 (left, +y) takes the end on the +y side
    r = skills.pull(
        ctx, (0, 1),
        s_left=(L - p["end_back"]) if left_is_last else p["end_back"],
        s_right=p["end_back"] if left_is_last else (L - p["end_back"]),
        direction_left=away_last if left_is_last else away_first,
        direction_right=away_first if left_is_last else away_last,
        distance=p["distance"], step=p["step"], height=p["height"],
    )
    return {"route": "pulled", "report": {k: v for k, v in r.items() if k != "rot"}, "state": state(ctx)}
