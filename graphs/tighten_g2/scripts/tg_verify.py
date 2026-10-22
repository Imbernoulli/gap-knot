"""Judge the settled knot hands-off, then retry only intact, loose knots."""
import json
import math
import os


def run(ctx, cycle: int, pull_used_steps: int) -> dict:
    with open(os.path.join(os.path.dirname(__file__), "..", "params.json")) as f:
        p = json.load(f)
    # pull has already set down, fully opened both hands, risen and settled.
    before = ctx.tool("rope.state")
    ctx.tool("robot.wait_steps", steps=p["settle_steps"])
    st = ctx.tool("rope.state")
    # Separate input/output names prevent this node's preceding visit from
    # shadowing the newer pull counter in the subgraph input binding.
    used_steps = pull_used_steps + int(math.ceil(1.10 * p["settle_steps"])) + 9
    # If tightening only became apparent during settling, give that state its
    # own hands-off second before accepting it, even on the last cycle.
    if st["tied"] and not before["tied"]:
        ctx.tool("robot.wait_steps", steps=p["settle_steps"])
        before, st = st, ctx.tool("rope.state")
        used_steps += int(math.ceil(1.10 * p["settle_steps"])) + 6
    tokens = st["code"].split()
    intact = (len(st["crossings"]) == 3 and len(tokens) == 6
              and [t[:-1] for t in tokens] == ["1", "2", "3", "1", "2", "3"]
              and all(tokens[i][-1] != tokens[i + 1][-1] for i in range(5)))
    if before["tied"] and st["tied"]:
        route = "tight"
    elif intact and cycle < p["cycles"] and used_steps < p["cycle_start_budget"]:
        route = "retry"
    else:
        route = "loose"
    return {"route": route, "used_steps": used_steps, "code": st["code"],
            "tied_before": before["tied"], "tied_after": st["tied"], "cycle": cycle}
