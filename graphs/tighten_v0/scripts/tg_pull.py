"""Pull the loose overhand knot tight, hand over hand: take both tails near the knot, draw them apart, let go, repeat."""
import json
import math
import os

from knotlab import skills

HERE = os.path.dirname(os.path.abspath(__file__))


def _p() -> dict:
    with open(os.path.join(HERE, "..", "params.json")) as f:
        return json.load(f)


def _knot(ctx) -> dict:
    st = ctx.tool("rope.state")
    ss = [c["s_a"] for c in st["crossings"]] + [c["s_b"] for c in st["crossings"]]
    lo, hi = (min(ss), max(ss)) if ss else (None, None)
    return {"st": st, "lo": lo, "hi": hi, "length": float(st["length"]), "in_knot": (hi - lo) if ss else 0.0, "n": len(st["crossings"])}


def run(ctx) -> dict:
    p = _p()
    k0 = _knot(ctx)
    a = math.radians(p["angle_deg"])  # each tail pulled outward along y, turned this much toward the robot
    left_dir, right_dir = (-math.sin(a), math.cos(a)), (-math.sin(a), -math.cos(a))
    cycles = []
    for c in range(int(p["cycles"])):
        k = _knot(ctx)
        if k["lo"] is None or k["n"] != 3:
            return {"route": "lost", "cycles": cycles, "start_in_knot": k0["in_knot"]}
        # Node 0's tail is the robot's right (-y): the right hand takes it; the last node's tail the left hand.
        s_right = max(0.03, k["lo"] - p["grasp_from_knot"])
        s_left = min(k["length"] - 0.03, k["hi"] + p["grasp_from_knot"])
        r = skills.pull(ctx, (0, 1), s_left=s_left, s_right=s_right, direction_left=left_dir, direction_right=right_dir,
                        distance=p["distance"], step=p["step"], height=p["height"])
        k2 = _knot(ctx)
        cycles.append({"cycle": c, "pulled": r.get("pulled"), "held": r.get("held"), "in_knot": round(k2["in_knot"], 4), "n": k2["n"]})
        if k2["n"] == 3 and k2["in_knot"] <= p["goal_fraction"] * k0["in_knot"]:
            return {"route": "tight", "cycles": cycles, "start_in_knot": k0["in_knot"]}
    return {"route": "pulled", "cycles": cycles, "start_in_knot": k0["in_knot"]}
