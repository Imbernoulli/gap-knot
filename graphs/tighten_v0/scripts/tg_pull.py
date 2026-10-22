"""Pull the loose overhand knot tight, hand over hand: take both tails, draw them apart, let go, repeat.

The grasp is the one measured to hold this cord (runs/probe/E_1): open to 45 mm, down to the tool centre 13.5 mm
over the bench, a ramped 0.9 close, a slow straight lift; the pull is slow straight moves of both hands in turn.
"""
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
BZ = 0.75


def _p() -> dict:
    with open(os.path.join(HERE, "..", "params.json")) as f:
        return json.load(f)


def _pose(xyz, rot):
    return {"position": {"x": xyz[0], "y": xyz[1], "z": xyz[2]}, "rotation": rot}


def _knot(ctx) -> dict:
    st = ctx.tool("rope.state")
    ss = [c["s_a"] for c in st["crossings"]] + [c["s_b"] for c in st["crossings"]]
    lo, hi = (min(ss), max(ss)) if ss else (None, None)
    return {"lo": lo, "hi": hi, "length": float(st["length"]), "in_knot": (hi - lo) if ss else 0.0, "n": len(st["crossings"])}


def _grasp(ctx, arm, s, p) -> dict:
    pt = ctx.tool("rope.point", s=s)
    x, y, z = pt["position"]
    rot = ctx.tool("robot.grasp_frame", close_heading_deg=pt["heading_deg"] + 90.0, arm_id=arm)["rotation"]
    ctx.tool("robot.set_grip", width_m=0.045, arm_id=arm, settle_steps=6)
    ctx.tool("robot.go_to_pose", pose=_pose((x, y, BZ + 0.07), rot), arm_id=arm, max_steps=200)
    ctx.tool("robot.go_to_pose", pose=_pose((x, y, BZ + p["low"]), rot), arm_id=arm, max_steps=200)
    ctx.tool("robot.set_grip", fraction_closed=p["fraction"], arm_id=arm, ramp_steps=48, settle_steps=30)
    ctx.tool("robot.go_to_pose_cartesian", pose=_pose((x, y, BZ + p["height"]), rot), arm_id=arm, speed_scale=p["speed"])
    after = ctx.tool("rope.point", s=s)["position"]
    return {"arm": arm, "rot": rot, "xy": [x, y], "held": after[2] - z > 0.005, "rose": round(after[2] - z, 4)}


def _release(ctx, arm, g) -> None:
    ctx.tool("robot.set_grip", preset="open", arm_id=arm, settle_steps=10)
    ctx.tool("robot.go_to_pose", pose=_pose((g["xy"][0], g["xy"][1], BZ + 0.08), g["rot"]), arm_id=arm, max_steps=150)


def run(ctx) -> dict:
    p = {"low": 0.0135, "fraction": 0.9, "speed": 0.25, **_p()}
    k0 = _knot(ctx)
    a = math.radians(p["angle_deg"])  # each tail pulled outward along y, turned this much toward the robot
    dirs = {0: (-math.sin(a), math.cos(a)), 1: (-math.sin(a), -math.cos(a))}
    cycles = []
    for c in range(int(p["cycles"])):
        k = _knot(ctx)
        if k["lo"] is None or k["n"] != 3:
            return {"route": "lost", "cycles": cycles, "start_in_knot": k0["in_knot"]}
        # Node 0's tail is the robot's right (-y): the right hand (1) takes it; the last node's tail the left (0).
        s_of = {1: max(0.03, k["lo"] - p["grasp_from_knot"]), 0: min(k["length"] - 0.03, k["hi"] + p["grasp_from_knot"])}
        grips = {arm: _grasp(ctx, arm, s_of[arm], p) for arm in (0, 1)}
        if all(g["held"] for g in grips.values()):
            # Lift the knot clear of the bench first (RoboSimStudio's own knot test lifts 120 mm, then pulls out),
            # so it draws up between the hands instead of being dragged along the bench.
            for arm, g in grips.items():
                ctx.tool("robot.go_to_pose_cartesian", pose=_pose((g["xy"][0], g["xy"][1], BZ + p.get("lift", 0.0) + p["height"]), g["rot"]),
                         arm_id=arm, speed_scale=p["speed"])
            moved = 0.0
            while moved < p["distance"] - 1e-9:
                d = min(p["step"], p["distance"] - moved)
                for arm, g in grips.items():
                    g["xy"] = [g["xy"][0] + dirs[arm][0] * d, g["xy"][1] + dirs[arm][1] * d]
                    ctx.tool("robot.go_to_pose_cartesian", pose=_pose((g["xy"][0], g["xy"][1], BZ + p.get("lift", 0.0) + p["height"]), g["rot"]),
                             arm_id=arm, speed_scale=p["speed"])
                moved += d
            for arm, g in grips.items():  # set the knot back down before letting go
                ctx.tool("robot.go_to_pose_cartesian", pose=_pose((g["xy"][0], g["xy"][1], BZ + p["height"]), g["rot"]),
                         arm_id=arm, speed_scale=p["speed"])
        for arm, g in grips.items():
            _release(ctx, arm, g)
        ctx.tool("robot.wait_steps", steps=30)
        k2 = _knot(ctx)
        cycles.append({"cycle": c, "held": {str(a_): g["held"] for a_, g in grips.items()}, "in_knot": round(k2["in_knot"], 4), "n": k2["n"]})
        if k2["n"] == 3 and k2["in_knot"] <= p["goal_fraction"] * k0["in_knot"]:
            return {"route": "tight", "cycles": cycles, "start_in_knot": k0["in_knot"]}
    return {"route": "pulled", "cycles": cycles, "start_in_knot": k0["in_knot"]}
