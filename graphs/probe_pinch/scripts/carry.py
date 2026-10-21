"""Pinch the rope at one point with the right hand, carry it, set it down: the grip and speed come from $PROBE_CFG."""
import json, os
from gap import NodeContext


def _pose(p, rot):
    return {"position": {"x": p[0], "y": p[1], "z": p[2]}, "rotation": rot}


def run(ctx: NodeContext, arms: list, grip: dict = None, ws: dict = None) -> None:
    cfg = {"s": 0.55, "dx": 0.06, "dy": 0.0, "low": 0.016, "carry": 0.035, "fraction": 0.9, "ramp": 48,
           "settle": 30, "speed": 0.25, "object": 0.0}
    cfg.update(json.loads(os.environ.get("PROBE_CFG", "{}")))
    right = next(a["arm_id"] for a in arms if a["name"] == "right")
    bz = 0.75
    pt = ctx.tool("rope.point", s=cfg["s"])
    x, y, _ = pt["position"]
    rot = ctx.tool("robot.grasp_frame", close_heading_deg=pt["heading_deg"] + 90.0, arm_id=right)["rotation"]
    ctx.tool("robot.set_grip", preset="open", arm_id=right)
    ctx.tool("robot.go_to_pose", pose=_pose((x, y, bz + 0.08), rot), arm_id=right)
    down = ctx.tool("robot.go_to_pose", pose=_pose((x, y, bz + cfg["low"]), rot), arm_id=right, max_steps=200)
    if cfg["object"]:
        g = ctx.tool("robot.set_grip", preset="grip", object_width_m=cfg["object"], arm_id=right, ramp_steps=cfg["ramp"], settle_steps=cfg["settle"])
    else:
        g = ctx.tool("robot.set_grip", fraction_closed=cfg["fraction"], arm_id=right, ramp_steps=cfg["ramp"], settle_steps=cfg["settle"])
    move = lambda p: ctx.tool("robot.go_to_pose_cartesian", pose=_pose(p, rot), arm_id=right, speed_scale=cfg["speed"])
    up = move((x, y, bz + cfg["carry"]))
    mid = ctx.tool("rope.point", s=cfg["s"])
    tx, ty = x + cfg["dx"], y + cfg["dy"]
    car = move((tx, ty, bz + cfg["carry"]))
    held = ctx.tool("rope.point", s=cfg["s"])
    move((tx, ty, bz + cfg["low"] + 0.002))
    ctx.tool("robot.set_grip", preset="open", arm_id=right, settle_steps=10)
    move((tx, ty, bz + 0.07))
    after = ctx.tool("rope.point", s=cfg["s"])
    r = lambda v: [round(c, 4) for c in v]
    print("PROBE", json.dumps({
        "cfg": cfg, "start": r(pt["position"]), "down": down.get("outcome"), "grip": {k: g.get(k) for k in ("command_width_m", "waited")},
        "lifted": r(mid["position"]), "up": up.get("outcome") if isinstance(up, dict) else up,
        "carried_to": [round(tx, 4), round(ty, 4)], "held": r(held["position"]), "after": r(after["position"]),
        "captured": round(((after["position"][0] - x) * cfg["dx"] + (after["position"][1] - y) * cfg["dy"]) / max(1e-9, cfg["dx"] ** 2 + cfg["dy"] ** 2), 3),
    }))
    return None
