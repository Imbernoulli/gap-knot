"""Runs one demo-plan step (knotlab.demo's step dicts) with the knotlab skills; keeps what each hand holds.

Kinds: lay, hold, lift_hold, carry_hold, lower, aim_end, push_under, pull_through, pull, release, settle.
See knotlab/demo.py for each kind's fields.
"""
from __future__ import annotations

import math

from knotlab import skills

BENCH_Z = skills.BENCH_Z
DEEP = 0.505
"""World x beyond which pinches came up empty: on the start line (x 0.53) 12 of 12 first pinches missed in the probes.
A point beyond it is taken with the jaws centred up to 25 mm short of it and opened to 65 mm, so the far finger
sweeps the cord in toward the robot as they close."""


def _pose(p, rot):
    return {"position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}, "rotation": rot}


def _cart(ctx, arm, xy, z, rot, speed=skills.SPEED):
    r = ctx.tool("robot.go_to_pose_cartesian", pose=_pose((xy[0], xy[1], BENCH_Z + z), rot), arm_id=arm, speed_scale=speed)
    return skills._outcome(r)


def _pt(ctx, s):
    return ctx.tool("rope.point", s=float(s))["position"]


def pinch(ctx, arm_id: int, s: float, *, low: float = skills.LOW, approach: float = skills.APPROACH,
          fraction: float = skills.FRACTION, ramp: int = skills.RAMP, settle: int = skills.SETTLE,
          verify_lift: float = 0.02, open_width: float = 0.045) -> dict:
    """skills.pinch (the tool centre ``low`` over the bench, or 8.5 mm over the strand's axis when that is higher), with
    two changes. A point beyond ``DEEP`` (the start line) is taken with the jaws centred short of it and opened
    wider. The descent may take 150 steps, not 90: a 90-step descent onto the cord timed out 3-6 mm high in a third
    of the probes' pinches (measured: pinch targets of 12 mm stopped at 17.5-18 mm), leaving the tips at the cord's
    axis."""
    pt = ctx.tool("rope.point", s=float(s))
    x, y, z0 = pt["position"]
    axis = float(z0) - BENCH_Z
    low = max(float(low), axis + 0.0085)
    rot = skills.frame(ctx, arm_id, pt["heading_deg"])
    if x > DEEP:  # beyond the depth the hand gets down at: centre the jaws short of it, open wider
        x = max(DEEP, x - 0.025)
        open_width = max(float(open_width), 0.065)
    ctx.tool("robot.set_grip", width_m=float(open_width), arm_id=arm_id, settle_steps=6)
    out = {"skill": "pinch", "arm_id": arm_id, "s": float(s), "at": [round(x, 4), round(y, 4)], "low": round(low, 4)}
    out["above"] = skills.hover(ctx, arm_id, (x, y, BENCH_Z + low + approach), rot)
    out["down"] = skills.descend(ctx, arm_id, (x, y, BENCH_Z + low), rot, max_steps=150)
    g = ctx.tool("robot.set_grip", fraction_closed=float(fraction), arm_id=arm_id, ramp_steps=int(ramp), settle_steps=int(settle))
    out["width"] = g.get("command_width_m")
    out["up"] = skills.descend(ctx, arm_id, (x, y, BENCH_Z + low + verify_lift), rot, tolerance=0.003)
    rise = float(ctx.tool("rope.point", s=float(s))["position"][2]) - float(z0)
    out.update(rise=round(rise, 4), held=bool(rise > 0.4 * verify_lift), rot=rot, z_tcp=BENCH_Z + low + verify_lift)
    return out


def pinch_retry(ctx, arm_id, s, tries, **kw) -> dict:
    p = pinch(ctx, arm_id, s, **kw)
    n = 1
    while not p["held"] and n <= tries:
        skills.retreat(ctx, arm_id, p["rot"], up=0.04)  # a miss usually nudged the cord aside: read it again
        p = pinch(ctx, arm_id, s, **kw)
        n += 1
    p["tries"] = n
    return p


def lay(ctx, arm_id, s, xy, *, via=(), height=skills.CARRY, retries=1, **pk) -> dict:
    """skills.lay with this module's pinch."""
    p = pinch_retry(ctx, arm_id, s, retries, **pk)
    out = {"skill": "lay", "s": s, "tries": p["tries"], "pinch": {k: v for k, v in p.items() if k != "rot"}}
    if not p["held"]:
        skills.retreat(ctx, arm_id, p["rot"])
        out.update(ok=False, failure="not_held")
        return out
    c = skills.carry(ctx, arm_id, [*via, xy], p["rot"], height=height, s=s)
    out["carry"] = c
    out["place"] = skills.place(ctx, arm_id, xy, p["rot"], s=s, height=height)
    out["ok"] = bool(c["reached"]) and out["place"].get("error", 0.0) < 0.02
    if not out["ok"]:
        out["failure"] = "slipped" if c.get("lag", 0.0) > 0.015 else ("unreached" if not c["reached"] else "off_target")
    return out


def pull(ctx, *, s_left, s_right, direction_left, direction_right, distance, step=0.01, height=0.03, pk=None) -> dict:
    """skills.pull with this module's pinch."""
    pk = pk or {}
    before = skills.rope_progress(ctx)
    grips = {0: pinch_retry(ctx, 0, s_left, 1, **pk), 1: pinch_retry(ctx, 1, s_right, 1, **pk)}
    out = {"skill": "pull", "before": before, "held": {a: g["held"] for a, g in grips.items()}}
    if not all(g["held"] for g in grips.values()):
        for a, g in grips.items():
            skills.retreat(ctx, a, g["rot"])
        out.update(ok=False, failure="not_held")
        return out
    pos = {a: list(g["at"]) for a, g in grips.items()}
    dirs = {0: direction_left, 1: direction_right}
    moved, best = 0.0, before["progress"]
    while moved < distance - 1e-9:
        d = min(step, distance - moved)
        for a in (0, 1):
            pos[a] = [pos[a][0] + dirs[a][0] * d, pos[a][1] + dirs[a][1] * d]
            skills.move(ctx, a, (pos[a][0], pos[a][1], BENCH_Z + height), grips[a]["rot"], speed=0.15)
        moved += d
        now = skills.rope_progress(ctx)
        if now["progress"] < best - 1e-9:
            out["stopped"] = "progress_drop"
            break
        best = max(best, now["progress"])
    out["pulled"] = round(moved, 4)
    for a in (0, 1):
        ctx.tool("robot.set_grip", preset="open", arm_id=a, settle_steps=10)
        skills.move(ctx, a, (pos[a][0], pos[a][1], BENCH_Z + height + 0.05), grips[a]["rot"], speed=0.5)
    ctx.tool("robot.wait_steps", steps=30)
    out["after"] = skills.rope_progress(ctx)
    out["ok"] = out["after"]["progress"] >= before["progress"]
    return out


class Hands:
    """What each arm holds: {arm: {"rot", "s", "z"}}."""

    def __init__(self):
        self.held = {}
        self.last_aim = None


def run_step(ctx, step: dict, P: dict, hands: Hands, length: float) -> dict:
    kind = step["kind"]
    arm = int(step.get("arm_id", 1))
    pk = dict(P.get("pinch", {}))
    out = {"k": step.get("k"), "kind": kind, "arm_id": arm, "event": step.get("event")}
    if kind == "lay":
        r = lay(ctx, arm, step["s"], tuple(step["xy"]), via=[tuple(v) for v in (step.get("via") or [])],
                height=float(step.get("height") or P["lay"]["height"]), retries=int(P["lay"]["retries"]), **pk)
        out.update(ok=bool(r.get("ok")), failure=r.get("failure"), error=r.get("place", {}).get("error"),
                   held=r.get("pinch", {}).get("held"), lag=r.get("carry", {}).get("lag"))
        return out
    if kind in ("hold", "lift_hold"):
        s = float(step.get("lift_s") if step.get("lift_s") is not None else step["s"])
        h = float(step.get("height") or (0.06 if kind == "lift_hold" else 0.0))
        p = pinch_retry(ctx, arm, s, int(P["lay"]["retries"]), **pk)
        out["held"] = bool(p["held"])
        if not p["held"]:
            skills.retreat(ctx, arm, p["rot"])
            out.update(ok=False, failure="not_held")
            return out
        x, y = p["at"]
        z = (skills.LOW + 0.002) if kind == "hold" else h
        out["move"] = skills.move(ctx, arm, (x, y, BENCH_Z + z), p["rot"])
        hands.held[arm] = {"rot": p["rot"], "s": s, "z": z, "xy": [x, y]}
        out["strand_z"] = round(float(_pt(ctx, s)[2]) - BENCH_Z, 4)
        out["ok"] = True if kind == "hold" else out["strand_z"] > 0.5 * h
        return out
    if kind == "carry_hold":
        hd = hands.held.get(arm)
        if hd is None:
            return {**out, "ok": False, "failure": "not_holding"}
        z = float(step.get("height", hd["z"]))
        for v in [*step.get("via", []), step["xy"]]:
            _cart(ctx, arm, v, z, hd["rot"])
        hd["xy"], hd["z"] = list(step["xy"]), z
        now = _pt(ctx, hd["s"])
        out.update(ok=True, lag=round(skills._dist(now, step["xy"]), 4))
        return out
    if kind == "lower":
        hd = hands.held.pop(arm, None)
        if hd is None:
            return {**out, "ok": False, "failure": "not_holding"}
        xy = step.get("xy") or hd["xy"]
        sweep = float(step.get("sweep", 0.0))
        aim = getattr(hands, "last_aim", None)
        if sweep and aim:
            xy = [xy[0] - aim["u"][0] * sweep, xy[1] - aim["u"][1] * sweep]
            _cart(ctx, arm, xy, hd["z"], hd["rot"])
        r = skills.place(ctx, arm, xy, hd["rot"], s=hd["s"], height=max(hd["z"], 0.02))
        out.update(ok=True, error=r.get("error"))
        return out
    if kind == "release":
        hd = hands.held.pop(arm, None)
        skills.retreat(ctx, arm, hd["rot"] if hd else None)
        return {**out, "ok": True}
    if kind == "aim_end":
        # Take the cord grasp_back from its end, set it down margin short of the crossing and drag it straight back:
        # the end trails behind the hand, pointing at the crossing. Keeps holding.
        end_first = bool(step.get("end_first", True))
        sign = 1.0 if end_first else -1.0
        tip_s = 0.0 if end_first else length
        gb, margin, drag = float(step.get("grasp_back", 0.06)), float(step.get("margin", 0.02)), float(step.get("drag", 0.075))
        tx, ty = step["through_xy"]
        bx, by = step["xy"]
        ux, uy = bx - tx, by - ty
        n = math.hypot(ux, uy) or 1.0
        ux, uy = ux / n, uy / n
        h1 = (tx - ux * margin, ty - uy * margin)
        h0 = (tx - ux * (margin + drag), ty - uy * (margin + drag))
        p = pinch_retry(ctx, arm, tip_s + sign * gb, 1, **pk)
        out["held"] = bool(p["held"])
        if not p["held"]:
            skills.retreat(ctx, arm, p["rot"])
            return {**out, "ok": False, "failure": "not_held"}
        rot = p["rot"]
        for v in (step.get("via") or []):
            _cart(ctx, arm, v, skills.CARRY, rot)
        out["to"] = _cart(ctx, arm, h1, skills.CARRY, rot)
        out["down"] = skills.descend(ctx, arm, (h1[0], h1[1], BENCH_Z + skills.LOW + 0.002), rot)
        out["drag_back"] = _cart(ctx, arm, h0, skills.LOW + 0.002, rot, 0.15)
        tip = _pt(ctx, tip_s)
        out["tip_short"] = round(-((tip[0] - tx) * ux + (tip[1] - ty) * uy), 4)
        hands.held[arm] = {"rot": rot, "s": tip_s + sign * gb, "z": skills.LOW + 0.002, "xy": list(h0),
                           "aim": {"through": [tx, ty], "u": [ux, uy], "margin": margin, "tip_s": tip_s}}
        out["ok"] = True
        return out
    if kind == "push_under":
        hd = hands.held.get(arm)
        if hd is None or "aim" not in hd:
            return {**out, "ok": False, "failure": "not_aimed"}
        a = hd["aim"]
        (tx, ty), (ux, uy) = a["through"], a["u"]
        h1 = (tx - ux * a["margin"], ty - uy * a["margin"])
        out["push"] = _cart(ctx, arm, h1, skills.LOW + 0.002, hd["rot"], 0.15)
        ctx.tool("robot.set_grip", width_m=0.06, arm_id=arm, settle_steps=12)
        skills.move(ctx, arm, (h1[0], h1[1], BENCH_Z + 0.11), hd["rot"], speed=1.0)
        hands.held.pop(arm, None)
        hands.last_aim = a
        tip = _pt(ctx, a["tip_s"])
        out["tip_past"] = round((tip[0] - tx) * ux + (tip[1] - ty) * uy, 4)
        out["ok"] = out["tip_past"] > 0.0
        return out
    if kind == "pull_through":
        end_first = bool(step.get("end_first", True))
        sign = 1.0 if end_first else -1.0
        tip_s = 0.0 if end_first else length
        s = tip_s + sign * float(step.get("grasp_back", 0.012))
        q = pinch_retry(ctx, arm, s, 1, open_width=float(step.get("open_width") or 0.035),
                        **{k: v for k, v in pk.items() if k != "open_width"})
        out["held"] = bool(q["held"])
        if not q["held"]:
            skills.retreat(ctx, arm, q["rot"])
            return {**out, "ok": False, "failure": "regrasp_missed"}
        h = float(step.get("height") or skills.CARRY)
        c = skills.carry(ctx, arm, [*[tuple(v) for v in (step.get("via") or [])], tuple(step["xy"])], q["rot"], height=h, s=s)
        pl = skills.place(ctx, arm, tuple(step["xy"]), q["rot"], s=s, height=h)
        out.update(ok=bool(c["reached"]), lag=c.get("lag"), error=pl.get("error"))
        return out
    if kind == "pull":
        r = pull(ctx, s_left=float(step["s_left"]), s_right=float(step["s_right"]),
                 direction_left=tuple(step["direction_left"]), direction_right=tuple(step["direction_right"]),
                 distance=float(step.get("distance") or P["pull"]["distance"]), step=float(P["pull"]["step"]),
                 height=float(P["pull"]["height"]), pk=pk)
        out.update(ok=bool(r.get("ok")), failure=r.get("failure"), stopped=r.get("stopped"), after=r.get("after"))
        return out
    if kind == "settle":
        ctx.tool("robot.wait_steps", steps=int(step.get("steps", 30)))
        return {**out, "ok": True}
    return {**out, "ok": False, "failure": f"unknown kind {kind}"}
