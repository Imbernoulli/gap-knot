"""The overhand knot's under-crossing, alone: B's free end made to pass UNDER A near crossing 2, by the mode in params.json."""
import json
import math
import os

import numpy as np

from knotlab import diagram, skills
from knotlab import thread_lab

HERE = os.path.dirname(os.path.abspath(__file__))
BZ = skills.BENCH_Z


def _params() -> dict:
    with open(os.path.join(HERE, "..", "params.json")) as f:
        return json.load(f)


def _geometry(ctx) -> dict:
    lay = ctx.tool("rope.layout")
    tgt = np.asarray(lay["target_nodes"]) - np.array([0.6, 0.0, 0.75])
    c2 = thread_lab.target_crossing(tgt, 2)
    s = diagram.arc_length(tgt)
    k = int(np.searchsorted(s, c2["s_b"]))
    t = tgt[min(k + 2, len(tgt) - 1), :2] - tgt[max(k - 2, 0), :2]
    u = t / np.linalg.norm(t)  # B's direction through the crossing
    X = np.array([c2["xy"][0] + 0.6, c2["xy"][1]])
    return {"X": X, "u": u, "sA": c2["s_a"], "sB": c2["s_b"], "L": float(s[-1]), "tgt": tgt}


def _score(ctx, g) -> dict:
    st = ctx.tool("rope.state")
    pts = np.asarray(st["nodes"]) - np.array([0.6, 0.0, 0.75])
    return thread_lab.segment_score(pts, g["tgt"])


def mode_thread_end(ctx, g, p) -> dict:
    beyond = g["X"] + g["u"] * p.get("beyond", 0.05)
    r = skills.thread_end(ctx, 1, 0, False, g["sA"], g["X"].tolist(), beyond.tolist(),
                          grasp_back=p["grasp_back"], drag=p["drag"], margin=p["margin"], lift=p["lift"])
    return {k: v for k, v in r.items() if k not in ("rot",)}


def mode_over_end(ctx, g, p) -> dict:
    """A passes over B's end: lay B's tip just short of A, lift A, step it back over the tip, hold it down, pull B through.

    Directions are the tip's own (the heading B has where the hand holds it, toward the end), not the target's:
    the jaws keep the heading the cord had when it was picked up.
    """
    X, L = g["X"], g["L"]
    out = {}
    hold_s = L - p["grip_back"]
    pt = ctx.tool("rope.point", s=hold_s)
    h = np.array([math.cos(math.radians(pt["heading_deg"])), math.sin(math.radians(pt["heading_deg"]))])
    # 1. B's end laid flat with its tip `short` before A along its own heading.
    hold_xy = X - h * (p["short"] + p["grip_back"])
    lay = skills.lay(ctx, 1, hold_s, hold_xy.tolist(), retries=2, low=0.012, fraction=0.95)
    out["lay_end"] = {k: lay.get(k) for k in ("ok", "failure")}
    skills.retreat(ctx, 1)
    tip = np.array(ctx.tool("rope.point", s=L)["position"][:2])
    back_pt = np.array(ctx.tool("rope.point", s=L - p["grip_back"])["position"][:2])
    h = (tip - back_pt) / max(np.linalg.norm(tip - back_pt), 1e-6)  # where the tip really points now
    out["tip_before"] = tip.round(4).tolist()
    out["tip_short"] = round(float(np.dot(X - tip, h)), 4)
    # 2. The helper takes A beside the crossing, lifts it, steps it back over the tip, lays it down.
    lf = skills.lift_strand(ctx, 0, g["sA"] + p.get("helper_ds", 0.0), height=p["lift"], low=0.012, fraction=0.95)
    out["lift"] = {k: lf.get(k) for k in ("ok", "strand_z", "failure")}
    if not lf.get("ok"):
        return out
    a = np.array(lf["pinch"]["at"])
    back = a - h * p["step"]
    skills.move(ctx, 0, (back[0], back[1], BZ + p["lift"]), lf["rot"])
    skills.descend(ctx, 0, (back[0], back[1], BZ + skills.LOW + 0.003), lf["rot"])
    if not p.get("hold_while_pull", True):
        ctx.tool("robot.set_grip", preset="open", arm_id=0, settle_steps=10)
        skills.retreat(ctx, 0, lf["rot"])
    out["mid"] = _score(ctx, g)
    # 3. The working hand takes B's tip, now past A, and draws it through along its heading.
    q = skills.pinch(ctx, 1, L - p["tip_grip"], low=0.012, fraction=0.95, open_width=0.035)
    out["regrasp"] = {k: q.get(k) for k in ("held", "rise")}
    if q.get("held"):
        here = ctx.tool("rope.point", s=L - p["tip_grip"])["position"]
        dest = np.array(here[:2]) + h * p["pull"]
        skills.carry(ctx, 1, [dest.tolist()], q["rot"], height=p.get("pull_height", 0.02))
        skills.place(ctx, 1, dest.tolist(), q["rot"], height=p.get("pull_height", 0.02))
    else:
        skills.retreat(ctx, 1, q.get("rot"))
    # 4. Let A go.
    ctx.tool("robot.set_grip", preset="open", arm_id=0, settle_steps=10)
    skills.retreat(ctx, 0, lf["rot"])
    return out


def _aim(ctx, g, p) -> dict:
    """Lay B's last stretch straight at the crossing along B's target direction u: two placements, the tip `short` before A."""
    X, u, L = g["X"], g["u"], g["L"]
    out = {}
    far_s, near_s = L - p.get("far_back", 0.12), L - p.get("near_back", 0.03)
    a = skills.lay(ctx, 1, far_s, (X - u * (p["short"] + p.get("far_back", 0.12) - 0.0)).tolist(), retries=2, low=0.012, fraction=0.95)
    b = skills.lay(ctx, 1, near_s, (X - u * (p["short"] + p.get("near_back", 0.03))).tolist(), retries=2, low=0.012, fraction=0.95)
    out["aim"] = [a.get("ok"), b.get("ok")]
    skills.retreat(ctx, 1)
    tip = np.array(ctx.tool("rope.point", s=L)["position"][:2])
    out["tip_short"] = round(float(np.dot(X - tip, u)), 4)
    out["tip_off_line"] = round(float(abs(np.cross(u, tip - X))), 4)
    return out


def mode_aim_push(ctx, g, p) -> dict:
    """Aim B at the crossing, lift A beside it into an arch, push B's tip under it from `grasp_back`, lay A back down in place."""
    X, u, L = g["X"], g["u"], g["L"]
    out = _aim(ctx, g, p)
    lf = skills.lift_strand(ctx, 0, g["sA"] + p.get("helper_ds", 0.035), height=p["lift"], low=0.012, fraction=0.95)
    out["lift"] = {k: lf.get(k) for k in ("ok", "strand_z", "failure")}
    if not lf.get("ok"):
        return out
    strand_xy = lf["pinch"]["at"]
    q = skills.pinch(ctx, 1, L - p["grasp_back"], low=0.012, fraction=0.95, open_width=0.035)
    out["push_grip"] = {k: q.get(k) for k in ("held", "rise")}
    if q.get("held"):
        here = np.array(ctx.tool("rope.point", s=L - p["grasp_back"])["position"][:2])
        stop = X - u * p["margin"]  # the fingers stop this far short of A's line
        push = here + u * max(0.0, float(np.dot(stop - here, u)))
        ctx.tool("robot.go_to_pose_cartesian", pose=skills._pose((push[0], push[1], BZ + skills.LOW + 0.002), q["rot"]), arm_id=1, speed_scale=0.15)
        ctx.tool("robot.set_grip", width_m=0.05, arm_id=1, settle_steps=10)
        skills.move(ctx, 1, (push[0], push[1], BZ + 0.10), q["rot"], speed=1.0)
        tip = np.array(ctx.tool("rope.point", s=L)["position"][:2])
        out["tip_past"] = round(float(np.dot(tip - X, u)), 4)
    else:
        skills.retreat(ctx, 1, q.get("rot"))
    out["lower"] = {k: v for k, v in skills.lower_strand(ctx, 0, lf["rot"], strand_xy).items() if k in ("error", "spring_back")}
    out["mid"] = _score(ctx, g)
    if p.get("pull", 0) > 0 and q.get("held"):
        # Hold A down beside the crossing again, then draw B's tip through along u.
        h2 = skills.pinch(ctx, 0, g["sA"] + p.get("helper_ds", 0.035), low=0.012, fraction=0.95, open_width=0.035)
        r = skills.pinch(ctx, 1, L - p.get("tip_grip", 0.015), low=0.012, fraction=0.95, open_width=0.035)
        if r.get("held"):
            here = np.array(ctx.tool("rope.point", s=L - p.get("tip_grip", 0.015))["position"][:2])
            dest = here + u * p["pull"]
            skills.carry(ctx, 1, [dest.tolist()], r["rot"], height=0.02)
            skills.place(ctx, 1, dest.tolist(), r["rot"], height=0.02)
        ctx.tool("robot.set_grip", preset="open", arm_id=0, settle_steps=10)
        skills.retreat(ctx, 0, h2.get("rot"))
    return out


def mode_aim_flip(ctx, g, p) -> dict:
    """Aim B at the crossing, then A steps back over B's tip: lift A beside the crossing, move it back along -u, lay it down."""
    X, u, L = g["X"], g["u"], g["L"]
    out = _aim(ctx, g, p)
    lf = skills.lift_strand(ctx, 0, g["sA"] + p.get("helper_ds", 0.035), height=p["lift"], low=0.012, fraction=0.95)
    out["lift"] = {k: lf.get(k) for k in ("ok", "strand_z", "failure")}
    if not lf.get("ok"):
        return out
    a = np.array(lf["pinch"]["at"])
    back = a - u * p["step"]
    skills.move(ctx, 0, (back[0], back[1], BZ + p["lift"]), lf["rot"])
    out["lower"] = {k: v for k, v in skills.place(ctx, 0, back.tolist(), lf["rot"], height=p["lift"]).items() if k in ("error", "spring_back")}
    return out


def _aim_at(ctx, g, p, P) -> dict:
    """Lay B's last stretch straight at point P along B's target direction u, the tip `short` before it."""
    u, L = g["u"], g["L"]
    out = {}
    far_s, near_s = L - p.get("far_back", 0.12), L - p.get("near_back", 0.03)
    a = skills.lay(ctx, 1, far_s, (P - u * (p["short"] + p.get("far_back", 0.12))).tolist(), retries=2, low=0.012, fraction=0.95)
    b = skills.lay(ctx, 1, near_s, (P - u * (p["short"] + p.get("near_back", 0.03))).tolist(), retries=2, low=0.012, fraction=0.95)
    out["aim"] = [a.get("ok"), b.get("ok")]
    skills.retreat(ctx, 1)
    tip = np.array(ctx.tool("rope.point", s=L)["position"][:2])
    out["tip_short"] = round(float(np.dot(P - tip, u)), 4)
    return out


def _go(ctx, arm, xyz, rot, steps=200):
    try:
        r = ctx.tool("robot.go_to_pose", pose=skills._pose(tuple(xyz), rot), arm_id=arm, max_steps=steps)
        return {"outcome": r.get("outcome"), "remaining": round(float(r.get("remaining_distance", 9) or 9), 4)}
    except Exception as e:
        return {"outcome": "refused " + str(e)[:40]}


def mode_tilt_pass(ctx, g, p) -> dict:
    """Aim B at the crossing; the helper lifts A beside it into an arch; the working hand, tilted along u, slides B's end under."""
    X, u, L = g["X"], g["u"], g["L"]
    out = _aim_at(ctx, g, p, X)
    lf = skills.lift_strand(ctx, 0, g["sA"] + p.get("helper_ds", 0.035), height=p["lift"], low=0.012, fraction=0.95)
    out["lift"] = {k: lf.get(k) for k in ("ok", "strand_z", "failure")}
    if not lf.get("ok"):
        return out
    strand_xy = lf["pinch"]["at"]
    q = skills.pinch(ctx, 1, L - p["grasp_back"], low=0.012, fraction=0.95, open_width=0.035)
    out["grip"] = {k: q.get(k) for k in ("held", "rise")}
    if q.get("held"):
        here = np.array(ctx.tool("rope.point", s=L - p["grasp_back"])["position"][:2])
        th = math.radians(p["tilt"])
        appr = {"x": float(math.sin(th) * u[0]), "y": float(math.sin(th) * u[1]), "z": float(-math.cos(th))}
        fr = ctx.tool("robot.grasp_frame", approach=appr, close_heading_deg=math.degrees(math.atan2(u[1], u[0])) + 90.0, arm_id=1)
        rot = fr["rotation_flipped" if p.get("wrist", "flip") == "flip" else "rotation"]
        out["tilt_up"] = _go(ctx, 1, (here[0], here[1], BZ + 0.04), rot)
        out["tilt_down"] = _go(ctx, 1, (here[0], here[1], BZ + p["z"]), rot)
        dest = X + u * p["pass"]
        out["slide"] = _go(ctx, 1, (dest[0], dest[1], BZ + p["z"]), rot, steps=300)
        ctx.tool("robot.set_grip", width_m=0.05, arm_id=1, settle_steps=10)
        back = dest - u * 0.05
        _go(ctx, 1, (back[0], back[1], BZ + p["z"]), rot)
        _go(ctx, 1, (back[0], back[1], BZ + 0.10), rot)
        tip = np.array(ctx.tool("rope.point", s=L)["position"][:2])
        out["tip_past"] = round(float(np.dot(tip - X, u)), 4)
    else:
        skills.retreat(ctx, 1, q.get("rot"))
    out["lower"] = {k: v for k, v in skills.lower_strand(ctx, 0, lf["rot"], strand_xy).items() if k in ("error",)}
    return out


def mode_video_tuck(ctx, g, p) -> dict:
    """As the person in the clip does: pinch AT crossing 1 and prop it ~1-2 cm; tuck B's tip low under the propped
    strand a few cm from the pinch; re-take the tip just beyond and draw it out over the loop's far side."""
    u, L, tgt = g["u"], g["L"], g["tgt"]
    c1 = thread_lab.target_crossing(tgt, 1)
    s = diagram.arc_length(tgt)
    # The tuck point: on A, `tuck_ds` along A from crossing 1 toward crossing 2's A strand.
    sP = c1["s_a"] + p["tuck_ds"]
    P_now = np.array(ctx.tool("rope.point", s=sP)["position"][:2])
    out = _aim_at(ctx, g, p, P_now)
    hp = skills.pinch(ctx, 0, c1["s_a"], low=0.012, fraction=0.95, open_width=0.035, verify_lift=p["prop"])
    out["prop"] = {k: hp.get(k) for k in ("held", "rise")}
    q = skills.pinch(ctx, 1, L - p["grasp_back"], low=0.012, fraction=0.95, open_width=0.035)
    out["grip"] = {k: q.get(k) for k in ("held", "rise")}
    if q.get("held"):
        here = np.array(ctx.tool("rope.point", s=L - p["grasp_back"])["position"][:2])
        P_now = np.array(ctx.tool("rope.point", s=sP)["position"][:2])
        stop = P_now + u * (p["tip_past"] - p["grasp_back"])  # the tip ends tip_past beyond A, the fingers short of it
        stop = here + u * max(0.0, float(np.dot(stop - here, u)))
        ctx.tool("robot.go_to_pose_cartesian", pose=skills._pose((stop[0], stop[1], BZ + p["z"]), q["rot"]), arm_id=1, speed_scale=0.15)
        ctx.tool("robot.set_grip", width_m=0.05, arm_id=1, settle_steps=10)
        skills.move(ctx, 1, (stop[0], stop[1], BZ + 0.08), q["rot"], speed=1.0)
        tip = np.array(ctx.tool("rope.point", s=L)["position"][:2])
        out["tip_past"] = round(float(np.dot(tip - P_now, u)), 4)
        r = skills.pinch(ctx, 1, L - 0.012, low=0.012, fraction=0.95, open_width=0.035)
        out["retake"] = {k: r.get(k) for k in ("held", "rise")}
        if r.get("held"):
            here = np.array(ctx.tool("rope.point", s=L - 0.012)["position"][:2])
            dest = here + u * p["draw"]
            skills.carry(ctx, 1, [dest.tolist()], r["rot"], height=0.02)
            skills.place(ctx, 1, dest.tolist(), r["rot"], height=0.02)
        else:
            skills.retreat(ctx, 1, r.get("rot"))
    else:
        skills.retreat(ctx, 1, q.get("rot"))
    ctx.tool("robot.set_grip", preset="open", arm_id=0, settle_steps=10)
    skills.retreat(ctx, 0, hp.get("rot"))
    return out


def run(ctx) -> dict:
    p = _params()
    g = _geometry(ctx)
    before = _score(ctx, g)
    rep = {"thread_end": mode_thread_end, "over_end": mode_over_end, "aim_push": mode_aim_push, "aim_flip": mode_aim_flip, "tilt_pass": mode_tilt_pass, "video_tuck": mode_video_tuck}[p["mode"]](ctx, g, p)
    for arm in (0, 1):
        try:
            skills.retreat(ctx, arm)
        except Exception:
            pass
    ctx.tool("robot.wait_steps", steps=40)
    after = _score(ctx, g)
    return {"route": "done", "before": before, "after": after, "report": rep}
