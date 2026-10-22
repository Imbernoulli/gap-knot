"""Under-pass probe: remeasure A after aiming B, lifting A, and grasping B.

Run on start_shape=pre_thread to isolate threading. This does not construct the
loop, and a segment success is not a from-straight knot-tying success.
"""
import json
import math
from pathlib import Path

import numpy as np

from knotlab import diagram, skills, task, thread_lab


def geometry(ctx, s_a, reference_u):
    """The crossing's current position and its in-plane normal toward B's exit."""
    point = ctx.tool("rope.point", s=float(s_a))
    heading = math.radians(point["heading_deg"])
    u = np.array([-math.sin(heading), math.cos(heading)])
    if np.dot(u, reference_u) < 0:
        u = -u
    return np.asarray(point["position"][:2]), u


def go(ctx, arm, xy, height, rot, steps=200):
    result = ctx.tool("robot.go_to_pose", arm_id=arm,
                      pose=skills._pose((*xy, skills.BENCH_Z + height), rot), max_steps=steps)
    return {k: result.get(k) for k in ("outcome", "remaining_distance")}


def run(ctx):
    p = json.loads((Path(__file__).resolve().parents[1] / "params.json").read_text())
    target = np.asarray(ctx.tool("rope.layout")["target_nodes"]) - task.BENCH_ORIGIN
    c2 = thread_lab.target_crossing(target, 2)
    s_a = c2["s_a"]
    target_arc = diagram.arc_length(target)
    i = int(np.searchsorted(target_arc, c2["s_b"]))
    reference_u = target[min(i + 2, len(target) - 1), :2] - target[max(i - 2, 0), :2]
    reference_u /= np.linalg.norm(reference_u)
    length = ctx.tool("rope.state")["length"]
    report = {"aim": []}

    def state(label):
        st = ctx.tool("rope.state")
        nodes = np.asarray(st["nodes"]) - task.BENCH_ORIGIN
        report[label] = thread_lab.segment_score(nodes, target)

    state("before")
    c1 = thread_lab.target_crossing(target, 1)
    anchor = skills.pinch(ctx, 0, c1["s_a"], low=0.012, fraction=0.95,
                          open_width=0.035, verify_lift=0.004)
    report["anchor"] = {k: anchor.get(k) for k in ("held", "rise")}
    state("after_anchor")
    # Each placement drags the loop. Read where A actually lies before aiming
    # the next material point instead of pushing toward the original drawing.
    for back in (0.12, 0.03):
        x, u = geometry(ctx, s_a, reference_u)
        r = skills.lay(ctx, 1, length - back, (x - u * (p["short"] + back)).tolist(),
                       retries=2, low=0.012, fraction=0.95)
        report["aim"].append({k: r.get(k) for k in ("ok", "failure", "error")})
    skills.retreat(ctx, 1)
    state("after_aim")

    grip = skills.pinch(ctx, 1, length - p["grasp_back"], low=0.012,
                        fraction=0.95, open_width=0.035)
    report["grip_before_lift"] = {k: grip.get(k) for k in ("held", "rise")}
    state("after_b_grip")
    anchor_xy = anchor["at"]
    lifted = go(ctx, 0, anchor_xy, p["lift"], anchor["rot"])
    lift = {"ok": True, "rot": anchor["rot"], "pinch": {"at": anchor_xy},
            "strand_z": ctx.tool("rope.point", s=s_a)["position"][2] - skills.BENCH_Z}
    report["anchor_lift_move"] = lifted
    report["lift"] = {k: lift.get(k) for k in ("ok", "strand_z", "failure")}
    if lift.get("ok"):
        state("after_lift")
        if grip.get("held"):
            x, u = geometry(ctx, s_a, reference_u)
            report["crossing_xy"] = x.tolist()
            theta = math.radians(p["tilt"])
            approach = {"x": float(math.sin(theta) * u[0]), "y": float(math.sin(theta) * u[1]),
                        "z": float(-math.cos(theta))}
            fr = ctx.tool("robot.grasp_frame", approach=approach,
                          close_heading_deg=math.degrees(math.atan2(u[1], u[0])) + 90, arm_id=1)
            rot = fr["rotation_flipped"]
            here = np.array(ctx.tool("rope.point", s=length - p["grasp_back"])["position"][:2])
            report["rotate"] = go(ctx, 1, here, 0.04, rot)
            report["descend"] = go(ctx, 1, here, p["z"], rot)
            x, u = geometry(ctx, s_a, reference_u)
            dest = x + u * p["pass"]
            report["pass"] = go(ctx, 1, dest, p["z"], rot, steps=300)
            state("after_pass")
            report["draw"] = go(ctx, 1, dest + u * 0.08, 0.035, rot, steps=300)
            state("after_draw")
        else:
            skills.retreat(ctx, 1, grip.get("rot"))
        skills.lower_strand(ctx, 0, lift["rot"], lift["pinch"]["at"])
        ctx.tool("robot.set_grip", width_m=0.06, arm_id=1, settle_steps=10)

    for arm in (0, 1):
        skills.retreat(ctx, arm)
    ctx.tool("robot.wait_steps", steps=120)
    state("after")
    return {"route": "done", "report": report}
