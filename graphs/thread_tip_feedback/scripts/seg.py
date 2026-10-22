"""A feedback-controlled under-pass probe.

This graph intentionally isolates one crossing on ``start_shape=pre_thread``.
After the working hand pinches the free tail, each move is planned from the
currently measured material point and tangent.  It is a diagnostic candidate,
not evidence that a straight rope was tied successfully.
"""

import json
import math
from pathlib import Path

import numpy as np

from knotlab import diagram, skills, task, thread_lab, tip_feedback


def geometry(ctx, s_a, reference_u):
    point = ctx.tool("rope.point", s=float(s_a))
    heading = math.radians(float(point["heading_deg"]))
    u = np.array([-math.sin(heading), math.cos(heading)], dtype=float)
    if np.dot(u, reference_u) < 0:
        u = -u
    return np.asarray(point["position"][:2], dtype=float), u


def _go(ctx, arm, plan, speed=0.15):
    r = ctx.tool("robot.go_to_pose_cartesian", pose=plan["pose"], arm_id=int(arm), speed_scale=float(speed))
    return {"outcome": r.get("outcome", "?") if isinstance(r, dict) else str(r),
            "error_xy": float(plan.get("error_xy", 0.0))}


def run(ctx):
    params = json.loads((Path(__file__).resolve().parents[1] / "params.json").read_text())
    target = np.asarray(ctx.tool("rope.layout")["target_nodes"], dtype=float) - task.BENCH_ORIGIN
    c2 = thread_lab.target_crossing(target, 2)
    s_a = float(c2["s_a"])
    target_arc = diagram.arc_length(target)
    i = int(np.searchsorted(target_arc, c2["s_b"]))
    reference_u = target[min(i + 2, len(target) - 1), :2] - target[max(i - 2, 0), :2]
    reference_u /= np.linalg.norm(reference_u)
    length = float(ctx.tool("rope.state")["length"])
    report = {"aim": [], "retarget": []}

    def state(label):
        st = ctx.tool("rope.state")
        nodes = np.asarray(st["nodes"], dtype=float) - task.BENCH_ORIGIN
        report[label] = thread_lab.segment_score(nodes, target)

    state("before")
    for back in (0.12, 0.03):
        x, u = geometry(ctx, s_a, reference_u)
        r = skills.lay(ctx, 1, length - back, (x - u * (params["short"] + back)).tolist(),
                       retries=2, low=0.012, fraction=0.90)
        report["aim"].append({k: r.get(k) for k in ("ok", "failure", "error")})
    skills.retreat(ctx, 1)
    state("after_aim")

    lift = skills.lift_strand(ctx, 0, s_a + params["helper_ds"], height=params["lift"], low=0.012, fraction=0.90)
    report["lift"] = {k: lift.get(k) for k in ("ok", "strand_z", "failure")}
    if lift.get("ok"):
        grasp_s = length - float(params["grasp_back"])
        grip = skills.pinch(ctx, 1, grasp_s, low=0.012, fraction=0.90, open_width=0.035)
        report["grip"] = {k: grip.get(k) for k in ("held", "rise")}
        if grip.get("held"):
            cap = tip_feedback.capture(ctx, 1, s=grasp_s, inset=params["inset"])
            # First approach the live crossing, then make up to N small passes.
            x, u = geometry(ctx, s_a, reference_u)
            for stage, dist in (("before", -float(params["short"])), ("past", float(params["pass"]))):
                for k in range(int(params["max_retargets"])):
                    x, u = geometry(ctx, s_a, reference_u)
                    target_xy = x + u * dist
                    plan = tip_feedback.retarget_pose(ctx, arm_id=1, s=grasp_s, target_xy=target_xy,
                                                      capture=cap, height=params["z"], inset=params["inset"])
                    result = _go(ctx, 1, plan, speed=0.15)
                    point = np.asarray(ctx.tool("rope.point", s=grasp_s)["position"][:2], dtype=float)
                    err = float(np.linalg.norm(point - target_xy))
                    report["retarget"].append({"stage": stage, "k": k, "result": result,
                                               "target": target_xy.tolist(), "tip_error": err})
                    if err <= float(params["tip_tolerance"]):
                        break
            ctx.tool("robot.set_grip", width_m=0.06, arm_id=1, settle_steps=10)
            skills.move(ctx, 1, (float(x[0] - u[0] * 0.05), float(x[1] - u[1] * 0.05), skills.BENCH_Z + 0.028),
                        plan["rotation"], speed=0.15)
            state("after_pass")
        else:
            skills.retreat(ctx, 1, grip.get("rot"))
        skills.lower_strand(ctx, 0, lift["rot"], lift["pinch"]["at"])

    for arm in (0, 1):
        skills.retreat(ctx, arm)
    ctx.tool("robot.wait_steps", steps=120)
    state("after")
    return {"route": "done", "report": report}
