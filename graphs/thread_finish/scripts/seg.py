"""Diagnose release order and finish the third crossing from a prepared loop.

Only robot tools move the cord. This is a segment probe, not a from-straight
knot-tying demonstration. Phase observations are retained even on failure.
"""
import json
import math
from pathlib import Path

import numpy as np

from knotlab import diagram, skills, task, thread_lab


def go(ctx, arm, xy, height, rot, steps=240):
    result = ctx.tool("robot.go_to_pose", arm_id=arm,
                      pose=skills._pose((*xy, skills.BENCH_Z + height), rot),
                      max_steps=steps)
    return {k: result.get(k) for k in ("outcome", "remaining_distance")}


def rotation_matrix(q):
    w, x, y, z = [float(q[k]) for k in ("w", "x", "y", "z")]
    return np.array([[1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)]])


def run(ctx):
    p = json.loads((Path(__file__).resolve().parents[1] / "params.json").read_text())
    target = np.asarray(ctx.tool("rope.layout")["target_nodes"]) - task.BENCH_ORIGIN
    c1, c2, c3 = [thread_lab.target_crossing(target, k) for k in (1, 2, 3)]
    arc = diagram.arc_length(target)
    i = int(np.searchsorted(arc, c2["s_b"]))
    ref = target[min(i + 2, len(target) - 1), :2] - target[max(i - 2, 0), :2]
    ref /= np.linalg.norm(ref)
    length = ctx.tool("rope.state")["length"]
    report = {"mode": p["finish"], "aim": []}

    def state(label):
        st = ctx.tool("rope.state")
        nodes = np.asarray(st["nodes"]) - task.BENCH_ORIGIN
        report[label] = {**thread_lab.segment_score(nodes, target),
                         "crossing_details": st["crossings"], "ends": st["ends"],
                         "max_height": st["max_height"],
                         "tcp": [skills.ee(ctx, arm)[0] for arm in (0, 1)]}
        return st

    def geometry():
        point = ctx.tool("rope.point", s=float(c2["s_a"]))
        h = math.radians(point["heading_deg"])
        u = np.array([-math.sin(h), math.cos(h)])
        if np.dot(u, ref) < 0:
            u = -u
        return np.asarray(point["position"][:2]), u

    def lower_helper():
        report["helper_down"] = go(ctx, 0, anchor["at"], 0.018, anchor["rot"])
        state("helper_down_closed")
        ctx.tool("robot.set_grip", width_m=p.get("helper_open", 0.035), arm_id=0, settle_steps=12)
        state("helper_open")
        if p.get("helper_side_clear", False):
            side = np.asarray(anchor["at"]) + np.array([0.075, -0.025])
            report["helper_side"] = go(ctx, 0, side, 0.025, anchor["rot"])
            state("helper_side")
            go(ctx, 0, side, 0.12, anchor["rot"])
        else:
            go(ctx, 0, anchor["at"], 0.12, anchor["rot"])
        state("helper_clear")

    def release_tip():
        ctx.tool("robot.set_grip", width_m=0.06, arm_id=1, settle_steps=10)
        state("tip_open")
        go(ctx, 1, dest - u * 0.05, p["z"], rot)
        state("tip_back")
        go(ctx, 1, dest - u * 0.05, 0.12, rot)
        state("tip_clear")

    state("before")
    anchor = skills.pinch(ctx, 0, c1["s_a"], low=0.012, fraction=0.95,
                          open_width=0.035, verify_lift=0.004)
    report["anchor"] = {k: anchor.get(k) for k in ("held", "rise")}
    state("after_anchor")
    for back in (0.12, 0.03):
        x, u = geometry()
        r = skills.lay(ctx, 1, length - back, (x - u * (p["short"] + back)).tolist(),
                       retries=2, low=0.012, fraction=0.95)
        report["aim"].append({k: r.get(k) for k in ("ok", "failure", "error")})
    skills.retreat(ctx, 1)
    state("after_aim")
    grip = skills.pinch(ctx, 1, length - p["grasp_back"], low=0.012,
                        fraction=0.95, open_width=0.035)
    for attempt in range(p.get("grip_retries", 0)):
        if grip["held"]:
            break
        skills.retreat(ctx, 1, grip["rot"], up=0.04)
        grip = skills.pinch(ctx, 1, length - p["grasp_back"], low=0.012,
                            fraction=0.95, open_width=0.035)
    report["grip_before_lift"] = {k: grip.get(k) for k in ("held", "rise")}
    state("after_tip_grip")
    report["lift"] = go(ctx, 0, anchor["at"], p["lift"], anchor["rot"])
    state("after_lift")
    if grip["held"]:
        x, u = geometry()
        theta = math.radians(p["tilt"])
        approach = {"x": float(math.sin(theta) * u[0]), "y": float(math.sin(theta) * u[1]),
                    "z": float(-math.cos(theta))}
        rot = ctx.tool("robot.grasp_frame", approach=approach,
                       close_heading_deg=math.degrees(math.atan2(u[1], u[0])) + 90,
                       arm_id=1)["rotation_flipped"]
        here = np.asarray(ctx.tool("rope.point", s=length - p["grasp_back"])["position"][:2])
        report["rotate"] = go(ctx, 1, here, 0.04, rot)
        report["descend"] = go(ctx, 1, here, p["z"], rot)
        x, u = geometry()
        dest = x + u * p["pass"]
        report["pass"] = go(ctx, 1, dest, p["z"], rot, steps=300)
        state("after_pass_closed")
        if p["finish"] == "draw_over":
            # The second under-pass only makes crossing 2.  The free B tail
            # must then travel over the next A strand to create crossing 3;
            # opening at ``dest`` loses the tail before that crossing exists.
            x3 = np.asarray(ctx.tool("rope.point", s=float(c3["s_a"]))["position"][:2])
            side = x3 + 0.012 * u
            exit_xy = x3 + p.get("draw_distance", 0.055) * u
            report["draw_over"] = skills.carry(ctx, 1,
                                                [dest.tolist(), side.tolist(), exit_xy.tolist()],
                                                rot, height=p.get("draw_height", 0.035),
                                                speed=0.18, s=length - p["grasp_back"])
            state("after_draw_over_closed")
            release_tip()
            lower_helper()
            state("after_draw_over_release")
            # Skip the generic release branch below.
            p["finish"] = "draw_over_done"
        if p["finish"] in ("regrasp_exit", "closed_exit"):
            if p["finish"] == "regrasp_exit":
                release_tip()
                grip = skills.pinch(ctx, 1, length - 0.02, low=0.012,
                                    fraction=0.95, open_width=0.035)
                report["exit_grip"] = {k: grip.get(k) for k in ("held", "rise")}
                rot = grip["rot"]
            st = state("before_exit")
            if p.get("pivot_upright", False):
                material = length - p["grasp_back"]
                point = ctx.tool("rope.point", s=material)
                held = np.asarray(point["position"])
                position, actual_rotation = skills.ee(ctx, 1)
                offset = rotation_matrix(actual_rotation).T @ (held - position)
                rot = skills.frame(ctx, 1, point["heading_deg"])
                pivot = held - rotation_matrix(rot) @ offset
                report["pivot"] = go(ctx, 1, pivot[:2], pivot[2] - skills.BENCH_Z, rot)
                st = state("after_pivot")
            if p.get("rise_in_eye", False):
                position, _ = skills.ee(ctx, 1)
                report["eye_rise"] = go(ctx, 1, position[:2], p.get("exit_height", 0.085), rot)
                st = state("after_eye_rise")
            nodes, s = np.asarray(st["nodes"]), np.asarray(st["arc"])
            loop = nodes[(s >= c1["s_a"]) & (s <= c1["s_b"]), :2]
            edge = np.asarray(ctx.tool("rope.point", s=float(c3["s_a"]))["position"][:2])
            outward = edge - loop.mean(axis=0)
            outward /= max(np.linalg.norm(outward), 1e-6)
            exit_xy = edge + p["exit_distance"] * outward
            report["exit_target"] = exit_xy.tolist()
            if p.get("exit_route") == "far_corner":
                edge = np.asarray(ctx.tool("rope.point", s=float(c3["s_a"] + 0.025))["position"][:2])
                exit_xy = edge + np.array([0.07, 0.015])
                report["exit_target"] = exit_xy.tolist()
            report["exit_move"] = skills.carry(ctx, 1, [edge.tolist(), exit_xy.tolist()],
                                               rot, height=p.get("exit_height", 0.085), s=length - 0.02)
            state("after_exit")
            lower_helper()
            report["tip_place"] = skills.place(ctx, 1, exit_xy, rot,
                                                s=length - 0.02, height=0.085)
            state("after_tip_place")
        elif p["finish"] == "helper_first":
            lower_helper()
            release_tip()
        elif p["finish"] != "draw_over_done":
            release_tip()
            lower_helper()
    else:
        report["skipped_pass"] = "tip not held"
        lower_helper()
    for arm in (0, 1):
        # Rise with the measured orientation before turning the wrist.
        position, rotation = skills.ee(ctx, arm)
        ctx.tool("robot.set_grip", preset="open", arm_id=arm, settle_steps=8)
        go(ctx, arm, position[:2], max(position[2] - skills.BENCH_Z, 0.20), rotation)
    ctx.tool("robot.wait_steps", steps=180)
    state("after")
    return {"route": "done", "report": report}
