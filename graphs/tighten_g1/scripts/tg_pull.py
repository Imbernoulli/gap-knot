"""One measured, grasp-verified tightening cycle; the graph owns the retry loop."""
import json
import math
import os

BZ = 0.75
HERE = os.path.dirname(os.path.abspath(__file__))


def _params():
    with open(os.path.join(HERE, "..", "params.json")) as f:
        return json.load(f)


class _Meter:
    """Conservative scene-step estimate, carried across graph visits.

    Gripper responses report the settle, but not the ramp. Also reserve time
    for tool overhead and the episode's initial settling, rather than treating
    the sum of motion response steps as the entire scene clock.
    """
    def __init__(self, ctx, used_steps):
        self.ctx, self.used_steps = ctx, used_steps

    def tool(self, name, **kw):
        r = self.ctx.tool(name, **kw)
        steps = int(r.get("steps", r.get("waited", 0)))
        if name == "robot.set_grip":
            steps += int(kw.get("ramp_steps", 24))
        self.used_steps += int(math.ceil(1.25 * steps)) + 3
        return r


def _pose(xyz, rot):
    return {"position": dict(zip(("x", "y", "z"), xyz)), "rotation": rot}


def _knot(st):
    ss = [c[key] for c in st["crossings"] for key in ("s_a", "s_b")]
    return {"lo": min(ss) if ss else 0.0, "hi": max(ss) if ss else 0.0,
            "in_knot": max(ss) - min(ss) if ss else 0.0}


def _overhand(st):
    tokens = st["code"].split()
    return (len(st["crossings"]) == 3 and len(tokens) == 6
            and [t[:-1] for t in tokens] == ["1", "2", "3", "1", "2", "3"]
            and all(tokens[i][-1] != tokens[i + 1][-1] for i in range(5)))


def _choose(st, knot, arm, p, tried):
    """Prefer an isolated, low tail point with room for an outward pull.

    Retain the nominal 100 mm crossing offset when it has space. A tail that
    migrated toward an edge may be taken closer only when nearby strands clear
    the open jaws. All candidates remain at least 75 mm beyond the crossings.
    """
    candidates = []
    for i, (s, xyz) in enumerate(zip(st["arc"], st["nodes"])):
        offset = s - knot["hi"] if arm == 0 else knot["lo"] - s
        if not p["min_grasp_offset"] <= offset <= p["max_grasp_offset"]:
            continue
        if not 0.03 <= s <= st["length"] - 0.03:
            continue
        if not (0.35 <= xyz[0] <= 0.62 and abs(xyz[1]) <= p["grasp_y_limit"]):
            continue
        if (arm == 0 and xyz[1] < -0.02) or (arm == 1 and xyz[1] > 0.02):
            continue
        if any(abs(s - old) < 0.012 for old in tried):
            continue
        clearance = min((math.dist(xyz[:2], other[:2])
                         for j, other in enumerate(st["nodes"])
                         if abs(st["arc"][j] - s) > 0.04), default=1.0)
        if clearance < p["strand_clearance"]:
            continue
        score = (abs(offset - p["grasp_from_knot"])
                 + 2.0 * max(0.0, abs(xyz[1]) - 0.135)
                 + 2.0 * max(0.0, xyz[0] - 0.55)
                 + 0.8 * max(0.0, xyz[2] - BZ - 0.015))
        candidates.append((score, i, s))
    return min(candidates)[2] if candidates else None


def _free_retreat(ctx, arm, g):
    ctx.tool("robot.set_grip", preset="open", arm_id=arm, ramp_steps=12, settle_steps=12)
    z = max(BZ + 0.08, g["xyz"][2] + 0.03)
    ctx.tool("robot.go_to_pose", pose=_pose((*g["xyz"][:2], z), g["rot"]),
             arm_id=arm, max_steps=90)
    g["held"] = False


def _grasp(ctx, arm, knot, p):
    tried, attempts, g = [], [], None
    for attempt in range(p["grasp_attempts"]):
        st = ctx.tool("rope.state")
        s = _choose(st, knot, arm, p, tried)
        if s is None or ctx.used_steps >= p["motion_budget"]:
            break
        tried.append(s)
        pt = ctx.tool("rope.point", s=s)
        x, y, z = pt["position"]
        heading = (pt["heading_deg"] + 180.0) % 180.0 - 90.0
        rot = ctx.tool("robot.grasp_frame", close_heading_deg=heading, arm_id=arm)["rotation"]
        # The fingers go 3.6 mm below the strand axis even when draped above
        # the board. Never descend through it to a fixed table height.
        low = max(BZ + p["low"], z + 0.0085)
        g = {"s": s, "rot": rot, "xyz": [x, y, low + p["verify_lift"]], "held": False}
        ctx.tool("robot.set_grip", width_m=0.045, arm_id=arm, ramp_steps=12, settle_steps=6)
        above = ctx.tool("robot.go_to_pose", pose=_pose((x, y, low + 0.05), rot),
                         arm_id=arm, max_steps=120)
        if above.get("reached", False):
            # Refresh after approaching a released, still moving stiff tail.
            pt = ctx.tool("rope.point", s=s)
            x, y, z = pt["position"]
            low = max(BZ + p["low"], z + 0.0085)
            g["xyz"] = [x, y, low + p["verify_lift"]]
            down = ctx.tool("robot.go_to_pose", pose=_pose((x, y, low), rot),
                            arm_id=arm, max_steps=90, tolerance=0.0025)
            if down.get("reached", False):
                ctx.tool("robot.set_grip", fraction_closed=p["fraction"] + 0.02 * attempt,
                         arm_id=arm, ramp_steps=48, settle_steps=30)
                up = ctx.tool("robot.go_to_pose_cartesian", pose=_pose(g["xyz"], rot),
                              arm_id=arm, speed_scale=p["speed"])
                after = ctx.tool("rope.point", s=s)["position"]
                rise, lag = after[2] - z, math.dist(after[:2], g["xyz"][:2])
                g["held"] = bool(up.get("reached", False) and rise > 0.008 and lag < p["slip_limit"])
                attempts.append({"s": round(s, 4), "axis_z": z, "rise": round(rise, 4),
                                 "lag": round(lag, 4), "held": g["held"]})
                if g["held"]:
                    g["attempts"] = attempts
                    return g
            else:
                attempts.append({"s": round(s, 4), "failure": "descent", "motion": down})
        else:
            attempts.append({"s": round(s, 4), "failure": "approach", "motion": above})
        _free_retreat(ctx, arm, g)
    if g is not None:
        g["attempts"] = attempts
    return g


def _move(ctx, arm, g, xyz, p):
    r = ctx.tool("robot.go_to_pose_cartesian", pose=_pose(xyz, g["rot"]),
                 arm_id=arm, speed_scale=p["speed"])
    if r.get("reached", False):
        g["xyz"] = list(xyz)
        return True
    # A refused/partial motion must not become the start of a further pull.
    ee = ctx.tool("robot.get_ee_pose", arm_id=arm)
    pos = ee.get("pose", ee)["position"]
    g["xyz"] = [pos[k] for k in ("x", "y", "z")]
    return False


def _holds(ctx, grips, p):
    samples, held = {}, True
    for arm, g in grips.items():
        at = ctx.tool("rope.point", s=g["s"])["position"]
        lag = math.dist(at[:2], g["xyz"][:2])
        dz = g["xyz"][2] - at[2]
        ok = lag <= p["slip_limit"] and 0.0 <= dz <= 0.035
        samples[str(arm)] = {"lag": round(lag, 4), "tcp_above_axis": round(dz, 4), "held": ok}
        held = held and ok
    return held, samples


def _directions(grips, p):
    # Follow measured placement/yaw, keeping the tested bias toward the robot.
    # Limit the yaw correction so neither arm drives toward the far edge.
    dx = grips[0]["xyz"][0] - grips[1]["xyz"][0]
    dy = grips[0]["xyz"][1] - grips[1]["xyz"][1]
    bias = math.sin(math.radians(p["angle_deg"]))
    ux = max(-bias, min(bias, dx / max(0.001, math.hypot(dx, dy))))
    uy = math.sqrt(1.0 - ux * ux)
    directions = {0: (ux - bias, uy), 1: (-ux - bias, -uy)}
    return {arm: [v / math.hypot(*d) for v in d] for arm, d in directions.items()}


def _room(xyz, direction, p):
    room = p["max_distance"]
    for x, d, low, high in zip(xyz, direction, (0.35, -p["pull_y_limit"]), (0.62, p["pull_y_limit"])):
        if abs(d) > 1e-6:
            room = min(room, ((high if d > 0 else low) - x) / d)
    return max(0.0, room - 0.005)


def _release(ctx, grips, p):
    # Put BOTH tails down before opening either. The old release opened 40 mm
    # above the bench, leaving a tail riding a finger at the next grasp.
    for arm, g in grips.items():
        if g is not None and g["held"]:
            _move(ctx, arm, g, (*g["xyz"][:2], BZ + p["height"]), p)
            r = ctx.tool("robot.go_to_pose", pose=_pose((*g["xyz"][:2], BZ + 0.0175), g["rot"]),
                         arm_id=arm, max_steps=90, tolerance=0.0025)
            g["release_reached"] = bool(r.get("reached", False))
    ctx.tool("robot.wait_steps", steps=12)
    for arm, g in grips.items():
        if g is not None:
            ctx.tool("robot.set_grip", width_m=0.06, arm_id=arm, ramp_steps=12, settle_steps=12)
    for arm, g in grips.items():
        if g is not None:
            # On a failed descent, rise from the actual hand, not through cord.
            ee = ctx.tool("robot.get_ee_pose", arm_id=arm)
            pos = ee.get("pose", ee)["position"]
            xyz = [pos[k] for k in ("x", "y", "z")]
            xyz[2] += 0.006
            ctx.tool("robot.go_to_pose", pose=_pose(xyz, g["rot"]), arm_id=arm, max_steps=60)
            ctx.tool("robot.set_grip", preset="open", arm_id=arm, ramp_steps=12, settle_steps=8)
            xyz[2] = max(BZ + 0.085, xyz[2] + 0.045)
            ctx.tool("robot.go_to_pose", pose=_pose(xyz, g["rot"]), arm_id=arm, max_steps=90)
    ctx.tool("robot.wait_steps", steps=30)
    return {str(arm): {"lowered": g.get("release_reached"),
                       "axis_z": ctx.tool("rope.point", s=g["s"])["position"][2]}
            for arm, g in grips.items() if g is not None}


def run(ctx, start_in_knot: float, cycle: int, next_distance: float,
        gain: float, used_steps: int) -> dict:
    p = _params()
    ctx = _Meter(ctx, used_steps)
    before = ctx.tool("rope.state")
    knot = _knot(before)
    report = {"cycle": cycle, "before": round(knot["in_knot"], 4)}
    route, moved, reason = "pulled", 0.0, "not_grasped"
    if not _overhand(before):
        route, reason = "lost", "settled_topology"
    elif before["tied"]:
        route, reason = "tight", "already_tight"
    else:
        grips = {}
        for arm in (0, 1):
            grips[arm] = _grasp(ctx, arm, knot, p)
            if grips[arm] is None or not grips[arm]["held"]:
                break
        report["grasps"] = {str(a): (g["attempts"] if g else []) for a, g in grips.items()}
        if len(grips) == 2 and all(g and g["held"] for g in grips.values()):
            dirs = _directions(grips, p)
            goal = p["goal_fraction"] * start_in_knot
            deficit = max(0.0, knot["in_knot"] - goal)
            distance = min(next_distance, max(p["min_distance"], 1.15 * deficit / max(gain, 0.15)),
                           *(_room(g["xyz"], dirs[a], p) for a, g in grips.items()))
            report.update(requested_distance=round(distance, 4), directions=dirs, gain_used=round(gain, 3))
            lifted = True
            for arm, g in grips.items():
                if not _move(ctx, arm, g, (*g["xyz"][:2], BZ + p["height"] + p["lift"]), p):
                    lifted = False
                    break
            held, sample = _holds(ctx, grips, p)
            report["lift_holds"] = sample
            if lifted and held:
                height = BZ + p["height"] + p["lift"]
                # A resistant knot still touching the bench gets more lift,
                # rather than another cycle dragging against table friction.
                if gain < 0.35 and ctx.used_steps < p["motion_budget"]:
                    airborne = ctx.tool("rope.state")
                    inner = [xyz[2] for s, xyz in zip(airborne["arc"], airborne["nodes"])
                             if knot["lo"] <= s <= knot["hi"]]
                    if inner and min(inner) < BZ + 0.012:
                        height = min(BZ + p["max_height"], height + 0.02)
                        for arm, g in grips.items():
                            if not _move(ctx, arm, g, (*g["xyz"][:2], height), p):
                                lifted = False
                                break
                        held, sample = _holds(ctx, grips, p)
                        report["extra_lift_holds"] = sample
                reason = "distance" if lifted and held else "extra_lift_miss"
                # Airborne crossings are projections. Monitor material holds
                # here; only the settled verifier judges topology.
                while lifted and held and moved < distance - 1e-6:
                    if ctx.used_steps >= p["motion_budget"]:
                        reason = "budget"
                        break
                    d = min(p["step"] if deficit > 0.03 else 0.01, distance - moved)
                    for arm, g in grips.items():
                        xyz = [g["xyz"][0] + dirs[arm][0] * d,
                               g["xyz"][1] + dirs[arm][1] * d, height]
                        if not _move(ctx, arm, g, xyz, p):
                            lifted, reason = False, "motion"
                            break
                    if not lifted:
                        break
                    moved += d
                    held, sample = _holds(ctx, grips, p)
                    report["last_holds"] = sample
                    if not held:
                        reason = "slip"
            else:
                reason = "lift_miss"
        report["release"] = _release(ctx, grips, p)
    after = ctx.tool("rope.state")
    final = _knot(after)
    improvement = knot["in_knot"] - final["in_knot"]
    # Learn only from complete, verified pulls: a miss is not evidence that
    # the material needs a longer stroke.
    if moved > 0.02 and reason == "distance" and _overhand(after):
        observed = max(0.05, min(1.5, improvement / moved))
        gain = 0.4 * gain + 0.6 * observed
    if _overhand(after):
        deficit = max(0.0, final["in_knot"] - p["goal_fraction"] * start_in_knot)
        next_distance = max(p["min_distance"], min(p["max_distance"], 1.15 * deficit / max(0.15, gain)))
    if after["tied"]:
        route = "tight"
    report.update(reason=reason, moved=round(moved, 4), after=round(final["in_knot"], 4),
                  reduction=round(improvement, 4), code=after["code"], gain_next=round(gain, 3))
    return {"route": route, "cycle": cycle + 1, "next_distance": next_distance,
            "gain": gain, "used_steps": ctx.used_steps, "report": report}
