"""Rope skills for the double coin knot: what a graph script calls, over the GaP tools.

Every function takes the node's ``ctx`` and talks only through ``ctx.tool``:
``robot.*`` to move, ``rope.*`` to read the rope. Positions are WORLD frame
(the bench's work-surface origin is world (0.6, 0, 0.75)); arms are named by
``arm_id`` (0 = left, 1 = right on ``yam_bimanual``). Each skill returns a small
dict of measurements -- what it asked for, what the rope did -- so a failed
rehearsal can be traced to the skill and the number that went wrong.

Defaults are the ones measured on the nominal variant (10 mm cord, SuperDex,
``feedback="off"``): see ``skills/rope/SKILL.md`` for the numbers behind them.
"""

from __future__ import annotations

import math
from typing import Any

BENCH_Z = 0.75
LOW = 0.0135
"""Tool centre over the bench to pinch [m]: the YAM fingers reach 12.1 mm past it, so the tips
stop ~1.4 mm over the bench, well under the 10 mm cord's equator (5 mm). At 16 mm the lifted
cord hung with its axis below the fingertips, held by the top half of its section, and one
left-hand pinch in two came up empty."""
CARRY = 0.035
"""Tool centre over the bench while carrying [m]: the held cord's underside rides ~18 mm up,
clear over a cord lying on the bench (10 mm), so a carried strand crosses OVER."""
APPROACH = 0.06
"""Pre-grasp hover over the pinch height [m]."""
FRACTION, RAMP, SETTLE = 0.9, 48, 30
"""Jaw close: fraction of full travel, ramp and settle steps. Snapped shut with no settle the
cord slid out of the tips on the first lift; ramped to 0.9 it is held through a 60 mm carry."""
SPEED = 0.25
"""Cartesian speed scale for every move holding the cord: 1.0 (~0.16 m/s) slips it."""
MAX_STEPS = 240
"""Cap on any one pose move [control steps]: a blocked descent must not burn 1000."""


def _pose(p, rot: dict) -> dict:
    return {"position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}, "rotation": rot}


def _outcome(r: Any) -> str:
    return str(r.get("outcome", "?")) if isinstance(r, dict) else str(r)


def _dist(a, b) -> float:
    return math.dist([float(x) for x in a[:2]], [float(x) for x in b[:2]])


def close_heading(heading_deg: float) -> float:
    """The jaws' closing heading across a rope running at ``heading_deg``, folded into [-90, 90).

    Either side of the rope is the same grasp; the folded one keeps the wrist
    near its canonical roll (jaws across world x) and away from its stops.
    """
    h = (heading_deg + 90.0 + 90.0) % 180.0 - 90.0
    return h


def frame(ctx, arm_id: int, heading_deg: float) -> dict:
    """The tool-down orientation with the jaws across a rope running at ``heading_deg``."""
    return ctx.tool("robot.grasp_frame", close_heading_deg=close_heading(heading_deg), arm_id=arm_id)["rotation"]


def ee(ctx, arm_id: int) -> tuple[list[float], dict] | tuple[None, None]:
    """Where the tool centre is now (world) and its rotation; ``robot.get_ee_pose`` nests both under ``pose``."""
    r = ctx.tool("robot.get_ee_pose", arm_id=arm_id)
    pose = r.get("pose", r) if isinstance(r, dict) else {}
    try:
        c = pose["position"]
        return [float(c["x"]), float(c["y"]), float(c["z"])], pose.get("rotation")
    except (KeyError, TypeError):
        return None, None


def _quat_angle_deg(a: dict, b: dict) -> float:
    dot = abs(sum(float(a[k]) * float(b[k]) for k in ("w", "x", "y", "z")))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def descend(ctx, arm_id: int, xyz, rot: dict, *, tolerance: float = 0.0025, max_steps: int = 90) -> str:
    """A short move with nothing dragging (down onto the cord, up off it), held to ``tolerance``.

    The servo's default ball is ~4 mm: descents stopped 2-4 mm high, leaving the
    fingertips barely under the cord's axis. At 1.5 mm half the descents onto the
    bench timed out; 2.5 mm is met.
    """
    r = ctx.tool("robot.go_to_pose", pose=_pose(xyz, rot), arm_id=arm_id, max_steps=int(max_steps), tolerance=float(tolerance))
    return _outcome(r)


def move(ctx, arm_id: int, xyz, rot: dict, *, speed: float = SPEED, seg: float = 0.012, seg_steps: int = 40) -> str:
    """A straight move of the tool centre in short servoed segments; returns the worst outcome.

    ``robot.go_to_pose_cartesian`` takes no step cap, and one unreachable descent
    burned its 1000-step default (5 minutes of a rehearsal). Segments of ``seg``
    metres, each capped at ``seg_steps``, bound a move and keep it slow: the
    servo ramps up and down inside every segment. ``speed`` > 0.5 doubles the
    segment length.
    """
    seg = seg * (2.0 if speed > 0.5 else 1.0)
    start, _ = ee(ctx, arm_id)
    if start is None:
        start = list(xyz)
    d = math.dist(start, [float(v) for v in xyz])
    n = max(1, int(math.ceil(d / seg)))
    worst = "reached"
    for i in range(1, n + 1):
        u = i / n
        p = [start[k] + u * (float(xyz[k]) - start[k]) for k in range(3)]
        r = ctx.tool("robot.go_to_pose", pose=_pose(p, rot), arm_id=arm_id, max_steps=int(seg_steps) if i < n else 2 * int(seg_steps))
        o = _outcome(r)
        if o != "reached" and i == n:
            worst = o
    return worst


def hover(ctx, arm_id: int, xyz, rot: dict, *, max_steps: int = 300) -> str:
    """A free move with nothing held (fast, servoed); returns its outcome."""
    r = ctx.tool("robot.go_to_pose", pose=_pose(xyz, rot), arm_id=arm_id, max_steps=max_steps)
    return _outcome(r)


def rope_progress(ctx) -> dict:
    """The knot's code against the target, and its extent: the widest spread of its crossings [m]."""
    st = ctx.tool("rope.state")
    pts = [c["at"] for c in st["crossings"]]
    extent = 0.0
    if len(pts) > 1:
        extent = max(max(p[k] for p in pts) - min(p[k] for p in pts) for k in (0, 1))
    return {"progress": st["progress"], "prefix": st["prefix"], "code": st["code"], "n_cross": len(st["crossings"]),
            "extent": round(extent, 4)}


def pinch(
    ctx,
    arm_id: int,
    s: float,
    *,
    low: float = LOW,
    approach: float = APPROACH,
    fraction: float = FRACTION,
    ramp: int = RAMP,
    settle: int = SETTLE,
    verify_lift: float = 0.02,
    open_width: float = 0.045,
    offset_xy=(0.0, 0.0),
) -> dict:
    """Take the rope ``s`` metres from its first end: over it, down, jaws across it, close, lift a little.

    ``held`` is measured, not assumed: the point must rise at least half of
    ``verify_lift`` with the hand. ``open_width`` > 0 opens the jaws only that wide
    (a crowded knot: the full 95 mm span sweeps neighbouring strands).
    """
    pt = ctx.tool("rope.point", s=float(s))
    x, y, z0 = pt["position"]
    x, y = x + offset_xy[0], y + offset_xy[1]
    # A strand lying on another sits ~9 mm up, not 5: the tips go 3.6 mm under its axis either way.
    low = max(float(low), float(z0) - BENCH_Z + 0.0085)
    rot = frame(ctx, arm_id, pt["heading_deg"])
    if open_width > 0:
        ctx.tool("robot.set_grip", width_m=float(open_width), arm_id=arm_id, settle_steps=6)
    else:
        ctx.tool("robot.set_grip", preset="open", arm_id=arm_id, settle_steps=6)
    out = {"skill": "pinch", "arm_id": arm_id, "s": float(s), "at": [round(x, 4), round(y, 4)]}
    out["above"] = hover(ctx, arm_id, (x, y, BENCH_Z + low + approach), rot)
    out["down"] = descend(ctx, arm_id, (x, y, BENCH_Z + low), rot)
    g = ctx.tool("robot.set_grip", fraction_closed=float(fraction), arm_id=arm_id, ramp_steps=int(ramp), settle_steps=int(settle))
    out["width"] = g.get("command_width_m")
    out["up"] = descend(ctx, arm_id, (x, y, BENCH_Z + low + verify_lift), rot, tolerance=0.003)
    after = ctx.tool("rope.point", s=float(s))
    rise = float(after["position"][2]) - float(z0)
    out.update(rise=round(rise, 4), held=bool(rise > 0.4 * verify_lift), rot=rot, z_tcp=BENCH_Z + low + verify_lift)
    return out


def carry(ctx, arm_id: int, path_xy, rot: dict, *, height: float = CARRY, speed: float = SPEED, s: float | None = None) -> dict:
    """Carry the held rope through ``path_xy`` (world xy waypoints) at ``height`` over the bench.

    With ``s`` the held point is read after the last waypoint: ``lag`` is how far it
    trails the hand in xy, which is slip in the fingers.
    """
    out = {"skill": "carry", "arm_id": arm_id, "height": height, "outcomes": []}
    for p in path_xy:
        # Speed-scaled cartesian, not servo segments: a servo segment runs at full speed and the
        # cord slid 40-120 mm through the pads (measured); at 0.25 it trailed the hand by <= 6 mm.
        r = ctx.tool("robot.go_to_pose_cartesian", pose=_pose((p[0], p[1], BENCH_Z + height), rot), arm_id=arm_id, speed_scale=speed)
        out["outcomes"].append(_outcome(r))
    if s is not None and path_xy:
        now = ctx.tool("rope.point", s=float(s))["position"]
        out["lag"] = round(_dist(now, path_xy[-1]), 4)
    out["reached"] = all(o == "reached" for o in out["outcomes"])
    return out


def place(
    ctx,
    arm_id: int,
    xy,
    rot: dict,
    *,
    s: float | None = None,
    overshoot_xy=(0.0, 0.0),
    release_z: float = LOW + 0.004,
    retreat: float = 0.05,
    height: float = CARRY,
    dwell: int = 20,
    release_width: float = 0.06,
) -> dict:
    """Set the held rope down at ``xy`` (+ ``overshoot_xy``), open, rise straight up.

    With ``s``: ``error`` is how far point s ends from ``xy`` once let go (spring-back
    plus any carry slip); ``spring_back`` is how far it moved on release.
    """
    tx, ty = float(xy[0]) + overshoot_xy[0], float(xy[1]) + overshoot_xy[1]
    out = {"skill": "place", "arm_id": arm_id, "to": [round(float(xy[0]), 4), round(float(xy[1]), 4)]}
    out["over"] = _outcome(ctx.tool("robot.go_to_pose_cartesian", pose=_pose((tx, ty, BENCH_Z + height), rot), arm_id=arm_id, speed_scale=SPEED))
    out["lower"] = descend(ctx, arm_id, (tx, ty, BENCH_Z + release_z), rot)
    if dwell > 0:
        ctx.tool("robot.wait_steps", steps=int(dwell))
    held = ctx.tool("rope.point", s=float(s))["position"] if s is not None else None
    # Opened to 60 mm, not the full 95 mm span, which sweeps the strands either side; at 30 mm a cord
    # under tension stays pressed on one finger and rides up with it (measured, mu 1.0: 51 mm up).
    ctx.tool("robot.set_grip", width_m=float(release_width), arm_id=arm_id, settle_steps=18)
    # Clear of the strands beside it by 6 mm, open the rest of the way: even at 60 mm a cord under
    # tension stayed on one finger and rode 30-50 mm up with the rising hand.
    descend(ctx, arm_id, (tx, ty, BENCH_Z + release_z + 0.006), rot, tolerance=0.004)
    ctx.tool("robot.set_grip", preset="open", arm_id=arm_id, settle_steps=8)
    out["rise"] = descend(ctx, arm_id, (tx, ty, BENCH_Z + release_z + retreat), rot, tolerance=0.004)
    if s is not None:
        now = ctx.tool("rope.point", s=float(s))["position"]
        out["error"] = round(_dist(now, xy), 4)
        out["spring_back"] = round(_dist(now, held), 4)
    return out


def lay(
    ctx,
    arm_id: int,
    s: float,
    xy,
    *,
    via=(),
    overshoot_xy=(0.0, 0.0),
    height: float = CARRY,
    speed: float = SPEED,
    place_kw: dict | None = None,
    retries: int = 1,
    **pinch_kw,
) -> dict:
    """Pinch point ``s``, carry it (through ``via``) to ``xy`` and set it down there: one placement step."""
    p = pinch(ctx, arm_id, s, **pinch_kw)
    tries = 1
    while not p["held"] and tries <= retries:
        # A miss usually nudged the cord aside: open, rise, read where it is now and go again.
        retreat(ctx, arm_id, p["rot"], up=0.04)
        p = pinch(ctx, arm_id, s, **pinch_kw)
        tries += 1
    out = {"skill": "lay", "s": s, "tries": tries, "pinch": {k: v for k, v in p.items() if k != "rot"}}
    if not p["held"]:
        out["ok"] = False
        out["failure"] = "not_held"
        ctx.tool("robot.set_grip", preset="open", arm_id=arm_id, settle_steps=6)
        retreat(ctx, arm_id, p["rot"])
        return out
    c = carry(ctx, arm_id, [*via, xy], p["rot"], height=height, speed=speed, s=s)
    out["carry"] = c
    out["place"] = place(ctx, arm_id, xy, p["rot"], s=s, overshoot_xy=overshoot_xy, height=height, **(place_kw or {}))
    out["ok"] = bool(c["reached"]) and out["place"].get("error", 0.0) < 0.02
    if not out["ok"]:
        out["failure"] = "slipped" if c.get("lag", 0.0) > 0.015 else ("unreached" if not c["reached"] else "off_target")
    return out


def lift_strand(ctx, arm_id: int, s: float, *, height: float = 0.06, **pinch_kw) -> dict:
    """The helper hand takes the strand at ``s`` and raises it, an arch the other hand can thread under.

    The strand's own height at the hand is read back (``strand_z``): the cord hangs
    from the fingertips, ~12 mm under the tool centre.
    """
    p = pinch(ctx, arm_id, s, **pinch_kw)
    out = {"skill": "lift_strand", "s": s, "pinch": {k: v for k, v in p.items() if k != "rot"}, "rot": p["rot"]}
    if not p["held"]:
        out["ok"] = False
        out["failure"] = "not_held"
        return out
    x, y = p["at"]
    out["up"] = move(ctx, arm_id, (x, y, BENCH_Z + height), p["rot"])
    out["strand_z"] = round(float(ctx.tool("rope.point", s=float(s))["position"][2]) - BENCH_Z, 4)
    out["ok"] = out["strand_z"] > 0.5 * height
    return out


def lower_strand(ctx, arm_id: int, rot: dict, xy, *, s: float | None = None) -> dict:
    """Lay the lifted strand back down at ``xy`` and let go."""
    return place(ctx, arm_id, xy, rot, s=s, height=0.06)


REACH_BENCH = ((-0.25, -0.05), (-0.20, 0.20))
"""Bench x and y both hands reach tool-down [m]."""


def in_reach(xy, margin: float = 0.0) -> bool:
    bx, by = float(xy[0]) - 0.6, float(xy[1])
    (x0, x1), (y0, y1) = REACH_BENCH
    return x0 + margin <= bx <= x1 - margin and y0 + margin <= by <= y1 - margin


FETCH_BENCH = ((-0.26, 0.0), (-0.215, 0.215))
"""Where a hand can still take the cord to pull it in [m, bench]: the servo reached bench x +0.02 at
y -0.2 in a probe, past the tool-down IK grid's -0.05."""


def fetch_end(ctx, arm_id: int, end_first: bool, *, pull: float = 0.15, to_xy=None) -> dict:
    """Bring a free end into reach: take the reachable point nearest to it and draw it on, away from the end.

    The end follows along the cord when it is pulled along the cord's own line
    (pulled sideways, the cord just bends and the end stays: measured). ``to_xy``
    overrides the destination.
    """
    st = ctx.tool("rope.state")
    nodes, arc = st["nodes"], st["arc"]
    (x0, x1), (y0, y1) = FETCH_BENCH
    ok_at = lambda p: x0 <= p[0] - 0.6 <= x1 and y0 <= p[1] <= y1
    order = list(range(len(nodes))) if end_first else list(range(len(nodes) - 1, -1, -1))
    k = next((i for i in order if ok_at(nodes[i])), None)
    end = nodes[order[0]]
    out = {"skill": "fetch_end", "end_first": end_first, "end_before": [round(v, 4) for v in end[:2]]}
    if in_reach(end):
        out.update(ok=True, already=True, end_at=out["end_before"])
        return out
    if k is None:
        out.update(ok=False, failure="nothing_in_reach")
        return out
    s = float(arc[k])
    p = nodes[k]
    if to_xy is None:
        ux, uy = p[0] - end[0], p[1] - end[1]
        n = math.hypot(ux, uy) or 1.0
        to_xy = (p[0] + ux / n * pull, p[1] + uy / n * pull)
    out["took_s"] = round(s, 4)
    out["to"] = [round(v, 4) for v in to_xy]
    out["lay"] = lay(ctx, arm_id, s, to_xy)
    tip = ctx.tool("rope.point", s=0.0 if end_first else float(st["length"]))["position"]
    out["end_at"] = [round(v, 4) for v in tip[:2]]
    out["ok"] = bool(out["lay"].get("pinch", {}).get("held")) and in_reach(tip, -0.01)
    if not out["ok"]:
        out["failure"] = "not_held" if not out["lay"].get("pinch", {}).get("held") else "end_still_out_of_reach"
    return out


def thread_end(
    ctx,
    arm_id: int,
    helper_arm: int,
    end_first: bool,
    lift_s: float,
    through_xy,
    beyond_xy,
    *,
    grasp_back: float = 0.06,
    drag: float = 0.075,
    margin: float = 0.02,
    lift: float = 0.06,
    pull_height: float = 0.022,
) -> dict:
    """Pass the rope's free end UNDER the strand at ``lift_s``, at ``through_xy``, heading for ``beyond_xy``.

    A tool-down hand cannot carry cord under a strand: its own column hits the
    strand. So the end is pushed under:

    1. take the cord ``grasp_back`` from its end; set it down ``margin`` short of
       the crossing and drag it straight back ``drag``: the end trails behind the
       hand, pointing at the crossing, its tip ~35 mm short of the strand;
    2. still holding, the helper lifts the strand into an arch;
    3. slide forward until the fingers are ``margin`` short of the crossing: the
       end goes under the arch, ``grasp_back - margin`` past it;
    4. let go, take the end again beyond the arch, draw it to ``beyond_xy``;
    5. the helper lays the strand back down, over the threaded cord.

    ``end_first``: thread node 0's end (half A) or the last node's (half B).
    """
    st = ctx.tool("rope.state")
    length = float(st["length"])
    tip_s = 0.0 if end_first else length
    sign = 1.0 if end_first else -1.0
    tx, ty = float(through_xy[0]), float(through_xy[1])
    ux, uy = float(beyond_xy[0]) - tx, float(beyond_xy[1]) - ty
    n = math.hypot(ux, uy) or 1.0
    ux, uy = ux / n, uy / n
    back = lambda d: (tx - ux * d, ty - uy * d)
    out = {"skill": "thread_end", "lift_s": lift_s, "through": [round(tx, 4), round(ty, 4)]}
    # 1. Take the cord near its end and drag it back from the crossing, so the end trails toward it.
    p = pinch(ctx, arm_id, tip_s + sign * grasp_back)
    out["pinch"] = {k: v for k, v in p.items() if k != "rot"}
    if not p["held"]:
        out.update(ok=False, failure="not_held")
        return out
    rot = p["rot"]
    h1, h0 = back(margin), back(margin + drag)
    cart = lambda xy, z, sp=SPEED: _outcome(
        ctx.tool("robot.go_to_pose_cartesian", pose=_pose((xy[0], xy[1], BENCH_Z + z), rot), arm_id=arm_id, speed_scale=sp)
    )
    out["to_strand"] = cart(h1, CARRY)
    out["down"] = descend(ctx, arm_id, (h1[0], h1[1], BENCH_Z + LOW + 0.002), rot)
    out["drag_back"] = cart(h0, LOW + 0.002, 0.15)
    tip = ctx.tool("rope.point", s=tip_s)["position"]
    out["tip_short"] = round(-((tip[0] - tx) * ux + (tip[1] - ty) * uy), 4)  # how far short of the crossing
    # 2. The helper lifts the strand.
    hl = lift_strand(ctx, helper_arm, lift_s, height=lift)
    out["lift"] = {k: v for k, v in hl.items() if k != "rot"}
    if not hl.get("ok"):
        if hl.get("rot"):
            retreat(ctx, helper_arm, hl["rot"])
        retreat(ctx, arm_id, rot)
        out.update(ok=False, failure="lift_failed")
        return out
    strand_xy = hl["pinch"]["at"]
    # 3. Push the end under.
    out["push"] = cart(h1, LOW + 0.002, 0.15)
    ctx.tool("robot.set_grip", width_m=0.06, arm_id=arm_id, settle_steps=12)
    move(ctx, arm_id, (h1[0], h1[1], BENCH_Z + 0.11), rot, speed=1.0)
    tip = ctx.tool("rope.point", s=tip_s)["position"]
    out["tip_past"] = round((tip[0] - tx) * ux + (tip[1] - ty) * uy, 4)
    # 4. Take the end again beyond the arch and draw it through.
    q = pinch(ctx, arm_id, tip_s + sign * 0.010, open_width=0.035)
    out["regrasp"] = {k: v for k, v in q.items() if k != "rot"}
    if not q["held"]:
        retreat(ctx, arm_id, q["rot"])
        lower_strand(ctx, helper_arm, hl["rot"], strand_xy)
        out.update(ok=False, failure="regrasp_missed")
        return out
    out["pull"] = carry(ctx, arm_id, [beyond_xy], q["rot"], height=pull_height, s=tip_s)
    out["place"] = place(ctx, arm_id, beyond_xy, q["rot"], s=tip_s, height=pull_height)
    # 5. The strand back down, over the threaded cord.
    out["lower"] = lower_strand(ctx, helper_arm, hl["rot"], strand_xy, s=lift_s)
    # 6. Did it go under? The crossing between the strand near lift_s and the threaded end's run.
    st = ctx.tool("rope.state")
    verdict = None
    for c in st["crossings"]:
        da, db = abs(c["s_a"] - lift_s), abs(c["s_b"] - lift_s)
        if min(da, db) > 0.03:
            continue
        strand_is_a = da < db
        other = c["s_b"] if strand_is_a else c["s_a"]
        if (end_first and other < lift_s) or (not end_first and other > lift_s):
            strand_over = c["a_over"] if strand_is_a else not c["a_over"]
            verdict = {"other_s": round(other, 4), "strand_over": bool(strand_over), "at": c["at"]}
            break
    out["crossing"] = verdict
    out["ok"] = bool(verdict and verdict["strand_over"])
    if not out["ok"]:
        out["failure"] = "no_crossing" if verdict is None else "went_over"
    return out


def pull(
    ctx,
    arm_ids=(0, 1),
    *,
    s_left: float | None = None,
    s_right: float | None = None,
    direction_left=(0.0, 1.0),
    direction_right=(0.0, -1.0),
    distance: float = 0.06,
    step: float = 0.01,
    height: float = 0.03,
    stop_progress_drop: bool = True,
) -> dict:
    """Both hands take the tails and draw them apart in small alternating steps, the knot read between.

    Stops early if the knot's code loses ground (``progress`` falls), which is the
    sign a crossing is being pulled out rather than dressed.
    """
    st = ctx.tool("rope.state")
    length = float(st["length"])
    sl = s_left if s_left is not None else length - 0.04
    sr = s_right if s_right is not None else 0.04
    before = rope_progress(ctx)
    grips = {}
    for arm, s in ((arm_ids[0], sl), (arm_ids[1], sr)):
        grips[arm] = pinch(ctx, arm, s)
    out = {"skill": "pull", "before": before, "held": {a: g["held"] for a, g in grips.items()}}
    if not all(g["held"] for g in grips.values()):
        out.update(ok=False, failure="not_held")
        for a, g in grips.items():
            ctx.tool("robot.set_grip", preset="open", arm_id=a, settle_steps=6)
        return out
    pos = {a: list(g["at"]) for a, g in grips.items()}
    dirs = {arm_ids[0]: direction_left, arm_ids[1]: direction_right}
    moved, best = 0.0, before["progress"]
    while moved < distance - 1e-9:
        d = min(step, distance - moved)
        for a in arm_ids:
            pos[a] = [pos[a][0] + dirs[a][0] * d, pos[a][1] + dirs[a][1] * d]
            move(ctx, a, (pos[a][0], pos[a][1], BENCH_Z + height), grips[a]["rot"], speed=0.15)
        moved += d
        now = rope_progress(ctx)
        if stop_progress_drop and now["progress"] < best - 1e-9:
            out["stopped"] = "progress_drop"
            break
        best = max(best, now["progress"])
    out["pulled"] = round(moved, 4)
    for a in arm_ids:
        ctx.tool("robot.set_grip", preset="open", arm_id=a, settle_steps=10)
        move(ctx, a, (pos[a][0], pos[a][1], BENCH_Z + height + 0.05), grips[a]["rot"], speed=0.5)
    ctx.tool("robot.wait_steps", steps=30)  # a tail the rising hand lifted falls back before the knot is read
    out["after"] = rope_progress(ctx)
    out["tightened"] = round(before["extent"] - out["after"]["extent"], 4)
    out["ok"] = out["after"]["progress"] >= before["progress"]
    return out


def retreat(ctx, arm_id: int, rot: dict | None = None, *, up: float = 0.08) -> str:
    """Open and rise straight up from wherever the hand is."""
    ctx.tool("robot.set_grip", width_m=0.06, arm_id=arm_id, settle_steps=6)
    p, _ = ee(ctx, arm_id)
    if p is None:
        return "unknown_pose"
    rot = rot or ctx.tool("robot.grasp_frame", arm_id=arm_id)["rotation"]
    return move(ctx, arm_id, (p[0], p[1], max(p[2], BENCH_Z) + up), rot, speed=0.5)


def home(ctx, arm_id: int) -> str:
    """Back to the arm's home posture, out of the board's way."""
    r = ctx.tool("robot.go_home", arm_id=arm_id)
    return _outcome(r)
