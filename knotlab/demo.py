"""The overhand knot as the Howcast clip ties it, retargeted to the YAM pair: ``demo_steps(layout)``.

The demonstration (``video/overhand/demo_plan.json``, extracted by ``video/overhand/demo_extract.py`` from the
clip's wide shots and close-ups; arc lengths ``s`` from the working end's tip, the person's RIGHT end):

- step 1 (19.3-25.8 s). The left hand holds the standing part (event 1); the right hand takes the rope 0.15 m
  from the tip and folds the end back alongside the standing part (2-3); the left hand lets go (4), takes the
  end's run 0.155 m from the tip and lays it ACROSS the standing part (5-6: X1, the end OVER), then holds X1
  (7) while the right hand turns the loop (8-9), takes the end 0.08 m from its tip and draws it out (10): the
  loop shrinks from 0.62 m of rope to 0.40 m. One crossing: "1O 1U" from the tip.
- step 2 (25.9-31.4 s). The left hand keeps X1 pinched, which holds the loop strand just past it (on the
  standing side) up off the table (11); the right hand takes the end (12), brings it across the eye (13-14),
  tucks the tip UNDER that strand (15: X2) and draws it out of the eye OVER the loop's far side (16: X3).
  Loose knot: "1O 2U 3O 1U 2O 3U" from the tip (the trefoil; the task's reference code is its mirror and
  either counts). From the tip: X3 at 0.104 / 0.425 m, X2 at 0.166 / 0.492, X1 at 0.219 / 0.541; 0.437 m of
  rope from the first crossing to the last.
- step 3 (33.5-35.8 s). The left hand takes the standing part at 0.57 m, the right hand the end at 0.07 m; they
  draw apart along the rope (0.14-0.21 m) and the knot closes to ~4 cm.

Retargeting (what changes on the YAM pair and why):

- Ends and hands. The working end is node 0 (s = 0), the robot's RIGHT end: the right arm (arm_id 1) works
  the end throughout, as the person's right hand does from event 10 on, and the left arm (arm_id 0) holds the
  standing part and X1 and lifts the strand, as the left hand does in steps 1-2. One swap: the person's LEFT
  hand lays the end over the standing part (events 5-6) while the right holds the fold; here the right arm
  carries the end across while the left holds the standing part (a tool-down hand that has just set the end
  down beyond the crossing is the one holding the end, and the left arm stays on the standing part).
- Mirror. The person's loop lies on the far side (+x) of the rope's line. The sim's line is at the far edge
  of reach (world x ~0.53), so the plan is mirrored across the line: the loop lies toward the robot and the
  end crosses the standing part going away from it. A planar reflection changes neither which strand is over
  at a crossing nor the order the crossings are met, so the Gauss code is the demo's; only the handedness flips.
- Bring the standing part in. The person's rope lies mid-table; the sim's lies at the edge of reach, with no
  room beyond it for the end to cross to. The standing part's run is first laid ``stage_in`` toward the robot
  (two lays, then the corner), the one step the person does not do.
- Scale. Arc lengths and distances scale by the knot: the demo's 0.437 m between the first and last crossing
  maps to the sim's 15 cm knot's 0.503 m (``rope.layout``'s target), x1.15. So the right hand takes the end
  0.07 m from its tip to lay it over (the person: 0.155 m from the tip, but the end then hung free in the air;
  on the bench the part beyond the hand must clear the standing part, so the hand holds it shorter and the end
  is drawn out afterwards, as the person does in event 10), draws it out to put ~0.15-0.25 m beyond X1, tucks
  it with 0.06 m (the person: >= 0.124 m), and the tails are taken 0.08 m (end) and 0.04 m (standing part)
  from their tips.
- Reach. Every target is clipped into world x 0.35-0.55, |y| <= 0.20 (``REACH``).
- Feedback. The person watches the rope; a placed cord here ends 2-5 cm off and drags its neighbours. So
  only step 1's lays are planned from the layout; the hold, the tuck, the draw-through and the pull are
  computed from the rope as it lies when their phase starts (``demo_steps(layout, state=rope.state())``):
  X1 is found in the diagram, the strand to lift is the one ``lift_back`` past it on the standing side, the
  push heads from outside that strand toward the eye's centre, and X3 is the loop strand the tip crosses on
  its way out.

A tool-down hand carries cord only OVER cord, and a mid-rope point carried across a strand drags both its sides
across (two crossings): a single crossing is made by a carry whose far side is free and short. Hence X1 is
laid with the hand near the tip, and X2, the one UNDER crossing, is made with the free end pushed under a
strand the other hand lifts.

Step dicts carry lay_plan's fields (k, half, s, d, xy (world), arm_id, kind, makes, helper_arm, lift_s,
reachable) plus ``phase`` (loop | over | thread | tail | pull: the graph's subgraphs) and ``event`` (the demo
event reproduced). Kinds:

- ``lay``: pinch ``s`` with ``arm_id``, carry through ``via`` (world xy) at ``height`` to ``xy``, place.
- ``hold``: pinch ``s`` and keep it pressed on the bench (the rope beyond it does not move) until ``release``.
- ``release``: open and rise.
- ``aim_end``: the working arm takes the cord ``grasp_back`` from its free end (``end_first``: node 0's), sets
  it down ``margin`` short of the crossing ``through_xy`` on the line toward ``xy`` and drags it straight back
  ``drag``, so the end trails toward the crossing; it keeps holding (the first half of ``skills.thread_end``).
- ``lift_hold``: the helper takes the strand at ``lift_s`` and raises it to ``height`` (``skills.lift_strand``),
  holding it.
- ``push_under``: the aimed arm slides forward until its fingers are ``margin`` short of the crossing, which
  pushes the end under the arch; it opens and rises (``tip_past``: how far beyond the crossing the tip went).
- ``lower``: the helper carries the lifted strand ``sweep`` back along the push (over the pushed tip) and lays
  it down there.
- ``pull_through``: the working arm takes the end again ``grasp_back`` from its tip (jaws ``open_width``),
  carries it through ``via`` to ``xy`` at ``height`` and places it (over whatever it passes).
- ``pull``: both arms take the tails (``s_left`` with arm 0, ``s_right`` with arm 1) and draw them apart along
  ``direction_left`` / ``direction_right`` by ``distance`` (``skills.pull``).

``graphs/overhand_demo/scripts/ov_exec.py`` runs every kind over ``knotlab.skills`` (its pinch takes a point on
the start line with the jaws centred short of it: pinches at world x 0.53 came up empty). The lay_plan graph's
scripts (graphs/overhand_v0) know only ``lay`` and ``thread``; the demo graph dispatches on every kind above.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from knotlab import diagram

DEMO_PLAN = Path(__file__).resolve().parents[1] / "video" / "overhand" / "demo_plan.json"
BENCH_ORIGIN = np.array([0.6, 0.0, 0.75])
REACH = ((0.35, 0.55), (-0.20, 0.20))  # world x, y both hands reach tool-down
DEMO_KNOT_ROPE = 0.437  # demo_plan.json retarget_numbers.knot_rope_first_to_last_crossing [m]

DEFAULTS = {
    # Step 1 (frame of the start line: x away from the robot, y along it toward the standing part's end; metres).
    "stage_in": -0.10,  # the standing part's run is laid this far from the start line (toward the robot)
    "stage_s": (0.42, 0.30, 0.20),  # ... these points of it, at these positions along the line:
    "stage_y": (0.05, -0.05, -0.15),
    "x1_grasp": 0.07,  # the end is laid over the standing part held this far from its tip
    "x1_via": ((-0.09, -0.10), (-0.08, 0.0)),  # ... round the robot's side of the loop (from the standing part) to be
    "x1_beyond": 0.07,  # ... set down this far beyond the standing part
    "x1_y": 0.0,  # ... crossing it here along the line
    "hold_beyond": 0.03,  # step 2: the left hand holds the standing part this far past X1 (outside the loop)
    "draw_grasp": 0.03,  # the right hand draws the end out taken this far from its tip
    "draw": 0.12,  # ... by this much, along the end's exit from X1
    "lift_back": 0.05,  # the strand lifted is this far past X1 into the loop on the standing side
    "lift": 0.06,
    "grasp_back": 0.06,
    "margin": 0.015,
    "drag": 0.05,
    "sweep": 0.02,
    "pull_grasp": 0.012,
    "tail_out": 0.045,  # the drawn-through tip is set down this far beyond X3
    "end_back": 0.08,  # step 3: the end is taken this far from its tip (demo 0.07 x 1.15)
    "stand_back": 0.04,  # the standing part this far from its tip (its tail is short on the 0.76 m cord)
    "pull_distance": 0.04,
}


def load_demo() -> dict:
    """The extracted demonstration, or {} if it has not been extracted."""
    try:
        return json.loads(DEMO_PLAN.read_text())
    except (OSError, ValueError):
        return {}


def line_frame(layout: dict) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The start line's middle (world xy), the unit along it toward the last node's end, and the unit away from
    the robot. From the layout's target, whose two tails lie on the start's line."""
    tgt = np.asarray(layout["target_nodes"], dtype=np.float64)
    a, b = tgt[0, :2], tgt[-1, :2]
    u = (b - a) / (np.linalg.norm(b - a) or 1.0)
    v = np.array([u[1], -u[0]])  # u turned -90 deg: away from the robot when u is +y
    return 0.5 * (a + b), u, v


def mirrored_target(layout: dict) -> np.ndarray:
    """The layout's target reflected across the start line and walked from the other end: the sim's 15 cm knot
    with the demo's code from node 0 (1O 2U 3O 1U 2O 3U)."""
    o, u, v = line_frame(layout)
    tgt = np.asarray(layout["target_nodes"], dtype=np.float64)[::-1].copy()
    rel = tgt[:, :2] - o
    tgt[:, :2] = o + np.outer(-(rel @ u), u) + np.outer(rel @ v, v)
    return tgt


def scale(layout: dict) -> float:
    """Demo metres to sim metres: the sim knot's rope between its first and last crossing over the demo's."""
    m = mirrored_target(layout)
    cs = diagram.crossings(m - BENCH_ORIGIN)
    if not cs:
        return 1.15
    ss = [c.s_i for c in cs] + [c.s_j for c in cs]
    return (max(ss) - min(ss)) / DEMO_KNOT_ROPE


def clip_reach(xy, margin: float = 0.005) -> list[float]:
    (x0, x1), (y0, y1) = REACH
    return [round(float(np.clip(xy[0], x0 + margin, x1 - margin)), 4), round(float(np.clip(xy[1], y0 + margin, y1 - margin)), 4)]


def _reachable(xy) -> bool:
    (x0, x1), (y0, y1) = REACH
    return x0 - 0.01 <= xy[0] <= x1 + 0.01 and y0 <= xy[1] <= y1


def _unit(a) -> np.ndarray:
    a = np.asarray(a, dtype=np.float64)
    return a / (np.linalg.norm(a) or 1.0)


def find_x1(state: dict) -> dict | None:
    """X1 in the rope as it lies: the crossing where the end side (the strand nearer node 0) lies OVER, the one
    enclosing the most rope (the loop); None if there is none."""
    best = None
    for c in state.get("crossings", []):
        if not c["a_over"]:
            continue
        span = c["s_b"] - c["s_a"]
        if span > 0.12 and (best is None or span > best["s_b"] - best["s_a"]):
            best = c
    return best


def _point(state: dict, s: float) -> np.ndarray:
    p = np.asarray(state["nodes"], dtype=np.float64)
    arc = np.asarray(state["arc"], dtype=np.float64)
    return np.array([np.interp(s, arc, p[:, 0]), np.interp(s, arc, p[:, 1])])


def _crosses(a, b, poly) -> int:
    """How many segments of the polyline ``poly`` the segment a-b crosses (xy)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = 0
    for p, q in zip(poly[:-1], poly[1:]):
        d1, d2 = b - a, q - p
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-12:
            continue
        r = p - a
        t_ = (r[0] * d2[1] - r[1] * d2[0]) / den
        u_ = (r[0] * d1[1] - r[1] * d1[0]) / den
        n += int(0.0 < t_ < 1.0 and 0.0 <= u_ <= 1.0)
    return n


def draw_target(state: dict, tip, x1_xy, sa: float, draw: float) -> list[float]:
    """Where the right hand draws the end out to (demo event 10): ``draw`` from the tip, in the direction
    (every 15 deg) that stays inside reach and away from X1, and whose straight path crosses no other strand;
    the longest such draw wins, ties to the end's own exit from X1."""
    p = np.asarray(state["nodes"], dtype=np.float64)[:, :2]
    arc = np.asarray(state["arc"], dtype=np.float64)
    rest = p[arc > sa + 0.02]  # the rope the end must not be drawn across (the loop, the standing part)
    exit_ = _unit(np.asarray(tip) - np.asarray(x1_xy))
    best, key = None, None
    for ang in np.radians(np.arange(0.0, 360.0, 15.0)):
        d = np.array([math.cos(ang), math.sin(ang)])
        if float(d @ exit_) < 0.0:
            continue
        dest = np.asarray(clip_reach(np.asarray(tip) + draw * d), dtype=np.float64)
        got = float(np.linalg.norm(dest - tip))
        if _crosses(tip, dest, rest):
            continue
        k = (round(got, 2), float(d @ exit_))
        if key is None or k > key:
            best, key = dest, k
    return [round(float(c), 4) for c in (best if best is not None else clip_reach(np.asarray(tip) + draw * exit_))]


def demo_steps(layout: dict, state: dict | None = None, **overrides) -> list[dict]:
    """The robot steps that reproduce the demonstration on this episode's rope.

    ``layout``: ``rope.layout``'s answer. ``state``: ``rope.state()`` now, to plan the hold, the tuck, the draw
    and the pull from the rope as it lies (the graph passes it at the start of each phase); without it they are
    planned on the reference knot's positions (a preview).
    """
    P = {**DEFAULTS, **overrides}
    o, u, v = line_frame(layout)
    m = mirrored_target(layout)
    arc_m = diagram.arc_length(m)
    L = float(arc_m[-1])
    W = lambda x, y: o + x * v + y * u  # line frame -> world xy
    steps: list[dict] = []

    def add(kind, phase, event, arm, s=None, xy=None, **kw):
        d = {"k": len(steps), "kind": kind, "phase": phase, "event": event, "arm_id": int(arm),
             "s": None if s is None else round(float(s), 4), "half": None if s is None else ("A" if s < 0.5 * L else "B"),
             "d": None if s is None else round(abs(float(s) - 0.5 * L), 4),
             "xy": None if xy is None else [round(float(xy[0]), 4), round(float(xy[1]), 4)],
             "makes": [], "helper_arm": None, "lift_s": None, "reachable": True}
        d.update(kw)
        if d["xy"] is not None:
            d["reachable"] = _reachable(d["xy"])
        steps.append(d)
        return d

    # ---- step 1 (phase loop): the standing part brought in toward the robot (demo event 1, the sim's extra).
    for s, y in zip(P["stage_s"], P["stage_y"]):
        add("lay", "loop", 1, 0 if y > 0 else 1, s, clip_reach(W(P["stage_in"], y)))

    # ---- step 1 (phase over): the left hand holds the standing part, the right hand lays the end over it (X1,
    # events 1, 5-6), then the left hand holds the standing part at X1 and the right draws the end out (7, 10).
    # Planned on the rope as it lies after the stage-in: X1 goes where the standing part actually is.
    want = W(P["stage_in"], P["x1_y"])
    if state:
        nodes = np.asarray(state["nodes"], dtype=np.float64)[:, :2]
        arc = np.asarray(state["arc"], dtype=np.float64)
        on_s = (arc > 0.25) & (arc < 0.62)
        k = int(np.argmin(np.where(on_s, np.linalg.norm(nodes - want, axis=1), 1e9)))
        s_x1, x1_at = float(arc[k]), nodes[k]
    else:
        s_x1, x1_at = 0.38, want
    off = float(np.dot(x1_at - o, v))  # where the standing part is, away from the line
    add("hold", "over", 1, 0, min(s_x1 + P["hold_beyond"] + 0.03, L - 0.02))
    st = add("lay", "over", 6, 1, P["x1_grasp"], clip_reach(x1_at + P["x1_beyond"] * v + 0.01 * u),
             via=[clip_reach(W(off + dx, float(np.dot(x1_at - o, u)) + dy)) for dx, dy in P["x1_via"]], replan_after=True)
    st["makes"] = [{"crossing": "X1", "working_over": True, "at": [round(float(c), 4) for c in x1_at], "other_s": round(s_x1, 4)}]
    add("release", "over", 4, 0)
    x1 = find_x1(state) if state else None
    if state and x1:
        sa, sb = x1["s_a"], x1["s_b"]
        x1_xy = np.asarray(x1["at"], dtype=np.float64)
        tip = _point(state, 0.0)
    else:  # preview: X1 where it is laid, the end beyond it
        sa, sb = 0.10, s_x1
        x1_xy = x1_at
        tip = x1_at + P["x1_beyond"] * v
    add("hold", "over", 7, 0, min(sb + P["hold_beyond"], L - 0.02))
    add("lay", "over", 10, 1, P["draw_grasp"], draw_target(state, tip, x1_xy, sa, P["draw"]) if (state and x1)
        else clip_reach(tip + P["draw"] * _unit(tip - x1_xy)), replan_after=True)
    add("release", "over", 7, 0)

    # ---- step 2: the tuck (events 11-15), planned on the rope as it lies when the phase starts.
    if state and x1:
        sa, sb = x1["s_a"], x1["s_b"]
        lift_s = sb - P["lift_back"]
        through = _point(state, lift_s)
        loop = np.array([_point(state, s) for s in np.linspace(sa, sb, 24)])
        eye = loop.mean(axis=0)
    else:
        lift_s = 0.564
        through = m[int(np.searchsorted(arc_m, lift_s)), :2]
        eye = W(P["stage_in"] - 0.05, -0.05)
    push = _unit(eye - through)
    beyond = through + 0.05 * push
    add("aim_end", "thread", 14, 1, P["grasp_back"], beyond, end_first=True, through_xy=[round(float(c), 4) for c in through],
        grasp_back=P["grasp_back"], margin=P["margin"], drag=P["drag"])
    add("lift_hold", "thread", 11, 0, lift_s, None, lift_s=round(float(lift_s), 4), height=P["lift"], helper_arm=0)
    st = add("push_under", "thread", 15, 1, P["grasp_back"], beyond, helper_arm=0, lift_s=round(float(lift_s), 4),
             end_first=True, through_xy=[round(float(c), 4) for c in through])
    st["makes"] = [{"crossing": "X2", "working_over": False, "at": [round(float(c), 4) for c in through], "other_s": round(float(lift_s), 4)}]
    add("lower", "thread", 11, 0, lift_s, None, sweep=P["sweep"])

    # ---- step 2, event 16: the tip drawn out of the eye over the loop's far side (X3).
    if state and x1:
        sa, sb = x1["s_a"], x1["s_b"]
        cand = [s for s in np.arange(sa + 0.02, sb - P["lift_back"] - 0.06, 0.01)]
        tipnow = _point(state, 0.0)
        best, best_score = None, -1e9
        for s in cand:
            p = _point(state, s)
            dest = p + P["tail_out"] * _unit(p - eye)
            (x0, x1_), (y0, y1) = REACH
            room = min(dest[0] - x0, x1_ - dest[0], dest[1] - y0, y1 - dest[1])
            ahead = float(np.dot(_unit(p - tipnow), push))
            score = min(room, 0.02) * 10 + ahead
            if score > best_score:
                best, best_score = (s, p, dest), score
        x3_s, x3_xy, dest = best if best else (None, eye, eye + 0.06 * push)
    else:
        x3_s, x3_xy = 0.415, m[int(np.searchsorted(arc_m, 0.415)), :2]
        dest = x3_xy + P["tail_out"] * _unit(x3_xy - eye)
    st = add("pull_through", "tail", 16, 1, P["pull_grasp"], clip_reach(dest), end_first=True, grasp_back=P["pull_grasp"],
             open_width=0.035, height=0.03)
    st["makes"] = [{"crossing": "X3", "working_over": True, "at": [round(float(c), 4) for c in x3_xy],
                    "other_s": None if x3_s is None else round(float(x3_s), 4)}]

    # ---- step 3: each hand takes a tail and they draw apart (events 19-22).
    if state:
        nodes = np.asarray(state["nodes"], dtype=np.float64)[:, :2]
        cs = state.get("crossings", [])
        centre = np.mean([c["at"] for c in cs], axis=0) if cs else nodes.mean(axis=0)
        d_left = _unit(_point(state, L - P["stand_back"]) - centre)
        d_right = _unit(_point(state, P["end_back"]) - centre)
    else:
        d_left, d_right = u, -u
    add("pull", "pull", 21, 0, None, None, s_left=round(L - P["stand_back"], 4), s_right=P["end_back"],
        direction_left=[round(float(c), 4) for c in d_left], direction_right=[round(float(c), 4) for c in d_right],
        distance=P["pull_distance"])
    return steps


def plan(layout: dict, state: dict | None = None, **overrides) -> dict:
    """``demo_steps`` wrapped like ``plan.lay_plan``'s answer (length, steps), for the graph scripts."""
    m = mirrored_target(layout)
    return {"length": round(float(diagram.arc_length(m)[-1]), 4), "steps": demo_steps(layout, state, **overrides),
            "code": diagram.code_str(diagram.gauss_code(m - BENCH_ORIGIN)), "scale": round(scale(layout), 3)}
