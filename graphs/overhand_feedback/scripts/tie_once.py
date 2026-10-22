"""Closed-loop straight-to-overhand attempt.

This is a deliberately small GaP policy prototype.  It follows the extracted
Howcast sequence, but keeps the state in one script so the helper hand remains
up while the free end makes both the under and the far-side over crossing.
Every later phase is regenerated from ``rope.state``; the old point-by-point
policy planned the crossing once and then released its precondition.
"""
from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ov_exec import Hands, run_step  # noqa: E402

from knotlab import demo  # noqa: E402


def _state(ctx):
    return ctx.tool("rope.state")


def _do(ctx, step, params, hands, length, rows):
    try:
        r = run_step(ctx, step, params, hands, length)
    except Exception as exc:  # retain the measured failure for GaP feedback
        r = {"k": step.get("k"), "kind": step.get("kind"), "ok": False,
             "failure": repr(exc)[:240]}
    rows.append(r)
    return r


def _phase(ctx, phase, params, hands, rows, *, state=None, skip=()):
    """Run a phase with GaP-style physical gates and bounded replanning."""
    layout = ctx.tool("rope.layout")
    st = _state(ctx) if state is None else state
    pl = demo.plan(layout, st, **params.get("plan", {}))
    L = float(st["length"])
    steps = [s for s in pl["steps"] if s.get("phase") == phase and s.get("kind") not in skip]
    out = []
    i = 0
    required = {"lay", "hold", "lift_hold", "aim_end", "push_under", "pull_through", "pull"}
    while i < len(steps):
        step = steps[i]
        result = _do(ctx, step, params, hands, L, rows)
        out.append(result)
        if not result.get("ok") and step.get("kind") in required:
            break
        if step.get("replan_after"):
            # demo.py marks the two over crossings as replan points.  Rebuild
            # only the not-yet-executed suffix from measured rope.state while
            # retaining hands.held and the already completed prefix.
            now = _state(ctx)
            replanned = demo.plan(layout, now, **params.get("plan", {}))["steps"]
            steps = [s for s in replanned if s.get("phase") == phase and s.get("kind") not in skip
                     and int(s.get("k", -1)) > int(step.get("k", -1))]
            i = 0
            continue
        i += 1
    return pl, out


def _attempt(ctx) -> dict:
    from knotlab import skills

    # Keep this in sync with overhand_demo/params.json; a copy makes this graph
    # independently runnable by the GaP validator.
    import json
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "..", "params.json")) as f:
        params = json.load(f)

    rows = []
    hands = Hands()
    st0 = _state(ctx)
    L = float(st0["length"])

    # 1. Bring the standing part in, then make X1.  The demo plan's own
    # replan_after flags use the actual strand position after each placement.
    _, rr = _phase(ctx, "loop", params, hands, rows, state=st0)
    if not all(x.get("ok") for x in rr):
        return {"route": "failed", "failure": "loop", "steps": rows}
    st = _state(ctx)
    _, rr = _phase(ctx, "over", params, hands, rows, state=st)
    if not all(x.get("ok") for x in rr):
        return {"route": "failed", "failure": "over", "steps": rows}
    # An action completing is not an X1 crossing.  Do not enter threading
    # unless the measured rope still contains the intended end-over loop.
    st = _state(ctx)
    if demo.find_x1(st) is None:
        return {"route": "failed", "failure": "x1_missing", "steps": rows, "state": st}

    # 2. Aim first, then lift the measured loop strand and push the end under.
    # Do not execute ``lower`` yet: it destroys the arch before X3 is made.
    st = _state(ctx)
    pl, rr = _phase(ctx, "thread", params, hands, rows, state=st,
                    skip=("lower",))
    push = next((r for r in rr if r.get("kind") == "push_under"), None)
    if not push or not push.get("ok"):
        return {"route": "failed", "failure": "under", "steps": rows,
                "state": _state(ctx)}

    # 3. Re-read the loop after the under pass.  Pull-through is planned from
    # this geometry, while the helper is still in Hands.held and physically up.
    st = _state(ctx)
    pl2, rr = _phase(ctx, "tail", params, hands, rows, state=st)
    if not all(x.get("ok") for x in rr):
        return {"route": "failed", "failure": "far_side", "steps": rows,
                "state": _state(ctx)}

    # Lower the helper only after the free end has crossed X3.
    st = _state(ctx)
    _, rr = _phase(ctx, "thread", params, hands, rows, state=st,
                    skip=("aim_end", "lift_hold", "push_under"))
    if not all(x.get("ok") for x in rr):
        return {"route": "failed", "failure": "lower", "steps": rows,
                "state": _state(ctx)}

    # 4. Pull from the current tails, not the target drawing.
    st = _state(ctx)
    _, rr = _phase(ctx, "pull", params, hands, rows, state=st)
    if not all(x.get("ok") for x in rr):
        return {"route": "failed", "failure": "pull", "steps": rows,
                "state": _state(ctx)}
    for arm in (0, 1):
        skills.retreat(ctx, arm)
    ctx.tool("robot.wait_steps", steps=int(params.get("verify", {}).get("settle_steps", 45)))
    final = _state(ctx)
    return {"route": "tied" if final.get("tied") else "untied",
            "steps": rows, "state": final}


def run(ctx) -> dict:
    """Always leave both hands open and vertically clear after an attempt."""
    from knotlab import skills

    try:
        result = _attempt(ctx)
    except Exception as exc:
        result = {"route": "failed", "failure": "phase_exception", "error": repr(exc)}
    cleanup = []
    for arm in (0, 1):
        try:
            tcp, rotation = skills.ee(ctx, arm)
            ctx.tool("robot.set_grip", preset="open", arm_id=arm, settle_steps=10)
            if tcp is not None:
                outcome = skills.descend(ctx, arm, (tcp[0], tcp[1], max(tcp[2], skills.BENCH_Z + 0.15)),
                                         rotation, max_steps=180, tolerance=0.004)
                cleanup.append({"arm": arm, "outcome": outcome})
        except Exception as exc:
            cleanup.append({"arm": arm, "error": repr(exc)})
    result["cleanup"] = cleanup
    return result
