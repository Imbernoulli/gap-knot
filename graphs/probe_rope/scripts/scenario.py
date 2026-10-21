"""Rope-skill feasibility scenarios ($PROBE), each recording its measurements in the node's output."""
import json
import math
import os
from typing import TypedDict

from gap import NodeContext

from knotlab import plan, skills


class Output(TypedDict):
    report: dict


def _strip(d):
    if isinstance(d, dict):
        return {k: _strip(v) for k, v in d.items() if k not in ("rot", "nodes", "arc")}
    if isinstance(d, list):
        return [_strip(v) for v in d]
    return d


def winding(nodes, centre, lo, hi):
    """Total angle the rope between node indices lo..hi turns round ``centre`` [turns]."""
    ang = [math.atan2(n[1] - centre[1], n[0] - centre[0]) for n in nodes[lo : hi + 1]]
    tot = 0.0
    for a, b in zip(ang, ang[1:]):
        d = (b - a + math.pi) % (2 * math.pi) - math.pi
        tot += d
    return tot / (2 * math.pi)


def run(ctx: NodeContext, layout: dict) -> Output:
    which = os.environ.get("PROBE", "ab")
    cfg = json.loads(os.environ.get("PROBE_CFG", "{}"))
    rep = {"probe": which, "cfg": cfg}
    p = plan.lay_plan(layout)
    steps = p["steps"]
    if which == "ab":
        last = int(cfg.get("last", 9))
        rep["steps"] = []
        for st in steps[: last + 1]:
            r = skills.lay(ctx, st["arm_id"], st["s"], st["xy"], overshoot_xy=tuple(cfg.get("overshoot", (0.0, 0.0))),
                           place_kw=cfg.get("place", {}), **cfg.get("pinch", {}))
            now = ctx.tool("rope.state")
            rep["steps"].append({"k": st["k"], "s": st["s"], "arm": st["arm_id"], "makes": st["makes"], "lay": _strip(r),
                                 "code": now["code"], "prefix": now["prefix"], "n_cross": len(now["crossings"]),
                                 "crossings": [[round(c["s_a"], 3), round(c["s_b"], 3), c["a_over"]] for c in now["crossings"]]})
            if st["k"] == 6:
                pin = layout["pins"]["left"]
                arc = now["arc"]
                lo = min(range(len(arc)), key=lambda i: abs(arc[i] - 0.28))
                hi = min(range(len(arc)), key=lambda i: abs(arc[i] - 0.58))
                rep["left_pin_winding_after_A"] = round(winding(now["nodes"], pin, lo, hi), 3)
                tgt = layout["target_nodes"]
                rep["left_pin_winding_target"] = round(winding(tgt, pin, lo, hi), 3)
        for a in (0, 1):
            skills.home(ctx, a)
        final = ctx.tool("rope.state")
        rep["final"] = {"code": final["code"], "prefix": final["prefix"], "progress": final["progress"],
                        "crossings": [[round(c["s_a"], 3), round(c["s_b"], 3), c["a_over"]] for c in final["crossings"]]}
    elif which == "c":
        # Bring half A's end into reach, then thread it under half B's leg, right to left.
        fe = skills.fetch_end(ctx, 1, True, pull=float(cfg.get("pull", 0.15)))
        rep["fetch"] = _strip(fe)
        mid = ctx.tool("rope.state")["length"] / 2
        lift_s = mid + float(cfg.get("lift_d", 0.09))
        strand = ctx.tool("rope.point", s=lift_s)["position"]
        through = (strand[0], strand[1])
        beyond = (strand[0], strand[1] + float(cfg.get("beyond", 0.07)))
        th = skills.thread_end(ctx, 1, 0, True, lift_s, through, beyond, **cfg.get("thread", {}))
        rep["thread"] = _strip(th)
        for a in (0, 1):
            skills.home(ctx, a)
        final = ctx.tool("rope.state")
        rep["final"] = {"code": final["code"], "crossings": [[round(c["s_a"], 3), round(c["s_b"], 3), c["a_over"]] for c in final["crossings"]]}
    elif which == "d":
        before = ctx.tool("rope.state")
        rep["settled"] = {"code": before["code"], "progress": before["progress"], "tied": before["tied"]}
        length = before["length"]
        tl = ctx.tool("rope.point", s=length - 0.04)["position"]
        tr = ctx.tool("rope.point", s=0.04)["position"]
        dist = float(cfg.get("distance", 0.06))
        # Each tail drawn out along the line from the knot's centre through it.
        c = layout["board_centre"]
        def away(t):
            dx, dy = t[0] - c[0], t[1] - c[1]
            n = math.hypot(dx, dy) or 1.0
            return (dx / n, dy / n)
        dl, dr = (away(tl), away(tr)) if cfg.get("dir") != "down" else ((-1.0, 0.0), (-1.0, 0.0))
        r = skills.pull(ctx, (0, 1), s_left=length - 0.04, s_right=0.04, direction_left=dl, direction_right=dr,
                        distance=dist, step=float(cfg.get("step", 0.01)))
        rep["pull"] = _strip(r)
        after = ctx.tool("rope.state")
        rep["after"] = {"code": after["code"], "progress": after["progress"], "tied": after["tied"],
                        "extent": None}
    return {"report": rep}
