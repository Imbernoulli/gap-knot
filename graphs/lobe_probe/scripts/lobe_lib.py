"""Development copy of the lobe skill: lay one half's lobe round its side pin.

Everything here talks only through ``ctx.tool`` and ``knotlab.skills``; once
validated it moves into ``knotlab/skills.py`` (additive).

Points of a half are named by ``d``, the arc length from the rope's middle
(``s = L/2 - d`` on half A, ``L/2 + d`` on half B), so the same numbers serve
both halves: the lobe's junction (where it meets the half's own run down from
the top pin) is d ~ 0.17, its apex (beyond the pin) d ~ 0.28, its far end
d ~ 0.38. The lobe's measured range is d in [0.082, 0.382].
"""
from __future__ import annotations

import math

import numpy as np

from knotlab import diagram, skills

BENCH_ORIGIN = np.array([0.6, 0.0, 0.75])
BENCH_Z = skills.BENCH_Z
LOBE_D = (0.082, 0.382)
PIN_TOP_Z = 0.050


# ----------------------------------------------------------------------------- geometry

class Lobe:
    """One half's lobe as rope.layout gives it: its pin, its target, d <-> s."""

    def __init__(self, layout: dict, half: str):
        self.half = half
        self.tgt = np.asarray(layout["target_nodes"], dtype=np.float64)
        self.arc = np.asarray(layout.get("target_arc") or diagram.arc_length(self.tgt), dtype=np.float64)
        self.L = float(self.arc[-1])
        self.mid = 0.5 * self.L
        self.pins = {k: np.asarray(v, dtype=np.float64)[:2] for k, v in layout["pins"].items()}
        self.pin = self.pins["left" if half == "A" else "right"]
        self.sign = -1.0 if half == "A" else 1.0

    def s(self, d: float) -> float:
        return float(np.clip(self.mid + self.sign * float(d), 0.0, self.L))

    def target(self, s: float) -> np.ndarray:
        return np.array([np.interp(s, self.arc, self.tgt[:, 0]), np.interp(s, self.arc, self.tgt[:, 1])])

    def outward(self, s: float) -> np.ndarray:
        """Unit vector from the pin through the target of s (overshoot direction)."""
        v = self.target(s) - self.pin
        n = float(np.linalg.norm(v)) or 1.0
        return v / n

    def range_s(self, d0: float = LOBE_D[0], d1: float = LOBE_D[1]) -> tuple[float, float]:
        a, b = self.s(d0), self.s(d1)
        return (min(a, b), max(a, b))


def _wind(xy: np.ndarray, c) -> float:
    a = np.arctan2(xy[:, 1] - c[1], xy[:, 0] - c[0])
    d = np.diff(a)
    d = (d + np.pi) % (2 * np.pi) - np.pi
    return float(d.sum() / (2 * np.pi))


def target_crossings(layout: dict) -> list[tuple[float, float, bool]]:
    tgt = np.asarray(layout["target_nodes"], dtype=np.float64)
    return [(c.s_i, c.s_j, c.i_over) for c in diagram.crossings(tgt - BENCH_ORIGIN)]


def classify_crossings(found, target, tol: float = 0.06) -> list[dict]:
    """Each found crossing (s_i, s_j, i_over) against the target's: ok (same strands, same over), flipped, or extra.

    Strands are matched by arc length within ``tol`` on both (dragged cord shifts a crossing along the rope),
    one target crossing per found one, nearest first.
    """
    pairs = []
    for i, c in enumerate(found):
        for j, t in enumerate(target):
            e = max(abs(c[0] - t[0]), abs(c[1] - t[1]))
            if e <= tol:
                pairs.append((e, i, j))
    pairs.sort()
    used_c, used_t, out = set(), set(), [None] * len(found)
    for e, i, j in pairs:
        if i in used_c or j in used_t:
            continue
        used_c.add(i)
        used_t.add(j)
        out[i] = {"s": [round(found[i][0], 3), round(found[i][1], 3)], "over": bool(found[i][2]), "target": j + 1,
                  "kind": "ok" if bool(found[i][2]) == bool(target[j][2]) else "flipped"}
    for i, c in enumerate(found):
        if out[i] is None:
            out[i] = {"s": [round(c[0], 3), round(c[1], 3)], "over": bool(c[2]), "target": None, "kind": "extra"}
    return out


def code_consistent(found, target, tol: float = 0.10) -> dict:
    """Is the rope's diagram a sub-diagram of the target's? Every crossing matched (same strands within ``tol``,
    same over/under, one-to-one) and met along the rope in the target's order (its labels and O/U a subsequence
    of the target's Gauss code). Crossings may sit away from the target's (dragged cord slides them); a crossing
    that is extra, flipped, or out of order is one the target does not have."""
    cls = classify_crossings(found, target, tol=tol)
    if any(c["kind"] != "ok" for c in cls):
        return {"ok": False, "why": "unmatched or flipped"}
    ev = sorted([(c["s"][0], c["target"], "O" if c["over"] else "U") for c in cls] +
                [(c["s"][1], c["target"], "U" if c["over"] else "O") for c in cls])
    tev = sorted([(t[0], k + 1, "O" if t[2] else "U") for k, t in enumerate(target)] +
                 [(t[1], k + 1, "U" if t[2] else "O") for k, t in enumerate(target)])
    want = [(lab, ou) for _, lab, ou in tev]
    it = iter(want)
    sub = all(any(x == y for y in it) for x in [(lab, ou) for _, lab, ou in ev])
    return {"ok": bool(sub), "why": "" if sub else "out of order"}


def lobe_metrics(nodes, layout: dict, half: str) -> dict:
    """Winding of the lobe's range about its pin (target sign positive), node error to target, crossings."""
    p = np.asarray(nodes, dtype=np.float64)
    lobe = Lobe(layout, half)
    arc = diagram.arc_length(p)
    lo, hi = lobe.range_s()
    m = (arc >= lo) & (arc <= hi)
    mt = (lobe.arc >= lo) & (lobe.arc <= hi)
    w_t = _wind(lobe.tgt[mt, :2], lobe.pin)
    w = _wind(p[m, :2], lobe.pin) * (1.0 if w_t >= 0 else -1.0)
    n = min(len(p), len(lobe.tgt))
    idx = np.nonzero(m[:n])[0]
    err = np.linalg.norm(p[idx, :2] - lobe.tgt[idx, :2], axis=1)
    found = [(c.s_i, c.s_j, c.i_over) for c in diagram.crossings(p - BENCH_ORIGIN)]
    tc = target_crossings(layout)
    cls = classify_crossings(found, tc)
    cons = code_consistent(found, tc)
    return {
        "consistent": cons["ok"], "inconsistent_why": cons["why"],
        "winding": round(w, 3), "winding_target": round(abs(w_t), 3),
        "err_median": round(float(np.median(err)), 4), "err_max": round(float(err.max()), 4),
        "err_p75": round(float(np.percentile(err, 75)), 4),
        "n_cross": len(found), "n_ok": sum(c["kind"] == "ok" for c in cls),
        "n_flipped": sum(c["kind"] == "flipped" for c in cls), "n_extra": sum(c["kind"] == "extra" for c in cls),
        "crossings": cls,
        "lowest_z": round(float(p[:, 2].min() - BENCH_Z), 4), "highest_z": round(float(p[:, 2].max() - BENCH_Z), 4),
    }


# ----------------------------------------------------------------------------- motion helpers

def route(p0, p1, obstacles, clear: float = 0.032, depth: int = 3) -> list[list[float]]:
    """Waypoints from p0 to p1 (xy) that keep the hand ``clear`` of every obstacle point (pins); p1 last."""
    p0, p1 = np.asarray(p0, dtype=float)[:2], np.asarray(p1, dtype=float)[:2]
    d = p1 - p0
    n2 = float(d @ d)
    worst = None
    for c in obstacles:
        c = np.asarray(c, dtype=float)[:2]
        u = 0.0 if n2 < 1e-12 else float(np.clip((c - p0) @ d / n2, 0.0, 1.0))
        q = p0 + u * d
        dist = float(np.linalg.norm(c - q))
        if dist < clear and 0.0 < u < 1.0 and (worst is None or dist < worst[0]):
            worst = (dist, c, q)
    if worst is None or depth == 0:
        return [p1.tolist()]
    _, c, q = worst
    off = q - c
    if float(np.linalg.norm(off)) < 1e-6:
        off = np.array([-d[1], d[0]])
    off = off / float(np.linalg.norm(off))
    w = c + off * clear * 1.15
    return route(p0, w, obstacles, clear, depth - 1)[:-1] + [w.tolist()] + route(w, p1, obstacles, clear, depth - 1)


def cart(ctx, arm_id: int, xy, z: float, rot: dict, speed: float = skills.SPEED) -> str:
    r = ctx.tool("robot.go_to_pose_cartesian", pose=skills._pose((xy[0], xy[1], BENCH_Z + z), rot), arm_id=arm_id,
                 speed_scale=float(speed))
    return skills._outcome(r)


PAD = 0.004  # a YAM finger pad's thickness at the tips [m] (collision box 2 x 0.002)


def pin_width(xy, pins, *, widest: float, narrowest: float = 0.022, margin: float = 0.003, strands=None) -> float:
    """The widest jaw opening at ``xy`` whose pads stay ``margin`` clear of every pin (pads open across any heading).

    ``strands``: other cord centreline points (xy) the pads must not push either (a 10 mm cord: 5 mm radius).
    """
    xy = np.asarray(xy, dtype=float)[:2]
    r = min((float(np.linalg.norm(xy - np.asarray(c, dtype=float)[:2])) for c in pins), default=1.0)
    w = 2.0 * (r - 0.0025 - PAD - margin)
    if strands is not None and len(strands):
        q = np.asarray(strands, dtype=float)[:, :2]
        rs = float(np.min(np.linalg.norm(q - xy, axis=1)))
        w = min(w, 2.0 * (rs - 0.005 - PAD - margin))
    return float(np.clip(w, narrowest, widest))


def other_strands(ctx, s: float, *, skip: float = 0.04):
    """Rope nodes (world) farther than ``skip`` along the rope from s: what a hand at s must not disturb."""
    st = ctx.tool("rope.state")
    arc = np.asarray(st["arc"], dtype=float)
    nodes = np.asarray(st["nodes"], dtype=float)
    return nodes[np.abs(arc - float(s)) > skip]


def release_clear(ctx, arm_id: int, xy, rot: dict, pins, *, s: float | None = None, release_z: float = skills.LOW + 0.004,
                  height: float = skills.CARRY, dwell: int = 20, release_width: float = 0.06, clear_z: float = 0.075,
                  lower_speed: float = 0.15) -> dict:
    """Set the held cord down at ``xy`` and let go without a pad sweeping a pin.

    Opened only as wide as the nearest pin allows; the pads then rise just past the cord's top (the tips at ~16 mm,
    so a cord pressed on a pad slides off the tip instead of riding up), then above the pins, and only then open fully.
    """
    out = {"to": [round(float(xy[0]), 4), round(float(xy[1]), 4)]}
    out["over"] = cart(ctx, arm_id, xy, height, rot, lower_speed)
    out["lower"] = skills.descend(ctx, arm_id, (xy[0], xy[1], BENCH_Z + release_z), rot)
    if dwell > 0:
        ctx.tool("robot.wait_steps", steps=int(dwell))
    held = ctx.tool("rope.point", s=float(s))["position"] if s is not None else None
    w = pin_width(xy, pins, widest=release_width)
    out["width"] = round(w, 4)
    ctx.tool("robot.set_grip", width_m=w, arm_id=arm_id, settle_steps=18)
    c = lift_off(ctx, arm_id, xy, rot, s=s, z=release_z, clear_z=clear_z, pins=pins)
    out.update(c)
    ctx.tool("robot.set_grip", preset="open", arm_id=arm_id, settle_steps=6)
    if s is not None:
        ctx.tool("robot.wait_steps", steps=10)
        now = ctx.tool("rope.point", s=float(s))["position"]
        out["error"] = round(skills._dist(now, xy), 4)
        out["spring_back"] = round(skills._dist(now, held), 4)
        out["z_after"] = round(float(now[2]) - BENCH_Z, 4)
    return out


def lift_off(ctx, arm_id: int, xy, rot: dict, *, s: float | None, z: float, clear_z: float = 0.075, pins=None) -> dict:
    """With the jaws open, leave the cord where it lies: centre the jaws on it (a cord pressed on one pad rides up
    with it), rise just past its top (tips at ~16 mm), then above the pins."""
    out = {"follow": []}
    cx, cy = float(xy[0]), float(xy[1])
    if s is not None:
        for _ in range(3):  # the cord springs toward where its neighbours pull it: keep the jaws centred on it
            ctx.tool("robot.wait_steps", steps=8)
            q = ctx.tool("rope.point", s=float(s))["position"]
            off = skills._dist(q, (cx, cy))
            if off < 0.0025 or off > 0.02:
                break
            if pins is not None and pin_width(q, pins, widest=1.0, narrowest=0.0) < pin_width((cx, cy), pins, widest=1.0, narrowest=0.0) - 0.004:
                break  # following it would bring a pad onto a pin
            cx, cy = float(q[0]), float(q[1])
            skills.descend(ctx, arm_id, (cx, cy, BENCH_Z + z), rot, tolerance=0.003, max_steps=30)
            out["follow"].append(round(off, 4))
    ctx.tool("robot.wait_steps", steps=6)
    skills.descend(ctx, arm_id, (cx, cy, BENCH_Z + 0.028), rot, tolerance=0.004, max_steps=60)
    if s is not None:
        q = ctx.tool("rope.point", s=float(s))["position"]
        out["z_at_28"] = round(float(q[2]) - BENCH_Z, 4)
        if float(q[2]) - BENCH_Z > 0.014:  # still on a pad: back down, re-centre, wait, up again
            skills.descend(ctx, arm_id, (float(q[0]), float(q[1]), BENCH_Z + z), rot, tolerance=0.003, max_steps=40)
            ctx.tool("robot.wait_steps", steps=12)
            cx, cy = float(q[0]), float(q[1])
            skills.descend(ctx, arm_id, (cx, cy, BENCH_Z + 0.028), rot, tolerance=0.004, max_steps=60)
            out["second_try"] = True
    out["rise"] = skills.move(ctx, arm_id, (cx, cy, BENCH_Z + clear_z), rot, speed=0.5)
    return out


def take(ctx, arm_id: int, s: float, pins, *, retries: int = 1, open_width: float = 0.045, low: float = skills.LOW,
         fraction: float = skills.FRACTION) -> dict:
    """skills.pinch with the jaws opened only as wide as nearby pins allow, re-pinched after a miss."""
    def once():
        here = ctx.tool("rope.point", s=float(s))["position"]
        w = pin_width(here, pins, widest=open_width)
        return skills.pinch(ctx, arm_id, s, open_width=w, low=low, fraction=fraction), w

    p, w = once()
    tries = 1
    while not p["held"] and tries <= retries:
        ctx.tool("robot.set_grip", width_m=w, arm_id=arm_id, settle_steps=6)
        lift_off(ctx, arm_id, p["at"], p["rot"], s=s, z=low, pins=pins)
        p, w = once()
        tries += 1
    p["tries"] = tries
    p["open_width"] = round(w, 4)
    return p


def carry_checked(ctx, arm_id: int, s: float, path, rot: dict, pins, *, height: float, speed: float = skills.SPEED,
                  regrasps: int = 2, fraction: float = skills.FRACTION, slip: float = 0.015) -> dict:
    """skills.carry one waypoint at a time, reading the held point after each: if it trails the hand by more than
    ``slip`` the cord slid through the pads, so let go where it is, take it again and redo that waypoint."""
    out = {"lags": [], "regrasps": 0, "reached": True}
    i = 0
    while i < len(path):
        wp = path[i]
        c = skills.carry(ctx, arm_id, [wp], rot, height=height, speed=speed, s=s)
        out["reached"] = out["reached"] and c["reached"]
        lag = float(c.get("lag", 0.0))
        out["lags"].append(round(lag, 4))
        if lag > slip and out["regrasps"] < regrasps:
            q = ctx.tool("rope.point", s=float(s))["position"]
            ctx.tool("robot.set_grip", width_m=pin_width(q, pins, widest=0.045), arm_id=arm_id, settle_steps=10)
            skills.move(ctx, arm_id, (q[0], q[1], BENCH_Z + 0.075), rot, speed=0.5)
            p = take(ctx, arm_id, s, pins, fraction=fraction)
            out["regrasps"] += 1
            if not p["held"]:
                out["dropped"] = True
                out["rot"] = p["rot"]
                return out
            rot = p["rot"]
            if height > skills.CARRY + 0.005:
                cart(ctx, arm_id, p["at"], height, rot, speed)
            continue  # redo this waypoint from where the cord is now
        i += 1
    out["lag"] = out["lags"][-1] if out["lags"] else 0.0
    out["rot"] = rot
    return out


# ----------------------------------------------------------------------------- ops

def _pin(lobe: Lobe, name: str) -> np.ndarray:
    other = "right" if lobe.half == "A" else "left"
    return lobe.pins[{"own": "left" if lobe.half == "A" else "right", "other": other}.get(name, name)].copy()


def _xy(lobe: Lobe, spec, s: float) -> np.ndarray:
    """A point named in an op, written for half B and mirrored (y offsets negated) for half A.

    "target" (s's target), "pin" (the lobe's own pin), {"pin": own|other|top, "off": [dx, dy]},
    {"target_off": [dx, dy]}, {"radial": r} (r from the pin along the ray through s's target), or [x, y].
    """
    m = np.array([1.0, -1.0]) if lobe.half == "A" else np.array([1.0, 1.0])
    if isinstance(spec, str):
        return lobe.target(s) if spec == "target" else lobe.pin.copy()
    if isinstance(spec, dict):
        if "pin" in spec:
            return _pin(lobe, spec["pin"]) + m * np.asarray(spec.get("off", (0.0, 0.0)), dtype=float)
        if "target_off" in spec:
            return lobe.target(s) + m * np.asarray(spec["target_off"], dtype=float)
        if "radial" in spec:
            return lobe.pin + float(spec["radial"]) * lobe.outward(s)
    return np.asarray(spec, dtype=float)[:2]


def lay_point(ctx, lobe: Lobe, op: dict, holding: dict) -> dict:
    """One op: take the cord at d (or s), carry it (low, or lifted over the pins) to its goal, set it down or hold it."""
    arm = int(op.get("arm", 0 if lobe.half == "A" else 1))
    if "arm_rel" in op:  # "own": the hand on the lobe's side; "other": the far hand
        arm = (0 if lobe.half == "A" else 1) if op["arm_rel"] == "own" else (1 if lobe.half == "A" else 0)
    if op.get("d") == "auto":
        st = ctx.tool("rope.state")
        arc, nodes = np.asarray(st["arc"], dtype=float), np.asarray(st["nodes"], dtype=float)
        lo, hi = op.get("d_range", (0.25, 0.31))
        best = None
        for d in np.arange(lo, hi + 1e-9, 0.01):
            q = np.array([np.interp(lobe.s(d), arc, nodes[:, 0]), np.interp(lobe.s(d), arc, nodes[:, 1])])
            r = min(float(np.linalg.norm(q - c)) for c in lobe.pins.values())
            z = float(np.interp(lobe.s(d), arc, nodes[:, 2])) - BENCH_Z
            key = (z < 0.016, min(r, 0.03), -abs(d - float(op.get("d_pref", 0.28))))
            if best is None or key > best[0]:
                best = (key, d)
        s = lobe.s(best[1])
    else:
        s = float(op["s"]) if "s" in op else lobe.s(float(op["d"]))
    dest = _xy(lobe, op.get("to", "target"), s)
    m = np.array([1.0, -1.0]) if lobe.half == "A" else np.array([1.0, 1.0])
    dest = dest + m * np.asarray(op.get("offset", (0.0, 0.0)), dtype=float) + float(op.get("radial", 0.0)) * lobe.outward(s)
    height = float(op.get("height", skills.CARRY))
    lift = op.get("lift")
    speed = float(op.get("speed", skills.SPEED))
    pins = list(lobe.pins.values())
    out = {"op": op.get("op", "lay"), "arm": arm, "s": round(s, 4), "dest": [round(float(v), 4) for v in dest]}
    before = ctx.tool("rope.point", s=s)["position"]
    out["from"] = [round(float(v), 4) for v in before[:2]]
    p = take(ctx, arm, s, pins, retries=int(op.get("retries", 1)), open_width=float(op.get("open_width", 0.045)),
             fraction=float(op.get("fraction", skills.FRACTION)))
    out["pinch"] = {"held": bool(p["held"]), "tries": p["tries"], "rise": p["rise"], "w": p["open_width"]}
    if not p["held"]:
        ctx.tool("robot.set_grip", width_m=p["open_width"], arm_id=arm, settle_steps=6)
        lift_off(ctx, arm, p["at"], p["rot"], s=s, z=skills.LOW, pins=pins)
        out["ok"] = False
        return out
    rot = p["rot"]
    here = np.asarray(p["at"], dtype=float)
    path = [_xy(lobe, w, s).tolist() for w in op.get("via", [])] + [dest.tolist()]
    fr = float(op.get("fraction", skills.FRACTION))
    if lift is not None:
        out["rise"] = cart(ctx, arm, here, float(lift), rot, speed)
    hc = float(lift) if lift is not None else height
    if op.get("checked", True):
        c = carry_checked(ctx, arm, s, path, rot, pins, height=hc, speed=speed, fraction=fr,
                          regrasps=int(op.get("regrasps", 2)))
        rot = c["rot"]
        if c.get("dropped"):
            ctx.tool("robot.set_grip", width_m=0.03, arm_id=arm, settle_steps=6)
            lift_off(ctx, arm, ctx.tool("rope.point", s=s)["position"], rot, s=s, z=skills.LOW, pins=pins)
            out["carry"] = {"dropped": True, "lags": c["lags"]}
            out["ok"] = False
            return out
    else:
        c = skills.carry(ctx, arm, path, rot, height=hc, speed=speed, s=s)
    out["carry"] = {"lag": c.get("lag"), "lags": c.get("lags"), "regrasps": c.get("regrasps"), "reached": c["reached"],
                    "path": [[round(v, 3) for v in w] for w in path]}
    if op.get("op") == "hold":
        cart(ctx, arm, dest, height, rot, 0.15)
        skills.descend(ctx, arm, (dest[0], dest[1], BENCH_Z + skills.LOW + 0.004), rot)
        holding[arm] = {"rot": rot, "xy": dest.tolist(), "s": s}
        now = ctx.tool("rope.point", s=s)["position"]
        out["error"] = round(skills._dist(now, dest), 4)
        out["ok"] = True
        return out
    out["place"] = release_clear(ctx, arm, dest, rot, pins, s=s, height=height, dwell=int(op.get("dwell", 20)),
                                 release_width=float(op.get("release_width", 0.06)))
    out["ok"] = bool(c["reached"])
    return out


def _let_go(ctx, arm: int, h: dict | None, pins, width: float = 0.06) -> bool:
    if not h:
        return False
    ctx.tool("robot.set_grip", width_m=pin_width(h["xy"], pins, widest=width), arm_id=arm, settle_steps=18)
    skills.descend(ctx, arm, (h["xy"][0], h["xy"][1], BENCH_Z + 0.028), h["rot"], tolerance=0.004, max_steps=60)
    skills.move(ctx, arm, (h["xy"][0], h["xy"][1], BENCH_Z + 0.075), h["rot"], speed=0.5)
    ctx.tool("robot.set_grip", preset="open", arm_id=arm, settle_steps=6)
    return True


def clear_area(ctx, lobe: Lobe, *, need: float = 0.032, d_other: float = 0.45, d_curve=(0.20, 0.36), tries: int = 2,
               extra: float = 0.006, arm: int | None = None) -> dict:
    """Push the other half's end run (its corridor and fold, d >= ``d_other``) out of the way of this lobe.

    Any such node closer than ``need`` to the lobe's target curve (d in ``d_curve``) is taken and carried radially
    out from the pin until it is ``need + extra`` clear: the apex is set down (and the jaws opened) there.
    """
    curve = np.array([lobe.target(lobe.s(d)) for d in np.linspace(d_curve[0], d_curve[1], 25)])
    out = {"op": "clear", "moves": []}
    for _ in range(tries):
        st = ctx.tool("rope.state")
        arc = np.asarray(st["arc"], dtype=float)
        nodes = np.asarray(st["nodes"], dtype=float)
        dist_mid = (arc - lobe.mid) * lobe.sign  # this half's d is positive, the other half's negative
        cand = np.nonzero(dist_mid <= -float(d_other))[0]
        if not len(cand):
            break
        dd = np.min(np.linalg.norm(nodes[cand, None, :2] - curve[None, :, :2], axis=2), axis=1)
        k = int(np.argmin(dd))
        out["closest"] = round(float(dd[k]), 4)
        if dd[k] >= need:
            break
        i = int(cand[k])
        # Take it a little away from the closest node, toward the middle of the offending run, and push it out.
        q = nodes[i, :2]
        ray = q - lobe.pin
        ray = ray / (float(np.linalg.norm(ray)) or 1.0)
        dest = q + ray * (need - float(dd[k]) + extra)
        a = (0 if lobe.half == "A" else 1) if arm is None else int(arm)  # the hand on the lobe side (the run lies there)
        r = lay_point(ctx, lobe, {"op": "lay", "s": float(arc[i]), "to": dest.tolist(), "arm": a}, {})
        out["moves"].append({k2: r.get(k2) for k2 in ("s", "from", "dest", "pinch", "place")})
    return out


def run_ops(ctx, layout: dict, half: str, ops: list[dict]) -> dict:
    lobe = Lobe(layout, half)
    pins = list(lobe.pins.values())
    holding: dict = {}
    log = []
    for op in ops:
        kind = op.get("op", "lay")
        if kind in ("lay", "hold"):
            r = lay_point(ctx, lobe, op, holding)
        elif kind == "release":
            arm = int(op["arm"]) if "arm" in op else ((1 if half == "A" else 0) if op.get("arm_rel") == "other" else (0 if half == "A" else 1))
            r = {"op": "release", "arm": arm, "had": _let_go(ctx, arm, holding.pop(arm, None), pins)}
        elif kind == "clear":
            r = clear_area(ctx, lobe, need=float(op.get("need", 0.032)), d_other=float(op.get("d_other", 0.45)),
                           arm=op.get("arm"))
        elif kind == "wait":
            ctx.tool("robot.wait_steps", steps=int(op.get("steps", 30)))
            r = {"op": "wait"}
        elif kind == "home":
            r = {"op": "home", "arm": op.get("arm"), "out": skills.home(ctx, int(op.get("arm", 0)))}
        elif kind == "metrics":
            r = {"op": "metrics", **lobe_metrics(ctx.tool("rope.state")["nodes"], layout, half)}
        else:
            r = {"op": kind, "error": "unknown op"}
        log.append(r)
    for arm in list(holding):
        _let_go(ctx, arm, holding.pop(arm), pins)
    return {"log": log}
