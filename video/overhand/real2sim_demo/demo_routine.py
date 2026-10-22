# SPDX-License-Identifier: Apache-2.0
"""The knotlab robot plan as a studio routine: the GaP graph's own scripts, driven through reach / grip / hand / wait.

A routine on the page is a generator that yields once per control step; on the
live page it also steps the scene itself (the page holds its own stepping while
a routine plays and drains the arm panels each pass), so every pass here is
"the panels have aimed; ``sim.run(1)``". The GaP graph's scripts (and
``knotlab.skills`` under them) are ordinary blocking code that talks only
through ``ctx.tool(name, **kwargs)``. So the plan runs on a worker thread with a
``ctx`` whose every tool call is handed to the routine on the page's main
thread and executed there:

- ``robot.go_to_pose`` / ``robot.go_to_pose_cartesian`` / ``robot.go_home`` ->
  :func:`robosimstudio_studio.routine.reach` (the arm panel's gizmo, its IK and
  setpoint law), at the speed the GaP call implies (free 0.25 m/s; cartesian
  0.16 m/s x speed_scale; a descend -- tolerance under 6 mm -- 0.05 m/s, then
  until the hand is still); a hand that does not arrive comes back as
  ``outcome: timeout`` with the miss, as the GaP servo reports it, instead of
  the RoutineError that would end the take;
- ``robot.set_grip`` -> the arm panel's grip command (what :func:`grip` sets),
  widths through the hand's own ``command_for_width``, ramped as GaP ramps it --
  except a pinch close, snapped (see ``SNAP_PINCH``);
- ``robot.wait_steps`` -> :func:`wait`; ``robot.get_ee_pose`` -> :func:`hand`;
- ``robot.grasp_frame`` -> the same ``grasp_quat`` over the hand's measured axes,
  with the wrist (q or its half-turn twin, the same grasp) the arm's IK reaches;
- ``rope.point`` / ``rope.state`` / ``rope.layout`` -> the rope's nodes read off the
  sim, scored with ``knotlab.task.score`` exactly as ``knotlab.tools`` does.

Nothing welds, pins or teleports: what a hand holds is what its pads hold.
Coordinates: the plan speaks knotlab's WORLD frame; this scene's world is that
plus ``offset`` (the same robot-relative layout, see scene_demo.py). Only this
module converts.

Where the plan comes from (the scene binds one routine per source: ``demo``
takes the first that exists; ``graph``, ``steps``, ``json``, ``smoke`` force one):

1. ``graph``: ``DEMO_GRAPH`` (default ``<gap-knot>/graphs/overhand_demo``): its
   ``workflow.json`` is walked from START, each subgraph's script run with the
   ``ctx``, routed by its ``route`` and the top level's ``conditional_edges``
   (a script that raises routes ``failed``) -- the GaP graph itself, node for node;
2. ``steps``: ``knotlab.demo.demo_steps(layout)``, each step run by the graph's own
   ``scripts/ov_exec.run_step`` when the graph has one (every kind knotlab.demo
   documents), else by the dispatcher below;
3. ``json``: ``demo_steps.json`` beside the scene (a list, or ``{"steps": [...]}``);
4. ``smoke``: lay s=0.10 5 cm toward the robot; lift s=0.40 and lower it.

This module's own dispatcher (no ov_exec): ``lay`` (knotlab.skills.lay of ``s``
to ``xy`` through ``via``), ``thread`` (knotlab.skills.thread_end, crossing at
``makes[0].at``), ``lift_hold`` (the helper takes ``s`` and holds it up
``height``), ``push_under`` (the working hand drags the end so its tip trails at
the crossing ``through_xy``, pushes it under the held strand, regrasps it beyond
and draws it to ``xy``), ``lower`` (the holding hand lays its strand back at
``xy``), ``pull`` (knotlab.skills.pull), and the probes ``pinch`` (take ``s``,
keep it) and ``stop`` (end the plan, hands where they are). After the last step
both hands retreat and the knot is read.

Do not Pause the page during a take: a paused page's arm panels do not drive.

Every tool call and every step's result is logged to ``<run_dir>/demo_take_<n>.json``.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os
import queue
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np

from robosimstudio_studio.routine import RoutineError, hand, reach, wait

ARMS = {0: "left", 1: "right"}
FREE_SPEED = 0.25
"""Gizmo glide for a free move [m/s]: under the setpoint law's 0.31 m/s, as GaP's servo moves free."""
SHORT_SPEED = float(os.environ.get("DEMO_SHORT_SPEED", "0.05"))
"""Glide for a short held-ball move -- a descent onto the cord or the 20 mm verify lift, knotlab's ``descend``
(tolerance under 6 mm) [m/s]. At 0.25 m/s the just-closed jaws lost the cord on the lift every time (4 of 4);
at 0.05 a probe pinch lifted it 14.6 mm of 20."""
CART_SPEED_AT_1 = 0.16
"""``robot.go_to_pose_cartesian`` at speed_scale 1.0 [m/s] (knotlab.skills.SPEED's note)."""
LOADED_TOL = 0.02
"""A hand dragging cord may stop this far from its target [m]: the cord pulls it back (route_the_cable's LOADED)."""
FREE_TOL_MIN = 0.006
"""The gizmo's arrival ball for a free move [m]: a YAM hand at rest sits 3-4 mm off its target (routine.reach)."""
SNAP_PINCH = os.environ.get("DEMO_SNAP_PINCH", "1") != "0"
"""Close a pinch (command >= 0.85) in one step instead of GaP's ramp (see set_grip)."""
Z_SAG = float(os.environ.get("DEMO_Z_SAG", "0.0"))
"""A descent ending under 60 mm over the bench aims this much lower than asked [m] (off by default). A pinch's
hand comes to rest with its tool centre 16.5 mm over the bench for the 13.5 mm asked: the fingertips are on the
bench there. Aiming 2.5 mm lower only pressed them on harder, and the jaws then could not close (measured)."""


class _Stop(Exception):
    """The take ended under the worker (Stop, Reset, or the routine's own error)."""


class _Bridge:
    """The worker's ``ctx``: each ``tool`` call waits for the routine to execute it on the main thread."""

    def __init__(self) -> None:
        self.req: queue.Queue = queue.Queue()
        self.res: queue.Queue = queue.Queue()
        self.calls: list[dict] = []

    def tool(self, name: str, **kwargs):
        self.req.put((name, kwargs))
        out = self.res.get()
        if isinstance(out, BaseException):
            raise out
        return out


def _wxyz(q_xyzw) -> dict:
    x, y, z, w = (float(v) for v in q_xyzw)
    return {"w": w, "x": x, "y": y, "z": z}


def _xyzw(rot: dict) -> np.ndarray:
    return np.array([float(rot["x"]), float(rot["y"]), float(rot["z"]), float(rot["w"])], dtype=np.float64)


class _Tools:
    """The tool calls the plan makes, each a generator run on the page's main thread."""

    def __init__(self, sim, session, *, layout: dict, variant, offset, knotlab) -> None:
        self.sim, self.session = sim, session
        self.layout_bench = layout
        self.variant = variant
        self.offset = np.asarray(offset, dtype=np.float64)
        self.kl = knotlab
        self.hz = max(1.0, float(session.env.request.physics.control_hz))
        self.bench = np.array([0.6, 0.0, 0.75])
        self._last_point = None  # knotlab world: the point the plan last asked about (its next grasp)
        self.wrist_choices: list = []

    # ------------------------------------------------------------------ rope
    def _nodes(self) -> np.ndarray:
        """The rope's nodes in knotlab WORLD [m]."""
        return np.asarray(self.sim["rope"].particle_q, dtype=np.float64).copy() - self.offset

    def rope_point(self, s: float):
        p = self._nodes()
        arc = self.kl.diagram.arc_length(p)
        s = float(np.clip(s, 0.0, arc[-1]))
        k = int(np.clip(np.searchsorted(arc, s) - 1, 0, len(p) - 2))
        u = (s - arc[k]) / max(arc[k + 1] - arc[k], 1e-9)
        at = p[k] + u * (p[k + 1] - p[k])
        self._last_point = at.copy()
        lo, hi = max(k - 2, 0), min(k + 3, len(p) - 1)
        t = p[hi] - p[lo]
        return {"position": at.round(5).tolist(), "heading_deg": float(np.degrees(np.arctan2(t[1], t[0]))), "node": k}
        yield  # noqa: B901 -- a generator that takes no steps

    def rope_state(self):
        p = self._nodes()
        bench = p - self.bench
        s = self.kl.diagram.arc_length(p)
        found = self.kl.diagram.crossings(bench)
        sc = self.kl.task.score(bench, knot="overhand")
        return {
            "nodes": p.round(5).tolist(),
            "arc": s.round(5).tolist(),
            "length": float(s[-1]),
            "code": sc["code"],
            "target_code": self.kl.diagram.code_str(self.kl.task.target_code("overhand")),
            "progress": sc["progress"],
            "prefix": sc["prefix"],
            "tied": sc["tied"],
            "crossings": [
                {"s_a": c.s_i, "s_b": c.s_j, "a_over": c.i_over, "gap": c.gap,
                 "at": [float(c.xy[0] + self.bench[0]), float(c.xy[1] + self.bench[1])]}
                for c in sorted(found, key=lambda c: c.s_i)
            ],
            "ends": {"first": p[0].round(5).tolist(), "last": p[-1].round(5).tolist()},
        }
        yield  # noqa: B901

    def rope_layout(self):
        lay = self.layout_bench
        tgt = np.asarray(lay["target"], dtype=np.float64)
        return {
            "pins": {k: (np.asarray(v) + self.bench).tolist() for k, v in lay["pins"].items()},
            "pin_radius": 0.0025,
            "pin_height": 0.05,
            "target_nodes": (tgt + self.bench).tolist(),
            "target_arc": self.kl.diagram.arc_length(tgt).round(5).tolist(),
            "target_code": self.kl.diagram.code_str(self.kl.task.target_code("overhand")),
            "rope_radius": float(self.variant.radius),
            "board_centre": (np.asarray(lay["board_centre"]) + self.bench).tolist(),
        }
        yield  # noqa: B901

    # ----------------------------------------------------------------- robot
    def _arm(self, arm_id) -> str:
        return ARMS[int(arm_id)]

    def _target(self, pose) -> tuple[np.ndarray, np.ndarray]:
        c = pose["position"]
        p = np.array([float(c["x"]), float(c["y"]), float(c["z"])]) + self.offset
        q = _xyzw(pose["rotation"]) if pose.get("rotation") else None
        return p, q

    def _sag(self, arm, p) -> np.ndarray:
        """A low descent aims Z_SAG lower (see Z_SAG)."""
        now = np.asarray(hand(self.session, arm)[0], dtype=np.float64)
        if Z_SAG and p[2] < 0.75 + 0.06 and p[2] < now[2] - 0.002:
            p = p.copy()
            p[2] -= Z_SAG
        return p

    def _reach(self, arm, p, q, *, speed, tolerance, settle, rest: bool = False):
        """reach(), with its RoutineError turned into GaP's routed ``timeout``.

        ``rest``: then also wait (inside ``settle``) until the hand has stopped -- under 0.15 mm a step for 6
        steps. A GaP descent holds a 2.5 mm ball the panel's hand never enters (it rests 3-4 mm off), and
        reach() returns as soon as the hand is inside its 6 mm ball, still moving: jaws closed then shut on
        the cord's top.
        """
        t0 = self.sim.time if hasattr(self.sim, "time") else None
        outcome = "reached"
        try:
            yield from reach(self.session, arm, p, q, speed=speed, turn=1.5, tolerance=max(tolerance, FREE_TOL_MIN), settle=settle)
        except RoutineError:
            outcome = "timeout"
        if rest:
            last, still = hand(self.session, arm)[0], 0
            for _ in range(int(settle * self.hz)):
                yield
                now = hand(self.session, arm)[0]
                still = still + 1 if float(np.linalg.norm(now - last)) < 1.5e-4 else 0
                last = now
                if still >= 6:
                    break
        miss = float(np.linalg.norm(hand(self.session, arm)[0] - p))
        if outcome == "reached" and miss > max(tolerance, FREE_TOL_MIN) * 1.5:
            outcome = "timeout"
        _ = t0
        at = np.asarray(hand(self.session, arm)[0], dtype=np.float64) - self.offset
        return {"outcome": outcome, "remaining": round(miss, 4), "reached": outcome == "reached",
                "hand": [round(float(v), 4) for v in at], "dz": round(float(at[2] + self.offset[2] - p[2]), 4)}

    def go_to_pose(self, pose, arm_id=0, max_steps=0, tolerance=0.0, **_):
        p, q = self._target(pose)
        p = self._sag(self._arm(arm_id), p)
        tol = float(tolerance or 0.0) or 0.004
        settle = (int(max_steps) / self.hz) if max_steps else 4.0
        short = tol < FREE_TOL_MIN
        return (yield from self._reach(self._arm(arm_id), p, q, speed=SHORT_SPEED if short else FREE_SPEED, tolerance=tol,
                                       settle=max(settle, 0.5) + (1.0 if short else 0.0), rest=short))

    def go_to_pose_cartesian(self, pose, arm_id=0, speed_scale=1.0, **_):
        p, q = self._target(pose)
        p = self._sag(self._arm(arm_id), p)
        speed = CART_SPEED_AT_1 * float(speed_scale)
        return (yield from self._reach(self._arm(arm_id), p, q, speed=speed, tolerance=LOADED_TOL, settle=4.0))

    def go_home(self, arm_id=0, **_):
        arm = self._arm(arm_id)
        p, q = self.session.arms[arm].ik.home_pose
        return (yield from self._reach(arm, np.asarray(p), np.asarray(q), speed=FREE_SPEED, tolerance=LOADED_TOL, settle=4.0))

    def set_grip(self, preset="", width_m=-1.0, fraction_closed=-1.0, arm_id=0, ramp_steps=24, settle_steps=0, **_):
        arm = self._arm(arm_id)
        panel = self.session.arms[arm]
        g = panel.gripper
        if fraction_closed is not None and float(fraction_closed) >= 0.0:
            command = float(np.clip(fraction_closed, 0.0, 1.0))
        elif preset == "open":
            command = 0.0
        elif preset == "close":
            command = 1.0
        else:
            command = float(np.clip(g.command_for_width(float(width_m)), 0.0, 1.0))
        start = float(panel.gripper_command)
        steps = max(1, int(ramp_steps))
        snap = SNAP_PINCH and command >= 0.85 and command > start
        if snap:
            # A pinch closes in one step here, then waits out the ramp: on this page the 48-step ramp let the
            # cord slip out from under the closing pads (A/B at s 0.10: ramped, rise 0.0 mm; snapped, 14.6 of 20).
            panel.gripper_command = command
            steps_wait, steps = steps, 0
        else:
            steps_wait = 0
        # Ramped as GaP ramps it, written straight to the panel's command (which its drain applies every step),
        # not through grip_request, which also writes the page's Grip slider (see below).
        for i in range(1, steps + 1):
            panel.gripper_command = start + (command - start) * (i / steps)
            yield
        for _ in range(int(settle_steps) + steps_wait):
            yield
        # The page's Grip slider is left alone: its callback writes the command back from the slider some steps
        # later, and a slider set to the opening width re-opened a pinch that had just closed (measured).
        measured = float(self.sim.gripper(arm).width(self.sim.state))
        return {"arm": arm, "arm_id": int(arm_id), "command_width_m": float(g.width_for_command(command)),
                "fraction_closed": command, "measured_width_m": round(measured, 4)}

    def open_gripper(self, arm_id=0, **_):
        return (yield from self.set_grip(preset="open", arm_id=arm_id, ramp_steps=6, settle_steps=6))

    def wait_steps(self, steps=30, **_):
        for _ in range(int(steps)):
            yield
        return {"waited": int(steps)}

    def get_ee_pose(self, arm_id=0, **_):
        p, q = hand(self.session, self._arm(arm_id))
        p = np.asarray(p, dtype=np.float64) - self.offset
        return {"pose": {"position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])}, "rotation": _wxyz(q)}}
        yield  # noqa: B901

    def grasp_frame(self, close_heading_deg=0.0, arm_id=0, approach=None, **_):
        from robosimstudio.solvers.grasp.antipodal import grasp_quat

        arm = self._arm(arm_id)
        h = self.sim.scene.embodiment.hands[arm]
        down = np.array([0.0, 0.0, -1.0])
        yaw = math.radians(float(close_heading_deg or 0.0))
        heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        q = grasp_quat(h.approach_axis, h.close_axis, h.tcp_offset_rot, down, heading)
        f = grasp_quat(h.approach_axis, h.close_axis, h.tcp_offset_rot, down, -heading)
        # The jaws are symmetric, so q and f are the same grasp; the arm is not. knotlab folds the heading into
        # [-90, 90) and takes q, which for the right arm at y -0.2 with a cord along x is -90 deg: 28 mm / 0.16 rad
        # off in this arm's IK, and the panel's setpoint law drove the hand into the robot's own base getting
        # there (measured). So the wrist that solves better near where the hand is going (the point rope.point
        # last named, 60 mm over it; else where the hand is) is taken, q on a tie.
        where = self._last_point if self._last_point is not None else (np.asarray(hand(self.session, arm)[0]) - self.offset)
        where = np.asarray(where, dtype=np.float64) + self.offset + np.array([0.0, 0.0, 0.06])
        ik = self.sim.arm(arm)
        seed = self.sim.state.joint_q.numpy().astype(np.float64)

        def err(quat):
            sol = ik.solve_q_full(seed, where, quat)
            e = ik.pose_errors(sol, where, quat)
            return float(e[0]) + 0.05 * float(e[1])

        eq, ef = err(q), err(f)
        if ef + 1e-3 < eq:
            q, f = f, q
        self.wrist_choices.append({"heading": round(float(close_heading_deg or 0.0), 1), "err_q": round(eq, 4),
                                   "err_flipped": round(ef, 4), "flipped": bool(ef + 1e-3 < eq)})
        return {"arm": arm, "rotation": _wxyz(q), "rotation_flipped": _wxyz(f)}
        yield  # noqa: B901

    def dispatch(self, name: str, kwargs: dict):
        table = {
            "rope.point": self.rope_point,
            "rope.state": self.rope_state,
            "rope.layout": self.rope_layout,
            "robot.go_to_pose": self.go_to_pose,
            "robot.go_to_pose_cartesian": self.go_to_pose_cartesian,
            "robot.go_home": self.go_home,
            "robot.set_grip": self.set_grip,
            "robot.open_gripper": self.open_gripper,
            "robot.wait_steps": self.wait_steps,
            "robot.get_ee_pose": self.get_ee_pose,
            "robot.grasp_frame": self.grasp_frame,
        }
        if name not in table:
            raise KeyError(f"tool {name!r} is not served on the live page")
        return table[name](**kwargs)


# ----------------------------------------------------------------------------- plan sources


def _forget_plan_modules() -> None:
    """Drop the graph's ov_* scripts and knotlab.demo from the import cache (and the package attribute that
    ``from knotlab import demo`` would otherwise reuse), so the next import reads them off disk."""
    for name in [m for m in sys.modules if m.startswith("ov_") or m == "knotlab.demo"]:
        del sys.modules[name]
    pkg = sys.modules.get("knotlab")
    if pkg is not None and "demo" in vars(pkg):
        delattr(pkg, "demo")


def _smoke_steps() -> list[dict]:
    """Two steps that exercise a carry and a held lift: lay s=0.10 5 cm toward the robot; lift s=0.40 and lower it."""
    return [
        {"k": 0, "kind": "lay", "s": 0.10, "xy": [0.39, -0.20], "arm_id": 1},
        {"k": 1, "kind": "lift_hold", "s": 0.40, "arm_id": 0, "height": 0.06},
        {"k": 2, "kind": "lower", "arm_id": 0, "xy": None, "s": 0.40},
    ]


def _graph_exec(graph: Path):
    """The demo graph's own step executor (``scripts/ov_exec.py``: ``run_step``, ``Hands``) and params, if there."""
    f = graph / "scripts" / "ov_exec.py"
    if not f.exists():
        return None
    _forget_plan_modules()
    sys.path.insert(0, str(f.parent))
    try:
        mod = _load_script(f)
    finally:
        sys.path.remove(str(f.parent))
    params = json.loads((graph / "params.json").read_text()) if (graph / "params.json").exists() else {}
    return mod, params


def _run_steps(ctx, kl, steps: list[dict], log: list, graph: Path | None = None) -> None:
    """Dispatch a step list by kind: through the demo graph's ``ov_exec.run_step`` when the graph has one (every
    kind knotlab.demo documents), else through knotlab.skills here (lay, thread, lift_hold, lower, push_under,
    pull, and the probes pinch / stop)."""
    skills = kl.skills
    ex = _graph_exec(graph) if graph is not None else None
    if ex is not None:
        mod, params = ex
        hands = mod.Hands()
        length = float(ctx.tool("rope.state")["length"])
        for step in steps:
            t0 = time.time()
            try:
                r = mod.run_step(ctx, step, params, hands, length)
            except _Stop:
                raise
            except Exception as exc:
                r = {"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()[-1500:]}
            st = ctx.tool("rope.state")
            log.append({"k": step.get("k"), "kind": step.get("kind"), "phase": step.get("phase"), "s": step.get("s"),
                        "result": _clean(r), "after": {"code": st["code"], "tied": st["tied"], "crossings": len(st["crossings"])},
                        "wall_s": round(time.time() - t0, 1)})
        for arm in (0, 1):
            skills.retreat(ctx, arm)
        ctx.tool("robot.wait_steps", steps=45)
        return
    held: dict[int, dict] = {}  # arm -> {"rot", "xy", "s"} of a strand it holds up
    length = float(ctx.tool("rope.state")["length"])
    for step in steps:
        kind = step.get("kind", "lay")
        t0 = time.time()
        r: dict = {}
        try:
            if kind == "lay":
                r = skills.lay(ctx, int(step["arm_id"]), float(step["s"]), tuple(step["xy"]),
                               via=tuple(tuple(v) for v in step.get("via", ()) or ()),
                               height=float(step.get("height", skills.CARRY)), retries=int(step.get("retries", 1)))
            elif kind == "thread":
                made = (step.get("makes") or [{}])[0]
                r = skills.thread_end(ctx, int(step["arm_id"]), int(step["helper_arm"]), float(step["s"]) < 0.5 * length,
                                      float(step["lift_s"]), made.get("at", step.get("through_xy")), step["xy"])
            elif kind == "lift_hold":
                r = skills.lift_strand(ctx, int(step["arm_id"]), float(step["s"]), height=float(step.get("height", 0.06)))
                if r.get("ok") or r.get("rot"):
                    held[int(step["arm_id"])] = {"rot": r.get("rot"), "xy": r.get("pinch", {}).get("at"), "s": float(step["s"])}
            elif kind == "lower":
                h = held.pop(int(step["arm_id"]), None)
                if h is not None:
                    xy = step.get("xy") or h["xy"]
                    r = skills.lower_strand(ctx, int(step["arm_id"]), h["rot"], xy, s=h["s"])
                else:
                    r = {"skipped": "nothing held"}
            elif kind == "pinch":  # a probe: take s and keep it (no carry, no release)
                r = skills.pinch(ctx, int(step["arm_id"]), float(step["s"]))
            elif kind == "stop":  # a probe: end the plan here, hands where they are
                log.append({"k": step.get("k"), "kind": "stop"})
                return
            elif kind == "push_under":
                r = _push_under(ctx, skills, step, length)
            elif kind == "pull":
                r = skills.pull(ctx, (0, 1), s_left=step.get("s_left"), s_right=step.get("s_right"),
                                direction_left=tuple(step.get("direction_left", (0.0, 1.0))),
                                direction_right=tuple(step.get("direction_right", (0.0, -1.0))),
                                distance=float(step.get("distance", 0.06)))
            else:
                r = {"skipped": f"unknown kind {kind!r}"}
        except _Stop:
            raise
        except Exception as exc:  # a step that crashes is logged and the plan goes on, as the graph's run_steps does
            r = {"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()[-1500:]}
        st = ctx.tool("rope.state")
        log.append({"k": step.get("k"), "kind": kind, "s": step.get("s"), "result": _clean(r),
                    "after": {"code": st["code"], "tied": st["tied"], "crossings": len(st["crossings"])},
                    "wall_s": round(time.time() - t0, 1)})
    for arm in (0, 1):
        skills.retreat(ctx, arm)
    ctx.tool("robot.wait_steps", steps=45)


def _push_under(ctx, skills, step: dict, length: float) -> dict:
    """The working hand passes the end under the strand the helper already holds up (thread_end's 1, 3, 4)."""
    end_first = bool(step.get("end_first", float(step.get("s", 0.0)) < 0.5 * length))
    tip_s = 0.0 if end_first else length
    sign = 1.0 if end_first else -1.0
    grasp_back, drag, margin = (float(step.get(k, d)) for k, d in (("grasp_back", 0.06), ("drag", 0.075), ("margin", 0.02)))
    tx, ty = (float(v) for v in step["through_xy"])
    bx, by = (float(v) for v in step["xy"])
    ux, uy = bx - tx, by - ty
    n = math.hypot(ux, uy) or 1.0
    ux, uy = ux / n, uy / n
    back = lambda d: (tx - ux * d, ty - uy * d)  # noqa: E731
    arm = int(step["arm_id"])
    out = {"skill": "push_under"}
    p = skills.pinch(ctx, arm, tip_s + sign * grasp_back)
    out["pinch"] = {k: v for k, v in p.items() if k != "rot"}
    if not p["held"]:
        skills.retreat(ctx, arm, p["rot"])
        return {**out, "ok": False, "failure": "not_held"}
    rot = p["rot"]
    z = 0.75
    pose = lambda xy, h: {"position": {"x": xy[0], "y": xy[1], "z": z + h}, "rotation": rot}  # noqa: E731
    h1, h0 = back(margin), back(margin + drag)
    out["to_strand"] = ctx.tool("robot.go_to_pose_cartesian", pose=pose(h1, skills.CARRY), arm_id=arm, speed_scale=0.25)["outcome"]
    out["down"] = skills.descend(ctx, arm, (h1[0], h1[1], z + skills.LOW + 0.002), rot)
    out["drag_back"] = ctx.tool("robot.go_to_pose_cartesian", pose=pose(h0, skills.LOW + 0.002), arm_id=arm, speed_scale=0.15)["outcome"]
    out["push"] = ctx.tool("robot.go_to_pose_cartesian", pose=pose(h1, skills.LOW + 0.002), arm_id=arm, speed_scale=0.15)["outcome"]
    ctx.tool("robot.set_grip", width_m=0.06, arm_id=arm, settle_steps=12)
    skills.move(ctx, arm, (h1[0], h1[1], z + 0.11), rot, speed=1.0)
    tip = ctx.tool("rope.point", s=tip_s)["position"]
    out["tip_past"] = round((tip[0] - tx) * ux + (tip[1] - ty) * uy, 4)
    q = skills.pinch(ctx, arm, tip_s + sign * 0.010, open_width=0.035)
    if not q["held"]:
        skills.retreat(ctx, arm, q["rot"])
        return {**out, "ok": False, "failure": "regrasp_missed"}
    out["pull"] = skills.carry(ctx, arm, [(bx, by)], q["rot"], height=0.022, s=tip_s)
    out["place"] = skills.place(ctx, arm, (bx, by), q["rot"], s=tip_s, height=0.022)
    out["ok"] = True
    return out


def _clean(r):
    if isinstance(r, dict):
        return {k: _clean(v) for k, v in r.items() if k != "rot"}
    if isinstance(r, (list, tuple)):
        return [_clean(v) for v in r]
    if isinstance(r, (np.floating, np.integer)):
        return r.item()
    return r


def _load_script(path: Path):
    spec = importlib.util.spec_from_file_location(f"_demo_graph_{path.stem}_{abs(hash(str(path)))}", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _run_graph(ctx, graph: Path, log: list) -> None:
    """Walk a GaP workflow.json of subgraph nodes (each a script ``act``), as the graph runtime routes it."""
    wf = json.loads((graph / "workflow.json").read_text())
    # The graph's scripts import their siblings (``from ov_common import ...``) and knotlab.demo: start them over,
    # so a take runs the plan code as it is on disk now (the page's process outlives edits to them).
    _forget_plan_modules()
    node = next(b for a, b in wf["edges"] if a == "START")
    seen = 0
    while node in wf["nodes"] and wf["nodes"][node]["type"] == "subgraph" and seen < 64:
        seen += 1
        sg = wf["subgraphs"][wf["nodes"][node]["ref"]]
        script = next(n["script"] for n in sg["nodes"].values() if n.get("type") == "script")
        t0 = time.time()
        step0 = ctx.tool("routine.step")["step"]
        try:
            out = _load_script(graph / script).run(ctx) or {}
            route = out.get("route", "")
            # The subgraph's own router sends a script's route to a noop named for its exit.
            exits = next(iter(sg.get("conditional_edges", {}).values()), {}).get("mapping", {})
            exit_ = exits.get(route, route)
        except _Stop:
            raise
        except Exception as exc:
            out, exit_ = {"error": f"{type(exc).__name__}: {exc}", "trace": traceback.format_exc()[-2000:]}, sg.get("on_error", "failed")
        st = ctx.tool("rope.state")
        log.append({"node": node, "exit": exit_, "steps": [step0, ctx.tool("routine.step")["step"]],
                    "report": _clean(out), "wall_s": round(time.time() - t0, 1),
                    "after": {"code": st["code"], "tied": st["tied"], "crossings": len(st["crossings"])}})
        nxt = wf.get("conditional_edges", {}).get(node, {}).get("mapping", {}).get(exit_)
        if nxt is None:
            nxt = next((b for a, b in wf["edges"] if a == node), None)
        node = nxt
    log.append({"ended_at": node})


def make_routine(*, layout, variant, offset, gap_knot, run_dir, steps_json, source: str = ""):
    """The page's routine: a fresh worker per Run; the plan from ``source``, else ``DEMO_SOURCE``, else the first that exists."""
    forced = source
    gap_knot, run_dir, steps_json = Path(gap_knot), Path(run_dir), Path(steps_json)

    def routine(sim, session):
        import knotlab.diagram
        import knotlab.skills
        import knotlab.task

        class _KL:  # the knotlab modules the tools and dispatcher use
            diagram, skills, task = knotlab.diagram, knotlab.skills, knotlab.task

        tools = _Tools(sim, session, layout=layout, variant=variant, offset=offset, knotlab=_KL)
        bridge = _Bridge()
        log: list = []
        source = forced or os.environ.get("DEMO_SOURCE", "")
        graph = Path(os.environ.get("DEMO_GRAPH", str(gap_knot / "graphs" / "overhand_demo")))
        if not source:
            if (graph / "workflow.json").exists():
                source = "graph"
            elif (gap_knot / "knotlab" / "demo.py").exists():
                source = "steps"
            elif steps_json.exists():
                source = "json"
            else:
                source = "smoke"
        meta = {"source": source, "graph": str(graph) if source == "graph" else None, "offset": list(map(float, offset)),
                "variant": json.loads(variant.to_json()), "started": time.strftime("%Y-%m-%d %H:%M:%S")}
        done = object()
        failure: list = []

        def work() -> None:
            try:
                if source == "graph":
                    _run_graph(bridge, graph, log)
                else:
                    if source == "steps":
                        _forget_plan_modules()  # the plan code as it is on disk now
                        import knotlab.demo

                        steps = knotlab.demo.demo_steps(bridge.tool("rope.layout"))
                    elif source == "json":
                        steps = json.loads(steps_json.read_text())
                        steps = steps.get("steps", steps) if isinstance(steps, dict) else steps
                    else:
                        steps = _smoke_steps()
                    meta["steps"] = _clean(steps)
                    _run_steps(bridge, _KL, steps, log, graph if source == "steps" else None)
            except _Stop:
                failure.append("stopped")
            except Exception:
                failure.append(traceback.format_exc()[-3000:])
            finally:
                bridge.req.put(done)

        worker = threading.Thread(target=work, name="demo-plan", daemon=True)
        worker.start()
        t_start = sim.time if hasattr(sim, "time") else 0.0
        n_steps = 0
        try:
            while True:
                msg = bridge.req.get(timeout=900)
                if msg is done:
                    break
                name, kwargs = msg
                if name == "routine.step":  # the plan asks how many control steps the take has run
                    bridge.res.put({"step": n_steps})
                    continue
                rec = {"tool": name, "args": _clean({k: v for k, v in kwargs.items() if k != "pose"})}
                if "pose" in kwargs:
                    rec["to"] = [round(float(kwargs["pose"]["position"][a]), 4) for a in "xyz"]
                try:
                    gen = tools.dispatch(name, kwargs)
                    while True:
                        try:
                            next(gen)
                        except StopIteration as stop:
                            result = stop.value
                            break
                        n_steps += 1
                        # The live page holds its own stepping while a routine plays and drains the arm
                        # panels each pass (the gizmo -> IK -> joint targets, the Grip control): the
                        # routine steps the scene itself, one control step per pass.
                        yield
                        sim.run(1)
                except GeneratorExit:
                    raise
                except Exception as exc:
                    result = exc
                    rec["error"] = f"{type(exc).__name__}: {exc}"
                if isinstance(result, dict) and "measured_width_m" in result:
                    rec["measured_width_m"] = result["measured_width_m"]
                if isinstance(result, dict) and "outcome" in result:
                    rec["outcome"] = result["outcome"]
                    rec["remaining"] = result.get("remaining")
                    rec["dz"] = result.get("dz")
                rec["step"] = n_steps
                bridge.calls.append(rec)
                bridge.res.put(result)
        except GeneratorExit:
            failure.append("the take was stopped or reset")
            bridge.res.put(_Stop())
            raise
        finally:
            st = None
            try:
                st = tools.rope_state().send(None)
            except StopIteration as stop:
                st = stop.value
            except Exception:
                pass
            meta.update(control_steps=n_steps, sim_s=round(n_steps / tools.hz, 2), failure=failure,
                        wrist_choices=tools.wrist_choices,
                        final={k: st[k] for k in ("code", "tied", "progress")} if st else None)
            k = len(list(run_dir.glob("demo_take_*.json")))
            (run_dir / f"demo_take_{k:03d}.json").write_text(
                json.dumps({"meta": meta, "log": log, "calls": bridge.calls}, indent=1, default=str)
            )
            print(f"[demo] {source}: {meta['final']}, {n_steps} steps; log {run_dir / f'demo_take_{k:03d}.json'}", flush=True)
        _ = t_start

    return routine
