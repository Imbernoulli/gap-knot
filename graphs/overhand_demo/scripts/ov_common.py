"""Shared by the overhand demo graph's scripts: its parameters, the demonstration's plan, and running a phase of it."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_exec import Hands, run_step  # noqa: E402

from knotlab import demo, skills  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def params() -> dict:
    with open(os.path.join(HERE, "..", "params.json")) as f:
        return json.load(f)


def the_plan(ctx, st: dict | None = None) -> dict:
    """The demonstration's events retargeted to this episode's rope (knotlab.demo.demo_steps on rope.layout),
    the later phases planned on the rope as it lies now (``st``: rope.state)."""
    return demo.plan(ctx.tool("rope.layout"), st, **params().get("plan", {}))


def _log(row: dict) -> None:
    """With params' log_states, append the rope after each step to the episode's --out dir (rehearsal probes)."""
    a = sys.argv
    out = next((a[i + 1] for i, x in enumerate(a) if x == "--out" and i + 1 < len(a)), None)
    if out:
        os.makedirs(out, exist_ok=True)
        with open(os.path.join(out, "states.jsonl"), "a") as f:
            f.write(json.dumps(row, default=str) + "\n")


def state(ctx) -> dict:
    st = ctx.tool("rope.state")
    return {"code": st["code"], "progress": st["progress"], "tied": st["tied"], "crossings": len(st["crossings"])}


def run_phase(ctx, phase: str) -> dict:
    """Run the plan's steps of one phase, in order; carries on past a failed step (the rope is read after).
    Whatever a hand still holds when the phase ends is let go."""
    P = params()
    st = ctx.tool("rope.state")
    pl = the_plan(ctx, st)
    L = float(st["length"])
    hands = Hands()
    done = []
    log = bool(P.get("log_states"))
    if log and phase == "loop":
        _log({"k": -1, "phase": phase, "nodes": st["nodes"], "code": st["code"]})
    ks = [s["k"] for s in pl["steps"] if s.get("phase") == phase]
    steps = {s["k"]: s for s in pl["steps"]}
    for k in ks:
        step = steps[k]
        try:
            r = run_step(ctx, step, P, hands, L)
        except Exception as e:  # a failed tool call inside a step: report it, read the rope after
            r = {"k": step.get("k"), "kind": step.get("kind"), "ok": False, "failure": repr(e)[:200]}
        done.append(r)
        now = ctx.tool("rope.state") if (log or step.get("replan_after")) else None
        if log:
            _log({"k": step.get("k"), "step": step, "result": r, "nodes": now["nodes"], "code": now["code"],
                  "crossings": now["crossings"]})
        if step.get("replan_after"):  # the rest of the phase planned on the rope as this step left it
            steps.update({s["k"]: s for s in the_plan(ctx, now)["steps"] if s["k"] > k})
    for arm, hd in list(hands.held.items()):
        skills.retreat(ctx, arm, hd["rot"])
    return {"steps": done, "ok": all(d.get("ok") for d in done), "state": state(ctx)}
