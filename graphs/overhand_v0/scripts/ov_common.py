"""Shared by the overhand graph's scripts: its parameters, the laying plan, and running a plan step."""
import json
import os

from knotlab import diagram, plan, skills

HERE = os.path.dirname(os.path.abspath(__file__))


def params() -> dict:
    with open(os.path.join(HERE, "..", "params.json")) as f:
        return json.load(f)


def the_plan(ctx) -> dict:
    """The laying plan for this episode's target (rope.layout), each half laid outward from the rope's middle."""
    p = params()["plan"]
    return plan.lay_plan(ctx.tool("rope.layout"), spacing=p["spacing"], start_d=p["start_d"])


def state(ctx) -> dict:
    st = ctx.tool("rope.state")
    return {"code": st["code"], "progress": st["progress"], "tied": st["tied"], "crossings": len(st["crossings"])}


def lay_step(ctx, step: dict) -> dict:
    """One laying step: pinch the rope at s, carry it (over whatever it passes), set it down at its target."""
    p = params()["lay"]
    pk = params().get("pinch", {})
    over = (step["xy"][0], step["xy"][1])
    r = skills.lay(ctx, step["arm_id"], step["s"], over, height=p["height"], retries=p["retries"], **pk)
    return {"k": step["k"], "kind": "lay", "ok": bool(r.get("ok")), "failure": r.get("failure"), "error": r.get("error")}


def thread_step(ctx, step: dict, length: float) -> dict:
    """The under-crossing: the helper lifts the strand at lift_s, the working hand pushes the free end under it."""
    p = params()["thread"]
    made = step["makes"][0]
    end_first = step["s"] < 0.5 * length
    last = {}
    for attempt in range(int(p["tries"])):
        last = skills.thread_end(
            ctx, step["arm_id"], step["helper_arm"], end_first, step["lift_s"], made["at"], step["xy"],
            grasp_back=p["grasp_back"], drag=p["drag"], margin=p["margin"], lift=p["lift"], pull_height=p["pull_height"],
        )
        if last.get("ok"):
            break
    return {"k": step["k"], "kind": "thread", "ok": bool(last.get("ok")), "failure": last.get("failure"), "attempts": attempt + 1}


def run_steps(ctx, which) -> dict:
    """Run the plan steps ``which(step)`` selects, in order; carries on past a failed step (the rope is read after)."""
    pl = the_plan(ctx)
    done = []
    for step in pl["steps"]:
        if not which(step, pl):
            continue
        done.append(thread_step(ctx, step, pl["length"]) if step["kind"] == "thread" else lay_step(ctx, step))
    return {"steps": done, "ok": all(d["ok"] for d in done), "state": state(ctx)}


def first_thread(pl) -> int:
    ks = [s["k"] for s in pl["steps"] if s["kind"] == "thread"]
    return ks[0] if ks else 10**6
