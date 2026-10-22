"""One rehearsal: a graph run on one variant of the task, with the rope read at every node boundary.

    python -m knotlab.episode --graph graphs/coin_v0 --variant '{"seed": 3}' --out runs/x/ep3

Writes ``result.json`` (outcome, sim time, per-node rope progress) and
``timeline.jsonl`` (the rope's score every ``--every`` steps) in ``--out``. The
per-node pre/post states are what the self-learning loop localizes failures with.
"""

from __future__ import annotations

import argparse
import json
import os
import time
import traceback
from pathlib import Path

import numpy as np


def run(graph: str, variant_json: str, out: str, *, max_steps: int = 6000, every: int = 30, frames: int = 0) -> dict:
    from robosimstudio_bench.run.runner import rollout
    from robosimstudio_policy_agent import SolutionGraph

    from knotlab import task, tools

    out_dir = Path(out)
    out_dir.mkdir(parents=True, exist_ok=True)
    v = task.Variant.from_json(variant_json) if variant_json else task.NOMINAL
    wall0 = time.perf_counter()
    env = task.build(v, max_steps=max_steps)
    origin = tools.BENCH_ORIGIN
    events: list[dict] = []
    timeline = open(out_dir / "timeline.jsonl", "w")
    snaps: list[np.ndarray] = []
    hands: list[list[float]] = []  # per saved frame: step, then each arm's TCP x, y, z and jaw width

    from knotlab import diagram

    def rope_score(detail: bool = False) -> dict:
        p = np.asarray(env["rope"].particle_q, dtype=np.float64) - origin
        sc = task.score(p)
        row = {k: sc[k] for k in ("code", "crossings", "prefix", "progress", "tied", "knot_length", "max_height")}
        if detail:  # each crossing: where along the rope its two strands are, and which is over
            row["x"] = [[round(c.s_i, 3), round(c.s_j, 3), int(c.i_over)] for c in sorted(diagram.crossings(p), key=lambda c: c.s_i)]
        return row

    class Traced(SolutionGraph):
        current: str = ""

        def _on_graph_node_start(self, name, success, generation=None):
            self.current = name
            events.append({"node": name, "event": "start", "step": int(env.step_count)})
            return super()._on_graph_node_start(name, success, generation)

        def _on_graph_node_end(self, name, success, generation=None):
            events.append({"node": name, "event": "end", "step": int(env.step_count), "ok": bool(success)})
            return super()._on_graph_node_end(name, success, generation)

    # bus_timeout: a tool call (an IK solve, a long carry) may take minutes of wall time on a 2-vCPU sandbox;
    # the default 30 s watchdog turned slow machines into graph failures.
    policy = Traced(env, graph=graph, task_tools=tools.register, trace_dir=str(out_dir / "trace"), bus_timeout=600.0)

    def observe(current, action, observed, earned, details):
        step = int(env.step_count)
        if step % every == 0:
            row = {"step": step, "t": round(step / 60.0, 3), "wall": round(time.time(), 2), **rope_score(detail=True)}
            timeline.write(json.dumps(row) + "\n")
            timeline.flush()
        if frames and step % frames == 0:
            snaps.append(np.asarray(env["rope"].particle_q, dtype=np.float32).copy())
            row = [float(step)]
            for arm in ("left", "right"):
                try:
                    p, _ = env.arm(arm).ee_pose(env.state)
                    row += [float(p[0]), float(p[1]), float(p[2]), float(env.gripper(arm).width(env.state))]
                except Exception:
                    row += [float("nan")] * 4
            hands.append(row)

    env.add_step_observer(observe)
    error = ""
    try:
        result = rollout(env, policy, episode_id=0, max_steps=max_steps)
        success, steps = bool(result.success), int(result.steps)
    except Exception:  # an episode that crashes is a failed episode, with its traceback
        error = traceback.format_exc()
        success, steps = False, int(getattr(env, "step_count", 0))
    final = rope_score(detail=True)
    np.save(out_dir / "rope_final.npy", np.asarray(env["rope"].particle_q, dtype=np.float64))
    # Per node: the rope's progress when it started and when it ended.
    nodes = []
    for e in events:
        if e["event"] == "end":
            nodes.append(e)
    summary = {
        "grader_version": task.GRADER_VERSION,
        "graph": graph,
        "variant": json.loads(v.to_json()),
        "success": success,
        "steps": steps,
        "sim_time_s": round(steps / 60.0, 2),
        "wall_s": round(time.perf_counter() - wall0, 1),
        "final": final,
        "events": events,
        "error": error[-4000:],
    }
    (out_dir / "result.json").write_text(json.dumps(summary, indent=1))
    if snaps:
        np.save(out_dir / "rope_frames.npy", np.stack(snaps).astype(np.float32))
        np.save(out_dir / "hand_frames.npy", np.asarray(hands, dtype=np.float32))
    timeline.close()
    try:
        policy.close()
    except Exception:
        pass
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--graph", required=True)
    ap.add_argument("--variant", default="")
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-steps", type=int, default=6000)
    ap.add_argument("--every", type=int, default=30)
    ap.add_argument("--frames", type=int, default=0, help="save the rope's nodes every N steps (0: never)")
    a = ap.parse_args()
    s = run(a.graph, a.variant, a.out, max_steps=a.max_steps, every=a.every, frames=a.frames)
    print(json.dumps({k: s[k] for k in ("success", "steps", "sim_time_s", "wall_s", "final")}))
    if s["error"]:
        print(s["error"][-2000:])
    os._exit(0)  # the graph's thread and the studio's servers do not always join


if __name__ == "__main__":
    main()
