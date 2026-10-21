"""Rehearse one graph on a list of task instances, in parallel, locally or on the Daytona fleet.

    python harness/rehearse.py GRAPH_DIR OUT_DIR --n 8 [--backend local|daytona] [--offset 0] [--test]

Each episode writes OUT_DIR/<variant seed>/ (knotlab.episode's result.json, timeline.jsonl, trace/).
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness import variants  # noqa: E402

MAX_STEPS = int(os.environ.get("KNOT_MAX_STEPS", "9000"))  # 150 s of scene time


def _episode_cmd(graph: str, variant_json: str, out: str, max_steps: int) -> list[str]:
    return [str(ROOT / "tools" / "rss.sh"), "-m", "knotlab.episode", "--graph", graph, "--variant", variant_json,
            "--out", out, "--max-steps", str(max_steps), "--every", "30", "--frames", "15"]


def run_local(graph: Path, out: Path, vs: list, *, slots: list[str], max_steps: int = MAX_STEPS, timeout_s: int = 7200) -> list[dict]:
    """One episode per slot at a time; a slot is a taskset core range like '0-31'."""
    out.mkdir(parents=True, exist_ok=True)
    queue = list(vs)
    results: list[dict] = []

    def worker(slot: str) -> None:
        while queue:
            try:
                v = queue.pop(0)
            except IndexError:
                return
            d = out / f"ep{v.seed:06d}"
            if (d / "result.json").exists():
                continue
            t0 = time.time()
            env = {**os.environ, "OMP_NUM_THREADS": str(min(16, int(slot.split("-")[1]) - int(slot.split("-")[0]) + 1))}
            with open(out / f"ep{v.seed:06d}.log", "w") as log:
                try:
                    p = subprocess.run(["taskset", "-c", slot, *_episode_cmd(str(graph), v.to_json(), str(d), max_steps)],
                                       stdout=log, stderr=subprocess.STDOUT, env=env, timeout=timeout_s, cwd=ROOT)
                    code = p.returncode
                except subprocess.TimeoutExpired:
                    code = -9
            results.append({"seed": v.seed, "exit": code, "wall_s": round(time.time() - t0, 1)})

    with ThreadPoolExecutor(len(slots)) as ex:
        list(ex.map(worker, slots))
    return results


def run_daytona(graph: Path, out: Path, vs: list, *, max_steps: int = MAX_STEPS, timeout_s: int = 7200, pool=None) -> list[dict]:
    """The same episodes on the Daytona fleet (fleet.pool.Pool): one episode per sandbox at a time."""
    from fleet.pool import Pool  # built by the fleet agent

    own = pool is None
    pool = pool or Pool.from_env()
    jobs = []
    for v in vs:
        rid = f"ep{v.seed:06d}"
        if (out / rid / "result.json").exists():
            continue
        jobs.append({
            "id": rid,
            "cmd": " ".join(shlex.quote(c) for c in ["python", "-m", "knotlab.episode", "--graph", "graph", "--variant", v.to_json(),
                                                      "--out", f"out/{rid}", "--max-steps", str(max_steps), "--every", "30", "--frames", "15"]),
            "files": {str(graph): "graph"},
            "outputs": [f"out/{rid}"],
            "timeout_s": timeout_s,
            "env": {},
        })
    try:
        res = pool.run(jobs, results_dir=str(out))
    finally:
        if own:
            pool.close()
    return res


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("graph")
    ap.add_argument("out")
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--offset", type=int, default=0)
    ap.add_argument("--test", action="store_true")
    ap.add_argument("--nominal", action="store_true", help="only the nominal instance")
    ap.add_argument("--backend", default="local")
    ap.add_argument("--slots", default="0-31,32-63,64-95,96-127")
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    a = ap.parse_args()
    from knotlab.task import NOMINAL

    vs = [NOMINAL] if a.nominal else (variants.test(a.n) if a.test else variants.train(a.n, a.offset))
    graph, out = Path(a.graph).resolve(), Path(a.out).resolve()
    if a.backend == "local":
        r = run_local(graph, out, vs, slots=a.slots.split(","), max_steps=a.max_steps)
    else:
        r = run_daytona(graph, out, vs, max_steps=a.max_steps)
    print(json.dumps(r))
    from harness.analyze import analyze

    stats, diags, md = analyze(out)
    (out / "REPORT.md").write_text(md)
    (out / "REPORT.json").write_text(json.dumps({"stats": stats, "episodes": diags}, indent=1))
    print(md[:2000])


if __name__ == "__main__":
    main()
