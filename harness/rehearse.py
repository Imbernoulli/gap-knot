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

MAX_STEPS = int(os.environ.get("KNOT_MAX_STEPS", "18000"))  # 300 s of scene time


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


def run_daytona_many(specs: list[tuple[Path, Path]], vs: list, *, max_steps: int = MAX_STEPS, timeout_s: int = 5400,
                     pool=None) -> list[dict]:
    """Several graphs, each on the same instances, as ONE batch on the Daytona fleet (fleet.pool.Pool).

    ``specs`` is [(graph_dir, out_dir)]; episode k of graph g lands in out_dir/ep<seed>. One episode per
    2-vCPU sandbox at a time; needs the daytona SDK, so run this under miniconda's python.
    """
    import shutil
    import tarfile
    import tempfile

    from fleet.pool import Job, Pool

    own = pool is None
    pool = pool or Pool("loop")
    tmp = Path(tempfile.mkdtemp(prefix="graphs_"))
    jobs, where = [], {}
    for g, (graph, out) in enumerate(specs):
        out.mkdir(parents=True, exist_ok=True)
        tgz = tmp / f"g{g}.tgz"
        with tarfile.open(tgz, "w:gz") as tar:
            tar.add(str(graph), arcname=".", filter=lambda ti: None if "__pycache__" in ti.name else ti)
        for v in vs:
            rid = f"ep{v.seed:06d}"
            if (out / rid / "result.json").exists():
                continue
            jid = f"g{g}-{int(time.time()) % 100000:05d}-{rid}"
            cmd = (f'mkdir -p "$GK_JOB_DIR/graph" && tar xzf "$GK_JOB_DIR/graph.tgz" -C "$GK_JOB_DIR/graph" && '
                   + " ".join(shlex.quote(c) for c in ["python", "-m", "knotlab.episode", "--variant", v.to_json(),
                                                        "--out", f"/opt/gk/out/{jid}", "--max-steps", str(max_steps), "--every", "30", "--frames", "15"])
                   + ' --graph "$GK_JOB_DIR/graph"')
            jobs.append(Job(id=jid, cmd=cmd, files={str(tgz): "graph.tgz"}, outputs=[f"/opt/gk/out/{jid}"], timeout_s=timeout_s))
            where[jid] = (out, rid)
    try:
        fleet_dir = specs[0][1].parent / f"_fleet_{int(time.time())}"
        # Exactly as many sandboxes as episodes (up to the cap), torn down after: nothing idles between batches.
        res = pool.run(jobs, results_dir=str(fleet_dir), size=min(len(jobs), 48), keep=False) if jobs else []
    finally:
        if own:
            pool.down()
    rows = []
    for r in res:
        out, rid = where[r.id]
        src = Path(r.outputs_dir or "") / r.id
        if src.exists():
            if (out / rid).exists():
                shutil.rmtree(out / rid)
            shutil.move(str(src), str(out / rid))
        rows.append({"id": r.id, "status": r.status, "exit": r.exit_code, "run_s": r.run_s, "attempts": r.attempts})
    shutil.rmtree(tmp, ignore_errors=True)
    return rows


def run_daytona(graph: Path, out: Path, vs: list, **kw) -> list[dict]:
    return run_daytona_many([(graph, out)], vs, **kw)


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
        r = run_daytona(graph, out, vs, max_steps=a.max_steps)  # needs the daytona SDK: run with miniconda's python
    print(json.dumps(r))
    from harness.analyze import analyze

    stats, diags, md = analyze(out)
    (out / "REPORT.md").write_text(md)
    (out / "REPORT.json").write_text(json.dumps({"stats": stats, "episodes": diags}, indent=1))
    print(md[:2000])


if __name__ == "__main__":
    main()
