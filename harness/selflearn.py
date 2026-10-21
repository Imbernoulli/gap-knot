"""The GaP loop for the double coin knot: generate a graph with Codex agents, then rehearse -> localize -> refine.

    python harness/selflearn.py --run RUN_DIR [--graph START_GRAPH] [--iters 8] [--n 12] [--cands 3] [--backend daytona]

State lives in RUN_DIR (resumable: a finished step is not redone):
  gen/                    orchestration + skill agents' workspaces, and gen/graph (the generated graph)
  iter_KK/incumbent/      the incumbent rehearsed on this iteration's instances (+ REPORT.md)
  iter_KK/cand_X/ws       a refiner's workspace (its candidate in ws/graph)
  iter_KK/cand_X/eval     that candidate rehearsed on the same instances
  best/                   the incumbent graph after the latest iteration
  history.json, HISTORY.md
"""
from __future__ import annotations

import argparse
import json
import random
import shutil
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from harness import codex_agent, rehearse, variants  # noqa: E402
from harness.analyze import analyze  # noqa: E402


def check(graph_dir) -> list[str]:
    """The static validator, run in RoboSimStudio's venv (this loop runs where the Daytona SDK is)."""
    import subprocess

    p = subprocess.run([str(ROOT / "harness" / "validate.sh"), str(graph_dir)], capture_output=True, text=True, timeout=600)
    lines = [l for l in p.stdout.splitlines() if l.strip() and not l.startswith("warning:")]
    return [] if p.returncode == 0 and lines and lines[-1] == "VALID" else (lines or [p.stderr[-500:]])

PROMPTS = ROOT / "harness" / "prompts"
IMAGES = [ROOT / "harness" / "docs" / "img" / "video_storyboard.jpg", ROOT / "harness" / "docs" / "img" / "start_and_target.png"]


def log(run: Path, msg: str) -> None:
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
    print(line, flush=True)
    with open(run / "loop.log", "a") as f:
        f.write(line + "\n")


# ------------------------------------------------------------------ generation
def generate(run: Path) -> Path:
    gen = run / "gen"
    final = gen / "graph"
    if (final / "workflow.json").exists() and not check(final):
        return final
    coord = codex_agent.prepare(gen / "coordinator")
    (coord / "graph" / "scripts").mkdir(parents=True, exist_ok=True)
    if not (coord / ".last_coordinator.md").exists() or check(coord / "graph"):
        log(run, "generation: orchestration agent decomposing the task")
        r = codex_agent.run("coordinator", coord, (PROMPTS / "coordinator.md").read_text(), images=IMAGES, timeout_s=3600)
        log(run, f"  coordinator exit={r['exit']} {r['wall_s']}s")
        probs = check(coord / "graph")
        if probs:
            fix = "The graph in graph/ does not validate:\n" + "\n".join(probs) + "\nFix it (keep the decomposition) and run ./validate.sh until VALID."
            codex_agent.run("coordinator_fix", coord, fix, timeout_s=1800)
            probs = check(coord / "graph")
            if probs:
                raise SystemExit("orchestration graph does not validate:\n" + "\n".join(probs))
    skeleton = json.loads((coord / "graph" / "workflow.json").read_text())
    sgs = list(skeleton.get("subgraphs", {}))
    log(run, f"generation: {len(sgs)} subgraphs {sgs}; skill agents in parallel")

    def fill(sg: str) -> dict:
        ws = codex_agent.prepare(gen / f"sg_{sg}", coord / "graph")
        if (ws / f".last_skill_{sg}.md").exists() and not check(ws / "graph"):
            return {"sg": sg, "exit": 0, "cached": True}
        prompt = (PROMPTS / "subgraph.md").read_text().replace("{sg}", sg)
        r = codex_agent.run(f"skill_{sg}", ws, prompt, images=IMAGES[1:], timeout_s=3600)
        return {"sg": sg, **{k: r[k] for k in ("exit", "wall_s")}}

    with ThreadPoolExecutor(len(sgs)) as ex:
        for r in ex.map(fill, sgs):
            log(run, f"  skill agent {r}")
    # Merge: the skeleton, each subgraph's body from its own agent, each agent's own scripts.
    merged = json.loads(json.dumps(skeleton))
    if final.exists():
        shutil.rmtree(final)
    shutil.copytree(coord / "graph", final)
    for sg in sgs:
        ws = gen / f"sg_{sg}" / "graph"
        mine = json.loads((ws / "workflow.json").read_text())
        merged["subgraphs"][sg] = mine["subgraphs"][sg]
        for sp in (ws / "scripts").glob(f"{sg}_*.py"):
            shutil.copy2(sp, final / "scripts" / sp.name)
        for sp in (ws / "scripts").glob("*.py"):  # anything else it wrote is noted, not merged
            if not sp.name.startswith(f"{sg}_") and not (coord / "graph" / "scripts" / sp.name).exists():
                log(run, f"  note: {sg}'s agent also wrote scripts/{sp.name} (not merged)")
    (final / "workflow.json").write_text(json.dumps(merged, indent=1))
    probs = check(final)
    if probs:
        log(run, "merged graph does not validate; orchestration agent fixes it")
        ws = codex_agent.prepare(gen / "merge_fix", final)
        msg = ("The graph in graph/ was assembled from the orchestration skeleton and each skill agent's subgraph. "
               "It does not validate:\n" + "\n".join(probs) + "\nFix only what is needed to make it valid and coherent "
               "(read graph/SEGMENTS.md for intent); run ./validate.sh until VALID.")
        codex_agent.run("merge_fix", ws, msg, timeout_s=1800)
        shutil.rmtree(final)
        shutil.copytree(ws / "graph", final)
        probs = check(final)
        if probs:
            raise SystemExit("generated graph does not validate:\n" + "\n".join(probs))
    log(run, f"generation done: {final}")
    return final


# ------------------------------------------------------------------ rehearsal
def rehearse_graph(graph: Path, out: Path, vs: list, backend: str, pool=None) -> dict:
    if backend == "local":
        rehearse.run_local(graph, out, vs, slots=["0-31", "32-63", "64-95", "96-127"])
    else:
        rehearse.run_daytona(graph, out, vs, pool=pool)
    stats, diags, md = analyze(out, title=f"Rehearsal of {graph.parent.name}/{graph.name}")
    (out / "REPORT.md").write_text(md)
    (out / "REPORT.json").write_text(json.dumps({"stats": stats, "episodes": diags}, indent=1))
    return stats


def rehearse_many(specs: list, vs: list, backend: str, pool=None) -> list[dict]:
    """Several graphs on the same instances in one batch, each then analysed on its own."""
    if not specs:
        return []
    if backend == "local":
        for graph, out in specs:
            rehearse.run_local(graph, out, vs, slots=["0-31", "32-63", "64-95", "96-127"])
    else:
        rehearse.run_daytona_many(specs, vs, pool=pool)
    stats = []
    for graph, out in specs:
        st, diags, md = analyze(out, title=f"Rehearsal of {out.parent.name}")
        (out / "REPORT.md").write_text(md)
        (out / "REPORT.json").write_text(json.dumps({"stats": st, "episodes": diags}, indent=1))
        stats.append(st)
    return stats


def better(a: dict, b: dict, margin: float = 0.0) -> bool:
    """Is a better than b: more successes, then a higher code prefix, then a shorter cycle."""
    if a["successes"] != b["successes"]:
        return a["successes"] > b["successes"] + margin
    if abs(a["mean_prefix"] - b["mean_prefix"]) > 0.5:
        return a["mean_prefix"] > b["mean_prefix"]
    ta, tb = a.get("mean_time_success_s") or 1e9, b.get("mean_time_success_s") or 1e9
    return ta < tb * 0.95


def focuses(stats: dict, report: dict, k: int) -> list[str]:
    """Different things for the k refiners to work on, from the report."""
    out = []
    causes = sorted(stats["causes"].items(), key=lambda kv: -kv[1])
    for c, m in causes:
        out.append(f"the most common failure: '{c}' ({m} of {stats['episodes']} episodes)")
    flipped = sorted(((cid, c.get("FLIPPED", 0) + c.get("MISSING", 0)) for cid, c in stats["crossings"].items()), key=lambda kv: -kv[1])
    for cid, m in flipped:
        if m:
            born = stats["flipped_born_in"].get(cid, {})
            where = max(born, key=born.get) if born else "unknown node"
            out.append(f"target crossing #{cid}, wrong or missing in {m} of {stats['episodes']} episodes (flipped ones made in {where})")
    if stats["success_rate"] >= 0.5:
        out.insert(min(2, len(out)), "throughput: cut the cycle time (sim seconds) of successful episodes without lowering the success rate — fewer moves, faster safe speeds, no redundant checks")
    out.append("robustness across task instances: look at which variants fail (stiffness, friction, jig offset/yaw, rope layout) and make the graph adapt to them")
    seen, uniq = set(), []
    for f in out:
        if f not in seen:
            uniq.append(f)
            seen.add(f)
    return (uniq + ["a fresh hypothesis for the most common failure, different from what HISTORY.md tried"] * k)[:k]


def evidence(src: Path, dst: Path, diags: list[dict], max_eps: int = 4) -> None:
    dst.mkdir(parents=True, exist_ok=True)
    fails = [d for d in diags if not d["success"]][: max_eps - 1] + [d for d in diags if d["success"]][:1]
    for d in fails:
        e = src / d["id"]
        if not e.exists():
            continue
        out = dst / d["id"]
        out.mkdir(exist_ok=True)
        for f in ("result.json", "timeline.jsonl"):
            if (e / f).exists():
                shutil.copy2(e / f, out / f)
        nd = e / "trace" / "0000" / "node_data"
        if nd.exists():
            shutil.copytree(nd, out / "node_data", dirs_exist_ok=True, ignore=shutil.ignore_patterns("assets", "*.npy", "*.png"))


def history_md(hist: list[dict]) -> str:
    L = ["# History of this self-learning run", ""]
    for h in hist:
        L.append(f"## Iteration {h['iter']}: incumbent {h['incumbent']['successes']}/{h['incumbent']['episodes']} "
                 f"(prefix {h['incumbent']['mean_prefix']:.1f}, success time {h['incumbent'].get('mean_time_success_s')})")
        for c in h["candidates"]:
            L.append(f"- cand {c['name']} [{c['focus']}]: {c.get('stats', {}).get('successes', '?')}/{c.get('stats', {}).get('episodes', '?')} "
                     f"(prefix {c.get('stats', {}).get('mean_prefix', 0):.1f}, success time {c.get('stats', {}).get('mean_time_success_s')}) "
                     f"{'ACCEPTED' if c.get('accepted') else 'rejected'}{' — ' + c['invalid'] if c.get('invalid') else ''}")
            if c.get("changes"):
                L.append("  changes: " + c["changes"][:1200].replace("\n", "\n  "))
        L.append("")
    return "\n".join(L)


# ------------------------------------------------------------------ the loop
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True)
    ap.add_argument("--graph", default="")
    ap.add_argument("--iters", type=int, default=8)
    ap.add_argument("--n", type=int, default=12)
    ap.add_argument("--cands", type=int, default=3)
    ap.add_argument("--backend", default="daytona")
    ap.add_argument("--test-every", type=int, default=3)
    ap.add_argument("--n-test", type=int, default=20)
    ap.add_argument("--generate-only", action="store_true")
    a = ap.parse_args()
    run = Path(a.run).resolve()
    run.mkdir(parents=True, exist_ok=True)
    hist_path = run / "history.json"
    hist = json.loads(hist_path.read_text()) if hist_path.exists() else []
    if hist:
        graph = run / "best"
    elif a.graph:
        graph = Path(a.graph).resolve()
    else:
        graph = generate(run)
    if a.generate_only:
        return
    best = run / "best"
    if not best.exists():
        shutil.copytree(graph, best, ignore=shutil.ignore_patterns("__pycache__"))
    pool = None
    if a.backend == "daytona":
        from fleet.pool import Pool

        pool = Pool("loop")
        pool.up(min(48, a.n * (a.cands + 1)))
    try:
        for it in range(len(hist), a.iters):
            itd = run / f"iter_{it:02d}"
            vs = variants.train(a.n, offset=it * a.n)  # fresh instances every iteration, shared by all candidates in it
            log(run, f"iter {it}: rehearsing the incumbent on {len(vs)} instances")
            inc = rehearse_graph(best, itd / "incumbent", vs, a.backend, pool)
            log(run, f"  incumbent {inc['successes']}/{inc['episodes']} prefix {inc['mean_prefix']:.1f} time {inc.get('mean_time_success_s')}")
            rep = json.loads((itd / "incumbent" / "REPORT.json").read_text())
            fs = focuses(inc, rep, a.cands)
            (run / "HISTORY.md").write_text(history_md(hist))

            def refine(k: int) -> dict:
                name = "abcdefgh"[k]
                cd = itd / f"cand_{name}"
                ws = codex_agent.prepare(cd / "ws", best)
                shutil.copy2(itd / "incumbent" / "REPORT.md", ws / "docs" / "REPORT.md")
                shutil.copy2(run / "HISTORY.md", ws / "docs" / "HISTORY.md")
                evidence(itd / "incumbent", ws / "evidence", rep["episodes"])
                prompt = (PROMPTS / "refiner.md").read_text().replace("{n}", str(len(vs))).replace("{focus}", fs[k])
                r = codex_agent.run("refiner", ws, prompt, timeout_s=3600)
                probs = check(ws / "graph")
                changes = (ws / "graph" / "CHANGES.md").read_text() if (ws / "graph" / "CHANGES.md").exists() else ""
                return {"name": name, "focus": fs[k], "exit": r["exit"], "wall_s": r["wall_s"], "graph": str(ws / "graph"),
                        "invalid": "; ".join(probs)[:500] if probs else "", "changes": changes}

            log(run, f"  refiners: {fs}")
            with ThreadPoolExecutor(a.cands) as ex:
                cands = list(ex.map(refine, range(a.cands)))
            valid = [c for c in cands if not c["invalid"]]
            log(run, f"  {len(valid)}/{len(cands)} candidates valid; rehearsing them on the same instances")
            stats = rehearse_many([(Path(c["graph"]), itd / f"cand_{c['name']}" / "eval") for c in valid], vs, a.backend, pool)
            for c, s in zip(valid, stats):
                c["stats"] = s
                log(run, f"  cand {c['name']}: {s['successes']}/{s['episodes']} prefix {s['mean_prefix']:.1f} time {s.get('mean_time_success_s')}")
            winner = None
            for c in valid:
                if better(c["stats"], inc) and (winner is None or better(c["stats"], winner["stats"])):
                    winner = c
            if winner:
                winner["accepted"] = True
                shutil.rmtree(best)
                shutil.copytree(winner["graph"], best, ignore=shutil.ignore_patterns("__pycache__"))
                log(run, f"  accepted cand {winner['name']}")
            else:
                log(run, "  kept the incumbent")
            hist.append({"iter": it, "incumbent": inc, "candidates": cands, "accepted": winner["name"] if winner else None})
            hist_path.write_text(json.dumps(hist, indent=1))
            (run / "HISTORY.md").write_text(history_md(hist))
            shutil.copytree(best, run / f"graph_after_iter_{it:02d}", dirs_exist_ok=True)
            if a.test_every and (it + 1) % a.test_every == 0:
                log(run, f"  held-out test of the incumbent on {a.n_test} instances")
                t = rehearse_graph(best, itd / "test", variants.test(a.n_test), a.backend, pool)
                log(run, f"  TEST {t['successes']}/{t['episodes']} time {t.get('mean_time_success_s')}")
                hist[-1]["test"] = t
                hist_path.write_text(json.dumps(hist, indent=1))
    finally:
        if pool is not None:
            pool.down()


if __name__ == "__main__":
    main()
