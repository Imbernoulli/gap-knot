"""Sweep the lobe probe graph over parameter sets and task variants on Daytona (at most --size sandboxes).

    /srv/home/bohanlyu/miniconda3/bin/python3 tools/lobe_sweep.py SWEEP.json OUT_DIR [--size 12] [--max-steps 9000]
    /srv/home/bohanlyu/miniconda3/bin/python3 tools/lobe_sweep.py --summarize OUT_DIR

SWEEP.json: {"graph": "graphs/lobe_probe", "configs": {name: params.json content, ...},
             "variants": {"nominal": true, "train": n, "offset": 50000}}
Each config becomes a copy of the graph with its params.json (OUT_DIR/graphs/<name>); episode k of config c lands in
OUT_DIR/<name>/ep<seed>/. The lobe's metrics are computed from the last saved rope frame (world) against
task.layout(variant), and written to OUT_DIR/summary.json.
"""
from __future__ import annotations

import argparse
import json
import shlex
import shutil
import sys
import tarfile
import tempfile
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "graphs" / "lobe_probe" / "scripts"))


def _variants(spec: dict):
    from harness import variants
    from knotlab import task

    vs = [task.NOMINAL] if spec.get("nominal", False) else []
    if spec.get("train"):
        vs += variants.train(int(spec["train"]), int(spec.get("offset", 50_000)))
    if spec.get("seeds"):
        vs += [variants.sample(int(s)) for s in spec["seeds"]]
    return vs


def run(sweep_path: Path, out: Path, *, size: int = 12, max_steps: int = 9000, timeout_s: int = 5400) -> None:
    from fleet.pool import Job, Pool

    sweep = json.loads(sweep_path.read_text())
    graph = ROOT / sweep.get("graph", "graphs/lobe_probe")
    vs = _variants(sweep.get("variants", {"nominal": True}))
    out.mkdir(parents=True, exist_ok=True)
    (out / "sweep.json").write_text(json.dumps(sweep, indent=1))
    tmp = Path(tempfile.mkdtemp(prefix="lobe_"))
    jobs, where = [], {}
    stamp = int(time.time()) % 100000
    for name, params in sweep["configs"].items():
        gdir = out / "graphs" / name
        if gdir.exists():
            shutil.rmtree(gdir)
        shutil.copytree(graph, gdir, ignore=shutil.ignore_patterns("__pycache__"))
        (gdir / "params.json").write_text(json.dumps(params, indent=1))
        tgz = tmp / f"{name}.tgz"
        with tarfile.open(tgz, "w:gz") as tar:
            tar.add(str(gdir), arcname=".", filter=lambda ti: None if "__pycache__" in ti.name else ti)
        for v in vs:
            rid = f"ep{v.seed:06d}"
            if (out / name / rid / "result.json").exists():
                continue
            jid = f"{name[:28]}-{stamp:05d}-{rid}".replace("_", "-")
            cmd = (f'mkdir -p "$GK_JOB_DIR/graph" && tar xzf "$GK_JOB_DIR/graph.tgz" -C "$GK_JOB_DIR/graph" && '
                   + " ".join(shlex.quote(c) for c in ["python", "-m", "knotlab.episode", "--variant", v.to_json(),
                                                        "--out", f"/opt/gk/out/{jid}", "--max-steps", str(max_steps),
                                                        "--every", "30", "--frames", "15"])
                   + ' --graph "$GK_JOB_DIR/graph"')
            jobs.append(Job(id=jid, cmd=cmd, files={str(tgz): "graph.tgz"}, outputs=[f"/opt/gk/out/{jid}"], timeout_s=timeout_s))
            where[jid] = (out / name, rid)
    print(f"{len(jobs)} jobs over {min(size, len(jobs))} sandboxes", flush=True)
    pool = Pool("skill")
    try:
        res = pool.run(jobs, results_dir=str(out / f"_fleet_{stamp}"), size=min(size, len(jobs)), keep=False) if jobs else []
    finally:
        pool.down()
    for r in res:
        d, rid = where[r.id]
        src = Path(r.outputs_dir or "") / r.id
        d.mkdir(parents=True, exist_ok=True)
        if src.exists():
            if (d / rid).exists():
                shutil.rmtree(d / rid)
            shutil.move(str(src), str(d / rid))
        print(r.id, r.status, r.exit_code, r.run_s and round(r.run_s), flush=True)
    shutil.rmtree(tmp, ignore_errors=True)
    shutil.rmtree(out / f"_fleet_{stamp}", ignore_errors=True)
    summarize(out)


def episode_metrics(ep: Path) -> dict | None:
    from knotlab import task, tools
    from lobe_lib import lobe_metrics

    try:
        r = json.loads((ep / "result.json").read_text())
    except Exception:
        return None
    v = task.Variant(**r["variant"])
    task.CURRENT.update(variant=v, layout=task.layout(v))
    lay = tools.RopeTools(None, None).layout()
    sweep_params = json.loads((ep.parent.parent / "graphs" / ep.parent.name / "params.json").read_text()) \
        if (ep.parent.parent / "graphs" / ep.parent.name / "params.json").exists() else {}
    half = sweep_params.get("half", "B")
    row = {"seed": v.seed, "mu": round(v.mu_cable, 2), "steps": r["steps"], "sim_s": r["sim_time_s"],
           "wall_s": r.get("wall_s"), "error": (r.get("error") or "")[-300:]}
    fr = ep / "rope_frames.npy"
    if fr.exists():
        m = lobe_metrics(np.load(fr)[-1].astype(np.float64), lay, half)
        row.update({k: m[k] for k in ("winding", "winding_target", "err_median", "err_p75", "err_max", "n_cross", "n_ok",
                                      "n_flipped", "n_extra", "highest_z", "consistent")})
        row["crossings"] = [f"{c['s'][0]:.2f}/{c['s'][1]:.2f}{'O' if c['over'] else 'U'}:{c['kind'][0]}{c['target'] or ''}"
                            for c in m["crossings"]]
    out_json = ep / "trace" / "0000" / "node_data" / "probe_sg.scenario" / "output.json"
    if out_json.exists():
        rep = json.loads(out_json.read_text()).get("report", {})
        log = rep.get("log", [])
        pin = [x.get("pinch", {}) for x in log if "pinch" in x]
        row["pinches"] = f"{sum(bool(p.get('held')) for p in pin)}/{len(pin)}"
        row["first_try"] = sum(p.get("tries") == 1 and bool(p.get("held")) for p in pin)
        row["place_err"] = [x.get("place", {}).get("error") for x in log if "place" in x]
        row["held_all"] = all(bool(p.get("held")) for p in pin)
    else:
        row["held_all"] = None
    row["success"] = bool(row.get("winding", 0) >= 0.8 and row.get("err_median", 1) <= 0.015 and row.get("consistent"))
    return row


def summarize(out: Path) -> dict:
    rows = {}
    for cdir in sorted(p for p in out.iterdir() if p.is_dir() and not p.name.startswith(("_", "graphs"))):
        rows[cdir.name] = [m for ep in sorted(cdir.glob("ep*")) if ep.is_dir() and (m := episode_metrics(ep))]
    (out / "summary.json").write_text(json.dumps(rows, indent=1))
    for name, eps in rows.items():
        print(f"\n== {name}: {len(eps)} episodes, success {sum(e['success'] for e in eps)}/{len(eps)}")
        for e in eps:
            print(f"  {e['seed']:6d} mu{e['mu']:.2f} t={e['sim_s']:6.1f}s wind {e.get('winding', float('nan')):+.2f}"
                  f"/{e.get('winding_target', 0):.2f} err med {1e3 * e.get('err_median', float('nan')):5.1f} p75 "
                  f"{1e3 * e.get('err_p75', float('nan')):5.1f} max {1e3 * e.get('err_max', float('nan')):5.1f} mm "
                  f"x {e.get('n_cross')} (ok {e.get('n_ok')} flip {e.get('n_flipped')} extra {e.get('n_extra')} cons {int(bool(e.get('consistent')))}) "
                  f"pinch {e.get('pinches')} {' '.join(e.get('crossings', []))} {('ERR ' + e['error'][-120:]) if e['error'] else ''}")
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("sweep", nargs="?")
    ap.add_argument("out", nargs="?")
    ap.add_argument("--size", type=int, default=12)
    ap.add_argument("--max-steps", type=int, default=9000)
    ap.add_argument("--summarize")
    a = ap.parse_args()
    if a.summarize:
        summarize(Path(a.summarize))
        return
    run(Path(a.sweep), Path(a.out), size=min(a.size, 12), max_steps=a.max_steps)


if __name__ == "__main__":
    main()
