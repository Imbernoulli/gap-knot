"""Benchmark the knot-pull control step across sandbox CPU shapes (one pool per shape, in parallel).

    /srv/home/bohanlyu/miniconda3/bin/python3 fleet/bench/run_bench.py \
        --grid "1:0,1,default;2:2,default;4:4,default;8:4,8,default;16:4,8,16;32:8,32;64:8,16,64" \
        --memory 4 --spacing 0.004 --repeats 1

``--grid`` is ``vCPU:variants;...``; a variant is a SuperDex worker-thread count (0 is
single-threaded), ``quota`` (= the sandbox's vCPUs) or ``default`` (RSS's -1: SuperDex
sizes its pool from the hardware it sees, which in a sandbox is the host's 48 cores, not
the quota). Every job is one run of fleet/bench/knot_step.py on its own sandbox; every
pool is deleted when its jobs finish. Writes summary.json and summary.md (with $/episode
for a 3500-step episode) under fleet/state/bench/<timestamp>/.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from fleet.common import MAX_ALIVE, STATE, client, describe, price_per_hour  # noqa: E402
from fleet.pool import Job, Pool  # noqa: E402
from fleet.snapshot import ensure_shape  # noqa: E402

EPISODE_STEPS = 3500


def parse_grid(text: str) -> dict[int, list[str]]:
    grid = {}
    for part in text.split(";"):
        cpu, _, variants = part.partition(":")
        grid[int(cpu)] = [v.strip() for v in variants.split(",") if v.strip()]
    return grid


def threads_for(variant: str, cpu: int) -> int | None:
    if variant == "quota":
        return cpu
    if variant == "default":
        return None
    return int(variant)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--grid", default="4:4,default;8:8,default;16:16,default;64:64,default")
    ap.add_argument("--repeats", type=int, default=1)
    ap.add_argument("--memory", type=int, default=4)
    ap.add_argument("--disk", type=int, default=30)
    ap.add_argument("--spacing", type=float, default=None, help="rope element length; 0.004 m ~ 176 nodes")
    ap.add_argument("--timeout", type=float, default=3600)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    grid = parse_grid(args.grid)
    total = sum(len(v) for v in grid.values()) * args.repeats
    if total > MAX_ALIVE:
        raise SystemExit(f"{total} sandboxes would exceed MAX_ALIVE={MAX_ALIVE}")
    out = args.out or STATE / "bench" / time.strftime("%Y%m%d-%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    (out / "args.json").write_text(json.dumps(vars(args), default=str, indent=2))
    d = client()
    snaps = {cpu: ensure_shape(d, cpu, args.memory, args.disk) for cpu in grid}

    rows: list[dict] = []
    lock = threading.Lock()

    def bench_shape(cpu: int) -> None:
        pool = Pool(f"bench-c{cpu}", cpu=cpu, memory=args.memory, disk=args.disk, snapshot=snaps[cpu], d=d)
        jobs = []
        for v in grid[cpu]:
            t = threads_for(v, cpu)
            extra = ("" if t is None else f" --threads {t}") + ("" if args.spacing is None else f" --spacing {args.spacing}")
            for r in range(args.repeats):
                # cwd is the RSS checkout (the bench imports its tests package); outputs are relative to it.
                # "default" must reach RSS's -1 untouched: switch fleet/remote/sitecustomize off for it.
                env = {"GK_SUPERDEX_THREADS": "-1"} if t is None else {}
                jobs.append(Job(id=f"c{cpu}-t{v}-r{r}", cwd="/opt/gk/robosimstudio", timeout_s=args.timeout, env=env,
                                cmd=f"rm -rf bench-out && python /opt/gk/gap-knot/fleet/bench/knot_step.py --out bench-out{extra}",
                                outputs=["bench-out"]))
        try:
            results = pool.run(jobs, results_dir=out / f"c{cpu}", size=len(jobs), keep=False)
        except Exception as exc:
            with lock:
                rows.append({"cpu": cpu, "error": describe(exc)})
            return
        for res in results:
            row = {"cpu": cpu, "memory": args.memory, "job": res.id, "status": res.status, "run_s": res.run_s,
                   "variant": res.id.split("-")[1][1:], "error": res.error, "sandbox": (res.sandboxes or [None])[-1]}
            base = Path(res.outputs_dir or "/nonexistent")
            bench = base / "bench-out" / "bench.json"
            if bench.exists():
                b = json.loads(bench.read_text())
                row.update({k: b.get(k) for k in ("tight_ms", "mean_ms", "median_ms", "p90_ms", "steps", "build_s",
                                                   "import_s", "drive_s", "superdex_threads", "cgroup_cpus",
                                                   "os_cpu_count", "cpu_model", "rope_nodes", "checks")})
            mp = base / "memory_peak"
            if mp.exists() and mp.read_text().strip().isdigit():
                row["memory_peak_gib"] = int(mp.read_text().strip()) / 2**30
            with lock:
                rows.append(row)

    threads = [threading.Thread(target=bench_shape, args=(c,)) for c in grid]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    for row in rows:
        if row.get("mean_ms"):
            price = price_per_hour(row["cpu"], row["memory"], args.disk)
            ep_s = EPISODE_STEPS * row["mean_ms"] / 1e3 + (row.get("build_s") or 0) + (row.get("import_s") or 0)
            ep_tight_s = EPISODE_STEPS * row["tight_ms"] / 1e3
            row.update(price_per_hour=price, episode_s=ep_s, episodes_per_hour=3600 / ep_s,
                       usd_per_episode=price * ep_s / 3600, usd_per_episode_tight=price * ep_tight_s / 3600,
                       episodes_per_hour_per_usd=3600 / ep_s / price)
    rows.sort(key=lambda r: (r["cpu"], str(r.get("variant")), str(r.get("job"))))
    (out / "summary.json").write_text(json.dumps(rows, indent=2))
    lines = [f"rope nodes: {sorted({r.get('rope_nodes') for r in rows if r.get('rope_nodes')})}, memory {args.memory} GiB, "
             f"disk {args.disk} GiB; episode = {EPISODE_STEPS} steps at the mean step + build",
             "",
             "| vCPU | SuperDex threads | tight ms/step | mean ms/step | build s | mem peak GiB | $/h | "
             "episode min | $/episode | episodes/h/$ | checks |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if not r.get("mean_ms"):
            lines.append(f"| {r['cpu']} | {r.get('variant')} | FAILED: {r.get('status')} {r.get('error')} |||||||||")
            continue
        failed = [k for k, v in (r.get("checks") or {}).items() if not v]
        lines.append(f"| {r['cpu']} | {r['superdex_threads']} ({r['variant']}) | {r['tight_ms']:.1f} | {r['mean_ms']:.1f} | "
                     f"{r['build_s']:.1f} | {r.get('memory_peak_gib', float('nan')):.2f} | {r['price_per_hour']:.3f} | "
                     f"{r['episode_s'] / 60:.1f} | {r['usd_per_episode']:.4f} | {r['episodes_per_hour_per_usd']:.1f} | "
                     f"{'all pass' if not failed else 'fail: ' + ','.join(failed)} |")
    table = "\n".join(lines)
    (out / "summary.md").write_text(table + "\n")
    print(table)
    print(f"\nresults: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
