"""Control-step time of RSS's two-hand knot pull, run inside a sandbox (or locally).

Drives exactly ``tests/test_knot_pull.py``'s ``_drive`` (the scene of
``examples/03_materials/03_knot.py``, both hands closing on the tails and pulling
the knot tight through the arm panels) and records:

* ``tight_ms``  -- the test's own number: mean of the last 60 steps, knot pulled tight
* ``mean_ms``   -- mean over every control step of the drive
* ``build_s``   -- scene build + page open (everything before the first step)
* the test's checks (crossings, knot length, clearance, jaw widths, slip)

``--spacing S`` resamples the same knot at element length S (0.004 m: ~176 nodes,
about the 1.5 m / 180-node rope the knot-tying episodes use). ``--threads N`` starts SuperDex with N worker threads before the scene does (RSS
keeps the first caller's count); the default -1 lets SuperDex size its pool from the
hardware it sees, which inside a Daytona sandbox is the host's cores, not the quota.

Run from the RSS checkout (``tests`` must be importable)::

    cd /opt/gk/robosimstudio && python /opt/gk/gap-knot/fleet/bench/knot_step.py --threads 16 --out out
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import time
from pathlib import Path


def _cgroup_cpus():
    try:
        q, p = Path("/sys/fs/cgroup/cpu.max").read_text().split()
        return None if q == "max" else int(q) / int(p)
    except Exception:
        return None


def _cpu_model():
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except Exception:
        pass
    return platform.processor()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--threads", type=int, default=None, help="SuperDex worker threads (default: let RSS pick, -1)")
    ap.add_argument("--spacing", type=float, default=None,
                    help="rope element length [m]; the example's 0.008 gives 88 nodes, 0.004 gives ~176 (the target rope)")
    ap.add_argument("--port", type=int, default=9785)
    ap.add_argument("--out", type=Path, default=Path("out"))
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    t_start = time.perf_counter()
    # RSS's ``tests`` package must win over any other ``tests`` on PYTHONPATH (gap-knot has one).
    import sys

    sys.path.insert(0, os.environ.get("GK_RSS") or os.getcwd())
    for name in [m for m in sys.modules if m == "tests" or m.startswith("tests.")]:
        del sys.modules[name]
    import numpy as np

    import tests.fixtures.grasp as grasp
    from tests import test_knot_pull as T

    if args.threads is not None:
        from robosimstudio.engines.superdex.bridge import ensure_initialized

        ensure_initialized(args.threads)
    if args.spacing is not None:
        # The example builds its rope with knot.overhand(spacing=0.008); the same knot, finer.
        import robosimstudio_materials.rod.knot as knot

        real_overhand = knot.overhand
        knot.overhand = lambda **kw: real_overhand(**{**kw, "spacing": args.spacing})

    steps: list[float] = []
    first_step_at: list[float] = []
    real_open = grasp.open_session

    def open_session(sim, port, page=None):
        session = real_open(sim, port, page)
        real_tick = session.tick

        def timed_tick():
            t = time.perf_counter()
            stepped = real_tick()
            if stepped:
                if not first_step_at:
                    first_step_at.append(t)
                steps.append(time.perf_counter() - t)
            return stepped

        session.tick = timed_tick
        return session

    grasp.open_session = open_session
    t_drive = time.perf_counter()
    run = T._drive(args.port)
    t_end = time.perf_counter()

    import superdex.physics as sdx

    sim = run.pop("sim")
    arr = np.asarray(steps) * 1e3
    widths_ok = all(0.004 < w < 0.012 for w in run["widths"].values())
    checks = {
        "starts_a_knot": run["before"]["crossings"] == 3,
        "superdex_self_contact": sim.request.physics.backend_for("rod").solver == "superdex"
        and bool(sim["rope"].entity.spec.self_contact),
        "pads_close_and_keep": widths_ok and run["slid"] < 0.01,
        "stays_a_knot": run["finite"] and run["crossings"] == 3 and run["clearance"] > -T.TOLERANCE,
        "draws_up": run["knot_length"] < 0.95 * run["before"]["knot_length"],
        "under_90ms": run["ms"] < 90.0,
    }
    result = {
        "tight_ms": run["ms"],
        "mean_ms": float(arr.mean()) if arr.size else None,
        "median_ms": float(np.median(arr)) if arr.size else None,
        "p90_ms": float(np.percentile(arr, 90)) if arr.size else None,
        "steps": int(arr.size),
        "build_s": (first_step_at[0] if first_step_at else t_end) - t_drive,
        "import_s": t_drive - t_start,
        "drive_s": t_end - t_drive,
        "superdex_threads": int(sdx.get_num_threads()),
        "threads_arg": args.threads,
        "spacing": args.spacing,
        "cgroup_cpus": _cgroup_cpus(),
        "os_cpu_count": os.cpu_count(),
        "cpu_model": _cpu_model(),
        "rope_nodes": int(np.asarray(sim["rope"].particle_q).shape[0]),
        "checks": checks,
        "test_values": {k: v for k, v in run.items() if k != "ms"},
        "env_threads": {k: os.environ.get(k) for k in ("OMP_NUM_THREADS", "GK_CPUS")},
    }
    (args.out / "bench.json").write_text(json.dumps(result, indent=2, default=float))
    print(json.dumps(result, default=float), flush=True)
    os._exit(0)  # the page's server threads would otherwise hold the process open


if __name__ == "__main__":
    main()
