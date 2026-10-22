"""Run every graph under runs/thread_lab/graphs on the same pre_thread instances (one Daytona batch) and score the segment."""
import dataclasses, json, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
from harness import rehearse, variants
from fleet.pool import Pool
from knotlab import task, thread_lab

root = Path(sys.argv[1]).resolve(); n = int(sys.argv[2]) if len(sys.argv) > 2 else 6
vs = [dataclasses.replace(variants.sample(k, "overhand"), start_shape="pre_thread") for k in range(n)]
graphs = sorted(p for p in (root / "graphs").iterdir() if p.is_dir())
pool = Pool("thread")
try:
    rehearse.run_daytona_many([(g, root / "eval" / g.name) for g in graphs], vs, max_steps=4000, pool=pool, timeout_s=3600)
finally:
    pool.down()
W = np.array([0.6, 0.0, 0.75])
summary = {}
for g in graphs:
    rows = []
    for ep in sorted((root / "eval" / g.name).glob("ep*")):
        try:
            r = json.loads((ep / "result.json").read_text())
            v = task.Variant(**r["variant"]); lay = task.layout(v)
            pts = np.load(ep / "rope_frames.npy")[-1].astype(np.float64) - W
            sc = thread_lab.segment_score(pts, lay["target"])
            rows.append({"ep": ep.name, **sc, "t": r["sim_time_s"], "err": r["error"][-200:] if r["error"] else ""})
        except Exception as e:
            rows.append({"ep": ep.name, "under_made": False, "err": f"unreadable: {e}"})
    k = sum(r.get("under_made") for r in rows)
    summary[g.name] = {"under_made": f"{k}/{len(rows)}", "rows": rows}
    print(f"{g.name:22s} under-crossing made {k}/{len(rows)}  ", [(r.get('c1'), r.get('c2'), r.get('code')) for r in rows])
(root / "summary.json").write_text(json.dumps(summary, indent=1))
