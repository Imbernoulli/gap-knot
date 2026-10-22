"""tools/sweep.py on a Daytona pool of its own name, so it neither borrows nor tears down another loop's sandboxes.

    python tools/sweep_pool.py SWEEP_DIR --n 5 --knot overhand --max-steps 12000 [--pool demo] [--nominal]

tools/sweep.py runs on Pool("loop"), the pool harness/selflearn.py runs on: started while a self-learning
loop is running, it would take that loop's sandboxes as its own and delete them when it finished
(Pool.run(keep=False) -> Pool.down() deletes every sandbox labelled with the pool's name). Same table.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from fleet.pool import Pool  # noqa: E402
from harness import rehearse, variants  # noqa: E402
from harness.analyze import analyze  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("sweep")
ap.add_argument("--n", type=int, default=5)
ap.add_argument("--offset", type=int, default=0)
ap.add_argument("--knot", default="overhand")
ap.add_argument("--max-steps", type=int, default=12000)
ap.add_argument("--pool", default="demo")
ap.add_argument("--nominal", action="store_true", help="only the nominal instance (probes)")
ap.add_argument("--no-analyze", action="store_true")
a = ap.parse_args()
if a.pool == "loop":
    sys.exit("pool 'loop' is the self-learning loop's; pick another name")
variants.KNOT = a.knot
root = Path(a.sweep).resolve()
graphs = sorted(p for p in (root / "graphs").iterdir() if p.is_dir())
vs = [variants.nominal()] if a.nominal else variants.train(a.n, a.offset)
pool = Pool(a.pool)
try:
    rehearse.run_daytona_many([(g, root / "eval" / g.name) for g in graphs], vs, max_steps=a.max_steps, pool=pool)
finally:
    pool.down()
if a.no_analyze:
    sys.exit(0)
rows = []
for g in graphs:
    st, diags, md = analyze(root / "eval" / g.name)
    (root / "eval" / g.name / "REPORT.md").write_text(md)
    causes = sorted(st["causes"].items(), key=lambda kv: -kv[1])[:2]
    rows.append((g.name, st["successes"], st["episodes"], st["mean_cross_ok"], st["mean_prefix"], st["mean_shape_near"], st["mean_time_success_s"], st["mean_time_all_s"], causes))
print(f"{'config':20s} succ  crossOK prefix shape  t_succ  t_all  top causes")
for r in rows:
    print(f"{r[0]:20s} {r[1]}/{r[2]}  {r[3]:.1f}    {r[4]:.1f}   {r[5]:.2f}  {r[6] if r[6] is None else round(r[6],1)}  {r[7]:.1f}  {r[8]}")
(root / "summary.json").write_text(json.dumps(rows, indent=1, default=str))
