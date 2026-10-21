"""Rehearse every graph under SWEEP/graphs/ on the same instances, in one Daytona batch, and tabulate.

    python tools/sweep.py SWEEP_DIR --n 5 --knot overhand --max-steps 12000
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from harness import rehearse, variants  # noqa: E402
from harness.analyze import analyze  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("sweep")
ap.add_argument("--n", type=int, default=5)
ap.add_argument("--offset", type=int, default=0)
ap.add_argument("--knot", default="overhand")
ap.add_argument("--max-steps", type=int, default=12000)
a = ap.parse_args()
variants.KNOT = a.knot
root = Path(a.sweep).resolve()
graphs = sorted(p for p in (root / "graphs").iterdir() if p.is_dir())
vs = variants.train(a.n, a.offset)
rehearse.run_daytona_many([(g, root / "eval" / g.name) for g in graphs], vs, max_steps=a.max_steps)
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
