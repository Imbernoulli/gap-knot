"""Paired tightening evaluation with the corrected world-to-bench goal.

Run with a Python that has the Daytona SDK. Uses six CPU sandboxes by default,
deletes them on completion, and exports the runtime verdicts and final nodes.
"""
import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from audit_results import export
from fleet.pool import Pool
from harness import rehearse, variants
from harness.analyze import analyze


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n", type=int, default=10)
    ap.add_argument("--offset", type=int, default=10)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--export", type=Path)
    args = ap.parse_args()
    if args.n < 1 or args.workers < 1:
        ap.error("n and workers must be positive")
    specs = [(ROOT / "graphs" / graph, args.out.resolve() / "eval" / label) for graph, label in
             (("tighten_v0", "G0_baseline"), ("tighten_g1", "G1_iter0c"), ("tighten_g2", "G2_iter1a"))]
    instances = [variants.sample(k, "overhand_tighten") for k in range(args.offset, args.offset + args.n)]
    pool = Pool("tighten-reeval")
    try:
        rehearse.run_daytona_many(specs, instances, max_steps=9000, pool=pool, workers=args.workers)
    finally:
        pool.down()
    for _, out in specs:
        _, _, report = analyze(out)
        (out / "REPORT.md").write_text(report)
    export(args.out.resolve() / "eval", args.export or args.out.resolve() / "audit.json")


if __name__ == "__main__":
    main()
