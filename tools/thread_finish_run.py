"""Small paired diagnostic of release order and the missing third crossing.

Starts from a constructed loop. These runs never establish from-straight tying.
"""
import argparse
import dataclasses
import json
import hashlib
import uuid
import shutil
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from fleet.pool import Pool
from harness import rehearse, variants
from knotlab import invariants, task, thread_lab


def graph_digest(graph):
    """Hash the exact frozen graph, excluding generated interpreter files."""
    digest = hashlib.sha256()
    for path in sorted(graph.rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts and path.suffix != ".pyc":
            digest.update(path.relative_to(graph).as_posix().encode() + b"\0")
            digest.update(path.read_bytes())
    return digest.hexdigest()


def audit(specs, instances):
    rows, missing = [], []
    for graph, out in specs:
        for expected in instances:
            ep = out / f"ep{expected.seed:06d}"
            row = {"graph": graph.name, "variant": dataclasses.asdict(expected),
                   "runtime_success": False, "audit_errors": []}
            if not (ep / "result.json").exists():
                missing.append({"graph": graph.name, "seed": expected.seed, "episode": str(ep)})
                continue
            try:
                result = json.loads((ep / "result.json").read_text())
                row.update({k: result.get(k) for k in ("sim_time_s", "error", "grader_version")})
                row["runtime_success"] = bool(result["success"])
                if result["variant"] != dataclasses.asdict(expected):
                    row["audit_errors"].append("variant differs from planned paired instance")
                if result.get("grader_version") != task.GRADER_VERSION:
                    row["audit_errors"].append("grader version differs from current evaluator")
                nodes = np.load(ep / "rope_final.npy", allow_pickle=False)
                if nodes.ndim != 2 or nodes.shape[1] != 3 or len(nodes) < 4 or not np.isfinite(nodes).all():
                    raise ValueError("final geometry must be finite Nx3, N>=4")
                nodes = nodes - task.BENCH_ORIGIN
                row["final"] = thread_lab.segment_score(nodes, task.layout(expected)["target"])
                row["final"]["max_height"] = float(nodes[:, 2].max())
                row["determinant_with_diagram_closure"] = invariants.determinant(nodes)
                row["geometry_source"] = "rope_final.npy (exact final, world-to-bench)"
                output = ep / "trace/0000/node_data/seg_sg.act/output.json"
                phases = json.loads(output.read_text()).get("report", {}) if output.exists() else {}
                row["phases"] = phases
                # Only state observations have an under_made field. Movement return values do not.
                states = [(name, value) for name, value in phases.items()
                          if isinstance(value, dict) and "under_made" in value]
                first_under = next((name for name, value in states if value["under_made"]), None)
                losses = [{"after": a, "at": b} for (a, av), (b, bv) in zip(states, states[1:])
                          if av["under_made"] and not bv["under_made"]]
                row["phase_invariants"] = {"observations": len(states), "first_under_made": first_under,
                    "under_made_any": first_under is not None,
                    "under_retained_at_final": bool(row["final"]["under_made"]),
                    "under_loss_transitions": losses,
                    "anchor_held": phases.get("anchor", {}).get("held"),
                    "tip_held_before_lift": phases.get("grip_before_lift", {}).get("held")}
                if not states:
                    row["audit_errors"].append("phase report missing (graph may have failed before returning)")
            except Exception as exc:
                row["audit_errors"].append(f"{type(exc).__name__}: {exc}")
            rows.append(row)
            print(graph.name, expected.seed, row["runtime_success"], row.get("final"), flush=True)
    totals = {}
    for graph, _ in specs:
        subset = [r for r in rows if r["graph"] == graph.name]
        totals[graph.name] = {"planned": len(instances), "completed": len(subset),
            "runtime_successes": sum(r["runtime_success"] for r in subset),
            "audited_runtime_successes": sum(r["runtime_success"] and not r["audit_errors"] for r in subset),
            "audit_errors": sum(bool(r["audit_errors"]) for r in subset),
            "under_made_any": sum(r.get("phase_invariants", {}).get("under_made_any", False) for r in subset),
            "under_retained_at_final": sum(r.get("final", {}).get("under_made", False) for r in subset),
            "final_determinant_3": sum(r.get("determinant_with_diagram_closure") == 3 for r in subset)}
    return rows, {"totals": totals, "missing_episodes": missing,
                  "claim_scope": "Constructed pre_thread loop only; no from-straight tying claim. "
                                 "Final geometry and sampled phase observations do not replace the runtime verdict."}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--n", type=int, default=1)
    ap.add_argument("--offset", type=int, default=0)
    ap.add_argument("--sampled", action="store_true", help="sample variants instead of nominal")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--max-steps", type=int, default=6500)
    ap.add_argument("--split", choices=("diagnostic", "train", "heldout"), default="diagnostic",
                    help="heldout requires sampled seeds >=100000 and frozen graphs")
    ap.add_argument("--audit-only", action="store_true", help="read existing episodes without contacting Daytona")
    ap.add_argument("--prepared", action="store_true", help="use the existing out/graphs subdirectories")
    args = ap.parse_args()
    if args.n < 1 or not 1 <= args.workers <= 6 or args.max_steps < 1 or args.offset < 0:
        ap.error("n/max-steps must be positive, offset nonnegative, workers in 1..6")
    if args.split == "heldout" and (not args.sampled or args.offset < 100000):
        ap.error("heldout requires --sampled --offset >=100000; freeze candidates before evaluation")
    args.out = args.out.absolute()
    instances = [dataclasses.replace(variants.sample(k, "overhand"), start_shape="pre_thread")
                 if args.sampled else task.Variant(seed=k, knot="overhand", board_x=-0.07, start_shape="pre_thread")
                 for k in range(args.offset, args.offset + args.n)]
    specs = []
    graph_root = args.out / "graphs"
    if args.prepared:
        if not graph_root.is_dir():
            ap.error("--prepared requires out/graphs with graph subdirectories")
        modes = [g.name for g in sorted(graph_root.iterdir()) if g.is_dir()]
        if not modes:
            ap.error("out/graphs contains no graph directories")
    else:
        modes = ("original_release", "tip_first", "helper_first", "closed_exit", "regrasp_exit")
    for mode in modes:
        dst = graph_root / mode
        src = (ROOT / "results/threading_2026-10-22/grip_before_lift_graph"
               if mode == "original_release" else ROOT / "graphs/thread_finish")
        if not dst.exists():
            shutil.copytree(src, dst, ignore=shutil.ignore_patterns("__pycache__"))
            p = json.loads((dst / "params.json").read_text())
            if mode != "original_release":
                p["finish"] = mode
            (dst / "params.json").write_text(json.dumps(p, indent=2) + "\n")
        if not (dst / "workflow.json").is_file() or not (dst / "params.json").is_file():
            ap.error(f"invalid prepared graph: {dst}")
        try:
            json.loads((dst / "params.json").read_text())
            json.loads((dst / "workflow.json").read_text())
        except (OSError, ValueError) as exc:
            ap.error(f"invalid graph JSON in {dst}: {exc}")
        specs.append((dst, args.out / "eval" / mode))
    plan = {"split": args.split, "sampled": args.sampled, "max_steps": args.max_steps,
            "grader_version": task.GRADER_VERSION, "workers": args.workers,
            "graphs": {g.name: graph_digest(g) for g, _ in specs},
            "instances": [dataclasses.asdict(v) for v in instances],
            "start_shape": "pre_thread", "claim_scope": "Constructed loop segment; not from straight rope"}
    manifest = args.out / "manifest.json"
    if manifest.exists():
        previous = json.loads(manifest.read_text())
        # Worker count does not affect paired inputs. Everything else must match on resume.
        if {k: v for k, v in previous.items() if k != "workers"} != {k: v for k, v in plan.items() if k != "workers"}:
            ap.error("existing manifest differs: use a new --out directory to avoid mixing experiments")
    elif not args.audit_only and any(list(out.glob("ep*/result.json")) for _, out in specs):
        ap.error("existing episodes have no manifest; audit them with --audit-only or use a new --out")
    manifest.write_text(json.dumps(plan, indent=2) + "\n")
    if not args.audit_only:
        # A unique pool prevents another simultaneous sweep from deleting this one's workers.
        pool = Pool("thread-" + uuid.uuid4().hex[:12])
        (args.out / "pool.json").write_text(json.dumps({"name": pool.name, "workers": args.workers}) + "\n")
        jobs, run_error = [], None
        try:
            jobs = rehearse.run_daytona_many(specs, instances, max_steps=args.max_steps,
                                             workers=args.workers, pool=pool)
        except Exception as exc:
            run_error = exc
            from fleet.common import describe
            (args.out / "run_error.json").write_text(json.dumps({"error": describe(exc)}) + "\n")
        finally:
            remaining = pool.down()
            (args.out / "cleanup.json").write_text(json.dumps({"pool": pool.name, "remaining": remaining}) + "\n")
        (args.out / "jobs.json").write_text(json.dumps(jobs, indent=2) + "\n")
    rows, report = audit(specs, instances)
    (args.out / "summary.json").write_text(json.dumps(rows, indent=2) + "\n")
    (args.out / "audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    if not args.audit_only and run_error is not None:
        raise run_error


if __name__ == "__main__":
    main()
