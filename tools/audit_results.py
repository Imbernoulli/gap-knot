"""Export per-episode outcomes and final rope geometry for a completed sweep.

This preserves the runtime verdict. A final frame satisfying a predicate is not
promoted to a success: only the runtime checks its continuous one-second hold.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from knotlab import invariants, task


def export(root: Path, output: Path):
    rows, geometry = [], {}
    previous = task.CURRENT.copy()
    try:
        for ep in sorted(root.glob("*/ep*")):
            if not (ep / "result.json").exists():
                continue
            result = json.loads((ep / "result.json").read_text())
            v = task.Variant(**result["variant"])
            task.CURRENT.clear()
            task.CURRENT.update(variant=v, L0=task.diagram_knot_length(task.layout(v)["start"]))
            exact = (ep / "rope_final.npy").exists()
            nodes = np.load(ep / "rope_final.npy") if exact else np.load(ep / "rope_frames.npy")[-1]
            local = nodes.astype(np.float64) - task.BENCH_ORIGIN
            key = f"{ep.parent.name}_{ep.name}"
            geometry[key] = nodes
            score = task.score(local)
            hand_frames = np.load(ep / "hand_frames.npy")
            hands = hand_frames[-1]
            widths = [float(hands[4]), float(hands[8])]
            distances = [float(np.linalg.norm(nodes - hands[start:start + 3], axis=1).min()) for start in (1, 5)]
            rows.append({
                "graph": ep.parent.name, "seed": v.seed, "variant": result["variant"],
                "grader_version": result.get("grader_version", "legacy"),
                "runtime_success": result["success"], "sim_time_s": result["sim_time_s"],
                "error": result["error"], "exact_final_geometry": exact,
                "final": score, "initial_knot_length": task.CURRENT["L0"],
                "length_fraction": score["knot_length"] / task.CURRENT["L0"] if task.CURRENT["L0"] else None,
                "determinant_with_diagram_closure": invariants.determinant(local),
                "last_sampled_hand_widths": widths, "last_sampled_tcp_distances_to_rope": distances,
                "last_hand_sample_step": int(hands[0]),
                "last_sampled_hands_open_and_clear": task.hands_clear(nodes, [hands[1:4], hands[5:8]], widths),
            })
    finally:
        task.CURRENT.clear()
        task.CURRENT.update(previous)
    summary = []
    for graph in sorted({r["graph"] for r in rows}):
        group = [r for r in rows if r["graph"] == graph]
        success = [r for r in group if r["runtime_success"]]
        summary.append({"graph": graph, "successes": len(success), "episodes": len(group),
                        "mean_time_success_s": round(float(np.mean([r["sim_time_s"] for r in success])), 2) if success else None,
                        "successful_final_hands_open_and_clear": sum(r["last_sampled_hands_open_and_clear"] for r in success)})
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output.with_suffix(".npz"), **geometry)
    strict = bool(rows) and all(r["grader_version"] == task.GRADER_VERSION for r in rows)
    document = {"criterion": ("trefoil diagram, knot length <=75% of initial, max node height <=0.05m above bench, both jaw widths >=0.05m and both TCPs >=0.06m from every rope node; runtime holds for 1s"
                              if strict else "Legacy or mixed graders; see each episode's grader_version. Runtime outcomes have not been upgraded by rescoring final frames."),
                "geometry_sha256": hashlib.sha256(output.with_suffix(".npz").read_bytes()).hexdigest(),
                "note": "Hand diagnostics are the last recorded sample, not a continuous contact test. Determinants use diagram closure.",
                "summary": summary, "episodes": rows}
    output.write_text(json.dumps(document, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("root", type=Path)
    ap.add_argument("--out", required=True, type=Path)
    args = ap.parse_args()
    export(args.root, args.out)
