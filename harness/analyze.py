"""Rehearsal results -> per-node failure localization, aggregate stats, and the report the refiners read.

    python harness/analyze.py RESULTS_DIR [--out REPORT.md]

RESULTS_DIR holds one directory per episode, each written by knotlab.episode
(result.json, timeline.jsonl, trace/0000/{dag_trace.json,node_data/...}).
"""
from __future__ import annotations

import argparse
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

from knotlab import diagram, geometry

TOL_S = 0.035  # how far [m along the rope] a crossing may sit from the target's and still be that crossing


def target_crossings(radius: float = 0.005) -> list[dict]:
    g = geometry.target(geometry.KnotSpec(radius=radius))
    cs = sorted(diagram.crossings(g), key=lambda c: c.s_i)
    return [{"id": k + 1, "s_a": c.s_i, "s_b": c.s_j, "a_over": c.i_over} for k, c in enumerate(cs)]


def wilson(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    r = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return ((c - r) / d, (c + r) / d)


def _match(x: list, tgt: dict) -> bool:
    return abs(x[0] - tgt["s_a"]) < TOL_S and abs(x[1] - tgt["s_b"]) < TOL_S


def load_episode(d: Path) -> dict | None:
    rp = d / "result.json"
    if not rp.exists():
        return {"id": d.name, "missing": True}
    r = json.loads(rp.read_text())
    rows = [json.loads(l) for l in (d / "timeline.jsonl").read_text().splitlines() if l.strip()] if (d / "timeline.jsonl").exists() else []
    events = []
    dag = d / "trace" / "0000" / "dag_trace.json"
    if dag.exists():
        try:
            events = json.loads(dag.read_text()).get("events", [])
        except ValueError:
            events = []
    return {"id": d.name, "dir": str(d), "result": r, "rows": rows, "events": events}


def _wall_to_step(rows: list[dict]):
    if len(rows) < 2:
        return lambda w: None
    w = np.array([r["wall"] for r in rows if "wall" in r])
    s = np.array([r["step"] for r in rows if "wall" in r])
    if len(w) < 2:
        return lambda x: None
    return lambda x: int(np.interp(x, w, s))


def _row_at(rows: list[dict], step: int | None) -> dict | None:
    if step is None or not rows:
        return None
    best = None
    for r in rows:
        if r["step"] <= step:
            best = r
        else:
            break
    return best or rows[0]


def node_spans(ep: dict) -> list[dict]:
    """Each node run: when (sim steps), how it ended, the rope's code progress before and after."""
    to_step = _wall_to_step(ep["rows"])
    open_: dict[str, dict] = {}
    spans = []
    for e in ep["events"]:
        t = e.get("event_type")
        name = e.get("node_name", "")
        if t == "node_started":
            open_[name] = {"node": name, "start": to_step(e["timestamp"]), "t0": e["timestamp"]}
        elif t == "node_finished" and name in open_:
            sp = open_.pop(name)
            sp["end"] = to_step(e["timestamp"])
            sp["status"] = e.get("status")
            spans.append(sp)
        elif t == "error_recorded":
            for sp in reversed(spans):
                if sp["node"] == name:
                    sp["error"] = (e.get("detail") or {}).get("error", "")[:400]
                    break
            else:
                if name in open_:
                    open_[name]["error"] = (e.get("detail") or {}).get("error", "")[:400]
    for name, sp in open_.items():  # never finished: the episode ended inside it
        sp["end"] = ep["rows"][-1]["step"] if ep["rows"] else None
        sp["status"] = "unfinished"
        spans.append(sp)
    for sp in spans:
        a, b = _row_at(ep["rows"], sp.get("start")), _row_at(ep["rows"], sp.get("end"))
        sp["prefix_before"] = a["prefix"] if a else None
        sp["prefix_after"] = b["prefix"] if b else None
        sp["sim_s"] = round(((sp.get("end") or 0) - (sp.get("start") or 0)) / 60.0, 2)
    return spans


def bad_calls(ep: dict, node: str, limit: int = 6) -> list[str]:
    """Tool calls inside a node that came back other than clean: timeouts, unreached poses, refusals, held=False."""
    out = []
    base = Path(ep["dir"]) / "trace" / "0000" / "node_data" / node / "calls"
    for call in sorted(base.glob("*")) if base.exists() else []:
        try:
            resp = json.loads((call / "response.json").read_text())
        except Exception:
            out.append(f"{call.name}: no response (did not return)")
            continue
        if isinstance(resp, dict):
            flags = []
            if resp.get("outcome") not in (None, "reached"):
                flags.append(f"outcome={resp.get('outcome')} remaining={resp.get('remaining_distance')}")
            if resp.get("held") is False:
                flags.append("held=False")
            if resp.get("error"):
                flags.append(f"error={str(resp.get('error'))[:120]}")
            if resp.get("clamped"):
                flags.append(f"clamped reason={resp.get('reason')}")
            if flags:
                out.append(f"{call.name}: " + "; ".join(flags))
        if len(out) >= limit:
            break
    return out


def diagnose(ep: dict, tcross: list[dict]) -> dict:
    r = ep["result"]
    rows = ep["rows"]
    final_x = (r.get("final") or {}).get("x") or (rows[-1].get("x") if rows else []) or []
    spans = node_spans(ep)

    def node_at(step: int) -> str:
        cands = [sp for sp in spans if sp.get("start") is not None and sp["start"] <= step <= (sp.get("end") or step) and "." in sp["node"]]
        if not cands:
            cands = [sp for sp in spans if sp.get("start") is not None and sp["start"] <= step <= (sp.get("end") or step)]
        return min(cands, key=lambda sp: (sp.get("end") or 0) - sp["start"])["node"] if cands else "?"

    def born(tc: dict) -> tuple[int | None, str]:
        for row in rows:
            for x in row.get("x", []):
                if _match(x, tc):
                    return row["step"], node_at(row["step"])
        return None, "?"

    per = []
    used = set()
    for tc in tcross:
        hit = next((k for k, x in enumerate(final_x) if k not in used and _match(x, tc)), None)
        if hit is None:
            per.append({"id": tc["id"], "state": "MISSING"})
            continue
        used.add(hit)
        ok = bool(final_x[hit][2]) == tc["a_over"]
        step, node = born(tc)
        per.append({"id": tc["id"], "state": "OK" if ok else "FLIPPED", "born_step": step, "born_node": node})
    extra = [x for k, x in enumerate(final_x) if k not in used]
    err_span = next((sp for sp in spans if sp.get("status") in ("error", "unfinished") and "." in sp["node"]), None)
    regress = [sp for sp in spans if sp.get("prefix_before") is not None and sp.get("prefix_after") is not None and sp["prefix_after"] < sp["prefix_before"] and "." in sp["node"]]
    if r.get("success"):
        cause = "success"
    elif r.get("error"):
        cause = "crash: " + r["error"].strip().splitlines()[-1][:160]
    elif err_span:
        cause = f"node {err_span['node']} {err_span['status']}" + (f": {err_span.get('error', '')[:160]}" if err_span.get("error") else "")
    elif any(p["state"] != "OK" for p in per) or extra:
        cause = "wrong diagram"
    else:
        cause = "ran out of steps"
    return {
        "id": ep["id"],
        "success": bool(r.get("success")),
        "sim_time_s": r.get("sim_time_s"),
        "wall_s": r.get("wall_s"),
        "final_code": (r.get("final") or {}).get("code"),
        "prefix": (r.get("final") or {}).get("prefix"),
        "crossings": per,
        "extra_crossings": len(extra),
        "cause": cause,
        "error_node": err_span["node"] if err_span else None,
        "error_calls": bad_calls(ep, err_span["node"]) if err_span else [],
        "regressions": [{"node": sp["node"], "from": sp["prefix_before"], "to": sp["prefix_after"]} for sp in regress],
        "spans": [{k: sp.get(k) for k in ("node", "start", "end", "sim_s", "status", "prefix_before", "prefix_after")} for sp in spans],
        "variant": r.get("variant"),
    }


def aggregate(diags: list[dict]) -> dict:
    n = len(diags)
    k = sum(d["success"] for d in diags)
    lo, hi = wilson(k, n)
    times_ok = [d["sim_time_s"] for d in diags if d["success"] and d["sim_time_s"] is not None]
    causes = Counter(d["cause"].split(":")[0] if d["cause"].startswith("crash") else d["cause"] for d in diags if not d["success"])
    cross = defaultdict(Counter)
    born = defaultdict(Counter)
    for d in diags:
        for c in d["crossings"]:
            cross[c["id"]][c["state"]] += 1
            if c["state"] == "FLIPPED":
                born[c["id"]][c.get("born_node", "?")] += 1
    node_time = defaultdict(list)
    node_err = Counter()
    for d in diags:
        for sp in d["spans"]:
            if "." in sp["node"]:
                node_time[sp["node"]].append(sp["sim_s"] or 0.0)
                if sp["status"] in ("error", "unfinished"):
                    node_err[sp["node"]] += 1
    return {
        "episodes": n,
        "successes": k,
        "success_rate": k / n if n else 0.0,
        "wilson95": [round(lo, 3), round(hi, 3)],
        "mean_prefix": float(np.mean([d["prefix"] or 0 for d in diags])) if n else 0.0,
        "mean_cross_ok": float(np.mean([sum(c["state"] == "OK" for c in d["crossings"]) for d in diags])) if n else 0.0,
        "mean_time_success_s": float(np.mean(times_ok)) if times_ok else None,
        "mean_time_all_s": float(np.mean([d["sim_time_s"] or 0 for d in diags])) if n else None,
        "causes": dict(causes),
        "crossings": {cid: dict(c) for cid, c in sorted(cross.items())},
        "flipped_born_in": {cid: dict(c) for cid, c in sorted(born.items())},
        "node_mean_sim_s": {k2: round(float(np.mean(v)), 2) for k2, v in sorted(node_time.items())},
        "node_errors": dict(node_err),
    }


def report(stats: dict, diags: list[dict], *, title: str = "Rehearsal report") -> str:
    L = [f"# {title}", ""]
    L.append(f"**Success {stats['successes']}/{stats['episodes']} = {stats['success_rate']:.0%}** (Wilson 95% {stats['wilson95'][0]:.2f}–{stats['wilson95'][1]:.2f}); "
             f"mean code prefix {stats['mean_prefix']:.1f}/16; mean target crossings right {stats['mean_cross_ok']:.1f}/8; mean sim time of successes {stats['mean_time_success_s'] or float('nan'):.1f} s, of all {stats['mean_time_all_s'] or float('nan'):.1f} s.")
    L.append("")
    L.append("## Why episodes failed")
    for c, m in sorted(stats["causes"].items(), key=lambda kv: -kv[1]):
        L.append(f"- {m}× {c}")
    L.append("")
    L.append("## The eight target crossings (walking from the first tail; target Gauss code `" + "1O 2U 3O 4U 5O 6U 2O 7U 4O 8U 6O 1U 7O 3U 8O 5U" + "`)")
    L.append("| crossing | OK | FLIPPED (over/under wrong) | MISSING | flipped crossings were made during |")
    L.append("|---|---|---|---|---|")
    for cid, c in stats["crossings"].items():
        born = ", ".join(f"{n} ({m})" for n, m in sorted(stats["flipped_born_in"].get(cid, {}).items(), key=lambda kv: -kv[1]))
        L.append(f"| {cid} | {c.get('OK', 0)} | {c.get('FLIPPED', 0)} | {c.get('MISSING', 0)} | {born} |")
    L.append("")
    L.append("## Nodes: mean sim seconds, and how often the episode died inside them")
    for node, t in stats["node_mean_sim_s"].items():
        L.append(f"- `{node}`: {t:.1f} s" + (f", **died here {stats['node_errors'][node]}×**" if stats["node_errors"].get(node) else ""))
    L.append("")
    L.append("## Episodes")
    for d in diags:
        v = d.get("variant") or {}
        L.append(f"### {d['id']}: {'SUCCESS' if d['success'] else 'FAIL'} — {d['cause']}")
        L.append(f"variant: young {v.get('young_modulus', 0):.3g} Pa, mu {v.get('mu_cable', 0):.2f}, board ({v.get('board_x', 0):+.3f}, {v.get('board_y', 0):+.3f}) yaw {v.get('board_yaw', 0):+.3f}, splay {v.get('splay', 0):.3f}, drop {v.get('drop', 0):.3f}, lay_noise {v.get('lay_noise', 0):.3f}")
        L.append(f"final code `{d['final_code']}` (prefix {d['prefix']}/16, extra crossings {d['extra_crossings']}), sim {d['sim_time_s']} s")
        bad = [c for c in d["crossings"] if c["state"] != "OK"]
        if bad:
            L.append("crossings wrong: " + "; ".join(f"#{c['id']} {c['state']}" + (f" (made in {c.get('born_node')} at step {c.get('born_step')})" if c["state"] == "FLIPPED" else "") for c in bad))
        if d["error_calls"]:
            L.append(f"bad calls in {d['error_node']}: " + " | ".join(d["error_calls"]))
        if d["regressions"]:
            L.append("rope got worse during: " + ", ".join(f"{r['node']} ({r['from']}→{r['to']})" for r in d["regressions"][:6]))
        L.append("nodes: " + " → ".join(f"{s['node'].split('.')[-1]}[{s['sim_s']}s,{s['prefix_before']}→{s['prefix_after']}{'' if s['status'] == 'ok' else ',' + str(s['status'])}]" for s in d["spans"] if "." in s["node"])[:1800])
        L.append("")
    return "\n".join(L)


def analyze(results_dir: str | Path, *, title: str = "Rehearsal report") -> tuple[dict, list[dict], str]:
    tc = target_crossings()
    eps = [load_episode(d) for d in sorted(Path(results_dir).iterdir()) if d.is_dir()]
    diags = []
    for ep in eps:
        if ep is None:
            continue
        if ep.get("missing"):
            diags.append({"id": ep["id"], "success": False, "sim_time_s": None, "wall_s": None, "final_code": None, "prefix": 0,
                          "crossings": [], "extra_crossings": 0, "cause": "no result (sandbox or runner failure)", "error_node": None,
                          "error_calls": [], "regressions": [], "spans": [], "variant": {}})
            continue
        diags.append(diagnose(ep, tc))
    stats = aggregate(diags)
    return stats, diags, report(stats, diags, title=title)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("results")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    stats, diags, md = analyze(a.results)
    if a.out:
        Path(a.out).write_text(md)
        Path(a.out).with_suffix(".json").write_text(json.dumps({"stats": stats, "episodes": diags}, indent=1))
    print(md[:3000])
