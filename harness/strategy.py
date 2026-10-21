"""How the self-learning loop aims its refiners: reloaded at the start of every iteration, so it can be tuned while a run goes on."""
from __future__ import annotations


def focuses(stats: dict, report: dict, k: int) -> list[str]:
    """Different strategies for the k refiners, aimed at where the graph is actually failing."""
    import re as _re

    n = stats["episodes"]
    # Group failure causes by the subgraph and route that ended the graph (the reason text varies).
    groups: dict[str, int] = {}
    reasons: dict[str, list[str]] = {}
    for c, m in stats["causes"].items():
        mm = _re.match(r"graph ended at '[^']+' after (\S+) routed '([^']+)'(?: \((.*)\))?", c)
        mc = _re.match(r"node (\S+?)\.(\S+) (error|unfinished)(?:: (.*))?", c)
        if mm:
            key, why = f"{mm.group(1)} routing '{mm.group(2)}'", mm.group(3)
        elif mc:
            key, why = f"{mc.group(1)} crashing ({mc.group(3)} in node {mc.group(1)}.{mc.group(2)})", (mc.group(4) or "")[-200:]
        else:
            key, why = c, None
        groups[key] = groups.get(key, 0) + m
        if why:
            reasons.setdefault(key, []).append(f"{why} ({m}x)")
    ranked = sorted(groups.items(), key=lambda kv: -kv[1])
    out = []
    if ranked:
        top, m = ranked[0]
        why = "; ".join(reasons.get(top, [])[:4])
        sg = top.split()[0]
        out.append(f"MINIMAL FIX of the dominant failure — {top} in {m}/{n} episodes" + (f" (reasons: {why})" if why else "")
                   + f". Make the smallest change that gets `{sg}` past it in most instances.")
        out.append(f"REDESIGN `{sg}`, the subgraph where {m}/{n} episodes end" + (f" ({why})" if why else "")
                   + ". Rewrite it more simply and more physically: try motions and judge them by their measured outcome, drop self-imposed"
                   " preconditions that make it give up before trying, and keep its declared inputs/outputs/exits so the rest of the graph still fits.")
        out.append(f"LOOK AHEAD: `{sg}` is where episodes stop now, so every subgraph after it has never run. Audit the downstream subgraphs"
                   " for the same failure patterns (over-strict reach margins, uncapped or unconverging moves, preconditions that abort"
                   " instead of trying) and fix them, so the graph keeps going once the dominant failure is fixed. You may also fix"
                   f" `{sg}` if you see a clear cause.")
        for key, m2 in ranked[1:]:
            out.append(f"the failure {key} ({m2}/{n} episodes): fix it")
    flipped = sorted(((cid, c.get("FLIPPED", 0)) for cid, c in stats["crossings"].items()), key=lambda kv: -kv[1])
    for cid, m in flipped:
        if m:
            born = stats["flipped_born_in"].get(cid, {})
            where = max(born, key=born.get) if born else "unknown node"
            out.insert(0 if not ranked else 1, f"target crossing #{cid} comes out with the wrong over/under in {m}/{n} episodes (made in {where}): fix how it is made")
    if stats["success_rate"] >= 0.5:
        out.insert(1, "THROUGHPUT: cut the cycle time (sim seconds) of successful episodes without lowering the success rate — fewer moves, faster safe speeds, no redundant checks")
    out.append("ROBUSTNESS: look at which variants fail (stiffness, friction, jig offset/yaw, rope layout) and make the graph adapt to them")
    seen, uniq = set(), []
    for f in out:
        if f not in seen:
            uniq.append(f)
            seen.add(f)
    return (uniq + ["a fresh hypothesis for the most common failure, different from what HISTORY.md tried"] * k)[:k]


