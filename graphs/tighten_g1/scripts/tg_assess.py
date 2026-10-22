"""Capture the original knot length once, never reset it on a retry."""
import json
import os


def run(ctx) -> dict:
    with open(os.path.join(os.path.dirname(__file__), "..", "params.json")) as f:
        p = json.load(f)
    st = ctx.tool("rope.state")
    ss = [c[key] for c in st["crossings"] for key in ("s_a", "s_b")]
    tokens = st["code"].split()
    valid = (len(st["crossings"]) == 3 and len(tokens) == 6
             and [t[:-1] for t in tokens] == ["1", "2", "3", "1", "2", "3"]
             and all(tokens[i][-1] != tokens[i + 1][-1] for i in range(5)))
    return {"route": "ready" if valid else "lost", "start_in_knot": max(ss) - min(ss) if ss else 0.0,
            "cycle": 0, "next_distance": p["distance"], "gain": p["initial_gain"],
            "used_steps": 300, "code": st["code"]}
