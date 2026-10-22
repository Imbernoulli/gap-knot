"""Step 2 of the demo, the bimanual tuck: the left hand lifts the loop strand next to the crossing and holds it,
the right hand pushes the end under it, the strand goes back down over the end (knotlab.demo, phase 'thread')."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import run_phase


def run(ctx) -> dict:
    r = run_phase(ctx, "thread")
    push = next((s for s in r["steps"] if s.get("kind") == "push_under"), {})
    return {"route": "threaded" if push.get("ok") else "missed", "report": r}
