"""The bimanual step: pass B's free end UNDER the loop's strand while the other hand lifts it."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import run_steps


def run(ctx) -> dict:
    r = run_steps(ctx, lambda st, pl: st["kind"] == "thread")
    return {"route": "threaded" if r["ok"] else "missed", "report": r}
