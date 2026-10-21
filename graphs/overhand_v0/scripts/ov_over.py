"""Lay the second half (B) up to its under-crossing: the steps that cross OVER the loop."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import first_thread, run_steps


def run(ctx) -> dict:
    r = run_steps(ctx, lambda st, pl: st["half"] == "B" and st["k"] < first_thread(pl))
    return {"route": "laid", "report": r}
