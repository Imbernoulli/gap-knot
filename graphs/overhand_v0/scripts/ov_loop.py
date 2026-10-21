"""Lay the first half (A, node 0's) into the loop: every plan step of half A."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import run_steps


def run(ctx) -> dict:
    r = run_steps(ctx, lambda st, pl: st["half"] == "A")
    return {"route": "laid", "report": r}
