"""Step 3 of the demo: each hand takes a tail and they draw apart along the start line (knotlab.demo, phase 'pull')."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import run_phase, state


def run(ctx) -> dict:
    return {"route": "pulled", "report": run_phase(ctx, "pull"), "state": state(ctx)}
