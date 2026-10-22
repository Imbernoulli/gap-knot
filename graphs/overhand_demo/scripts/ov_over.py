"""The demo plan's 'over' phase (knotlab.demo): see knotlab/demo.py for what each step reproduces."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import run_phase


def run(ctx) -> dict:
    return {"route": "laid", "report": run_phase(ctx, "over")}
