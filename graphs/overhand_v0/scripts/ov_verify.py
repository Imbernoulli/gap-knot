"""Hands off and clear, let the rope settle, read the knot."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the runtime does not put scripts/ on the path
from ov_common import params, state
from knotlab import skills


def run(ctx) -> dict:
    for arm in (0, 1):
        skills.retreat(ctx, arm)
    ctx.tool("robot.wait_steps", steps=int(params()["verify"]["settle_steps"]))
    st = state(ctx)
    return {"route": "tied" if st["tied"] else "untied", "state": st}
