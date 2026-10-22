"""Hands clear, the rope settled: is it still an overhand knot, drawn tight?"""
import json
import os

from knotlab import skills

HERE = os.path.dirname(os.path.abspath(__file__))


def run(ctx) -> dict:
    with open(os.path.join(HERE, "..", "params.json")) as f:
        p = json.load(f)
    for arm in (0, 1):
        skills.retreat(ctx, arm)
    ctx.tool("robot.wait_steps", steps=int(p["settle_steps"]))
    st = ctx.tool("rope.state")
    return {"route": "tight" if st["tied"] else "loose", "code": st["code"]}
