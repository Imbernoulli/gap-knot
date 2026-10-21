"""Lobe probe: run params.json's op list for one half, home both hands, report the lobe's metrics."""
import json
import os
import sys
from pathlib import Path
from typing import TypedDict

from gap import NodeContext

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lobe_lib import lobe_metrics, run_ops  # noqa: E402

from knotlab import skills  # noqa: E402


class Output(TypedDict):
    report: dict


def run(ctx: NodeContext, layout: dict) -> Output:
    params = json.loads((Path(__file__).resolve().parents[1] / "params.json").read_text())
    half = params.get("half", "B")
    rep = {"params": params, "before": lobe_metrics(ctx.tool("rope.state")["nodes"], layout, half)}
    rep.update(run_ops(ctx, layout, half, params.get("ops", [])))
    for a in (0, 1):
        skills.home(ctx, a)
    ctx.tool("robot.wait_steps", steps=int(params.get("settle", 40)))
    rep["after"] = lobe_metrics(ctx.tool("rope.state")["nodes"], layout, half)
    print(json.dumps({k: v for k, v in rep["after"].items() if k != "crossings"}))
    return {"report": rep}
