"""The laying plan for the double coin knot from rope.layout: ordered steps, the crossings each makes."""
from typing import TypedDict

from gap import NodeContext

from knotlab import plan


class Output(TypedDict):
    steps: list
    phases: list
    under_crossings: list
    n_steps: int


def run(ctx: NodeContext, layout: dict, spacing: float = 0.05, after: float = 0.025) -> Output:
    del ctx
    p = plan.lay_plan(layout, spacing=spacing, after=after)
    return {"steps": p["steps"], "phases": p["phases"], "under_crossings": p["under_crossings"], "n_steps": len(p["steps"])}
