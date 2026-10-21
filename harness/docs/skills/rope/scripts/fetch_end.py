"""Bring a half's free end into reach before it is laid or threaded. Routes on `outcome`."""
from typing import TypedDict

from gap import NodeContext

from knotlab import skills


class Output(TypedDict):
    outcome: str  # "fetched" | "nothing_in_reach" | "not_held" | "end_still_out_of_reach"
    end_at: list
    report: dict


def run(ctx: NodeContext, half: str, arm_id: int, pull: float = 0.15) -> Output:
    r = skills.fetch_end(ctx, int(arm_id), half == "A", pull=pull)
    return {"outcome": "fetched" if r.get("ok") else r.get("failure", "failed"), "end_at": r.get("end_at", []), "report": r}
