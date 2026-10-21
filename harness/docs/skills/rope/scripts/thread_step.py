"""An UNDER crossing: the helper lifts the strand, the working hand threads the free end beneath. Routes on `outcome`."""
from typing import TypedDict

from gap import NodeContext

from knotlab import skills


class Output(TypedDict):
    outcome: str  # "threaded" | "lift_failed" | "push_not_held" | "regrasp_missed" | "went_over" | "no_crossing"
    report: dict


def run(ctx: NodeContext, step: dict, beyond: float = 0.07, grasp_back: float = 0.06, margin: float = 0.02, lift: float = 0.06) -> Output:
    m = step["makes"][0]
    through = m["at"]
    tx, ty = step["xy"]
    # Draw the end on past the crossing, along the step's own heading from the crossing to its target.
    dx, dy = tx - through[0], ty - through[1]
    n = (dx * dx + dy * dy) ** 0.5 or 1.0
    far = (through[0] + dx / n * beyond, through[1] + dy / n * beyond)
    end_first = step["half"] == "A"
    r = skills.thread_end(ctx, int(step["arm_id"]), int(step["helper_arm"]), end_first, float(step["lift_s"]), through, far,
                          grasp_back=grasp_back, margin=margin, lift=lift)
    r.pop("rot", None)
    return {"outcome": "threaded" if r.get("ok") else r.get("failure", "failed"), "report": r}
