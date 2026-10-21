"""One placement: pinch the rope at s, carry it to xy, set it down. Routes on `outcome`."""
from typing import TypedDict

from gap import NodeContext

from knotlab import skills


class Output(TypedDict):
    outcome: str  # "laid" | "not_held" | "slipped" | "unreached" | "off_target"
    error_m: float
    report: dict


def run(ctx: NodeContext, step: dict, overshoot: list | None = None, height: float = skills.CARRY,
        speed: float = skills.SPEED, low: float = skills.LOW, dwell: int = 20) -> Output:
    r = skills.lay(ctx, int(step["arm_id"]), float(step["s"]), step["xy"], overshoot_xy=tuple(overshoot or (0.0, 0.0)),
                   height=height, speed=speed, low=low, place_kw={"dwell": dwell})
    r.get("pinch", {}).pop("rot", None)
    return {"outcome": "laid" if r.get("ok") else r.get("failure", "failed"),
            "error_m": float(r.get("place", {}).get("error", -1.0)), "report": r}
