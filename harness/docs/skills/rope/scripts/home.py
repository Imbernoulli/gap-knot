"""Both arms home, clear of the board."""
from typing import TypedDict

from gap import NodeContext

from knotlab import skills


class Output(TypedDict):
    outcome: str


def run(ctx: NodeContext) -> Output:
    for a in (0, 1):
        skills.home(ctx, a)
    return {"outcome": "home"}
