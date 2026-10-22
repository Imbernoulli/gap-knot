import json
from pathlib import Path
from knotlab import skills

def run(ctx):
    p = json.loads((Path(__file__).resolve().parents[1] / "params.json").read_text())
    s = float(p["s"]); arm = int(p["arm"])
    pt = ctx.tool("rope.point", s=s)
    g = skills.pinch(ctx, arm, s, low=float(p["low"]), fraction=float(p["fraction"]),
                     open_width=float(p["open_width"]), verify_lift=float(p["verify_lift"]),
                     offset_xy=(float(p.get("offset_x", 0.0)), float(p.get("offset_y", 0.0))))
    tcp, _ = skills.ee(ctx, arm)
    skills.retreat(ctx, arm, g.get("rot"))
    return {"route": "done", "point": pt, "grasp": {k: v for k, v in g.items() if k != "rot"}, "tcp": tcp}
