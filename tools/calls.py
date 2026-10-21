"""Print a node's tool calls from a GaP trace compactly: python tools/calls.py RUN_DIR [node-substring]."""
import json, sys
from pathlib import Path

run = Path(sys.argv[1])
want = sys.argv[2] if len(sys.argv) > 2 else ""
KEEP = ("outcome", "remaining_distance", "steps", "position", "heading_deg", "command_width_m", "waited", "progress", "code", "error", "reason")
for node in sorted((run / "trace" / "0000" / "node_data").glob("*")):
    if want not in node.name:
        continue
    for call in sorted((node / "calls").glob("*")):
        try:
            req = json.loads((call / "request.json").read_text())
            resp = json.loads((call / "response.json").read_text()) if (call / "response.json").exists() else {}
        except Exception as e:
            print(call.name, "unreadable", e); continue
        pos = req.get("pose", {}).get("position") if isinstance(req.get("pose"), dict) else None
        rq = {k: round(v, 4) for k, v in pos.items()} if pos else {k: v for k, v in req.items() if k != "pose"}
        rs = {k: (round(v, 4) if isinstance(v, float) else ([round(x, 4) for x in v] if isinstance(v, list) and len(v) <= 3 else v)) for k, v in resp.items() if k in KEEP} if isinstance(resp, dict) else resp
        print(f"{node.name:28s} {call.name:24s} {json.dumps(rq)[:110]:110s} -> {json.dumps(rs)[:200]}")
