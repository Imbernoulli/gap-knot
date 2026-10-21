"""Print a probe's report (the scenario node's output) and the episode result."""
import json, sys
from pathlib import Path
run = Path(sys.argv[1])
res = json.loads((run / "result.json").read_text())
print({k: res[k] for k in ("success", "steps", "sim_time_s", "wall_s")}, res["error"][-600:])
for out in (run / "trace").glob("*/node_data/*scenario*/output.json"):
    print(json.dumps(json.loads(out.read_text()), indent=None)[:20000])
