"""Write the tool catalog a graph on this task can call (name, signature, summary) to harness/docs/TOOLS.md."""
import inspect, sys
from pathlib import Path
from knotlab import task, tools
from robosimstudio_policy_agent import SolutionGraph

env = task.build(task.NOMINAL)
env.reset(0)
policy = SolutionGraph(env, graph=str(Path(__file__).resolve().parents[1] / "graphs" / "probe_pinch"), task_tools=tools.register)
conn = policy._new_connector()
reg = None
for name in ("registry", "tool_registry", "_registry", "tools"):
    reg = getattr(conn, name, None)
    if reg is not None:
        break

out = ["# Tool catalog\n", "Every tool a graph node or a script's `ctx.tool(name, **kwargs)` can call on this task (privileged sim run).",
       "Positions are WORLD frame [m]; the bench work-surface origin is world (0.6, 0, 0.75); arm_id 0 = left, 1 = right.\n"]
tools_ = reg._tools if isinstance(reg._tools, dict) else {}
for tname in sorted(tools_):
    t = tools_[tname]
    fn = getattr(t, "fn", None) or getattr(t, "callable", None) or getattr(t, "func", None)
    summary = getattr(t, "summary", "") or ""
    try:
        sig = str(inspect.signature(fn)) if fn else ""
    except (TypeError, ValueError):
        sig = ""
    doc = inspect.getdoc(fn) if fn else ""
    first = (doc or summary).strip().split("\n\n")[0].replace("\n", " ")
    out.append(f"- `{tname}{sig}` — {summary or first}")
Path(__file__).resolve().parent.joinpath("docs").mkdir(exist_ok=True)
Path(__file__).resolve().parent.joinpath("docs", "TOOLS.md").write_text("\n".join(out) + "\n")
print(len(tools_), "tools;", sorted(tools_)[:80])
