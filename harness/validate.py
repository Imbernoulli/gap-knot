"""Static check of a candidate graph before it is rehearsed: python harness/validate.py GRAPH_DIR

- gap's own validator on the staged graph (RoboSimStudio's check_graph);
- every tool a node names, or a script calls as ctx.tool("<name>", ...), is in the task's catalog;
- no forbidden tools (welding the rope to a hand, reading the grader);
- every script imports and defines run(ctx, ...).
Exit 0 and "VALID" when it passes; otherwise the problems, one per line.
"""
from __future__ import annotations

import ast
import importlib.util
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FORBIDDEN = {"sim.attach", "sim.detach", "sim.check_success", "sim.query"}
CATALOG = {m.group(1) for m in re.finditer(r"^- `([a-z_]+\.[a-z_]+)", (HERE / "docs" / "TOOLS.md").read_text(), re.M)}


def _tools_in_script(path: Path) -> set[str]:
    names = set()
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "attr", "") == "tool" and node.args:
            a = node.args[0]
            if isinstance(a, ast.Constant) and isinstance(a.value, str):
                names.add(a.value)
    return names


def check(graph_dir: str | Path) -> list[str]:
    graph_dir = Path(graph_dir)
    problems: list[str] = []
    wf_path = graph_dir / "workflow.json"
    if not wf_path.exists():
        return [f"no workflow.json in {graph_dir}"]
    try:
        wf = json.loads(wf_path.read_text())
    except ValueError as e:
        return [f"workflow.json is not JSON: {e}"]
    used: dict[str, str] = {}

    def walk(nodes: dict, where: str) -> None:
        for name, n in nodes.items():
            if n.get("type") == "tool":
                used[n.get("tool", "")] = f"{where}.{name}"
            if n.get("type") == "script":
                sp = graph_dir / n.get("script", "")
                if not sp.exists():
                    problems.append(f"{where}.{name}: script {n.get('script')} does not exist")
                    continue
                for t in _tools_in_script(sp):
                    used.setdefault(t, f"{where}.{name} ({sp.name})")

    walk(wf.get("nodes", {}), "top")
    for sg, body in wf.get("subgraphs", {}).items():
        walk(body.get("nodes", {}), sg)
    for helper in (graph_dir / "scripts").glob("**/*.py") if (graph_dir / "scripts").exists() else []:
        for t in _tools_in_script(helper):
            used.setdefault(t, f"helper {helper.name}")
    for t, where in sorted(used.items()):
        if t in FORBIDDEN:
            problems.append(f"{where}: tool {t} is forbidden (it welds the rope to a hand or reads the grader)")
        elif t not in CATALOG:
            problems.append(f"{where}: tool {t!r} is not in the catalog (harness/docs/TOOLS.md)")
    # Scripts import and expose run().
    sys.path.insert(0, str(graph_dir / "scripts"))
    for sp in sorted((graph_dir / "scripts").glob("*.py")) if (graph_dir / "scripts").exists() else []:
        try:
            spec = importlib.util.spec_from_file_location(f"_cand_{sp.stem}", sp)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)  # type: ignore[union-attr]
        except Exception as e:  # noqa: BLE001
            problems.append(f"scripts/{sp.name}: import failed: {type(e).__name__}: {e}")
            continue
    # gap's validator on the staged graph.
    try:
        from robosimstudio_policy_agent.graphs import check_graph
        from robosimstudio_policy_agent.skills import pinned_registries

        errors, warnings, _ = check_graph(graph_dir, pinned_registries(None), None)
        problems += [f"gap: {e}" for e in errors]
        for w in warnings:
            if "TypedDict" not in w:  # a script returning a plain dict is fine here
                print(f"warning: {w}")
    except Exception as e:  # noqa: BLE001
        problems.append(f"gap validator could not run: {type(e).__name__}: {e}")
    return problems


if __name__ == "__main__":
    probs = check(sys.argv[1])
    if probs:
        print("\n".join(probs))
        sys.exit(1)
    print("VALID")
