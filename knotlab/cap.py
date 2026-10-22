"""Small, auditable robot-action caps for rehearsed rope skills.

A cap is a recording of *robot commands* made while a human or a graph
rehearses one bounded move (for example, lifting one end over a strand).  It
is deliberately not a simulator snapshot: it never stores or restores rope
particles, pins, contacts, or a gripper latch.  Replaying a cap calls the
ordinary ``robot.*`` tools on the current rope and checks that motions reach
their targets and that the rope does not lose topological progress.

Typical use from a graph/script::

    rec = CapRecorder(ctx, "lift_end", requires_hold=True)
    result = rec.run(lambda c: my_rehearsed_skill(c))
    rec.save("caps/lift_end.json")

The JSON is intentionally plain and reviewable.  A cap is a local skill
artifact, never evidence that the complete knot was made; the final strict
grader must still be run on the full graph.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any, Callable


CAP_VERSION = 1
_ROBOT_PREFIX = "robot."
_FORBIDDEN = ("rope.", "sim.", "env.", "state.", "physics.")


def _json(value: Any) -> Any:
    """Convert numpy/scalar values to JSON without hiding non-finite values."""
    if hasattr(value, "tolist"):
        return _json(value.tolist())
    if isinstance(value, dict):
        return {str(k): _json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json(v) for v in value]
    if isinstance(value, float):
        if value != value or value in (float("inf"), float("-inf")):
            raise ValueError("cap contains a non-finite number")
        return value
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


def _state_summary(state: dict[str, Any]) -> dict[str, Any]:
    """Keep enough state to audit a cap without making it a particle snapshot."""
    out = {}
    for key in ("code", "target_code", "prefix", "progress", "tied", "length", "knot_length"):
        if key in state:
            out[key] = _json(state[key])
    crossings = state.get("crossings")
    if isinstance(crossings, list):
        out["n_cross"] = len(crossings)
    return out


def _fingerprint(summary: dict[str, Any]) -> str:
    blob = json.dumps(summary, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(blob).hexdigest()[:16]


class CapRecorder:
    """Proxy a context while recording only physical robot calls and guards."""

    def __init__(self, ctx: Any, name: str, *, requires_hold: bool = False, metadata: dict | None = None):
        self.ctx = ctx
        self.name = str(name)
        self.requires_hold = bool(requires_hold)
        self.metadata = dict(metadata or {})
        self.calls: list[dict[str, Any]] = []
        self.initial: dict[str, Any] | None = None
        self.final: dict[str, Any] | None = None

    def _state(self) -> dict[str, Any] | None:
        try:
            value = self.ctx.tool("rope.state")
        except Exception:
            return None
        return _state_summary(value) if isinstance(value, dict) else None

    def tool(self, name: str, **kwargs: Any) -> Any:
        if self.initial is None:
            self.initial = self._state()
        before = self._state()
        result = self.ctx.tool(name, **kwargs)
        after = self._state()
        if str(name).startswith(_ROBOT_PREFIX):
            self.calls.append({
                "tool": str(name),
                "args": _json(kwargs),
                "result": _json(result),
                "before": before,
                "after": after,
            })
        self.final = after
        return result

    def run(self, fn: Callable[["CapRecorder"], Any]) -> Any:
        result = fn(self)
        self.final = self._state()
        return result

    def as_dict(self) -> dict[str, Any]:
        if self.initial is None:
            self.initial = self._state()
        if self.final is None:
            self.final = self._state()
        return {
            "cap_version": CAP_VERSION,
            "name": self.name,
            "kind": "robot_action_cap",
            "metadata": self.metadata,
            "requires_hold": self.requires_hold,
            "initial": self.initial,
            "initial_fingerprint": _fingerprint(self.initial or {}),
            "final": self.final,
            "calls": self.calls,
            "audit": {
                "robot_only": all(c["tool"].startswith(_ROBOT_PREFIX) for c in self.calls),
                "call_count": len(self.calls),
                "replay_required": True,
            },
        }

    def save(self, path: str | Path) -> Path:
        out = Path(path)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(self.as_dict(), indent=2, sort_keys=True) + "\n")
        return out


def load(path: str | Path) -> dict[str, Any]:
    cap = json.loads(Path(path).read_text())
    validate(cap)
    return cap


_TRACE_ROBOT_TOOLS = {
    "go_to_pose", "go_to_pose_cartesian", "set_grip", "wait_steps",
    "get_ee_pose", "grasp_frame", "go_home",
}


def from_trace(trace_node: str | Path, *, name: str | None = None,
               requires_hold: bool = False, metadata: dict | None = None) -> dict[str, Any]:
    """Extract a robot-only cap from one completed GaP trace node.

    Trace directories name calls like ``013_go_to_pose`` and keep request JSON
    beside the response.  Perception/state calls (``rope.point`` etc.) are
    intentionally omitted: replay reads the live rope, while the cap preserves
    only commands that could physically move the hands.  The extracted cap is
    untrusted until the caller reviews it and replays it on a fresh straight
    rope; this helper never labels a trace successful.
    """
    root = Path(trace_node)
    calls_dir = root / "calls" if (root / "calls").is_dir() else root
    entries = []
    for req in calls_dir.glob("*/request.json"):
        stem = req.parent.name
        tool_stem = stem.split("_", 1)[1] if "_" in stem else stem
        if tool_stem not in _TRACE_ROBOT_TOOLS:
            continue
        try:
            args = json.loads(req.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"cannot read trace request {req}: {exc}") from exc
        resp_path = req.parent / "response.json"
        response = json.loads(resp_path.read_text()) if resp_path.exists() else {}
        try:
            order = int(stem.split("_", 1)[0])
        except ValueError:
            order = len(entries)
        entries.append((order, stem, args, response))
    entries.sort(key=lambda x: (x[0], x[1]))
    if not entries:
        raise ValueError(f"trace node has no recognized robot calls: {calls_dir}")
    cap = {
        "cap_version": CAP_VERSION,
        "name": name or root.name,
        "kind": "robot_action_cap",
        "metadata": {**(metadata or {}), "source": "gap_trace", "trace_node": str(root)},
        "requires_hold": bool(requires_hold),
        "initial": None,
        "initial_fingerprint": _fingerprint({}),
        "final": None,
        "calls": [{"tool": f"robot.{stem.split('_', 1)[1]}", "args": _json(args),
                   "result": _json(response), "before": None, "after": None}
                  for _, stem, args, response in entries],
        "audit": {"robot_only": True, "call_count": len(entries),
                  "replay_required": True, "source_trace_untrusted": True},
    }
    validate(cap)
    return cap


def validate(cap: dict[str, Any]) -> None:
    """Reject caps that could alter simulation state or bypass robot tools."""
    if cap.get("cap_version") != CAP_VERSION or cap.get("kind") != "robot_action_cap":
        raise ValueError("unsupported cap format")
    calls = cap.get("calls")
    if not isinstance(calls, list) or not calls:
        raise ValueError("cap must contain at least one robot call")
    for i, call in enumerate(calls):
        tool = call.get("tool") if isinstance(call, dict) else None
        if not isinstance(tool, str) or not tool.startswith(_ROBOT_PREFIX):
            raise ValueError(f"call {i} is not a robot tool")
        if any(tool.startswith(p) for p in _FORBIDDEN):
            raise ValueError(f"forbidden tool in call {i}: {tool}")
        if not isinstance(call.get("args"), dict):
            raise ValueError(f"call {i} has no argument object")


def replay(ctx: Any, cap: dict[str, Any], *, require_same_start: bool = False,
           progress_tolerance: float = 0.02) -> dict[str, Any]:
    """Replay a cap using normal robot tools and return an auditable report.

    ``require_same_start`` compares the compact rope-state fingerprint.  It is
    off by default because a skill may intentionally accept small geometric
    variation.  In either mode the replay never writes rope state and never
    invokes non-robot calls from the cap.
    """
    validate(cap)
    start = _state_summary(ctx.tool("rope.state"))
    expected = cap.get("initial") or {}
    start_match = _fingerprint(start) == cap.get("initial_fingerprint")
    if require_same_start and not start_match:
        return {"ok": False, "failure": "start_state_mismatch", "start": start, "expected": expected}
    reports = []
    last_progress = float(start.get("progress", 0.0) or 0.0)
    for i, call in enumerate(cap["calls"]):
        result = ctx.tool(call["tool"], **call["args"])
        result = _json(result)
        after = _state_summary(ctx.tool("rope.state"))
        outcome = result.get("outcome") if isinstance(result, dict) else None
        progress = float(after.get("progress", last_progress) or 0.0)
        bad = outcome not in (None, "reached")
        if progress + float(progress_tolerance) < last_progress:
            bad = True
        reports.append({"index": i, "tool": call["tool"], "outcome": outcome,
                        "progress": progress, "failure": "unreached" if outcome not in (None, "reached") else
                        ("progress_drop" if progress + progress_tolerance < last_progress else None)})
        last_progress = max(last_progress, progress)
        if bad:
            return {"ok": False, "failure": reports[-1]["failure"], "start_match": start_match,
                    "calls": reports, "final": after}
    return {"ok": True, "start_match": start_match, "calls": reports,
            "final": _state_summary(ctx.tool("rope.state"))}


__all__ = ["CAP_VERSION", "CapRecorder", "from_trace", "load", "replay", "validate"]
