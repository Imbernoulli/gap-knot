"""Closed-loop helpers for threading a rope end with RoboSimStudio.

The route-the-cable demo in RSS does not assume that the material point is at
the TCP after a grasp.  It measures the point's offset from the hand, and
re-reads the cable's tangent before each turn.  This module keeps that small
piece of bookkeeping independent from a GaP graph.  It deliberately uses only
the ``ctx.tool`` interface, so it can be used by a graph script, a local
rehearsal, or a fake context in a unit test.

``capture`` is called immediately after a successful pinch.  ``tcp_for_tip``
then converts a desired rope-material position into the TCP position that
should be commanded.  ``tip_state`` re-measures a free endpoint and its local
tangent; callers should use it again after every carry or crossing rather than
reusing a drawing waypoint.

These helpers do not pin or latch a rod and do not claim that a TCP move was a
successful grasp.  The caller still has to verify ``pinch(...)["held"]`` and
measure drift after each move.
"""

from __future__ import annotations

import math
from typing import Any, Iterable

import numpy as np

from knotlab import skills


def _state(ctx) -> tuple[np.ndarray, np.ndarray]:
    state = ctx.tool("rope.state")
    nodes = np.asarray(state["nodes"], dtype=np.float64).reshape(-1, 3)
    arc = np.asarray(state.get("arc", ()), dtype=np.float64)
    if len(nodes) < 2:
        raise ValueError("rope.state returned fewer than two nodes")
    if len(arc) != len(nodes):
        arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(nodes, axis=0), axis=1))))
    if not np.all(np.isfinite(nodes)) or not np.all(np.isfinite(arc)):
        raise ValueError("rope.state contains non-finite geometry")
    return nodes, arc


def _unit(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm < 1e-9:
        raise ValueError("rope endpoint tangent is degenerate")
    return v / norm


def point_state(ctx, *, s: float, inset: float = 0.02) -> dict[str, Any]:
    """Read a material point and its local tangent from the current rope geometry.

    ``s`` is clamped to the rope's current arc-length range.  The tangent is
    estimated by points on either side, with a one-sided estimate at either
    endpoint.  This is the general form used by :func:`tip_state` and is useful
    when the gripper holds a few centimetres behind a free end.
    """

    nodes, arc = _state(ctx)
    length = float(arc[-1])
    s = float(np.clip(float(s), 0.0, length))
    inset = max(0.0, min(float(inset), max(length * 0.25, 1e-6)))
    left_s, right_s = max(0.0, s - inset), min(length, s + inset)

    def sample(at: float) -> np.ndarray:
        i = int(np.clip(np.searchsorted(arc, at, side="right") - 1, 0, len(nodes) - 2))
        u = (at - arc[i]) / max(arc[i + 1] - arc[i], 1e-9)
        return nodes[i] + u * (nodes[i + 1] - nodes[i])

    position = sample(s)
    tangent = _unit(sample(right_s) - sample(left_s))
    heading = math.degrees(math.atan2(float(tangent[1]), float(tangent[0])))
    return {
        "s": s,
        "length": length,
        "position": position.copy(),
        "tangent": tangent.copy(),
        "heading_deg": heading,
        "node": int(np.clip(np.searchsorted(arc, s, side="right") - 1, 0, len(nodes) - 2)),
    }


def tip_state(ctx, *, end_first: bool, inset: float = 0.02) -> dict[str, Any]:
    """Read an endpoint and its inward tangent from the current rope geometry.

    ``end_first`` selects ``s=0`` or ``s=length``.  ``inset`` is an arc-length
    distance used to estimate a tangent, not a fixed node count, so this stays
    stable when a rope is re-sampled.  Positions are in the world frame used by
    ``rope.state``.
    """

    nodes, arc = _state(ctx)
    out = point_state(ctx, s=0.0 if end_first else float(arc[-1]), inset=inset)
    if not end_first:
        out["tangent"] = -np.asarray(out["tangent"], dtype=np.float64)
        out["heading_deg"] = math.degrees(math.atan2(float(out["tangent"][1]), float(out["tangent"][0])))
    return out


def capture(ctx, arm_id: int, *, s: float, inset: float = 0.02) -> dict[str, Any]:
    """Capture the material point's measured offset from the TCP after a pinch."""

    point = ctx.tool("rope.point", s=float(s))
    _, arc = _state(ctx)
    tcp, rotation = skills.ee(ctx, int(arm_id))
    if tcp is None:
        raise ValueError("robot.get_ee_pose returned no TCP position")
    tip = np.asarray(point["position"], dtype=np.float64)
    tcp = np.asarray(tcp, dtype=np.float64)
    return {
        "arm_id": int(arm_id),
        "s": float(s),
        "length": float(arc[-1]),
        "tip": tip.copy(),
        "tcp": tcp.copy(),
        "offset": tip - tcp,
        "heading_deg": float(point.get("heading_deg", 0.0)),
        "inset": float(inset),
        "rotation": rotation,
    }


def tcp_for_tip(target_xy: Iterable[float], capture: dict[str, Any], *, z: float) -> np.ndarray:
    """Return a TCP target for a desired XY rope position.

    The measured XY offset is subtracted because ``offset = rope_point - TCP``.
    Z is supplied by the caller (carry height), keeping the helper independent
    of the gripper's tool-centre offset and bench origin.
    """

    target = np.asarray(tuple(target_xy), dtype=np.float64)
    if target.shape != (2,):
        raise ValueError("target_xy must have two coordinates")
    offset = np.asarray(capture["offset"], dtype=np.float64)
    if offset.shape != (3,):
        raise ValueError("capture offset must have three coordinates")
    return np.array([target[0] - offset[0], target[1] - offset[1], float(z)], dtype=np.float64)


def drift(ctx, capture: dict[str, Any], *, s: float) -> dict[str, float]:
    """Measure material-point displacement from the TCP offset captured after grasp."""

    point = ctx.tool("rope.point", s=float(s))
    tcp, _ = skills.ee(ctx, int(capture.get("arm_id", 0)))
    if tcp is None:
        raise ValueError("robot.get_ee_pose returned no TCP position")
    now = np.asarray(point["position"], dtype=np.float64) - np.asarray(tcp, dtype=np.float64)
    expected = np.asarray(capture["offset"], dtype=np.float64)
    delta = now - expected
    return {
        "xy": float(np.linalg.norm(delta[:2])),
        "xyz": float(np.linalg.norm(delta)),
        "tip_tcp_offset_z": float(now[2]),
    }


def retarget_pose(ctx, *, arm_id: int, s: float, target_xy: Iterable[float], capture: dict[str, Any],
                  height: float, inset: float = 0.02) -> dict[str, Any]:
    """Build a pose using the latest material-point tangent and measured grasp offset.

    This is intentionally a pure planning helper: it reads the current tip and
    returns a pose for the caller to execute with ``robot.go_to_pose`` or
    ``robot.go_to_pose_cartesian``.  Re-call it after each material move.
    """

    geom = point_state(ctx, s=float(s), inset=inset)
    # The measured offset is in world coordinates.  Preserve the grasp's wrist
    # orientation while holding; rotating without rotating that offset would
    # invalidate the feedback target.  The live tangent remains in the report.
    rot = capture.get("rotation") or skills.frame(ctx, int(arm_id), float(geom["heading_deg"]))
    tcp = tcp_for_tip(target_xy, capture, z=skills.BENCH_Z + float(height))
    # Keep the returned plan JSON-safe: GaP traces serialize script results,
    # while the numpy arrays are only needed internally to compute this pose.
    tip = {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in geom.items()}
    return {"pose": skills._pose(tcp, rot), "tip": tip, "tcp": tcp.tolist(), "rotation": rot}


def move_tip(ctx, *, arm_id: int, s: float, target_xy: Iterable[float], capture: dict[str, Any],
             height: float = skills.CARRY, speed: float = skills.SPEED, inset: float = 0.02,
             verify: bool = True) -> dict[str, Any]:
    """Move the captured endpoint to one target, then re-read tip and drift.

    This performs one bounded cartesian tool call.  It does not silently retry
    or call ``pinch`` again; a graph should route a failed ``outcome`` or large
    drift to its own recovery node.
    """

    plan = retarget_pose(ctx, arm_id=arm_id, s=s, target_xy=target_xy, capture=capture,
                         height=height, inset=inset)
    result = ctx.tool("robot.go_to_pose_cartesian", pose=plan["pose"], arm_id=int(arm_id), speed_scale=float(speed))
    point = ctx.tool("rope.point", s=float(s))
    actual = np.asarray(point["position"], dtype=np.float64)
    wanted = np.asarray(tuple(target_xy), dtype=np.float64)
    error = float(np.linalg.norm(actual[:2] - wanted))
    out = {"outcome": str(result.get("outcome", "?")) if isinstance(result, dict) else str(result),
           "error_xy": error, "tip": actual.tolist(), "plan": plan}
    if verify:
        out["drift"] = drift(ctx, {**capture, "arm_id": int(arm_id)}, s=s)
    return out


__all__ = ["capture", "drift", "move_tip", "point_state", "retarget_pose", "tcp_for_tip", "tip_state"]
