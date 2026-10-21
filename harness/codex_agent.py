"""Run one GaP agent as a Codex session through the account pool (`coco`).

Each agent works in its own workspace directory with workspace-write sandboxing: it reads the
shared docs copied in, edits the graph there, and runs the static validator; it never runs the
simulator (generation and testing are kept apart, as GaP keeps them).
"""
from __future__ import annotations

import json
import os
import shutil
import subprocess
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "harness" / "docs"
MODEL = os.environ.get("GAP_CODEX_MODEL", "gpt-6.1-sol")
# `coco` is codex routed through the friend account pool (its own CODEX_HOME and key); the plain `codex`
# login is the user's own ChatGPT account and must not carry this harness's load.
CODEX_BIN = os.environ.get("GAP_CODEX_BIN", "coco")
# The pool is shared: never more than this many sessions at once from this harness.
_SLOTS = __import__("threading").BoundedSemaphore(int(os.environ.get("GAP_CODEX_CONCURRENCY", "5")))
EFFORT = os.environ.get("GAP_CODEX_EFFORT", "xhigh")


def prepare(ws: Path, graph: Path | None = None) -> Path:
    """A fresh workspace: the shared docs, the validator, and (optionally) a copy of a graph to edit."""
    ws = Path(ws).resolve()
    ws.mkdir(parents=True, exist_ok=True)
    shutil.copytree(DOCS, ws / "docs", dirs_exist_ok=True, ignore=shutil.ignore_patterns("__pycache__"))
    if graph is not None:
        if (ws / "graph").exists():
            shutil.rmtree(ws / "graph")
        shutil.copytree(graph, ws / "graph", ignore=shutil.ignore_patterns("__pycache__", ".cache"))
    (ws / "validate.sh").write_text(
        "#!/bin/bash\n# Static check of the graph in this workspace: ./validate.sh [graph_dir]\n"
        f'export WARP_CACHE_PATH="$PWD/.cache/warp" XDG_CACHE_HOME="$PWD/.cache" MPLCONFIGDIR="$PWD/.cache/mpl"\n'
        f'exec {ROOT}/harness/validate.sh "${{1:-graph}}"\n'
    )
    (ws / "validate.sh").chmod(0o755)
    return ws


def run(role: str, ws: Path, prompt: str, *, images: list[Path] = (), timeout_s: int = 3600, effort: str = EFFORT) -> dict:
    """One Codex session; returns its final message, wall time, exit code and token use, and logs JSONL events."""
    ws = Path(ws).resolve()
    (ws / f".prompt_{role}.md").write_text(prompt)
    cmd = [
        CODEX_BIN, "exec", "--skip-git-repo-check", "-C", str(ws), "-s", "workspace-write",
        "-m", MODEL, "-c", f'model_reasoning_effort="{effort}"', "--json",
        "-o", str(ws / f".last_{role}.md"),
    ]
    for im in images:
        cmd += ["-i", str(im)]
    cmd.append("-")  # the prompt on stdin
    env = dict(os.environ)
    with _SLOTS:
        t0 = time.time()
        with open(ws / f".codex_{role}.jsonl", "w") as log:
            try:
                p = subprocess.run(cmd, input=prompt, text=True, stdout=log, stderr=subprocess.STDOUT, timeout=timeout_s, cwd=ws, env=env)
                code = p.returncode
            except subprocess.TimeoutExpired:
                code = -9
    last = (ws / f".last_{role}.md").read_text() if (ws / f".last_{role}.md").exists() else ""
    log_text = (ws / f".codex_{role}.jsonl").read_text(errors="replace") if (ws / f".codex_{role}.jsonl").exists() else ""
    limited = "usage limit" in log_text[-4000:] or "rate limit" in log_text[-4000:].lower()
    tokens = 0
    try:
        for line in open(ws / f".codex_{role}.jsonl"):
            if '"usage"' in line or "token" in line:
                try:
                    ev = json.loads(line)
                except ValueError:
                    continue
                u = (ev.get("usage") or (ev.get("msg") or {}).get("usage") or {})
                tokens = max(tokens, int(u.get("total_tokens", 0) or 0) + int(u.get("input_tokens", 0) or 0) * 0)
    except OSError:
        pass
    return {"role": role, "exit": code, "wall_s": round(time.time() - t0, 1), "last": last, "tokens": tokens, "limited": limited}
