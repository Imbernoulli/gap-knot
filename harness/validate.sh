#!/bin/bash
# Validate a graph without uv (runs inside a Codex sandbox): harness/validate.sh GRAPH_DIR
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export PYTHONPATH="$HERE:$PYTHONPATH" PYTHONDONTWRITEBYTECODE=1 CUDA_VISIBLE_DEVICES=""
exec "$HERE/third_party/robosimstudio/.venv/bin/python" "$HERE/harness/validate.py" "$@"
