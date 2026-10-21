#!/bin/bash
# Run a python script in RoboSimStudio's venv, with gap-knot on the path. CPU-only by default, as the fleet runs.
HERE="$(cd "$(dirname "$0")/.." && pwd)"
export UV_NO_GROUP=rtx PYTHONPATH="$HERE:$PYTHONPATH" CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES-}"
exec uv run --project "$HERE/third_party/robosimstudio" --extra gap --extra superdex python "$@"
