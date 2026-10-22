# The robot take against the clip (RoboSimStudio live page)

`scene_demo.py` is the Codex Real2Sim scene of the Howcast clip (`third_party/robosimstudio/examples/live/scene.py`,
report `examples/live/REAL2SIM.md`) with the robot given the job: the rope re-laid as the knotlab overhand
task's start (same place relative to the robot), knotlab's rope contact and friction, a `knot` camera at the
clip's wide-shot scale over the arms' workspace, and the GaP plan bound as routines. `demo_routine.py` runs the
GaP graph's own scripts (`graphs/overhand_demo`) on a worker thread whose `ctx.tool` calls are executed on the
page through the studio's `reach` / `grip` / `hand` / `wait`. Every change against the Codex scene is commented
in `scene_demo.py`; every way the executor differs from GaP's tools is documented in `demo_routine.py`.

The working copies live in `third_party/robosimstudio/examples/live/` (the page imports them from there);
these are copies for the gap-knot repository. The Codex scene (`scene.py`, `ref/`, `run/`) is untouched; this
page writes to `run_demo/` and its clip timeline is `ref_demo/timeline.json`.

```bash
cd third_party/robosimstudio/examples/live
cp /srv/home/bohanlyu/gap-knot/video/overhand/real2sim_demo/{scene_demo.py,demo_routine.py} .   # if not there
# the clip's wide-shot anchors (19.4 straight, 20.9 loop_oval, 22.7 loop, 31.0 threaded (close-up), 33.6 loose, 35.8 tight)
./ctl --run-dir run_demo ref frames ../../../../video/overhand/SwdJ5op25SM.mp4 \
    --at 19.4:straight,20.9:loop_oval,22.7:loop,31.0:threaded,33.6:loose,35.8:tight --out ref_demo   # needs the page up
UV_NO_GROUP=rtx CUDA_VISIBLE_DEVICES=6 ./ctl --run-dir run_demo up --scene scene_demo.py   # runs on CPU here: the driver is too old for the venv's CUDA warp
./ctl --run-dir run_demo status                       # routines: demo, graph, steps, json, smoke
./ctl --run-dir run_demo reset
./ctl --run-dir run_demo play graph --record take1 --camera knot --every 60   # the GaP graph, node for node; ~8 control steps/s on CPU (~25 min)
./ctl --run-dir run_demo status | grep playing        # wait for "playing: 0" (do not Pause: paused arm panels do not drive)
./ctl --run-dir run_demo compare --take take1 --key rope=blue --timeline ref_demo/timeline.json
# Better: key the clip's rope by its own colour (the "blue" word finds only fragments of this dim, braided rope:
# 99-900 px against 1700-4600 px for the hex key) and pin each anchor to the take step where the plan's phase
# that reproduces it ends (sim_step, from the per-node "steps" in run_demo/demo_take_NNN.json), e.g.
#   ref_demo/timeline_take1.json: straight 0, loop_oval 2650, loop 5303, threaded 8613, loose 9773, tight 12593
./ctl --run-dir run_demo compare --take take1 --key 'rope=#40425f~0.06' --timeline ref_demo/timeline_take1.json
# read run_demo/cmp_NNN_strip.png and the verdict line; the plan's own log (every tool call, per node and per
# step held/ok, the rope's code after each node) is run_demo/demo_take_NNN.json
./ctl --run-dir run_demo quit
```

Other plan sources: `play steps` (`knotlab.demo.demo_steps(layout)`, each step run by the graph's
`ov_exec.run_step`), `play json` (`examples/live/demo_steps.json`), `play smoke` (two steps). `DEMO_GRAPH`,
`DEMO_VARIANT` (a knotlab Variant as JSON), `DEMO_MU`, `DEMO_CONTACT=codex` (the Codex rope contact),
`DEMO_SNAP_PINCH=0` and `DEMO_Z_SAG` are read when the page starts.

Each take re-imports `knotlab.demo` and the graph's `ov_*` scripts from disk, so edits to the plan land
without restarting the page; edits to `scene_demo.py` rebuild the scene.

What the executor does differently from GaP's tools, measured on this page: a pinch close is snapped, not
ramped (ramped: 0 of 4 pinches held; snapped: every pinch held, rise 14-15 mm of 20); the wrist is the
half-turn twin of `grasp_frame`'s when that solves better (the right arm's -90 deg wrist at y -0.2 drove the
hand into its own base); short descents glide at 0.05 m/s and wait until the hand is still (at 0.25 m/s the
just-closed jaws lost the cord on the lift). The rope uses knotlab's contact, not the Codex's (with skin off,
4 mm mirror padding and a 5 mm gap the jaws stopped at 43 mm over the cord; with skin off alone the cord's axis
sank to 1 mm). The CPU id tracer labels the rod's particles as the rope (it is not a collider shape, so the
scene's segmentation had no rope label at all and every take read "rope missing").
