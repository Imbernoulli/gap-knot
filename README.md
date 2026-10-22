# gap-knot: bimanual knot tying from a video, with RoboSimStudio and Graph-as-Policy

We take a knot-tying task from an online video and rebuild it in simulation with [RoboSimStudio](https://robosimstudio.github.io/) (RSS, Newton physics). A [Graph-as-Policy](https://graph-robots.github.io/gap/) (GaP) computation graph then drives a YAM bimanual robot through the task. Codex agents write the graph and improve it by rehearsing it on varied task instances in parallel on Daytona: they localize each failure to a node and edit the graph to raise the success rate and cut cycle time.

Status (2026-10-22): work in progress. From-straight knot tying is still unsolved. The results below distinguish it from tightening a knot already tied at the start. The updated results page with videos is in [site/index.html](site/index.html). The older claude.ai artifact has not been synced and contains superseded results.

## Results so far

| Task (source video) | What the robot must do | Outcome |
|---|---|---|
| **Pull an overhand knot tight** (Howcast, [youtu.be/SwdJ5op25SM](https://www.youtube.com/watch?v=SwdJ5op25SM), step 3) | From a loose overhand knot, draw it up to ≤ 75% of its rope-in-knot, return it to the table, open and withdraw both hands, and hold for 1 s | **G0: 0/10; G1: 5/10; G2: 6/10.** Paired re-evaluation of existing graphs on reused training seeds 10–19. Mean simulated time among successes: G1 105.92 s, G2 65.42 s. This is not a new learning run or a held-out generalization test. |
| Overhand knot (单结) from a straight rope (same video, steps 1–3) | Make a loop, pass the end under, pull | **Not achieved: 0 successes.** Five self-learning iterations, a 16-instance held-out test (0/16), and a segment-level study of the under-crossing all failed (below). |
| Double coin knot (双钱结) on a pinned board (Art Sprouts, [youtu.be/dg6XrRY_FzI](https://www.youtube.com/watch?v=dg6XrRY_FzI)) | Lay two lobes round pins, weave the tails over and under | **Not achieved: 0 successes.** The graphs never form the 8-crossing weave. |

**Why from-scratch knot tying fails here.** Every knot needs at least one crossing where a free end passes UNDER another strand.
- **Tool-down passes.** A tool-down parallel-jaw hand cannot carry cord under a strand anchored at both ends: its own column hits the strand.
- **Segment study.** We rehearsed only that step, from a constructed "loop already made" start, with 16 mechanisms on 6 instances each:
  - pushing under a lifted arch;
  - laying the strand back over the end;
  - a tilted (60–75°) gripper;
  - the person's local tuck extracted from the video.
- **Result.** The under-crossing was made in **1 of 60** episodes on a graspable cord.
- **Common cause.** Lifting or pinching a strand next to the first crossing moves that crossing, and at this friction every pull-through drags the whole loop.
- **Lead.** Holding the first crossing down (a third hand, or the pins the double coin video uses) is the most promising fix.

**Real2Sim (RSS's own method).** A Codex agent rebuilt the Howcast clip on RSS's live page and measured it against stills with RSS's compare and check (`video/overhand/REAL2SIM.md`).
- Straight rope: silhouette IoU 0.883.
- Loose knot: 0.711.
- Loop: 0.31.
- Gate PASS.

It set a 10 mm cord, and E = 2e4 Pa from the loose knot's settled shape. At that stiffness the YAM's pads cannot hold the SuperDex rod (a 5 cm carry moved the pinched point 4 mm, against 49 mm at 1e5). The task therefore keeps 1e5, the value RSS's own knot example uses.

The `thread_feedback` graph remeasures the crossing after moving the rope; it is a threading segment probe from `start_shape=pre_thread`, not a demonstration of tying a knot from a straight rope. Eight follow-up probes all failed at the end. One made the intended under-crossing while retaining the first crossing immediately after the pass, then lost that structure on release. Variants, graphs, and intermediate diagnostics are archived in [results/threading_2026-10-22](results/threading_2026-10-22/README.md).

The old tightening curve (0/10 → 9/10) is withdrawn because it accepted airborne knots. Later diagnostic runs also used an incorrect height coordinate. The corrected runtime requires every rope node ≤5 cm above the bench, both jaws ≥5 cm open, both TCPs ≥6 cm from every rope node, and the trefoil/length conditions continuously for one second. [The audit](results/README.md) links all 30 verdicts, variants, and exact final rope coordinates.

## What is here

| Path | What |
|---|---|
| `knotlab/` | The tasks on RSS: double coin and overhand geometry, the Gauss-code knot diagram used as the success test, the `rope.*` tools, the episode runner, the rope skill library (`skills.py`), the laying planner, the demo-plan extraction, and the under-crossing segment lab |
| `harness/` | The GaP self-learning loop: Codex orchestration, skill and refiner agents (`codex_agent.py`, `prompts/`); per-node and per-crossing failure localization (`analyze.py`); paired rehearsal of candidates; strategy (`strategy.py`); briefs and docs for the agents (`docs/`) |
| `fleet/` | Daytona CPU sandbox pool: a snapshot with RSS installed, one rollout per 2-vCPU sandbox, retries, teardown |
| `graphs/` | GaP baselines, the previously refined tightening graphs (`tighten_g1`, `tighten_g2`), and the `thread_feedback` segment probe |
| `skills/rope/` | The rope skill bundle (SKILL.md), measured behaviour, lobe-laying results |
| `video/` | Storyboards of the source clips, the Real2Sim report and scene, the extracted demonstration plan |
| `tools/` | Sweeps, renders, trace readers |

## Running

1. **Install RSS.** Clone it as `third_party/robosimstudio`, with graph-as-policy and open-robot-skills beside it (commits pinned in `THIRD_PARTY.lock`). Run `UV_NO_GROUP=rtx uv sync --extra gap --extra superdex` there.
2. **One episode.** `tools/rss.sh -m knotlab.episode --graph graphs/tighten_v0 --variant '{"knot":"overhand","board_x":-0.07,"start_shape":"loose_knot","knot_size":0.15}' --out runs/x`
3. **Self-learning.** Run with a Python that has the Daytona SDK; Codex runs through the account pool via `coco`:

   ```
   python harness/selflearn.py --run runs/tg --graph graphs/tighten_v0 --knot overhand_tighten --max-steps 9000 --iters 8 --n 10 --cands 3
   ```
4. **Draw an episode.** `python tools/render_episode.py runs/x/ep000000 --mp4`

## Recheck tightening with the corrected grader

```sh
python tools/reevaluate_tighten.py --out runs/tighten-check --offset 10 --n 10 --workers 6 --export results/tighten-check.json
```

Use an interpreter with the Daytona SDK. This compares `tighten_v0`, `tighten_g1`, and `tighten_g2` on exactly the same variants, deletes its sandboxes, and exports verdicts plus final rope nodes. The small unit suite runs with `python -m unittest discover -v`.

To open the report as one file with embedded images and videos:

```sh
python tools/pack_site.py --out runs/report.html
```

## Credits

- **RoboSimStudio** (Chen, Xie et al., UC Berkeley and NVIDIA) and **Graph-as-Policy** (Chen, Xie et al., UC Berkeley, NVIDIA and Bosch), Apache-2.0. The overhand geometry in `knotlab/overhand.py` is copied from RSS's `robosimstudio_materials.rod.knot`, and `harness/docs/gap/` holds GaP's own agent prompts and docs.
- **Source clips** are linked, not redistributed: Art Sprouts, "How to tie a Double Coin Knot"; Howcast, "How to Tie an Overhand Knot". `video/` holds only stills used as references.
