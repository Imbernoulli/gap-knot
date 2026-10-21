---
name: rope
description: Lays, threads and dresses a cord on a pinned board with two tool-down
  parallel-jaw hands, for flat decorative knots (the double coin knot, 双钱结) —
  pinches the cord at an arc length with the jaws across it and verifies the hold,
  carries it at a height that crosses OVER whatever it passes, sets it down with a
  dwell and a part-opened release, threads a free end UNDER a strand the other hand
  lifts, brings an out-of-reach end into reach, and draws both tails apart in small
  steps that stop if a crossing starts to go. Also plans the laying order of a
  target knot from its Gauss code with the fewest under-crossings. Use when a graph
  must shape a rope on a table rather than pick and place rigid parts.
license: Apache-2.0
compatibility: requires gap>=0.1, knotlab on PYTHONPATH, the knotlab task's rope.* tools
metadata: {category: manipulation, tags: [deformable, rope, knot, bimanual, sim-only, cpu]}
gap:
  requires: {connector: [rope.state, rope.point, rope.layout, robot.grasp_frame, robot.go_to_pose, robot.set_grip, robot.wait_steps, robot.get_ee_pose, robot.go_home]}
  allowed_tools:
    - rope.state
    - rope.point
    - rope.layout
    - robot.grasp_frame
    - robot.go_to_pose
    - robot.set_grip
    - robot.wait_steps
    - robot.get_ee_pose
    - robot.go_home
  exit_conditions:
    laid: lay_step set the point down within 20 mm of its target.
    not_held: the pinch came up without the cord (it did not rise with the hand).
    slipped: the cord trailed the hand by more than 15 mm during the carry.
    off_target: held and carried, but ended more than 20 mm from the target after release (spring-back).
    threaded: thread_step's end went UNDER the lifted strand (the crossing reads strand-over).
    went_over: the threaded end crossed the strand, but over it.
    regrasp_missed: the pushed end could not be taken again beyond the arch.
    dressed: dress pulled the tails apart without the knot's code losing ground.
  produces_outputs:
    outcome: string
    report: dict
  canonical_scripts:
    - plan_lay: scripts/plan_lay.py
    - lay_step: scripts/lay_step.py
    - thread_step: scripts/thread_step.py
    - fetch_end: scripts/fetch_end.py
    - dress: scripts/dress.py
    - home: scripts/home.py
  streaming: false
---

# rope

Shape a cord lying on a board into a flat knot with two tool-down hands. The
skills live in `knotlab/skills.py` (each takes the node's `ctx` and returns a dict of
measurements); the scripts here are thin wrappers that route on `outcome`.

## Conventions

- Positions are **world** frame; the bench top is z = 0.75 and its work-surface
  origin is world (0.6, 0, 0.75). Arms by `arm_id`: **0 = left, 1 = right**.
- A point of the rope is named by its **arc length `s`** from node 0 (the end of
  half A, which starts on the robot's right). The rope's middle `s = L/2` hangs on
  the top pin. `rope.point(s)` gives where it is now and its heading.
- The target knot is `rope.layout().target_nodes`: node k of the rope belongs at
  target node k (same count, same spacing), so "put s where it goes" is
  `target(s)`.
- Reach (tool-down, both hands): bench x in [-0.25, -0.05], y in [-0.20, 0.20]
  (world x 0.35–0.55). The servo reaches a little further (world x 0.62 at
  y -0.2 was reached); `fetch_end` uses that wider box to take a far point.

## Skills

| Skill | Signature (defaults) | What it does, measured |
|---|---|---|
| `pinch` | `pinch(ctx, arm_id, s, low=0.0135, approach=0.06, fraction=0.9, ramp=48, settle=30, verify_lift=0.02, open_width=0.045)` | Over `s`, jaws across its tangent (closing heading folded into [-90°, 90°)), down (`descend`, 2.5 mm tolerance) to the tool centre `low` over the bench — raised to 8.5 mm over the strand's own axis if it lies on another strand — ramped close to 0.9, lift 20 mm. `held` = the point rose ≥ 8 mm. |
| `carry` | `carry(ctx, arm_id, path_xy, rot, height=0.035, s=None)` | Through world-xy waypoints at `height`. At 35 mm the held cord's underside rides ~18 mm up: it crosses **over** any cord on the board. `lag` = held point to hand, xy. |
| `place` | `place(ctx, arm_id, xy, rot, s=None, overshoot_xy=(0,0), dwell=20, release_width=0.06)` | Down to 17.5 mm, hold `dwell` steps, open to 60 mm, rise 6 mm, open fully, rise. `error` = point to target after release; `spring_back` = movement on release. |
| `lay` | `lay(ctx, arm_id, s, xy, via=(), overshoot_xy=(0,0), height=0.035, place_kw=None, retries=1, **pinch_kw)` | pinch (re-pinched once on a miss) → carry → place. `ok` iff reached and `error` < 20 mm; else `failure` in {not_held, slipped, unreached, off_target}. |
| `lift_strand` / `lower_strand` | `lift_strand(ctx, arm_id, s, height=0.06)` | The helper takes a strand and raises it into an arch (`strand_z` ≈ 47 mm at the hand for 60 mm). `lower_strand(ctx, arm_id, rot, xy)` lays it back. |
| `fetch_end` | `fetch_end(ctx, arm_id, end_first, pull=0.15)` | Takes the reachable point nearest a free end and draws it 15 cm on along the cord, away from the end, so the end follows. |
| `thread_end` | `thread_end(ctx, arm_id, helper_arm, end_first, lift_s, through_xy, beyond_xy, grasp_back=0.05, gap=0.012, margin=0.018, lift=0.06)` | The UNDER crossing (see below). `ok` iff the strand reads over the threaded cord afterwards. |
| `pull` | `pull(ctx, arm_ids=(0,1), s_left, s_right, direction_left, direction_right, distance=0.06, step=0.01, height=0.03)` | Both hands take the tails and draw them apart 10 mm at a time, the knot's code read after each; stops on a progress drop. `tightened` = how much the crossings' extent shrank. |
| `retreat` / `home` | `retreat(ctx, arm_id)`, `home(ctx, arm_id)` | Open and rise / back to the arm's home posture. |
| `lobe_metrics` | `lobe_metrics(nodes, layout, half)` | A half's lobe against the target (see "Laying a lobe" below): `winding`, `err_median`/`err_max`, crossings each `ok`/`flipped`/`extra`, `consistent`, `ok`. `crossings_consistent(found, target, tol=0.10)` is the crossing check; `lobe_s(layout, half, d)` names points by distance from the middle. |
| `move` / `descend` | `move(ctx, arm_id, xyz, rot)`, `descend(ctx, arm_id, xyz, rot, tolerance=0.0025, max_steps=90)` | Motions with nothing dragging: 12 mm servoed segments capped at 40 steps each / one capped servo move held to 2.5 mm. Anything dragging cord (carry, the thread push) is `robot.go_to_pose_cartesian` at speed 0.25 or less, which has no step cap: keep its targets inside the reach box. |

## Why an UNDER crossing is threaded, never carried

A tool-down hand cannot pass cord under a strand anchored at both ends: wherever
its column crosses the strand's footprint, the strand is above the fingertips
and the column runs from the fingertips up to the arm. So the only under-crossing
is made with a **free end**: lay the end straight at the crossing, stopping
`gap` short; the helper lifts the strand; the working hand takes the cord
`grasp_back` from its end and slides it until its fingers are `margin` short of
the crossing, which pushes the end under; lets go, takes the end again beyond the
arch with the jaws only 30 mm open, and draws it through; the helper lays the
strand back down over it.

`knotlab.plan.lay_plan(layout)` schedules the laying (each half outward from the
top pin, interleaved) to need as few of these as possible. For the double coin
knot the minimum is **2** — one is a crossing of half A with itself, which no
order avoids. The plan's steps carry `kind: "lay" | "thread"`, the crossing each
makes, and, for a thread step, `helper_arm` and `lift_s`.

## Recommended subgraph state flow

```text
plan_lay ─▶ for each step:
              lay_step ──laid──▶ next
                 ├─not_held──▶ lay_step again (once), then abort
                 └─off_target/slipped──▶ lay_step again with overshoot
              thread_step (kind == thread) ──threaded──▶ next
                 └─went_over/regrasp_missed──▶ undo: lay_step back, fetch_end, retry
          ─▶ dress ──dressed──▶ home ─▶ done
```

Read `rope.state().progress` (0..1, the fraction of the target Gauss code matched
from node 0) after each step: it only rises once a step makes a crossing, so a
step that makes none is judged by its `error`.

## Measured behaviour (nominal cord: 10 mm, E = 1e5 Pa, SuperDex, feedback off)

Probes are `graphs/probe_rope` with `$PROBE` = ab (plan steps 0–9: half A's
lobe round the left pin, then B over it), c (fetch half A's end, thread it under
B's leg), d (start tied, pull both tails). Summarised:

- Snapped shut with no settle the cord slid out of the tips on the first lift;
  ramped to 0.9 with 30 settle steps it was carried 60 mm with ≤ 6 mm lag.
- With the tips at 3.9 mm (tool centre 16 mm) the lifted cord hung with its axis
  below the fingertips and about half the left-hand pinches came up empty.
- SuperDex gives the cord one friction coefficient, sqrt(mu_cable), for strands,
  pads and the bench alike, and none static below 1 mm/s. At mu_cable 0.25 a point
  carried across the board sprang back 20–60 mm on release (its own bending and the
  dragged cord behind it); at mu_cable 1.0 held placements ended 0.5–5 mm from target
  with 1–3 mm spring-back.
- Opened to only 30 mm at release, a cord under tension stayed pressed on one
  finger and rode up 51 mm with the rising hand; at 60 mm it is let go.
- A pull of the tied knot with both tails held, 10 mm steps to 60–80 mm (apart or
  both toward the robot), kept the full double coin code at every step; the
  crossings' spread grew 8–10 mm (pulling the tails spreads the bottom of the
  knot rather than shrinking its lobes). Right after release a lifted tail briefly
  shows two extra projected crossings until it falls back (hence the 30-step wait
  before the knot is read), and pulled toward the robot the released tails can
  splay into a ninth crossing.
- The tied knot left alone holds its code: started tied, the task's goal
  (`hold-for 1.0`) was met at step 61.
- `fetch_end` (half A's end, 0.12 m beyond reach up the board's side) took the
  reachable point nearest it and drew it 15 cm along the cord: the end came into
  reach in 3 of 3 runs once carries were speed-limited (placement error 5–15 mm).
- Across ten `lay` steps per run, pinches miss 30–50% on the first try
  (rise ~0: the descent nudged the cord sideways or stopped high); with one
  re-pinch 7 of 10 steps held (mu 1.0). Held placements ended 7–81 mm from
  target (median ~30 mm): **half A's lobe is not yet laid round the left pin**
  (winding 0.07 turns against the target's 0.92) — each 5 cm placement drags the
  rest of the half across the board and it recoils. Laying a lobe needs the
  remaining half staged near the knot first, or placements past the pin
  (`overshoot_xy`), or a pin-wrap motion; that is the open problem for the graph.
- **Threading is not yet validated.** It fails before the push: after
  `fetch_end` the cord within ~10 cm of the end is left 25–35 mm off the bench
  (draped, not lying), and a pinch there comes up empty. When an earlier variant
  did reach the push, the end stopped 12 mm short of the arch and the regrasp
  pulled it over (`went_over`).

## Laying a lobe round its side pin (probes, October 2026; not yet a reliable skill)

Probes: `graphs/lobe_probe` (experimental recipe in its `scripts/lobe_lib.py`), results and pictures in
`skills/rope/LOBE_RESULTS.md`. Measure a lobe with `skills.lobe_metrics(nodes, layout, half)` (winding over
d = 0.082..0.382 from the middle about the half's pin, node error to target, every crossing checked against the
target by `skills.crossings_consistent`; `ok` = winding ≥ 0.8, median ≤ 15 mm, consistent). **No `lay_lobe` is in
the library yet: 0 of 71 probe episodes met that criterion; 4 ended wrapped with only target crossings (best median
17 mm), all on half B.** What was measured:

- **A lobe needs the cord to pass over its pin.** Each half winds 0.19 turns about its side pin at the start and
  1.25 in the knot. Cord on the bench cannot cross a 50 mm pin and the free end does not go round it, so
  point-by-point placements (any order, with overshoot) left the lobe at winding ≤ 0.07. What wrapped it, every
  time it completed (0.83–1.01 turns): the **lift-over**. Take the lobe's apex (d ≈ 0.28), lift to TCP 80 mm, carry
  it straight over the pin, and set it down on its target + 8–12 mm radially out. Its two legs come down on either
  side of the pin. The far end (d 0.38) must already be on its target first, so the far leg passes the pin's far side.
- **Only half B can be laid this way; half A must be woven.** The target's chirality makes A's lobe pass UNDER B's
  run from the top pin (crossings 4, 6), and A's tail pass UNDER A's own run (crossing 2). B's lobe passes OVER A's
  run (6, 7) and B's tail OVER B's run (8). A tool-down hand only makes the carried strand go over. Measured: the
  lift-over recipe on B made crossings 6, 7, 8 all right; mirrored on A it wrapped 0.93 turns at 12 mm median, but
  crossings 2, 4, 6 were all flipped (and a_points flipped 2). So lay B's lobe first, as the video does with its
  first lobe. Then make A's by threading A's free end: round pin_left, under B's run at 4, over B's tail at 3,
  under A's run at 2, over B's lobe at 1. `plan.lay_plan`'s A-first order cannot be carried out without flipped
  crossings.
- **Route the far end round the other pin's robot side.** B's start tail (side run, fold, corridor) lies round
  pin_left's outer side. Carrying B's far end to the right on a straight or far-side path hooks that tail on
  pin_left. Waypoints at other-pin + (−45, +30) mm and other-pin + (−30, −35) mm (mirrored for A) avoided it.
- **Clear the other half's end run first.** A's corridor lies 5–42 mm (centre line) from B's lobe outline across
  the train variants (25 mm nominal). Push it radially out to ≥ 35 mm before the lift-over. Otherwise the pads
  catch it at the apex release, and once the release lifted A's end and the arm dropped it off the board.
- **Jaws near a pin:** a pad landing on a pin collides. Open no wider than 2·(r − 9.5 mm), where r is the distance
  from the TCP to the pin. But ≤ 30 mm releases, and releases on top of another strand, let the cord ride up the
  pad 30–77 mm with the rising hand, which can lift a wrapped lobe back off its pin. Centring the jaws on the cord
  before rising helped at the apex, not at the far end next to pin_top.
- **Do not re-pinch the lobe near its pin.** Shaping placements at d 0.22–0.34 (3–4 cm from the pin) undid
  wraps that had formed in every shaped run.
- **Slip:** carrying the far end drags ~45 cm of cord across both runs from the top pin. It slid 7–25 cm through
  the pads in every run at mu 0.68 and in some at mu 1.0. Grip fraction 1.0 stopped that slip. Re-grasping after a slip made
  things worse: re-wraps of −1 turn.
- **Error floor:** with only the far end and the apex placed, the half's own run from the top pin (d 0.08–0.17)
  stays 25–40 mm off and the far end drifts ~25 mm. That holds the median at ~17–22 mm even when the lobe proper
  (d 0.20–0.32) is within 6–15 mm.

## Known failure modes

- **Pinching a strand that lies on another** (near a crossing): it sits ~9 mm
  up; the tips are sent 3.6 mm under its axis, but the hold is weaker. Pinch
  points 2–3 cm from crossings.
- **A free end out of reach** (the start lays the tails up the board's sides):
  `fetch_end` first.
- **Carrying across the board drags the rest of the half**: the dragged cord
  recoils on release until a lobe has wrapped its pin; place lobe points past the
  pin (`overshoot_xy`) or lay the lobe in short steps.
- **Projected crossings count tails in the air**: read `rope.state` after the
  hands are clear and the cord has settled.
- **Cord off the bench after a fetch or a release**: check `rope.point(s)[2]`
  (a lying 10 mm cord's axis is ~0.756) before pinching; wait, or take a point
  that is lying.
- **The jaws' closing heading is folded but not checked**: the wrist pose
  `robot.get_ee_pose` returns is the flange's, not the tool frame `grasp_frame`
  builds, so the two cannot be compared directly.
