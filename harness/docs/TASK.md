# Task: tie a double coin knot (双钱结) with two arms

## Where it comes from

A YouTube tutorial (Art Sprouts, "How to tie a Double Coin Knot – Chinese Knot tutorial", youtu.be/dg6XrRY_FzI; storyboard in `img/video_storyboard.jpg`). On a foam board with pins:

1. The cord's middle is hung over a top pin; both halves run down the board.
2. One half is laid into a lobe round a side pin, crossing the other half's top part.
3. The other half's tail is brought up and woven through alternately over and under to make the other lobe.
4. Both tails are pulled to even the knot up.

The double coin knot is the 3-lead 4-bight Turk's head (the Carrick mat) cut open at one bight: 8 crossings, met alternately over and under along the cord.

## The simulated cell (RoboSimStudio on Newton; rope on SuperDex)

- **Robot**: the YAM bimanual pair. `arm_id 0` = left arm (world +y side), `arm_id 1` = right arm (world -y side). Each has a parallel-jaw gripper: 95 mm span, fingertips reach 12.1 mm past the tool centre (TCP), 4 mm clearance.
- **Frames**: every tool takes and returns WORLD coordinates in metres. The bench top is z = 0.75; its centre is world (0.6, 0, 0.75). World +x points away from the robot; +y is the robot's left.
- **Reach**: tool-down (gripper pointing straight down), both hands reach about world x ∈ [0.35, 0.55] and y ∈ [-0.20, +0.20] at bench height. Outside that, IK fails or the servo stalls. Every move you command must be inside it.
- **Rope**: a 10 mm cord, 1.32 m long, 162 nodes about 8 mm apart, with self-contact (strands cannot pass through each other). It is soft (Young's modulus ~1e5 Pa). Its friction coefficient (~1.0 nominal, one value for strands, pads and the bench) was raised from the engine default because a cord laid on the video's foam board stays put. It rests on the bench (axis at z ≈ 0.755). Arc length s runs from node 0 (s = 0) to the last node (s ≈ 1.32).
- **Board and pins**: a foam board (drawn only). There are three pins (posts 2.5 mm in radius, 50 mm tall, about 5 cord diameters as in the video): `pin_top`, `pin_left`, `pin_right`, each in the eye of one lobe of the finished knot. Fingers that land on a pin collide with it. A pin holds a loop laid round it, and cord carried below 50 mm cannot pass over a pin, so route carries round the pins.
- **Start** (`img/start_and_target.png`, left): the cord's middle (s ≈ 0.66) is hung round `pin_top`. Each half runs down toward the robot ~14 cm, then out sideways ~17 cm, then hooks back up the side of the board. The ends lie beyond reach. Node 0 is the end of the robot's RIGHT half (world -y); the last node is the end of the LEFT half.
- **Target** (`img/start_and_target.png`, right): `rope.layout()` gives `target_nodes`, where every node of the rope lies in the tied knot for this episode (world frame, already shifted with the jig), and `target_arc`. The 8 crossings, numbered by where strand A (the earlier along the rope) meets them:

| crossing | strand A at s | strand B at s | strand A is |
|---|---|---|---|
| 1 | 0.149 | 0.888 | over |
| 2 | 0.201 | 0.548 | under |
| 3 | 0.260 | 1.066 | over |
| 4 | 0.313 | 0.725 | under |
| 5 | 0.437 | 1.177 | over |
| 6 | 0.489 | 0.836 | under |
| 7 | 0.600 | 1.014 | under |
| 8 | 0.777 | 1.126 | under |

Walking the rope from node 0, the Gauss code is `1O 2U 3O 4U 5O 6U 2O 7U 4O 8U 6O 1U 7O 3U 8O 5U` (O = this strand is over there).
  - The half s ∈ [0, 0.66] makes the knot's LEFT lobe (round `pin_left`; robot's +y side).
  - The half s ∈ [0.66, 1.32] makes the RIGHT lobe (round `pin_right`).
  - The middle stays on `pin_top` as the top lobe.
  - Both tails leave the knot toward the robot, s < ~0.12 to the robot's right and s > ~1.20 to its left.

  The halves cross over to opposite sides: the right half of the start becomes the left lobe.

## Success and score

- **Success**: the rope's diagram seen from above has exactly the target Gauss code (no extra crossings, every crossing's over/under right), held for 1 s, before the step budget runs out. The grader uses the same measure as `rope.state()["tied"]` / `progress`.
- **Second objective**: cycle time, the scene time to success. The step budget is 18000 control steps at 60 Hz (300 s of scene time). A simulated control step costs ~0.11 s of wall time, so motion is expensive: plan economical moves.

## What varies between episodes (the graph must cope; read positions at run time)

- Rope stiffness: ×0.5–2.
- Friction: `mu_cable` 0.6–1.4 (friction coefficient 0.77–1.18).
- Axial stiffness: 200–450 N.
- Where the jig sits: centre shifted ±1 cm in x and ±1.5 cm in y, turned up to ±4.6°. The pins move with it, each also jittered up to 3 mm.
- How the rope was laid: bight width, how far the halves run down (12–15.5 cm) and out (14–19 cm), and a lateral wiggle up to 6 mm.

## Measured behaviour and the open problems

`skills/rope/SKILL.md` holds the measured numbers, read it in full. The short version:

- **What works**:
  - Pinch-and-carry: jaws across the cord, ramped 0.9 close, ≤ 35 mm carry height.
  - Placing a single point.
  - Over-crossings by carrying over a strand.
  - `fetch_end`, which brings an out-of-reach end into reach.
  - A two-hand pull of a tied knot, which keeps all 8 crossings.
  - A tied knot left alone holds its code.
- **What does not work yet**:
  1. **Laying a lobe round a side pin.** Laying half A point by point dragged the rest of the half across the board, and it recoiled on release. Placements landed a median ~30 mm off, and the lobe never wrapped `pin_left` (0.07 of the 0.92 turns needed).
  2. **Threading an end UNDER a strand.** After `fetch_end`, the last ~10 cm of cord hangs 25–35 mm off the bench and pinches there come up empty. Once the pushed end stopped 12 mm short and the regrasp pulled it over.

  Solving these is the graph's job: staging the half near the knot before laying the lobe, placements past the pin (`overshoot_xy`), a wrap round the pin, waiting or flattening before a pinch, a different laying order, or a different way to make the under-crossings. A script may compose `robot.*` tools into new motions when the library's skills are not enough.
- **Physical facts**:
  - Release spring-back is 1–3 mm at the nominal friction, more when the dragged cord recoils.
  - A move that cannot converge burns up to 1000 steps unless capped (`max_steps`, or `skills.move` / `skills.descend`). `robot.go_to_pose_cartesian` has no cap: keep its targets inside reach.
  - A pinch near a crossing is weaker; pinch 2–3 cm away from crossings.
  - Read `rope.state` only after the hands are clear and the cord has settled. A tail in the air adds projected crossings.
- **Budget**: 18000 control steps (300 s of scene time). The library's minimum-under-crossing plan is 26 lay steps plus 2 thread steps (`knotlab.plan.lay_plan`). That plan is one option, not a requirement.

## Rules

- Allowed: the `robot.*`, `motion.*` and `rope.*` tools, and the `sim.*` read tools.
- Forbidden (the validator rejects them): `sim.attach` / `sim.detach` (they weld the rope to a hand), `sim.check_success` and `sim.query` (they read the grader).
- `rope.state` / `rope.point` / `rope.layout` are the robot's "perception" in sim (privileged ground truth standing in for a top-down camera tracker) and may be used freely, including for closed-loop checks.
- Scripts: `def run(ctx, **inputs) -> dict | None` under `graph/scripts/`. They may import `knotlab` (`from knotlab import skills, plan, diagram`) and numpy.
- Every subgraph declares `"skill": "generic"`. The rope library is imported as code; it is not a registry bundle. Copy and adapt the wrappers in `skills/rope/scripts/` into `graph/scripts/` as needed.

## Skill library

See `skills/rope/SKILL.md` (code: `knotlab/skills.py` and `knotlab/plan.py`, readable at `/srv/home/bohanlyu/gap-knot/knotlab/`).
