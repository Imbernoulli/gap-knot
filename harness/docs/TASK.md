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
- **Rope**: a 10 mm cord, 1.32 m long, 162 nodes about 8 mm apart, with self-contact (strands cannot pass through each other). It is soft (Young's modulus ~1e5 Pa), with friction coefficient ~0.5 at the pads and between strands. It rests on the bench (axis at z ≈ 0.755). Arc length s runs from node 0 (s = 0) to the last node (s ≈ 1.32).
- **Board and pins**: a foam board (drawn only). There are three pins (posts 2.5 mm in radius, 30 mm tall): `pin_top`, `pin_left`, `pin_right`, each in the eye of one lobe of the finished knot. Fingers that land on a pin collide with it. A pin holds a loop laid round it.
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
- **Second objective**: cycle time, the scene time to success. The step budget is 9000 control steps at 60 Hz (150 s of scene time). A simulated control step costs ~0.11 s of wall time, so motion is expensive: plan economical moves.

## What varies between episodes (the graph must cope; read positions at run time)

- Rope stiffness: ×0.5–2.
- Friction coefficient: 0.15–0.40.
- Axial stiffness: 200–450 N.
- Where the jig sits: centre shifted ±1 cm in x and ±1.5 cm in y, turned up to ±4.6°. The pins move with it, each also jittered up to 3 mm.
- How the rope was laid: bight width, how far the halves run down (12–15.5 cm) and out (14–19 cm), and a lateral wiggle up to 6 mm.

## Measured behaviour (from probes in this simulator)

- **Pinch**:
  - Jaws across the rope's tangent: `robot.grasp_frame(close_heading_deg = rope heading + 90, arm_id=...)`.
  - Descend with `robot.go_to_pose(..., max_steps=200)` to TCP z = 0.766 (16 mm over the bench). The fingertips are then just below the rope's equator. Lower stalls on the bench.
  - Close with `robot.set_grip(fraction_closed=0.9, ramp_steps=48, settle_steps=30)`. A fully closed snap (`preset="close"`), or lifting before the jaws settle, lets the rope slip out.
- **Carry**:
  - Use `robot.go_to_pose_cartesian(pose, arm_id, speed_scale=0.25)` (~7 cm/s). With the pinch above it holds: a 6 cm carry moves the pinched point 5.7 cm.
  - The default `robot.go_to_pose` servo moves ~16 cm/s; carrying at that speed lost most of the motion to slip.
  - The pinched point rides ~14 mm below the TCP.
- **Release**: after letting go the rope springs back toward its old shape by 1.2–1.6 cm. Plan an overshoot, or pin it.
- **Unbounded moves**: a `go_to_pose` without `max_steps` that cannot converge burns 1000 steps (~2 minutes of wall time). Always bound it.

## Rules

- Allowed: the `robot.*`, `motion.*` and `rope.*` tools, and the `sim.*` read tools.
- Forbidden (the validator rejects them): `sim.attach` / `sim.detach` (they weld the rope to a hand), `sim.check_success` and `sim.query` (they read the grader).
- `rope.state` / `rope.point` / `rope.layout` are the robot's "perception" in sim (privileged ground truth standing in for a top-down camera tracker) and may be used freely, including for closed-loop checks.
- Scripts: `def run(ctx, **inputs) -> dict | None` under `graph/scripts/`. They may import `knotlab` (`from knotlab import skills, plan, diagram`) and numpy.

## Skill library

See `skills/rope/SKILL.md`.
