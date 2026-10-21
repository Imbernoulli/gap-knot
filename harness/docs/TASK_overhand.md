# Task: tie an overhand knot (单结) with two arms

The overhand knot is the first knot in every knot-tying tutorial, Chinese knotting included. From a cord lying on the board the robot must:

1. lay one half into a loop;
2. lay the other half over it;
3. pass that half's free end UNDER the loop's strand while the other hand lifts the strand (a bimanual step);
4. lay the tail;
5. pull both tails to tighten.

## The simulated cell (RoboSimStudio on Newton; rope on SuperDex)

- **Robot**: the YAM bimanual pair. `arm_id 0` = left arm (world +y side), `arm_id 1` = right arm (world -y side). Each has a parallel-jaw gripper with a 95 mm span; the fingertips reach 12.1 mm past the tool centre.
- **Frames**: every tool takes and returns WORLD coordinates in metres. The bench top is z = 0.75; its centre is world (0.6, 0, 0.75). World +x points away from the robot; +y is the robot's left.
- **Reach**: tool-down at bench height, IK reliably reaches world x ∈ [0.35, 0.55] and y ∈ [-0.20, +0.20]. This is guidance: attempt capped moves at the edge and judge them by their outcome.
- **Rope**: a 10 mm cord, 0.76 m long, 96 nodes 8 mm apart, with self-contact. Its stiffness comes from a Real2Sim of the source clip on RoboSimStudio's live page (`video/overhand/REAL2SIM.md`): E = 2e4 Pa matched the clip's settled loose knot. Friction is high (`mu_cable` ~1.0: SuperDex uses one friction for strands, pads and bench, and a grip needs it). Arc length s runs from node 0 (s = 0, the robot's RIGHT end) to the last node (s ≈ 0.76, the LEFT end).
- **Start**: the cord lies straight along world y at x ≈ 0.53 for |y| ≤ 0.20, both ends bent toward the robot to x ≈ 0.34. Everything lies flat, at or inside the edge of reach. This is the Howcast clip's first frame (the rope straight on the table).
- **Target**: `rope.layout()` gives `target_nodes`, a reference knot about 15 cm across for this episode. Its tails run along y on the start's line; the knot sits toward the robot of it (world x ≈ 0.41–0.53). Success does NOT require matching this shape, only the topology (below).

  The 3 crossings, numbered by where strand A (the earlier along the rope) meets them:

  | crossing | strand A at s | strand B at s | strand A is |
  |---|---|---|---|
  | 1 | 0.126 | 0.412 | under |
  | 2 | 0.194 | 0.561 | over |
  | 3 | 0.344 | 0.629 | under |

  Walking from node 0 the Gauss code is `1U 2O 3U 1O 2U 3O` (the trefoil).

## Success and score

- **Success**: the rope's diagram seen from above is an overhand knot of either handedness, held for 1 s, before the step budget (12000 control steps, 200 s of scene time) runs out.
  - An overhand knot here means three crossings met along the rope as 1 2 3 1 2 3, alternately over and under. The reference above is `1U 2O 3U 1O 2U 3O`; its mirror `1O 2U 3O 1U 2O 3U` counts too.
  - `rope.state()["tied"]` reports exactly this.
- **Second objective**: cycle time, the scene seconds to success. A control step costs ~0.1 s of wall time.

## What varies between episodes (read positions at run time)

- Rope stiffness: ×0.5–2.
- Friction: `mu_cable` 0.6–1.4.
- Axial stiffness: 200–450 N.
- Where the cord lies: its line shifted ±1 cm in x and ±1.5 cm in y, turned up to ±4.6°, with a lateral wiggle up to 6 mm.

## What the baseline graph does now (rehearsal evidence, 45 episodes)

The baseline lays the rope point by point to the reference knot (`knotlab.plan.lay_plan`), with one `thread` step. It runs all its phases, but 0 of 45 episodes tied the knot:
- **Most end with 0–2 crossings.** The loop and the over-crossing do not reliably form, because placements land ~2–3 cm off.
- **The thread step never once put the end UNDER the lifted strand.**
- **The two episodes with 3 crossings were wrong**: `1O 2O 3O 1U 2U 3U` (the end laid over everything) and `1O 2U 3U 3O 2O 1U` (a loop inside a loop).

How a person ties it in the clip (`docs/img/video_storyboard.jpg`, the Howcast clip):
1. Make a loop by crossing the end over the standing part.
2. Pass the end through the loop from below.
3. Pull both ends.

Any method that ends in the overhand topology is fine; the reference shape is only a guide.

## The baseline graph and the laying plan

`knotlab.plan.lay_plan(rope.layout(), start_d=0.0)` lays half A (s < 0.30) outward from the middle into the loop, then half B outward. Only one crossing then has to be made UNDER: B at s ≈ 0.44 under A's strand at s ≈ 0.19. That is a `thread` step (`knotlab.skills.thread_end`): the left hand lifts the strand, the right hand pushes the end under. Every other crossing is made by carrying OVER.

Parameters live in `graph/params.json`.

## Measured behaviour

Read `skills/rope/SKILL.md` for the skills and their measured numbers:
- pinch (jaws across the cord, ramped 0.9 close), carry at ≤ 35 mm, place;
- lift_strand / thread_end;
- pull.

Known risks:
- **Threading is the least proven skill.** It needs the end lying flat, and the pushed end can stop short of the arch.
- **Spring-back**: released cord springs back 1–3 mm, more when dragged cord recoils.
- **Pinching near a crossing** is weaker; pinch 2–3 cm away from crossings.
- **Read `rope.state` only after the hands are clear** and the cord has settled.

## Rules

- Allowed: the `robot.*`, `motion.*` and `rope.*` tools, and the `sim.*` read tools.
- Forbidden (the validator rejects them): `sim.attach` / `sim.detach`, `sim.check_success` and `sim.query`.
- Scripts are `def run(ctx, **inputs) -> dict | None` under `graph/scripts/`. They may import `knotlab` (`from knotlab import skills, plan, diagram`) and numpy. Code: `/srv/home/bohanlyu/gap-knot/knotlab/`.
