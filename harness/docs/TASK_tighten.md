# Task: pull a loose overhand knot tight with two arms (单结收紧)

This is step 3 of the Howcast overhand-knot clip (`docs/img/video_storyboard.jpg`, ~33 s → ~36 s). A loose overhand knot (trefoil) lies on the table. The robot takes both tails and draws the knot up while keeping it an overhand knot.

## The simulated cell (RoboSimStudio on Newton; rope on SuperDex)

- **Robot**: the YAM bimanual pair. `arm_id 0` = left arm (world +y side), `arm_id 1` = right arm (world -y side). Parallel-jaw grippers; the fingertips reach 12.1 mm past the tool centre.
- **Frames**: WORLD coordinates in metres. The bench top is z = 0.75; world +x points away from the robot; +y is the robot's left.
- **Reach**: tool-down at bench height, about world x ∈ [0.35, 0.55] and y ∈ [-0.20, 0.20]. Higher up, the reach is wider. Attempt capped moves and judge them by their outcome.
- **Rope**: a 10 mm cord, ~0.9 m long, 8 mm node spacing, with self-contact. Node 0's tail is on the robot's right (-y); the last node's tail is on its left (+y).
- **Start**: a loose overhand knot ~13–17 cm across at world x ≈ 0.39–0.53, centred near y = 0, with 12 cm tails running out along ±y.

## Success and score

- **Success**: the rope is still an overhand knot (3 crossings, trefoil, either handedness), and the rope between its first and last crossing is at most 75% of what it was at the start. This must hold for 1 s with the hands off, within 9000 control steps (150 s of scene time).
  - `rope.state()["tied"]` reports exactly this.
  - The rope between the crossings is `max(s) - min(s)` over `rope.state()["crossings"]` (s_a, s_b).
- **Second objective**: cycle time, the scene seconds to success.

## What varies (read positions at run time)

- Rope stiffness: 0.5–2e5 Pa.
- Friction: `mu_cable` 0.6–1.4.
- Axial stiffness.
- The knot's size: 13–17 cm across.
- Where it lies: ±1 cm in x, ±1.5 cm in y, turned up to ±4.6°.

## The baseline graph (`graph/`), and what was measured

`scripts/tg_pull.py` works hand over hand. Each cycle:
1. Takes each tail `grasp_from_knot` along the rope beyond the outermost crossing. A grasp closer to the knot catches neighbouring strands in the open jaws and fails.
2. Grasps as follows: open to 45 mm, down to a tool centre 13.5 mm over the bench, a ramped 0.9 close, a slow straight lift. "Held" means the cord rose more than 5 mm.
3. Lifts both tails `lift` above the bench, so the knot hangs clear of the table, then draws them apart `distance` in `step` increments along ±y (turned `angle_deg` toward the robot).
4. Sets the knot down, lets go and re-reads the knot.

Measured on the nominal knot:
- Pulling along the bench without lifting tightened ~1% per cycle: the knot is dragged rather than drawn up.
- Lifting 8 cm then pulling 8 cm tightened ~8% per cycle at friction 0.25 or 1.0, and kept the trefoil.
- 0.503 m → 0.42 m of rope in the knot after 2 cycles; each cycle is ~20 s of scene time.

Parameters are in `graph/params.json`.

## Rules

- Allowed: the `robot.*`, `motion.*` and `rope.*` tools, and the `sim.*` read tools.
- Forbidden: `sim.attach` / `sim.detach` / `sim.check_success` / `sim.query`.
- Scripts are `def run(ctx, **inputs) -> dict | None` under `graph/scripts/`. They may import `knotlab` and numpy.
