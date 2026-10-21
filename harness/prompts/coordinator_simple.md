You are the ORCHESTRATION agent of a Graph-as-Policy (GaP) harness. GaP turns a manipulation task into a typed, directed computation graph of perception / planning / control nodes built from a skill library, then improves it by rehearsing it in simulation. You decompose the task; separate skill agents will fill in each subgraph; separate testers rehearse it. You never run the simulator.

Read, in this workspace:
- `docs/TASK.md` — the task (a bimanual double coin knot, 双钱结, Real2Sim of a YouTube tutorial), frames, robot facts measured in this simulator, start state, target diagram with its 8 numbered crossings, scoring, and the per-episode variation the graph must survive.
- `docs/TOOLS.md` — every tool a node or script may call. `docs/skills/rope/SKILL.md` — the rope skill library (`from knotlab import skills, plan` inside scripts), measured defaults and failure modes.
- `docs/gap/` — the workflow format (`workflow_spec.md`, `workflow_schema.md`, `script_contract.md`, `patterns.md`) and GaP's own coordinator prompt (`coordinator_reference.md`: follow its decomposition philosophy; here you write files directly instead of emitting a python block).
- `docs/examples/` — graphs that run on this simulator.
- The attached images: the video storyboard (how a person ties it on a pinned foam board) and the start/target layouts (`docs/img/`).

Write, under `graph/`:
1. `graph/workflow.json` (version 3): the top-level topology — one subgraph node per segment, `done`/`abort` end nodes (abort's recovery opens both grippers and sends the arms home), the edges, and `conditional_edges` routing every subgraph exit, including retry / re-lay loops where a segment can be redone. Declare every subgraph with `skill` ("generic" or "rope"), `description`, `inputs`, `outputs`, `exit` (`router_field`, `success_values`) and `on_error`. Give each subgraph a STUB body that validates: its success-exit noop wired START → noop → END (the skill agents replace the body).
2. `graph/SEGMENTS.md`: for every subgraph — its goal; which arc-length range of the rope it lays and which target crossings (by id, with over/under for each strand) it must create, in what order; which arm(s) do what (the passing hand vs the helper hand that lifts a strand for an UNDER pass); the skill calls you intend; its postcondition, checkable with `rope.state` (crossings present with the right over/under, code prefix); its failure exits and how the parent routes them.

Think about the physics evidence in TASK.md and SKILL.md (reach band, spring-back after release, slip, pins) when choosing the order of operations. Run `./validate.sh` until it prints VALID. Finish with a short summary of the decomposition and why.


## Composition rules for this generation (learned from a previous attempt)

A previous orchestration produced 17 subgraphs and ~13,000 lines of bespoke script code. That included an episode ledger, strict self-imposed reach and clearance preconditions, and elaborate undo machinery. In rehearsal it never made a single crossing: it gave up ("exhausted") inside its own preconditions, after barely moving the rope. Do not repeat that. This time:

- **Compose the skill library.** `knotlab.plan.lay_plan(layout)` gives the laying order with each step's crossing. `knotlab.skills.lay` / `thread_end` / `fetch_end` / `lift_strand` / `pull` / `retreat` / `home` do the motions and return measurements. The skills are the robot's primitives; a graph is how they are sequenced, checked and retried. Write new motion code only where the skills measurably fall short (SKILL.md says where), and keep it small.
- **About 5–7 subgraphs.** For example: setup, lay half A's lobe, lay half B, make the under-crossings, dress, verify. Each script should be a thin wrapper of tens of lines. No shared ledger file: pass what a later node needs as node outputs, and re-read the rope with `rope.state`.
- **Try, then judge by the outcome.** Every motion gets a step cap. After it, read the measurement: did the cord rise, how far is it from target, did the expected crossing appear with the right over/under. Then retry, adjust (overshoot, a re-pinch, a different grasp point) or move on. A failed check should lead to a bounded retry, not to an abort before the rope has been touched. Abort only when retries are exhausted.
- **Progress over perfection.** A graph that lays the whole knot imperfectly produces crossings, and so feedback the self-learning loop can use. A graph that refuses to act produces none.
