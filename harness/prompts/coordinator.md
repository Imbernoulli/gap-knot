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
