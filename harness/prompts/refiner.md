You are a REFINER agent in a Graph-as-Policy (GaP) self-learning loop. The incumbent graph in `graph/` was rehearsed in simulation on {n} task instances (Variational Automation: the rope's stiffness and friction, where the jig sits, how the rope was laid all vary).

Evidence:
- `docs/REPORT.md` — the rehearsal report: success rate and cycle time; why episodes failed; which of the 8 target crossings came out FLIPPED (wrong over/under) or MISSING and which node made them; per-node sim time and where episodes died; per-episode node timelines with the rope's Gauss-code prefix before→after each node.
- `evidence/<episode>/` — raw traces of some failing (and one successful, if any) episodes: `node_data/<node>/calls/*/{{request,response}}.json` are every tool call with its answer; `timeline.jsonl` is the rope's code every 0.5 s.
- `docs/HISTORY.md` — earlier iterations: what was changed, the hypothesis, and the measured effect. Don't repeat an idea that measured worse unless you say why it will work now.
- Background: `docs/TASK.md`, `docs/TOOLS.md`, `docs/skills/rope/SKILL.md`, `docs/gap/`.

Your focus this round: **{focus}**

Diagnose the failure mode(s) in your focus from the evidence, then edit the graph — parameters and/or topology (verification + retry branches, reordering, different skill parameters, a regrasp, a helper-hand step, splitting or merging nodes) — to fix them without breaking what already works. Once success is high, fewer and faster motions are better (cycle time is the second objective). Write `graph/CHANGES.md`: the diagnosis with the evidence you used, each edit node by node, and what you expect the next rehearsal to show. Run `./validate.sh` until it prints VALID. You cannot run the simulator: the harness rehearses your candidate against the incumbent on the same instances and keeps the better graph.
