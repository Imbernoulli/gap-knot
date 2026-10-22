# Threading probes, 2026-10-22

All eight episodes ended without retaining the required under-crossing or forming an overhand knot.
These trials start from a constructed `pre_thread` loop, not a straight rope.
They are negative results, not a knot-tying demonstration or a new held-out test.

- Fixed target versus feedback: two matched instances, nominal and current sampled seed 5. Both graphs fail 0/2. Feedback remeasures the crossing after placing the end and lifting the strand. The first crossing is already lost during end preparation.
- Anchor first: the nominal instance, with the helper pinching the first crossing before end preparation. Tool heights 16 and 28 mm, helper lift 75 mm. Both fail, 0/2.
- Grasp the end before raising the helper: the nominal trial made the required under-crossing and retained crossing 1 immediately after the pass, but lost the structure on release. A subsequent 8 cm draw while retaining the grip also failed. These are one-instance diagnostics, not paired evidence of improved reliability.
- Every final diagram has determinant 1 under the diagram-closure convention.

`episodes.json` records each complete variant. `fixed_target_graph/` is the archived 75° original probe; `anchor_graph/` is the new anchor-first probe with height 16 mm (change params `z` to 0.028 for the other case). The feedback graph is `../../graphs/thread_feedback/`.

Run one trial using RSS's interpreter, as described in the repository README:

```sh
tools/rss.sh -m knotlab.episode --graph results/threading_2026-10-22/anchor_graph --variant '{"knot":"overhand","board_x":-0.07,"start_shape":"pre_thread"}' --out runs/thread-anchor --max-steps 5000 --frames 15
```

The success predicate for the tightening experiment changed after these probes: the probes' graders did not yet enforce tool clearance. None passed even their weaker knot-tying condition.

`grip_before_lift_graph/` and `draw_before_release_graph/` archive those two one-instance follow-ups. `phase_report` in the JSON preserves the intermediate under-crossing, so it is distinguishable from a settled success.
