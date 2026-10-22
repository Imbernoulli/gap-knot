# Results checkpoint — 2026-10-22

**An ordinary overhand knot from a straight rope is still unsolved.** Tightening
starts with a knot already tied; none of the tightening successes demonstrates
forming that knot. The earlier from-straight held-out test was 0/16.

## Tightening: corrected paired evaluation

| Existing graph | Runtime successes | Mean simulated time among successes |
|---|---:|---:|
| [G0 baseline](../graphs/tighten_v0) | 0/10 | — |
| [G1 / iter0c](../graphs/tighten_g1) | 5/10 | 105.92 s |
| [G2 / iter1a](../graphs/tighten_g2) | 6/10 | 65.42 s |

All three graphs ran exactly the same variants, seeds **10–19**. These instances
were used in the earlier training run. This is a paired re-evaluation of existing
graphs, not a new learning run or a held-out generalization test. Each sample is
only ten episodes. The time means use different successful subsets, so they are
not a paired speed comparison.

The grader is `2026-10-22-hands-clear-v1`. A runtime success requires all of the
following continuously for one simulated second:

- An exact three-crossing trefoil Gauss diagram, either handedness.
- Rope material inside the knot at most 75% of its initial length.
- Every rope node at most 5 cm above the bench.
- Both measured jaw openings at least 5 cm.
- Both tool centres at least 6 cm from every rope node.

The runtime scorer converts world positions to bench coordinates. The 5 cm height
threshold accommodates stacked strands in a resting knot (observed 28–42 mm).
This checks tool clearance, not absence of all possible robot/rope contact or
long-term stability after the one-second hold.

The old 0/10 → 9/10 curve is withdrawn: it accepted knots held in the air. Later
diagnostic batches mixed world and bench height, or omitted hand clearance, and
are also superseded. Only this complete 30-episode batch is reported above.

- [Per-episode audit JSON](tighten_hands_off_2026-10-22.json): variants, runtime
  verdicts, grader versions, final geometry scores, and last sampled hand data.
- [Final rope coordinates](tighten_hands_off_2026-10-22.npz): one world-coordinate
  array per episode, keyed by graph and episode ID; SHA-256 is in the JSON.
- All 30 arrays are finite and come from exact `rope_final.npy` files. All eleven
  runtime successes also pass final trefoil, length, height, determinant-3, and
  sampled hand-clearance checks. Determinants use diagram closure. Sampled hand
  diagnostics alone do not establish the continuous hold; that verdict comes
  from the runtime, and the audit never upgrades final frames to success.
- Successful seeds: G1 = 12, 13, 17, 18, 19; G2 = 11, 12, 13, 15, 17, 18.

Reproduce with a Daytona-SDK interpreter, from the repository root:

```sh
python tools/reevaluate_tighten.py --out runs/tg_hands_off --n 10 --offset 10 --workers 6 --export results/tighten_hands_off_2026-10-22.json
```

The evaluation used at most six 2-vCPU/4-GiB sandboxes. Ownership-filtered cleanup
was checked after completion: zero project sandboxes remained. Raw rollouts and
regenerated analyzer reports are in the ignored local `runs/tg_hands_off/eval/`.

## Threading: eight final failures

[The threading archive](threading_2026-10-22/README.md) contains all eight variants,
source graphs, final results and intermediate phase diagnostics. These probes
start from a constructed loop (`pre_thread`), not a straight rope.

Fixed-target versus feedback passes failed on both matched instances. Anchoring
the first crossing before preparing the end also failed at both tested heights.
Grasping the end before lifting the helper retained the first crossing and made
the intended under-crossing immediately after the pass, then lost the structure
on release. Drawing the end another 8 cm also failed. Every final diagram has
determinant 1 under diagram closure.

The next unresolved step is maintaining the crossing structure through
follow-through and release, then incorporating it into a from-straight policy.

## Report

[The current webpage](../site/index.html) includes the corrected crossing render,
an invalid historical airborne example labelled as such, and fresh baseline
(seed 10, failure at 71.42 s) and G2 (seed 12, success at 62.18 s) replays.
These renders show simulated rope and tool-centre markers.

The previous Claude Artifact has not been updated. For a single local HTML file
with embedded images and videos, run:

```sh
python tools/pack_site.py --out runs/report.html
```
