# Robot-action caps

A cap records one bounded, rehearsed physical action after it works (for
example, carrying an end over one strand).  It contains ordinary `robot.*`
requests and compact rope-state guards.  It does **not** contain a simulator
snapshot, pinned particles, a pre-made knot, or a gripper latch.

Use `knotlab.cap.CapRecorder` around a manual/script rehearsal:

```python
from knotlab.cap import CapRecorder

rec = CapRecorder(ctx, "end_over_strand", requires_hold=True,
                  metadata={"operator": "agent", "seed": 7})
rec.run(lambda c: rehearse_end_over_strand(c))
rec.save("caps/end_over_strand.json")
```

Replay the cap on a fresh, straight-rope episode with `knotlab.cap.replay`.
Replay may call only normal robot tools; every motion must reach its target and
the rope's measured topological progress may not go backwards.  A passing cap
is evidence for that local action only.  The full graph still has to run from
the straight initial rope and pass the strict knot grader (three crossings,
tension, release, and hold time).

Review the JSON before using it.  `validate()` rejects any state mutation tool,
and the audit field records the call count and robot-only property.
