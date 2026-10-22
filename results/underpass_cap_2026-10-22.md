# Prepared-loop under-pass cap (2026-10-22)

This is a **diagnostic segment**, not a from-straight tying result. The initial state is `start_shape=pre_thread`: one loose overhand crossing is already present. The cap holds the A strand at the target X2, takes the remote tail, approaches from the far side, passes under, carries toward X3, and then separates tip release from helper release. Every stage records crossing code, tip drift, helper-anchor drift, and post-release state.

The experiments used Daytona 8 vCPU / 16 GiB sandboxes, two workers, and `mu_cable=1.0`; only `young_modulus` changed.

| E [Pa] | stage | crossings / code | diagnostic |
|---:|---|---|---|
| 1.0e5 | helper hold | 1 / `1U 1O` | anchor drift 2.1 mm; tip rise 18.3 mm |
| 1.0e5 | under pass | 2 / `1U 2O 1O 2U` | `c2=ok`, transient `under_made=true`; tip past 12.4 mm; anchor drift 37.8 mm |
| 1.0e5 | X3 carry | 2 / `1U 2O 1O 2U` | anchor drift 135.2 mm; tip carry lag 29.5 mm; X2 no longer retained by the segment score |
| 1.0e5 | 3 s after release | 2 / `1U 1O 2O 2U` | `under_made=false` |
| 1.5e5 | helper hold | 1 / `1U 1O` | anchor drift 1.7 mm; tip rise 18.4 mm |
| 1.5e5 | under pass | 1 / `1U 1O` | motion reached, but no X2 (`c2=missing`); tip ended 76.0 mm short; tip drift 241.6 mm |
| 1.5e5 | X3 carry | 1 / `1U 1O` | carry lag 253.6 mm; anchor drift 142.4 mm |
| 1.5e5 | 3 s after release | 2 / `1U 2O 1O 2U` | `under_made=false` |

The softer cord can briefly produce the desired X2 relation, but the helper anchor moves beyond the 25 mm drift gate before the far-side action and the relation is lost after release. The harder cord's pass is reachable but misses the crossing and slips badly. Neither run is a knot success; neither run begins from a straight cord.

Code and exact rerunnable graph: `graphs/underpass_cap/`. Raw Daytona episodes are kept under the ignored `runs/underpass_cap_v2/` directory; the manifest records the 8-vCPU shape and paired variants.
