# Helper-arch under-pass probe (2026-10-22)

This is an isolated `start_shape=pre_thread` diagnostic, not a complete knot-tying result. The loop/crossing was constructed before the robot acted. The probe tests only the proposed primitive: lift one strand as an arch, pass the other end underneath, then release.

| E (Pa) | Seed | Result | Final crossings | Final code | Wall time |
|---:|---:|---|---:|---|---:|
| 100,000 | 0 | failure | 2 | `1U 1O 2O 2U` | 141.0 s |
| 150,000 | 1 | failure | 2 | `1U 2O 1O 2U` | 124.6 s |

At `E=100,000`, the moving tip did pass the under-entry by 12.4 mm and the transient `under_made` check became true. The anchored material point then drifted 37.8 mm during the pass and 135.2 mm during the attempted third crossing; after release and three seconds, only two crossings remained. The harder cord did not fix the route: its tip moved backward by 76.0 mm and the under gate did not become valid.

These measurements support fixing the anchor-preserving motion and exit path before changing stiffness further. Exact results and final geometries are in `summary.json` and the per-episode files. Neither episode counts as a straight-rope success.
