# Lobe-laying probes: results

Probe graph `graphs/lobe_probe` (its `params.json` holds an op list; `scripts/lobe_lib.py` is the experimental
implementation), swept on Daytona with `tools/lobe_sweep.py` (≤ 12 sandboxes). Metrics are taken from the last
saved rope frame against `task.layout(variant)` with `knotlab.skills.lobe_metrics`: winding of the lobe's range
(d = 0.082..0.382 from the middle) about its pin (target 0.89 B / 0.90 A), median node error over that range, and
whether every crossing is one the target has (`consistent`: same strands within 10 cm of arc, same over/under,
met in the target's order). Success = winding ≥ 0.8 and median ≤ 15 mm and consistent.

**Overall: 71 episodes, 0 met the full criterion. 4 ended wrapped (≥ 0.8) with only target crossings (all half B,
the lift-over recipe), best end median 17 mm. Validation across ≥ 10 variants per half was not reached (work stopped
at the coordinator's request).**

Ops: `F` = take the lobe's far end (d 0.38) and carry it to its target round the robot side of the *other* side pin;
`lift` = take the lobe's apex (d ≈ 0.28), lift to TCP 80 mm, carry it over its own pin, set it down on its target +
radial overshoot; `clear` = push the other half's end run ≥ 35 mm out of the lobe's way; shaping = further single
placements (d values listed).

| sweep | config (half, ops) | variants | winding (end) | median err | crossings | note |
|---|---|---|---|---|---|---|
| s1 | B points d .20/.24/.28/.32/.36, 35 mm carries | nominal | +0.03 | 33 mm | 4 extra | baseline: no wrap; carry slipped 81 mm, spring-back 72 mm |
| s1 | A points (mirror) | nominal | −0.11 | 35 mm | 1 flipped, 3 extra | |
| s1 | B J, F (straight route), lift 0.10 | nominal | +0.81 | 52 mm | 6 extra | first wrap; F route dragged B's tail round pin_left |
| s1 | B lift directly from start | nominal | +0.05 | 33 mm | 3 extra | tail hooked on pin_left |
| s2 | **B F(route) + lift r 12 mm, no shaping** | nominal | **+0.85** | **17 mm** | **6, 7, 8 all right** | 36 s sim; release lifted A's end and the homing arm dropped it off the board |
| s2 | B F + lift (apex d .26) + shaping .20/.24/.33 | nominal | +0.83 | 20 mm | consistent | |
| s2 | B F + lift + shaping (r 5 / 20 mm, slow, lift 0.10) | nominal ×4 | −0.17..+0.87 | 24–92 mm | 1–4 extra | wrap was +0.85 after the lift-over in all 4; near-pin shaping pinches lifted it off the pin in all 4 (one re-wrapped by chance) |
| s2 | **A F + lift (mirror)** | nominal | +0.93 mid-run, +0.83 end | 12 mm mid-run | **2, 4, 6 all flipped** | the chirality obstruction, measured |
| s3 | B clear + F + lift (jaws also limited by nearby strands) | nominal, 50004/7/8 | −0.17, +0.96, +0.39, +0.92 | 13–86 mm | 0–3 extra | 22 mm jaws: cord rode up the pads 70 mm; apex pinches missed |
| s4 | **B clear + F + lift (pin-limited jaws, centred lift-off, auto lift point)** | nominal / 50004 / 50007 / 50008 | **+0.95** / +0.88 / +0.01 / +0.01 | 22 / 30 / 71 / 165 mm | cons / 1 extra / – / – | 50008 (mu 0.68): F and apex slid 18–25 cm through the pads |
| s4 | same + shaping J .19, F .37 (, run .12) | same ×2 | −0.11..+0.77 | 30–183 mm | mostly extra | 0/8 ended wrapped; the shaping pinches/releases undid wraps that had formed |
| s5 | F at d 0.36 (further from pin_top) | same | +0.04, +0.75, +0.98, 0.00 | 22–175 mm | extra | F carry slipped 72 mm; release popped the cord 30–77 mm up |
| s5 | grip fraction 1.0 | same | −0.19, −0.04, **+1.01**, **+0.84** | 31–133 mm | 50008 consistent | firmer grip fixed the low-friction slip, not the rest |
| s6 | fraction 1.0 + re-grasp on slip | nominal, 50000/1/4/7/8 (11 runs) | −1.06..+0.99 | 28–148 mm | 0/11 consistent | re-grasps re-wrapped the cord the wrong way |

Sandbox time: 71 episodes, 5.7 sandbox-hours of run time (~$1.2 with set-up), 2 vCPU sandboxes, pool `skill`.

## Pictures (top view, robot's left on the left; magenta = half A's end, yellow = half B's end)

- Best: `lobe_img/best_B_liftover_nominal.png` (s4, B clear + F + lift: winding 0.95, 22 mm, only crossings 6, 7, 8).
- `lobe_img/B_liftover_noshape_nominal.png` (s2: 0.85, 17 mm; note A's end lifted by the release and thrown off the
  board at the end).
- `lobe_img/A_mirror_flipped.png` (s2, the same recipe on half A: wraps 0.93 but crossings 2, 4, 6 flipped).
- Worst: `lobe_img/worst_B_points_baseline.png` (s1, point-by-point: no wrap) and
  `lobe_img/worst_B_regrasp_wrongwrap_50000.png` (s6: re-grasps wrapped the lobe −1.06 turns).
