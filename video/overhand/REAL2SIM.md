# Howcast overhand rope: scene and calibration

The delivered default in `scene.py` is the straight strand at 16 s, on a wooden tabletop with a parked `yam_bimanual` at the near edge. The rope is a **10 mm diameter, 1.05 m long SuperDex rod**, with 177 nodes, `self_contact=True`, effective bending Young's modulus **20,000 Pa**, `mu_cable=0.05`, and axial stiffness **300 N**. Curved presets are independent initial conditions; they are not baked stress-free knots.

This report is saved in `examples/live/REAL2SIM.md` because the requested `video/overhand/REAL2SIM.md` destination is outside this session's writable roots. No file was written there.

## Scene parameters

All object poses below are metres in the table's work-surface frame; add 0.75 m to z for world coordinates. Camera coordinates are world coordinates.

| Parameter | Final value |
| --- | --- |
| Work surface height | 0.750 m; assumed conventional table height |
| Table half extents / centre | `(0.55, 0.75, 0.025)` / `(0, 0, -0.025)` |
| Table base RGB / roughness | `(0.47, 0.35, 0.24)` / 0.8 |
| Wood finish | `assets/table_wood.jpg`, sampled from the clip's clean tabletop; UV scale `(26,26)` |
| Plank seams | x = -0.062 and +0.105 m, 1.2 mm wide; dark brown, visual-only |
| Table friction | 0.5; the SuperDex rod's actual friction is controlled by `mu_cable` |
| Robot | shipped `yam_bimanual`, mount `(-0.55, 0, 0.022)`, default home joints, facing +x |
| Rope radius / nominal length | 0.005 / 1.050 m |
| Nodes / mean spacing | 177 / 5.966 mm |
| Young's modulus / axial stiffness | 20,000 Pa / 300 N |
| `mu_cable` / `friction_falloff` | 0.05 / 0.001 m/s |
| Density | 350 kg/m³, assumed |
| Rest shape | `stress_free="straight"`, including curved initializations |
| Backend | SuperDex, 2 substeps, `feedback="off"` |
| Contact | `self_contact=True`, `skin=False`, `contact_gap=0.005`, `contact_penalty=4e8`, backend `mirror_padding=0.004` m |
| Rope colour / finish | `(0.025,0.075,0.20)`; clip-sampled blue sheath texture, roughness 0.85 |

The straight centreline is parameterized by arc coordinate s in [-0.525, +0.525]: `x=s/sqrt(1+0.018²)`, `y=0.018*s/sqrt(1+0.018²)-0.0049`, initial `z=0.0053`. It settles onto the table. Its endpoints are about `(-0.5249,-0.01435,0.0053)` and `(+0.5249,+0.00455,0.0053)`. The near table edge is negative x, at the bottom of the phone image. The arms lie outside the macro framing.

| Camera / reference | Eye | Look at | Up | Vertical FOV / image |
| --- | --- | --- | --- | --- |
| `phone` / 16 s | `(0,0,0.827)` | `(0,0,0.750)` | `(1,0,0)` | 45° / 640×480 |
| `loop` / 24 s | `(0.040,0,1.117)` | `(0.040,0,0.750)` | `(0.819152,-0.573576,0)` | 45° / 640×480 |
| `loose` / 33 s | `(-0.021,0,0.955)` | `(-0.021,0,0.750)` | `(1,0,0)` | 45° / 640×480 |
| `wide` / 36 s | `(0.010,0,1.245)` | `(0.010,0,0.750)` | `(1,0,0)` | 45° / 640×480 |

These are equivalent virtual projections for the clip's cuts and zoom changes. A phone's physical height, focal length and crop cannot be separately recovered from these images. In particular, the 77 mm macro camera clearance is an equivalent projection, not a claim that a phone was physically this close.

## Measurements that set scale

The 10 mm diameter is the requested YAM-compatible metric scale. The clip provides ratios, not an independent metric ruler. The straight 16 s strand is roughly 80 pixels across; the 24 s loop is about 220×300 pixels with a roughly 16 pixel strand; the 33 s loose lobe is about 380×300 pixels with a roughly 30 pixel strand; the 36 s compact knot is about 54–60×35–40 pixels with a roughly 12 pixel strand. Thus the loop is roughly 0.14×0.19 m, the loose lobe 0.13×0.10 m, and the compact knot roughly 45–50×30–33 mm at this scale. Cameras were calibrated separately for those shots.

The straight frame hides both ends and cannot measure length. At the calibrated 24 s projection, the traced centreline inside the image covers **0.8403 m**, including the assumed path behind the hand. A conservative exposed subset (`v<265 or u>355`, inside frame) covers **0.5723 m**. I chose **1.05 m** to include the hidden/off-frame tails consistently in every preset. The extra length is an explicit assumption; 1.05 m is not a measured unique length. `fit_length` changes the unseen tails to retain this length instead of changing material length between shots.

The 8 mm and 12 mm controls held the camera fixed. Their pairs show the former too thin and the latter too thick; the final 10 mm strand lies between them. Colour-mask area alone is insufficient: pale braid flecks fragment the blue mask, and bounding-box width also includes strand tilt.

## Method and measurement provenance

I followed the repository's `real-to-sim` method: the straight 16 s pair, look sheet and gate came first (log 7–8, `first_04`, silhouette IoU 0.8769, gate PASS, measured node drift 0.0592 mm/60 steps). Only then were loop and knot initializations tried. Final parameters were rechecked after changing contact and friction.

**CLI limitation:** sandbox socket connections/binds returned EPERM, so `./ctl status`, snapshot/compare/check/log could not reach the running page. An attempted `./ctl up` also unlinked its socket pathname after misidentifying it as stale; the original page continued rebuilding on save. Restoring ordinary CLI access requires restarting that page outside this sandbox. These results must not be described as successful CLI commands.

Instead, `scene.py` binds its own `sim` as allowed by REFERENCE.md. An optional `probe.json` calls `live_probe.capture` inside the existing page's rebuild. That uses the repository's own `checks.compare_still`, picture writer, gate and log functions. Every log entry states `source="live save (socket denied)"`, carries the Python scene hash and the probe configuration. `run/log.txt` is generated with the same `checks.log_report` used by `./ctl log`. Probe configuration is included explicitly because the library's scene hash does not include JSON.

**Rendering limitation:** this CPU environment's native collider snapshot rendered the foreign rod as beads and its particle segmentation failed on a sentinel ID. `visual_probe.py` is a rendering adapter: native Newton CPU tracing of the original rigid poses plus a continuous tube swept around the actual live SuperDex nodes, with the clip-sampled finishes. It changes no physics or node positions. This is diagnostic rendering, not a successful native RTX/path-traced `ctl snapshot`. Tube winding is corrected for the CPU tracer. Finish UV orientation/tint in the browser or RTX backend may differ from this adapter.

The diagnostic silhouette mask uses blue/red ratio and HSV, closing radius 6 pixels, retaining significant components. It preserves loop holes. Each JSON also records the unmodified blue-key score. Full-frame masks include blue titles and visible tails and omit hand-occluded rope. Exposed-region comparisons are reported separately with their exact rectangles. They do not replace the full-frame scores.

## Material and contact selection

- **Radius 5 mm:** YAM scale plus first-frame apparent width; the 4 and 6 mm radius controls bracket the observed strand. Loop/knot sizes in diameters provide the cross-check above.
- **Young's modulus 20,000 Pa:** with the same valid loose-knot initialization and contact settings, E=2,000,000 Pa gave IoU 0.3542 and 14.0146 mm maximum planar movement after 3 s. E=20,000 Pa at mu=0.25 gave IoU 0.7044 and 3.9302 mm planar movement. The high-E lobe narrowed and shifted visibly. Final E with mu=0.05 retains the loop and loose knot; see final readings below. This is effective bending stiffness, with axial stiffness overridden separately, not an identification of the fibres' tensile modulus.
- **Friction 0.05:** a working compatible value after trying 0.0025, 0.05 and 0.25. The loose knot held three crossings even at 0.0025, so the requested resting image does not identify a friction lower bound. With identical 120 mm outward travel per end over 2 s, the 0.25 and 0.0025 trials reduced knot length from about 483 mm to 204 and 185 mm after a 3 s hold. The 0.05 trial with a 5 s hold reached 179 mm, retained three crossings, had +0.036 mm self-clearance, 0.171 mm table penetration and 0.609 mm subsequent drift. Its exposed-region IoU was 0.5938. Different hold durations prevent treating those scores as a controlled ranking of friction. This selects a useful representative value, not a unique estimate from the video.
- **Contact settings:** the early skin configuration allowed roughly 3–4 mm of table penetration. `skin=False` plus mirror padding 4 mm brings actual axis height close to the 5 mm radius. Zero contact gap permitted 4.73 mm of strand overlap and was rejected despite better image overlap; the default 10 mm gap over-separated strands. A 5 mm gap produces approximately touching strands with submillimetre numerical contact error.

All curved presets start with positive self-clearance and a straight natural rest shape. Loop and loose presets trace the clip centreline, unproject through their camera, lift alternating over-crossings, and allow gravity/contact to settle. The compact preset uses the library's open-trefoil helper, mirrored to face the observed lobe direction. It is independently initialized; it is not a continuation of a tying policy.

A separate endpoint-load diagnostic starts from the actual loose preset and tightens it while preserving three crossings. Earlier `loose_pull_mu025` / `loose_pull_mu00025` rows drove the loose preset's reversed endpoint order inward and are rejected probe errors. Only rows named `loose_pull_outward_*` are outward-pull results. The arms never tie or grasp the rope, and no continuous robot-policy take is claimed.

## Visual review and limits

I opened both the full-size pairs and look sheets for the initial match, final stages, diameter controls, stiffness control and friction/pull comparisons. The rope's blue patterned sheath, the wood colour/grain, straight orientation, loop scale and compact-knot scale are represented. The final loose lobe agrees well; the loop retains a small kink at its upper-right bend and is several pixels offset. The table has approximate matching plank seams, but the exact dark grain features, broad real shadows and blurred video lighting differ. Braid is an image texture on a smooth tube, not geometric fibres. Human hands and Howcast graphics are omitted. Their pixels explain part of the low full-frame scores in occluded shots; the differences in the rope itself remain visible in the look sheets.

Unresolved: real camera intrinsics/crop, true table height/extent, unseen tails, strand heights at occluded crossings, forces and trajectories of the hands, density, torsional response, and a unique friction/bending pair. Some exploratory and subsequent free-running curved scenes emitted SuperDex linear-solver warnings. The gates below validate their specified settling windows, not arbitrary long-run stability or robot grasp reliability.

## Validation and replay

The stock gate's rigid-body drift/penetration checks do not meaningfully measure the foreign rod (`drift_mm={}`). The probe therefore adds actual node drift ≤2 mm/60 steps, table penetration ≤1 mm, self-clearance ≥-1 mm, and preservation of the initial crossing count. A stock PASS alone was not accepted as deformable evidence.

Local changed-file Ruff checks/format and Python compilation pass (available Ruff 0.15.12; requested 0.16.1 was not installed). The repository fast tier was attempted with the installed interpreter and a writable cache. It emitted test errors and CPU compilation errors, then stalled; it was interrupted after several minutes (exit 130), so the full suite did **not** pass. Output is retained in `run/fast-tests.txt`. No repository engine code was changed.

To repeat a stage on a working live page, copy its JSON's `config` object to `probe.json` and save/touch `scene.py`. Its capture runs during the live rebuild. Remove `probe.json` and save again to restore the delivered straight state. Saved probe configs and final measurement images are in `evidence/`; the full exploration log remains in `run/`. The final default has no active diagnostic pins or auto-running routine.

## Final readings and exact log lines

All four final independent stages pass both the stock and added rod gates. The compact preset uses a 35 mm per-end pull over 2 s followed by a 5 s hold; it retains held endpoint constraints for that diagnostic only. The delivered straight default has none.

| Stage | Log compare/check | Full silhouette IoU | Exposed-region IoU | Crossings | Node drift / 60 steps (mm) | Table penetration (mm) | Self-clearance at capture (mm) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 16 s straight | 62/63 | 0.8832 | — | 0 | 0.0001 | 0.0473 | 7.8977 |
| 24 s loop | 58/59 | 0.3108 | 0.4157 | 1 | 0.7453 | 0.0833 | 0.1560 |
| 33 s loose | 56/57 | 0.7109 | — | 3 | 0.2820 | 0.1001 | 0.3598 |
| 36 s compact | 64/65 | 0.1813 | 0.1792 | 3 | 0.5457 | 0.2015 | -0.0334 |

The loop ROI is `[180,0,440,265]`, above the occluding hand; the compact ROI is `[60,190,530,285]`, excluding hands and graphics. The compact knot has a 39 pixel overall vertical silhouette in that ROI (reference 36), but the tails/knot are about 9 pixels high. Its knot-length probe is 148.99 mm. The stronger 45 mm pull made the knot smaller, 131.47 mm of rope between crossings; it was not selected merely for a higher mask score. The final loose knot has 483.23 mm between crossings.

Diagnostic equivalent of `./ctl log` (actual source is save-driven, as explained above):

```text
  n  at        cmd       scene       score   note
  1  10:27:46  compare   39dbd723eb   0.039  first_01
  2  10:27:51  check     39dbd723eb    PASS  first_01
  3  10:30:22  compare   d860622005       -  first_02
  4  10:30:29  check     d860622005    PASS  first_02
  5  10:30:46  compare   53a33a1df5   0.858  first_03
  6  10:30:53  check     53a33a1df5    PASS  first_03
  7  10:31:50  compare   5f88fbb42c   0.877  first_04
  8  10:31:56  check     5f88fbb42c    PASS  first_04
  9  10:34:55  compare   dca9ea2796   0.179  loop_E1e5_mu025
 10  10:36:33  compare   dca9ea2796   0.056  loop_E2e6_mu025
 11  10:37:35  compare   d66cd44a26   0.246  loose_E1e5_mu025
 12  10:38:30  compare   1f34cbd420   0.278  loose_E1e5_mu00025
 13  10:38:47  compare   1f34cbd420   0.332  loose_padded_E1e5_mu025
 14  10:45:26  compare   19f4ff8939   0.614  loose_clear_E2e4_mu025
 15  10:46:10  compare   19f4ff8939   0.527  loose_clear_E1e5_mu025
 16  10:47:18  compare   a0d47bef65   0.698  loose_gap0_E2e4_mu025
 17  10:48:52  compare   bc702aa39d   0.704  loose_final_E2e4_mu025
 18  10:48:53  check     bc702aa39d    PASS  loose_final_E2e4_mu025
 19  10:49:46  compare   bc702aa39d   0.354  loose_final_E2e6_mu025
 20  10:51:24  compare   bc702aa39d   0.721  loose_final_E2e4_mu00025
 21  10:54:13  compare   325293958e   0.721  loose_final_E2e4_mu00025
 22  10:54:26  compare   325293958e   0.292  tight_final
 23  10:54:26  check     325293958e    PASS  tight_final
 24  10:55:18  compare   e3fcc05d41   0.312  loop_final
 25  10:55:20  check     e3fcc05d41    PASS  loop_final
 26  10:55:57  compare   e3fcc05d41   0.037  tight_pull
 27  10:55:58  check     e3fcc05d41    FAIL  tight_pull
 28  10:57:22  compare   f003b77c70   0.070  tight_pull_aligned
 29  10:57:23  check     f003b77c70    PASS  tight_pull_aligned
 30  10:58:57  compare   0c6e445137   0.036  tight_background_final
 31  10:58:58  check     0c6e445137    PASS  tight_background_final
 32  11:01:10  compare   98b9298151   0.526  tight_background_final
 33  11:01:11  check     98b9298151    PASS  tight_background_final
 34  11:01:28  compare   98b9298151   0.012  loose_pull_mu025
 35  11:01:29  check     98b9298151    PASS  loose_pull_mu025
 36  11:01:58  compare   98b9298151   0.012  loose_pull_mu00025
 37  11:01:59  check     98b9298151    PASS  loose_pull_mu00025
 38  11:02:22  compare   c93041390c   0.287  loose_pull_outward_mu025
 39  11:02:23  check     c93041390c    FAIL  loose_pull_outward_mu025
 40  11:03:07  compare   c93041390c   0.263  loose_pull_outward_mu00025
 41  11:03:08  check     c93041390c    PASS  loose_pull_outward_mu00025
 42  11:03:49  compare   c93041390c   0.465  loose_pull_outward_mu005
 43  11:03:51  check     c93041390c    PASS  loose_pull_outward_mu005
 44  11:04:38  compare   c93041390c   0.864  diameter_8mm
 45  11:04:39  check     c93041390c    PASS  diameter_8mm
 46  11:05:35  compare   78c09cfb15   0.736  diameter_12mm
 47  11:05:36  check     78c09cfb15    PASS  diameter_12mm
 48  11:05:47  compare   78c09cfb15   0.736  diameter_12mm
 49  11:05:47  check     78c09cfb15    PASS  diameter_12mm
 50  11:07:06  compare   78c09cfb15   0.310  loop_validated
 51  11:07:07  check     78c09cfb15    PASS  loop_validated
 52  11:08:08  compare   78c09cfb15   0.711  loose_validated
 53  11:08:09  check     78c09cfb15    PASS  loose_validated
 54  11:09:50  compare   f1f6862029   0.711  loose_validated_flat
 55  11:09:51  check     f1f6862029    PASS  loose_validated_flat
 56  11:10:05  compare   f1f6862029   0.711  loose_validated_flat
 57  11:10:06  check     f1f6862029    PASS  loose_validated_flat
 58  11:11:47  compare   f1f6862029   0.311  loop_validated_flat
 59  11:11:49  check     f1f6862029    PASS  loop_validated_flat
 60  11:13:09  compare   f1f6862029   0.406  tight_validated
 61  11:13:10  check     f1f6862029    PASS  tight_validated
 62  11:15:52  compare   f1f6862029   0.883  straight_validated
 63  11:15:53  check     f1f6862029    PASS  straight_validated
 64  11:18:13  compare   f1f6862029   0.181  tight_validated_035
 65  11:18:14  check     f1f6862029    PASS  tight_validated_035
```

### straight_validated

[Pair](evidence/straight_validated_beside.png) · [Look sheet](evidence/straight_validated_look.png) · [Numbers/config](evidence/straight_validated.json)

```text
PASS  builds       the last rebuild landed
PASS  finite       every number in the state is finite
PASS  camera       phone 640x480, fov 45 deg; clip 640x480
PASS  objects      all of rope, table are in the scene
PASS  penetration  worst overlap 0.00 mm
PASS  rest         nothing moved more than 0.0 mm in 60 steps
PASS  finite_after still finite after 60 steps
PASS  rod_rest  max node drift 0.0001 mm / 60 steps; limit 2 mm
PASS  rod_table  table penetration 0.0473 mm; limit 1 mm
PASS  rod_self  self clearance 7.8977 mm; limit -1 mm
PASS  rod_crossings  0 crossings; initial 0
```

### loop_validated_flat

[Pair](evidence/loop_validated_flat_beside.png) · [Look sheet](evidence/loop_validated_flat_look.png) · [Numbers/config](evidence/loop_validated_flat.json)

```text
PASS  builds       the last rebuild landed
PASS  finite       every number in the state is finite
PASS  camera       loop 640x480, fov 45 deg; clip 640x480
PASS  objects      all of rope, table are in the scene
PASS  penetration  worst overlap 0.00 mm
PASS  rest         nothing moved more than 0.0 mm in 60 steps
PASS  finite_after still finite after 60 steps
PASS  rod_rest  max node drift 0.7453 mm / 60 steps; limit 2 mm
PASS  rod_table  table penetration 0.0833 mm; limit 1 mm
PASS  rod_self  self clearance 0.1422 mm; limit -1 mm
PASS  rod_crossings  1 crossings; initial 1
```

### loose_validated_flat

[Pair](evidence/loose_validated_flat_beside.png) · [Look sheet](evidence/loose_validated_flat_look.png) · [Numbers/config](evidence/loose_validated_flat.json)

```text
PASS  builds       the last rebuild landed
PASS  finite       every number in the state is finite
PASS  camera       loose 640x480, fov 45 deg; clip 640x480
PASS  objects      all of rope, table are in the scene
PASS  penetration  worst overlap 0.00 mm
PASS  rest         nothing moved more than 0.0 mm in 60 steps
PASS  finite_after still finite after 60 steps
PASS  rod_rest  max node drift 0.2820 mm / 60 steps; limit 2 mm
PASS  rod_table  table penetration 0.1001 mm; limit 1 mm
PASS  rod_self  self clearance 0.3521 mm; limit -1 mm
PASS  rod_crossings  3 crossings; initial 3
```

### tight_validated_035

[Pair](evidence/tight_validated_035_beside.png) · [Look sheet](evidence/tight_validated_035_look.png) · [Numbers/config](evidence/tight_validated_035.json)

```text
PASS  builds       the last rebuild landed
PASS  finite       every number in the state is finite
PASS  camera       wide 640x480, fov 45 deg; clip 640x480
PASS  objects      all of rope, table are in the scene
PASS  penetration  worst overlap 0.00 mm
PASS  rest         nothing moved more than 0.0 mm in 60 steps
PASS  finite_after still finite after 60 steps
PASS  rod_rest  max node drift 0.5457 mm / 60 steps; limit 2 mm
PASS  rod_table  table penetration 0.2015 mm; limit 1 mm
PASS  rod_self  self clearance -0.0317 mm; limit -1 mm
PASS  rod_crossings  3 crossings; initial 3
```
