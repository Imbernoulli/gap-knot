# SPDX-License-Identifier: Apache-2.0
"""The Howcast overhand scene with the robot doing the clip's job: a copy of scene.py for the take-vs-clip check.

Everything the Codex Real2Sim built is kept -- the wood table and its plank seams,
the parked yam_bimanual at (-0.55, 0, 0.022), the four clip cameras, the rope's
radius, braid finish, stiffness (E = 2e4 Pa), contact settings and the SuperDex
backend. What changes, deliberately (each marked CHANGED below):

(a) the rope is the knotlab overhand task's start (96 nodes, 0.758 m, straight
    along y with both ends bent toward the robot), placed where it lies relative
    to the robot in knotlab, so a GaP plan written in knotlab's world frame runs
    here shifted by one constant offset (KNOTLAB_TO_SCENE);
(b) mu_cable 1.0, knotlab's nominal: at the Codex 0.05 a pinch does not lift the
    cord (SuperDex uses one friction for pads, strands and bench);
(c) an added camera "knot": the clip's "wide" camera (0.495 m over the table,
    45 deg, 640x480, up = +x) moved over the arms' workspace, so a take is filmed
    at the clip's wide-shot scale (~0.85 mm/px) -- every Codex camera looks at the
    table's centre, 0.3 m beyond the arms' reach;
(d) ``routines = {"demo": ...}``: the robot plan, run by demo_routine.py through
    the studio's reach/grip/hand/wait (see that module for where the plan comes from).

Units are metres in the table's work-surface frame (world origin + 0.75 m z).
"""

import json
import os
import sys
from pathlib import Path

import numpy as np
import warp as wp

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.robots.registry import get_embodiment
from robosimstudio.scene.registry import register_cell

HERE = Path(__file__).resolve().parent
GAP_KNOT = Path(os.environ.get("GAP_KNOT", "/srv/home/bohanlyu/gap-knot"))
if str(GAP_KNOT) not in sys.path:
    sys.path.insert(0, str(GAP_KNOT))  # knotlab (pure numpy) and harness.variants
from harness import variants  # noqa: E402
from knotlab import task as knot_task  # noqa: E402

import demo_routine  # noqa: E402  (re-imported on every save, like any helper beside the scene)

RUN_DIR = HERE / os.environ.get("DEMO_RUN_DIR", "run_demo")
RUN_DIR.mkdir(exist_ok=True)
wp.config.kernel_cache_dir = str(RUN_DIR / "warp_cache")  # not the Codex run/: that folder is its evidence
RADIUS = 0.005
YOUNG_MODULUS = 2.0e4
MU_CABLE = float(os.environ.get("DEMO_MU", "1.0"))  # CHANGED (b): Codex 0.05
AXIAL_STIFFNESS = 300.0
TABLE_HEIGHT = 0.75
# CHANGED (e): the rope's contact as knotlab has it (the rod's default skin and contact gap, the backend's default
# mirror padding), which is what the GaP plan's heights were measured against. With the Codex values (skin off,
# mirror padding 4 mm, contact gap 5 mm) a hand over the cord came to rest with its tool centre 16.5 mm up
# (13.5 asked) and the jaws stopped at 43 mm: no pinch held (measured, 4 of 4); with skin off alone the cord's
# axis sank to 1 mm over the table (the Codex report's 3-4 mm penetration). DEMO_CONTACT=codex restores them.
CONTACT = os.environ.get("DEMO_CONTACT", "knotlab")
engine = (
    rss.backend("superdex", substeps=2, feedback="off", mirror_padding=RADIUS - 0.001)
    if CONTACT == "codex"
    else rss.backend("superdex", substeps=2, feedback="off")
)
physics = rss.replace(rss.GENERIC, backends=(("rod", engine), *rss.GENERIC.backends))
cell = rss.CellSpec(name="howcast_table", surface_z=TABLE_HEIGHT, half=(0.55, 0.75, 0.025))
register_cell(cell)
scene = rss.Scene(cell=cell.name, physics=physics, name="Howcast overhand, robot take")
scene.add(
    rss.slab(
        name="table",
        half=(0.55, 0.75, 0.025),
        at=(0, 0, -0.025),
        color=(0.47, 0.35, 0.24),
        friction=0.5,
        finish=Finish(roughness=0.8, texture=str(HERE / "assets/table_wood.jpg"), uv_scale=(26, 26)),
    )
)
for index, x in enumerate((-0.062, 0.105)):
    scene.add(
        rss.slab(
            name=f"plank_seam_{index}",
            half=(0.0006, 0.75, 0.0001),
            at=(x, 0, 0.0001),
            color=(0.22, 0.14, 0.08),
            collide=False,
            finish=Finish(roughness=0.9),
        )
    )
yam = get_embodiment("yam_bimanual")
yam = rss.replace(yam, asset=rss.replace(yam.asset, path=str(HERE / "assets/yam/yam_bimanual_arms.xml")))
ROBOT_AT = (-0.55, 0.0, 0.022)
scene.add(rss.robot(yam, at=ROBOT_AT))

# Frames. knotlab's bench_075 puts yam_bimanual at its registered base_offset, (-0.3475, 0, 0.022) from the
# bench centre, and the bench centre is knotlab world (0.6, 0, 0.75). This table's centre is world (0, 0, 0.75)
# and the robot stands at ROBOT_AT. Same robot-relative layout: scene world = knotlab world + KNOTLAB_TO_SCENE.
KNOTLAB_BASE_WORLD = np.array([0.6 - 0.3475, 0.0, 0.75 + 0.022])
SCENE_BASE_WORLD = np.array([ROBOT_AT[0], ROBOT_AT[1], TABLE_HEIGHT + ROBOT_AT[2]])
KNOTLAB_TO_SCENE = SCENE_BASE_WORLD - KNOTLAB_BASE_WORLD  # (-0.8025, 0, 0)

# CHANGED (a): the knotlab overhand start. DEMO_VARIANT (a knotlab Variant as JSON) picks another instance.
variants.KNOT = "overhand"
VARIANT = (
    knot_task.Variant.from_json(os.environ["DEMO_VARIANT"]) if os.environ.get("DEMO_VARIANT") else variants.nominal("overhand")
)
LAYOUT = knot_task.layout(VARIANT)  # bench frame
KNOTLAB_BENCH = np.array([0.6, 0.0, 0.75])
start_world_knotlab = np.asarray(LAYOUT["start"], dtype=np.float64) + KNOTLAB_BENCH
nodes = start_world_knotlab + KNOTLAB_TO_SCENE - np.array([0.0, 0.0, TABLE_HEIGHT])  # table frame
nodes[:, 2] = RADIUS + 0.0003

CAMERAS = {
    "phone": rss.CameraSpec(
        name="phone", width=640, height=480, fov_y=45, eye=(0, 0, 0.827), look_at=(0, 0, 0.75), up=(1, 0, 0)
    ),
    "loop": rss.CameraSpec(
        name="loop",
        width=640,
        height=480,
        fov_y=45,
        eye=(0.040, 0, 1.117),
        look_at=(0.040, 0, 0.75),
        up=(0.819152, -0.573576, 0),
    ),
    "loose": rss.CameraSpec(
        name="loose", width=640, height=480, fov_y=45, eye=(-0.021, 0, 0.955), look_at=(-0.021, 0, 0.75), up=(1, 0, 0)
    ),
    "wide": rss.CameraSpec(
        name="wide", width=640, height=480, fov_y=45, eye=(0.010, 0, 1.245), look_at=(0.010, 0, 0.750), up=(1, 0, 0)
    ),
}
# CHANGED (c): the wide camera over the arms' workspace. Its centre sits between the cord's line (table x -0.27)
# and the bent ends (-0.46), so the line falls in the upper half of the frame as in the clip's wide shots.
KNOT_CAMERA_X = -0.32
CAMERAS["knot"] = rss.CameraSpec(
    name="knot",
    width=640,
    height=480,
    fov_y=45,
    eye=(KNOT_CAMERA_X, 0, TABLE_HEIGHT + 0.495),
    look_at=(KNOT_CAMERA_X, 0, TABLE_HEIGHT),
    up=(1, 0, 0),
)
for camera in CAMERAS.values():
    scene.add(camera)
rope = scene.add(
    rss.rod(
        name="rope",
        radius=RADIUS,
        nodes=tuple(map(tuple, nodes)),
        color=(0.025, 0.075, 0.20),
        finish=Finish(roughness=0.85, texture=str(HERE / "assets/rope_braid.jpg")),
        self_contact=True,
        young_modulus=float(VARIANT.young_modulus),
        axial_stiffness=float(VARIANT.axial_stiffness),
        mu_cable=float(VARIANT.mu_cable) if os.environ.get("DEMO_VARIANT") else MU_CABLE,
        friction_falloff=1e-3,
        contact_penalty=4e8,
        skin_samples="P1Q6",
        # knotlab's rod passes nothing more; the Codex's straight rest shape, skin off, 5 mm contact gap and
        # 350 kg/m3 come back with DEMO_CONTACT=codex. (With the default skin, stress_free="straight" laid the
        # bent start out straight along +x from node 0: measured.)
        **({"skin": False, "stress_free": "straight", "contact_gap": RADIUS, "density": 350} if CONTACT == "codex" else {}),
    )
)

sim = scene.build(control_mode="joint_position")


def _label_particles_as_rope() -> None:
    """CPU workaround (this box's driver is too old for the venv's CUDA warp, so the page runs on CPU).

    On CPU the id tracer draws the rod as particles and writes 0xFFFFFFFE for a particle hit; the
    tiled renderer's shape->label table has no such row, so ``--record`` died on its first frame with
    ``IndexError: index 4294967294`` (the Codex report hit the same sentinel). The rope is the only
    particle thing here, so a particle hit is labelled the rope. Patched in this process only.
    """
    from robosimstudio.solvers.render import tiled

    # The rod is not a collider shape, so the scene's segmentation names do not list it at all: give it the next
    # label, on this build's handles, so a take's per-frame masks and `compare --take --key rope=...` see it.
    handles = sim.scene.handles
    names = dict(handles.segmentation_names())
    label = next((int(k) for k, v in names.items() if v == "rope"), None)
    if label is None:
        label = max([int(k) for k in names] + [0]) + 1
        names[label] = "rope"
        handles.segmentation_names = lambda names=names: dict(names)
    tiled._demo_rope_label = label  # this build's
    if getattr(tiled.RendererTiled, "_demo_particles_patched", False):
        return

    class _Lut:
        def __init__(self, lut):
            self.lut = np.asarray(lut)

        def __getitem__(self, idx):
            idx = np.asarray(idx)
            out = self.lut[np.clip(idx, 0, len(self.lut) - 1)]
            return np.where(idx == 0xFFFFFFFE, tiled._demo_rope_label, out)

    original = tiled.RendererTiled.__init__

    def __init__(self, *a, **k):
        original(self, *a, **k)
        if self._label_lut is not None:
            self._label_lut = _Lut(self._label_lut)

    tiled.RendererTiled.__init__ = __init__
    tiled.RendererTiled._demo_particles_patched = True


_label_particles_as_rope()

# CHANGED (d): the robot plan as a Run button. demo_routine.py says where the plan comes from.
# "demo" takes the first plan source that exists (the GaP graph, knotlab.demo.demo_steps, demo_steps.json, the
# smoke plan); the others force one.
_kw = dict(layout=LAYOUT, variant=VARIANT, offset=KNOTLAB_TO_SCENE, gap_knot=GAP_KNOT, run_dir=RUN_DIR,
           steps_json=HERE / "demo_steps.json")
routines = {name: demo_routine.make_routine(**_kw, source=src)
            for name, src in (("demo", ""), ("graph", "graph"), ("steps", "steps"), ("json", "json"), ("smoke", "smoke"))}
(RUN_DIR / "scene_demo_frames.json").write_text(
    json.dumps(
        {
            "knotlab_to_scene": KNOTLAB_TO_SCENE.tolist(),
            "variant": json.loads(VARIANT.to_json()),
            "knot_camera_x": KNOT_CAMERA_X,
            "rope_start_table": nodes.round(5).tolist(),
        },
        indent=1,
    )
)
