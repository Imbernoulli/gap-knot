# SPDX-License-Identifier: Apache-2.0
"""Howcast overhand rope on wood, with a pair of YAM arms at the near edge.

Units are metres in the table's work-surface frame (world origin + 0.75 m z).
Default: the straight strand in the 16 s close-up. Optional probe.json selects
reference-shaped initial conditions and save-driven live-page measurements.
"""

import json
from pathlib import Path

import numpy as np
import warp as wp
from rope_shapes import layout

import robosimstudio as rss
from robosimstudio.core.render.finish import Finish
from robosimstudio.robots.registry import get_embodiment
from robosimstudio.scene.registry import register_cell

HERE = Path(__file__).resolve().parent
wp.config.kernel_cache_dir = str(HERE / "run" / "warp_cache")
PROBE = json.loads((HERE / "probe.json").read_text()) if (HERE / "probe.json").exists() else {}
STAGE = PROBE.get("stage", "straight")
RADIUS = float(PROBE.get("radius", 0.005))
LENGTH = 1.05
NODE_COUNT = 177
YOUNG_MODULUS = float(PROBE.get("young_modulus", 2.0e4))
MU_CABLE = float(PROBE.get("mu_cable", 0.05))
AXIAL_STIFFNESS = 300.0
TABLE_HEIGHT = 0.75
SKIN = bool(PROBE.get("skin", False))
engine = rss.backend("superdex", substeps=2, feedback="off", mirror_padding=0 if SKIN else RADIUS - 0.001)
physics = rss.replace(rss.GENERIC, backends=(("rod", engine), *rss.GENERIC.backends))
cell = rss.CellSpec(name="howcast_table", surface_z=TABLE_HEIGHT, half=(0.55, 0.75, 0.025))
register_cell(cell)
scene = rss.Scene(cell=cell.name, physics=physics, name="Howcast overhand")
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
# The importer creates a temporary XML beside its source. Keep that in the
# writable workspace; the copy is the unmodified shipped YAM asset.
yam = rss.replace(yam, asset=rss.replace(yam.asset, path=str(HERE / "assets/yam/yam_bimanual_arms.xml")))
scene.add(rss.robot(yam, at=(-0.55, 0, 0.022)))
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
        name="wide", width=640, height=480, fov_y=45, eye=(0.010, 0, 1.245), look_at=(0.010, 0, 0.75), up=(1, 0, 0)
    ),
}
for camera in CAMERAS.values():
    scene.add(camera)
if STAGE == "straight":
    arc = np.linspace(-LENGTH / 2, LENGTH / 2, NODE_COUNT)
    nodes = np.column_stack(
        (
            arc / np.sqrt(1 + 0.018**2),
            0.018 * arc / np.sqrt(1 + 0.018**2) - 0.0049,
            np.full(NODE_COUNT, RADIUS + 0.0003),
        )
    )
else:
    nodes = layout(
        STAGE, CAMERAS[{"loop": "loop", "loose": "loose", "tight": "wide"}[STAGE]], RADIUS, LENGTH, NODE_COUNT
    )
rope = scene.add(
    rss.rod(
        name="rope",
        radius=RADIUS,
        nodes=tuple(map(tuple, nodes)),
        color=(0.025, 0.075, 0.20),
        finish=Finish(roughness=0.85, texture=str(HERE / "assets/rope_braid.jpg")),
        self_contact=True,
        skin=SKIN,
        stress_free="straight",
        young_modulus=YOUNG_MODULUS,
        axial_stiffness=AXIAL_STIFFNESS,
        mu_cable=MU_CABLE,
        friction_falloff=1e-3,
        contact_penalty=4e8,
        contact_gap=RADIUS,
        skin_samples="P1Q6",
        density=350,
    )
)

sim = scene.build(control_mode="joint_position")
if PROBE:
    from live_probe import capture

    capture(sim, scene, __file__, PROBE)
