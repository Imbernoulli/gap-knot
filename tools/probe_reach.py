"""Where the yam_bimanual hands reach tool-down over bench_075: IK error over a grid, per arm."""
import math, numpy as np
import robosimstudio as rss
from robosimstudio.robots.registry import get_embodiment

scene = rss.Scene(cell="bench_075", name="reach")
scene.add(rss.robot(get_embodiment("yam_bimanual")))
scene.add(rss.box(name="cube", half=(0.01,)*3, at=(0.0, 0.0, 0.05)))
sim = scene.build(control_mode="joint_position")
origin = np.array([0.6, 0.0, 0.75])
q0 = sim.state.joint_q.numpy().astype(np.float64)
for arm in ("left", "right"):
    ik = sim.arm(arm)
    p, q = ik.ee_pose(sim.state)
    print(arm, "home ee (bench frame)", np.round(np.asarray(p) - origin, 3), "coords", list(ik.arm_coords))
down = np.array([1.0, 0.0, 0.0, 0.0])  # tool down, xyzw (as route_the_cable's yaw_quat(0))
xs = np.arange(-0.25, 0.36, 0.05); ys = np.arange(-0.45, 0.46, 0.05)
for arm in ("left", "right"):
    ik = sim.arm(arm)
    print(f"\n{arm}: IK error mm at z=0.03 (rows x from {xs[0]:.2f} to {xs[-1]:.2f}, cols y from {ys[0]:.2f} to {ys[-1]:.2f})")
    right_seed = (0.5459, 2.2689, 2.0725, -1.3745, 0.0, -1.0249)
    seeds = []
    for arm_q in (right_seed, (0.0, 1.047, 1.047, 0.0, 0.0, 0.0), (0.0, 1.8, 1.6, -0.9, 0.0, 0.0)):
        for sign in (1, -1):
            q = q0.copy(); qa = np.array(arm_q); qa[0] *= sign; qa[5] *= sign
            idx = list(ik.arm_coords)
            if idx is None: break
            q[np.asarray(idx)] = qa; seeds.append(q)
    if not seeds: seeds = [q0]
    for x in xs:
        row = []
        for y in ys:
            target = origin + np.array([x, y, 0.03])
            best = None
            for sd in seeds:
                qs = ik.solve_q_full(sd, target, down)
                e = ik.pose_errors(qs, target, down)
                if best is None or e[0] + 0.05 * e[1] < best[0] + 0.05 * best[1]: best = e
            err = best
            row.append("  ." if err[0] < 0.003 and err[1] < 0.05 else f"{min(99, err[0]*1e3):3.0f}")
        print(f"x={x:+.2f} " + " ".join(row))
