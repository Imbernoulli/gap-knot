"""Build the nominal double coin task, let the rope settle, and read its diagram."""
import sys, time, numpy as np
from knotlab import task
t = time.perf_counter()
env = task.build(task.NOMINAL)
env.reset(0)
print("built in %.1fs" % (time.perf_counter() - t), "names", env.sim.names() if hasattr(env, "sim") else "")
rope = env["rope"]
p0 = np.asarray(rope.particle_q, dtype=np.float64)
print("rope nodes", p0.shape, "z range", p0[:, 2].min().round(4), p0[:, 2].max().round(4))
t = time.perf_counter()
for _ in range(60):
    env.step(None) if hasattr(env, "step") else env.sim.run(1)
dt = (time.perf_counter() - t) / 60
p1 = np.asarray(rope.particle_q, dtype=np.float64)
print("ms/step %.1f" % (1e3 * dt), "moved max %.4f m" % np.abs(p1 - p0).max(), "z range", p1[:, 2].min().round(4), p1[:, 2].max().round(4))
print("score", task.score(p1))
print("pins", {k: v.round(3) for k, v in task.layout(task.NOMINAL)["pins"].items()})
