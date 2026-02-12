"""Profile where time is spent in the main loop."""
import time
import mujoco
from pathlib import Path
from sim import SwerveSimulator

model_path = Path(__file__).parent / "models" / "robot.xml"
sim = SwerveSimulator(model_path)

n_iters = 500

# Benchmark: mj_step only
mujoco.mj_resetData(sim.model, sim.data)
t0 = time.perf_counter()
for _ in range(n_iters):
    mujoco.mj_step(sim.model, sim.data)
t1 = time.perf_counter()
print(f"mj_step:       {(t1-t0)/n_iters*1000:.2f} ms")

# Benchmark: sim.step (includes motor model + collision toggle)
mujoco.mj_resetData(sim.model, sim.data)
t0 = time.perf_counter()
for _ in range(n_iters):
    sim.step(1)
t1 = time.perf_counter()
print(f"sim.step:      {(t1-t0)/n_iters*1000:.2f} ms")

# Benchmark: get_outputs (pose extraction for all bodies)
mujoco.mj_resetData(sim.model, sim.data)
t0 = time.perf_counter()
for _ in range(n_iters):
    sim.get_outputs()
t1 = time.perf_counter()
print(f"get_outputs:   {(t1-t0)/n_iters*1000:.2f} ms")

# Benchmark: viewer sync
sim.launch_viewer()
time.sleep(0.5)  # let viewer initialize
t0 = time.perf_counter()
for _ in range(n_iters):
    sim.sync_viewer()
t1 = time.perf_counter()
print(f"sync_viewer:   {(t1-t0)/n_iters*1000:.2f} ms")

print(f"\nTotal per iter: {sum_ms:.2f} ms" if False else "")

# Combined
mujoco.mj_resetData(sim.model, sim.data)
t0 = time.perf_counter()
for _ in range(n_iters):
    sim.step(1)
    sim.get_outputs()
    sim.sync_viewer()
t1 = time.perf_counter()
print(f"\nCombined:      {(t1-t0)/n_iters*1000:.2f} ms  ({1000/((t1-t0)/n_iters):.0f} Hz max)")
