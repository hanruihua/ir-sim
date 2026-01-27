import time

import numpy as np
from shapely import prepare

import irsim
from irsim.util.util import geometry_transform, transform_point_with_state

env = irsim.make(save_ani=False, full=False, display=False)

robot = env.robot
sensor = robot.sensors[0]
print(f"LiDAR rays: {sensor.number}")
print(f"Grid map shape: {env.objects[1].grid_map.shape}")
print()

n_steps = 30

# ============================================================
# Benchmark NEW method (grid-based raycasting)
# ============================================================
print("=" * 60)
print("NEW METHOD: Grid-based raycasting")
print("=" * 60)

new_times = []
for i in range(n_steps):
    start = time.perf_counter()
    env.step()
    elapsed = time.perf_counter() - start
    new_times.append(elapsed)
    if i % 10 == 0:
        print(f"Step {i}: {elapsed*1000:.2f} ms")

new_avg = np.mean(new_times) * 1000
print(f"Average: {new_avg:.2f} ms")
print()

# ============================================================
# Benchmark OLD method (geometry-based difference)
# ============================================================
print("=" * 60)
print("OLD METHOD: Geometry-based (shapely difference)")
print("=" * 60)

old_times = []
for i in range(n_steps):
    # Manually run the old geometry-based method
    state = sensor._state
    start = time.perf_counter()

    sensor.lidar_origin = transform_point_with_state(sensor.offset, state)
    new_geometry = geometry_transform(sensor._original_geometry, state)
    prepare(new_geometry)
    # This is the expensive part - geometry difference operation
    new_geometry, intersect_indices = sensor.laser_geometry_process(new_geometry)
    new_geometry = sensor._ensure_multi_linestring(new_geometry)

    elapsed = time.perf_counter() - start
    old_times.append(elapsed)
    if i % 10 == 0:
        print(f"Step {i}: {elapsed*1000:.2f} ms")

old_avg = np.mean(old_times) * 1000
print(f"Average: {old_avg:.2f} ms")
print()

# ============================================================
# Summary
# ============================================================
print("=" * 60)
print("COMPARISON SUMMARY")
print("=" * 60)
print(f"Old method (geometry):  {old_avg:.2f} ms")
print(f"New method (grid):      {new_avg:.2f} ms")
print(f"Speedup:                {old_avg/new_avg:.1f}x faster")
print("=" * 60)

env.end()
