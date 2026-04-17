import irsim

env = irsim.make("fmcw_lidar_world.yaml")

for step in range(120):
    env.step()

    scan = env.get_lidar_scan()
    valid_count = int(scan["valid"].sum())
    center_idx = len(scan["ranges"]) // 2
    if scan["valid"][center_idx]:
        print(
            f"step={step:03d} valid_beams={valid_count:03d} "
            f"center_range={scan['ranges'][center_idx]:.3f} "
            f"center_radial_velocity={scan['radial_velocity'][center_idx]:.3f}"
        )
    else:
        print(f"step={step:03d} valid_beams={valid_count:03d} center beam invalid")

    env.render(0.05, mode="all")

    if env.done():
        break

env.end(3)
