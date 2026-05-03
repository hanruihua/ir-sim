import irsim

env = irsim.make("fmcw_lidar_world.yaml")

for step in range(120):
    env.step()

    scan = env.get_lidar_scan()
    valid_count = int(scan["valid"].sum())
    valid_indices = scan["valid"].nonzero()[0]
    if len(valid_indices) > 0:
        beam_idx = max(valid_indices, key=lambda idx: abs(scan["radial_velocity"][idx]))
        print(
            f"step={step:03d} valid_beams={valid_count:03d} "
            f"beam={beam_idx:03d} range={scan['ranges'][beam_idx]:.3f} "
            f"radial_velocity={scan['radial_velocity'][beam_idx]:.3f}"
        )
    else:
        print(f"step={step:03d} valid_beams={valid_count:03d} no valid returns")

    env.render(0.05, mode="all")

    if env.done():
        break

env.end(3)
