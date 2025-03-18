"""
Habitat-Sim NavMesh Vertex Extraction Tool

Extracts and processes navigation mesh vertices from 3D scenes for 2D mapping applications.
"""

import os
from typing import List, Tuple

import numpy as np
import habitat_sim
from habitat_sim.utils.settings import default_sim_settings, make_cfg


def extract_navmesh_vertices(
    scene_path: str,
    output_dir: str,
    scene_name: str
) -> Tuple[np.ndarray, str]:
    """
    Extract navigation mesh vertices from a 3D scene and save as 2D coordinates.

    Args:
        scene_path: Path to scene file (.glb, .ply, or other supported format)
        output_dir: Directory to save output files
        scene_name: Base name for output files

    Returns:
        Tuple containing:
        - NumPy array of 2D vertices in XZ plane
        - Path to saved output file

    Raises:
        FileNotFoundError: If input scene file doesn't exist
        RuntimeError: If NavMesh computation fails
    """
    # Verify input file existence
    if not os.path.exists(scene_path):
        raise FileNotFoundError(f"Scene file not found: {scene_path}")

    # Configure simulator settings
    sim_settings = default_sim_settings.copy()
    sim_settings.update({
        "scene": scene_path,
        "enable_physics": False,  # Physics not required for NavMesh generation
        "default_agent_navmesh": False
    })

    try:
        # Initialize simulator
        cfg = make_cfg(sim_settings)
        with habitat_sim.Simulator(cfg) as sim:
            # Compute NavMesh if not loaded
            if not sim.pathfinder.is_loaded:
                print(f"Computing NavMesh for {scene_name}...")
                navmesh_settings = habitat_sim.NavMeshSettings()
                navmesh_settings.set_defaults()
                
                # Configure agent parameters from first agent specification
                agent_cfg = cfg.agents[0]
                navmesh_settings.agent_height = agent_cfg.height
                navmesh_settings.agent_radius = agent_cfg.radius
                navmesh_settings.include_static_objects = True
                
                sim.recompute_navmesh(sim.pathfinder, navmesh_settings)

            # Extract and transform vertices
            vertices_3d = sim.pathfinder.build_navmesh_vertices()
            vertices_2d = np.array([[v[0], -v[2]] for v in vertices_3d])

            # Create output directory if needed
            output_subdir = os.path.join(output_dir, "binary_map_dataset_hm3d")
            os.makedirs(output_subdir, exist_ok=True)

            # Save results
            output_path = os.path.join(
                output_subdir,
                f"{scene_name}_navmesh_vertices.npy"
            )
            np.save(output_path, vertices_2d)

            return vertices_2d, output_path

    except Exception as e:
        raise RuntimeError(
            f"NavMesh extraction failed for {scene_name}"
        ) from e


if __name__ == "__main__":
    # Example usage
    scene_name = "00824-Dd4bFSTQ8gi"
    scene_path = f"/path/to/scenes/{scene_name}.glb"  # Update with actual path
    output_dir = os.path.dirname(__file__)

    try:
        vertices, output_file = extract_navmesh_vertices(
            scene_path,
            output_dir,
            scene_name
        )
    
    except Exception as e:
        print(f"Error: {str(e)}")
        exit(1)