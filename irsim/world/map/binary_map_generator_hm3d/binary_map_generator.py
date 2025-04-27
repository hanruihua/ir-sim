"""
Generate navigable 2D binary maps from 3D scene datasets.

Extract and processes navigation mesh vertices from 3D scene datasets 
(like Habitat-Matterport 3D) to create 2D occupancy binary maps. 

Author: Guoliang Li
"""

import os
import numpy as np
import habitat_sim
import matplotlib.pyplot as plt
from typing import Tuple
from habitat_sim.utils.settings import default_sim_settings, make_cfg
from shapely.geometry import Polygon as ShapelyPolygon, box
from shapely.ops import unary_union
from PIL import Image


def extract_navmesh_vertices(
    scene_path: str,
    scene_name: str
) -> np.ndarray:
    """
    Extract navigation mesh vertices from a 3D scene and save as 2D coordinates.

    Args:
        scene_path: Path to scene file (.glb, .ply, or other supported format)
        scene_name: Name of scene file

    Returns:
        NumPy array of 2D vertices in XZ plane

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

            return vertices_2d

    except Exception as e:
        raise RuntimeError(
            f"NavMesh extraction failed for {scene_name}"
        ) from e


def create_regions(
    vertices: np.ndarray
) -> Tuple[ShapelyPolygon, float, float, float, float]:
    """
    Process navigation mesh vertices to identify navigable regions and boundaries.

    Args:
        vertices: Array of shape (N, 2) containing navigation mesh vertices

    Returns:
        Tuple containing:
        - Unreachable regions as Shapely geometry (Polygon/MultiPolygon)
        - Bounding box coordinates (min_x, min_y, max_x, max_y)
    """
    # Create triangles from vertex triplets
    triangles = [vertices[idx:idx + 3] for idx in range(0, len(vertices), 3)]

    # Combine navigable areas into a single polygon
    reachable_region = unary_union([ShapelyPolygon(tri) for tri in triangles]).buffer(0)

    # Calculate scene bounding box
    min_x, min_y, max_x, max_y = reachable_region.bounds
    bounding_box = box(min_x, min_y, max_x, max_y)

    # Identify non-navigable areas
    unreachable_region = bounding_box.difference(reachable_region).buffer(0)

    return unreachable_region, min_x, min_y, max_x, max_y


def draw_binary_map(
    unreachable_region: ShapelyPolygon,
    bounds: Tuple[float, float, float, float]
) -> Tuple[plt.Figure, plt.Axes]:
    """
    Generate matplotlib visualization of the binary map with hatched obstacles.

    Args:
        unreachable_region: Geometry representing non-navigable areas
        bounds: Tuple of (min_x, min_y, max_x, max_y) for map boundaries

    Returns:
        Matplotlib figure and axes containing the visualization
    """
    min_x, min_y, max_x, max_y = bounds
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_aspect("equal", adjustable="datalim")

    if not unreachable_region.is_empty:
        hatch_style = {
            "facecolor": "none",
            "edgecolor": "black",
            "hatch": "/",  # Diagonal line pattern
            "linewidth": 2
        }

        def add_hatched_polygon(polygon: ShapelyPolygon) -> None:
            """Add a hatched polygon representation to the plot."""
            patch = plt.Polygon(
                np.array(polygon.exterior.coords),
                closed=True,
                **hatch_style
            )
            ax.add_patch(patch)

        # Handle both single and multi-polygon regions
        if unreachable_region.geom_type == "Polygon":
            add_hatched_polygon(unreachable_region)
        elif unreachable_region.geom_type == "MultiPolygon":
            for polygon in unreachable_region.geoms:
                add_hatched_polygon(polygon)

    # Configure plot boundaries
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.axis("off")
    ax.set_frame_on(False)

    return fig, ax


def save_binary_map(fig: plt.Figure, output_file: str) -> None:
    """
    Save binary map visualization to file and convert to grayscale.

    Args:
        fig: Matplotlib figure object to save
        output_file: Path for output image file (PNG format)
    """
    # Save as high-quality PNG
    plt.savefig(
        output_file,
        format="png",
        dpi=100,
        bbox_inches="tight",
        transparent=False,
        pad_inches=0
    )
    plt.show()
    plt.close()

    # Convert to grayscale using PIL
    with Image.open(output_file) as img:
        img.convert("L").save(output_file)


if __name__ == "__main__":
    # Example usage with scenario 00824-Dd4bFSTQ8gi of HM3D dataset
    scene_name = "00824-Dd4bFSTQ8gi"
    scene_path = f"/path/to/scenes/{scene_name}.glb"
    current_dir = os.path.dirname(__file__)

    try:
        # Extracts navigation mesh vertices from 3D scenes
        navmesh_vertices = extract_navmesh_vertices(
            scene_path,
            scene_name
        )

        # Process extracted navigation mesh vertices
        obstacles, *bounds = create_regions(navmesh_vertices)

        # Generate and save visualization
        map_fig, _ = draw_binary_map(obstacles, bounds)
        save_binary_map(map_fig, f"{current_dir}/hm3d_2.png")

    except Exception as e:
        print(f"Error: {str(e)}")
        exit(1)