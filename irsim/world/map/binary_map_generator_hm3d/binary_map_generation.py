"""
Generate navigable 2D binary maps from 3D spaces datasets.

Processes navigation mesh vertices from 3D spaces datasets 
(like Habitat-Matterport 3D) to create 2D occupancy binary maps. 
Key features:
- Identifies navigable vs. non-navigable regions using geometric analysis
- Visualizes obstacles with diagonal hatch patterns
- Exports grayscale images (pure white=navigable, hatched=obstacles)
"""

import os
from typing import Tuple

import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon as ShapelyPolygon, box
from shapely.ops import unary_union
from PIL import Image


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
    triangles = [vertices[idx:idx+3] for idx in range(0, len(vertices), 3)]
    
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


def main() -> None:
    """Main execution flow for generating and saving binary map."""
    # Example usage with scenario 00824-Dd4bFSTQ8gi of HM3D dataset
    scene_name = "00848-ziup5kvtCCR"
    current_dir = os.path.dirname(__file__)
    vertices_path = os.path.join(current_dir, f"{scene_name}_navmesh_vertices.npy")
    
    # Process navigation mesh data
    navmesh_vertices = np.load(vertices_path)
    obstacles, *bounds = create_regions(navmesh_vertices)
    
    # Generate and save visualization
    map_fig, _ = draw_binary_map(obstacles, bounds)
    save_binary_map(map_fig, f"{current_dir}/{scene_name}_binary_map.png")


if __name__ == "__main__":
    main()