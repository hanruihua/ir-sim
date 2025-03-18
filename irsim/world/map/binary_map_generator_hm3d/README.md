# Binary Map Generator

A toolkit for generating 2D binary navigation maps from 3D spaces datasets like [HM3D](https://aihabitat.org/datasets/hm3d/), [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), etc. Convert complex 3D spaces into simple 2D representations showing navigable and non-navigable areas.

<div style="display: flex; justify-content: center; align-items: center; gap: 20px;">
  <div style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/16be3bb1-5022-4cc4-b4e1-275d31600e96" alt="Example real map" width="400"/>
    <br>
    <em>Example Real Map (00824-Dd4bFSTQ8gi in HM3D) </em>
  </div>
  <div style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/0486949d-9f16-4633-b8ff-90cbe5b79e19" alt="Example binary map" width="400"/>
    <br>
    <em>Example Binary Map (00824-Dd4bFSTQ8gi in HM3D)</em>
  </div>
</div>

## 🔍 Overview

This toolkit provides tools to:

- Extract navigation mesh vertices from 3D scenes using Habitat-Sim
- Process these vertices to generate intuitive 2D binary maps
- Visualize navigable regions and obstacles with clear difference

## ✨ Features

- Transform complex 3D environments into simplified 2D navigation maps
- Clearly distinguish between navigable areas and obstacles
- Represent obstacles with diagonal hatch patterns for visibility
- Generate high-quality images suitable for path planning algorithms

## 📋 Usage

You have two options for using this toolkit:

### Option 1: Use Pre-extracted Data (Recommended)

We provide pre-extracted navigation mesh vertices of HM3D dataset in the `binary_map_dataset_hm3d` directory, so you can generate maps without setting up Habitat-Sim and downloading the full HM3D dataset.

```bash
# Generate binary maps from pre-extracted vertices
python binary_map_generation.py
```

### Option 2: Extract Your Own Vertices

If you need to work with custom scenes or want to extract vertices yourself:

1. Set up [Habitat-Sim](https://github.com/facebookresearch/habitat-sim) (Follow the official Habitat-Sim installation guide) and download the 3D spaces datasets
2. Configure the scene path in the extraction script (`navmesh_vertices_extraction.py`):

```python
scene_name = "your-scene-id"  # E.g., "00824-Dd4bFSTQ8gi"
scene_path = f"/path/to/your/scenes/{scene_name}.glb"
```

3. Extract vertices:

```bash
python navmesh_vertices_extraction.py
```

4. Generate binary maps:

```bash
python binary_map_generation.py
```

## 📁 Toolkit Structure

```
.
├── binary_map_generation.py       # Converts vertices to binary maps
├── navmesh_vertices_extraction.py # Extracts vertices from 3D scenes
├── binary_map_dataset_hm3d/       # Directory with pre-extracted data
│   ├── 00824-Dd4bFSTQ8gi_navmesh_vertices.npy
│   ├── 00824-Dd4bFSTQ8gi_binary_map.png
│   └── ...
└── README.md                      # This file
```

## 🖼️ Output Format

The generated binary maps are saved as grayscale PNG images where:

- White areas represent navigable space
- Hatched areas represent obstacles/walls

These maps can be used for:

- Path planning algorithms
- Navigation studies
- Robot simulation
- Environmental analysis

## 🙏 Acknowledgments

- [habitat-sim](https://github.com/facebookresearch/habitat-sim)
- [HM3D dataset](https://github.com/matterport/habitat-matterport-3dresearch)