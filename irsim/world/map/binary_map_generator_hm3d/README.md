# Binary Map Generator Configuration

The Binary Map Generator converts 3D scene datasets into 2D binary occupancy maps. Configure it by specifying the input scene file and output path directly in the Python script.

## Dependencies

- [habitat-sim](https://github.com/facebookresearch/habitat-sim) (for NavMesh extraction)
- `shapely`, `matplotlib`, `numpy` (for geometry processing and visualization)

## Configuration Parameters

The main script (`binary_map_generator.py`) uses the following parameters:

| Parameter    | Description                                                                 |
|--------------|-----------------------------------------------------------------------------|
| `scene_name` | Identifier for the scene (e.g., `"00824-Dd4bFSTQ8gi"`).                     |
| `scene_path` | Path to the 3D scene file (e.g., `"/path/to/scenes/00824-Dd4bFSTQ8gi.glb"`).|

## Usage

1. Set up [Habitat-Sim](https://github.com/facebookresearch/habitat-sim) (Follow the official Habitat-Sim installation guide) and download the 3D scene datasets like [HM3D](https://aihabitat.org/datasets/hm3d/), [MatterPort3D](https://niessner.github.io/Matterport/), [Gibson](http://gibsonenv.stanford.edu/database/), etc.
2. Configure the parameters in the script (`binary_map_generator.py`):
```python
scene_name = "your-scene-id"  # E.g., "00824-Dd4bFSTQ8gi"
scene_path = f"/path/to/your/scenes/{scene_name}.glb"
```
3. Generate binary maps:

```bash
python binary_map_generator.py
```

# hm3d Real Maps vs. Binary Maps

| Real Map | Binary Map |
|:--------:|:----------:|
| <img src="https://github.com/user-attachments/assets/588e4ab9-abf3-411e-9290-d5dffcfd6870" alt="Real map 1" width="400"/><br><em>hm3d Real Map 1 (00813-svBbv1Pavdk)</em> | <img src="https://github.com/user-attachments/assets/3e2d2c78-8ab3-47db-b30b-40d4032d184b" alt="Binary map 1" width="400"/><br><em>hm3d Binary Map 1 (00813-svBbv1Pavdk)</em> |
| <img src="https://github.com/user-attachments/assets/16be3bb1-5022-4cc4-b4e1-275d31600e96" alt="Real map 2" width="400"/><br><em>hm3d Real Map 2 (00824-Dd4bFSTQ8gi)</em> | <img src="https://github.com/user-attachments/assets/0486949d-9f16-4633-b8ff-90cbe5b79e19" alt="Binary map 2" width="400"/><br><em>hm3d Binary Map 2 (00824-Dd4bFSTQ8gi)</em> |
| <img src="https://github.com/user-attachments/assets/dbcdef8e-d985-4ddc-bb89-5ed651ebf2f4" alt="Real map 3" width="400"/><br><em>hm3d Real Map 3 (00827-BAbdmeyTvMZ)</em> | <img src="https://github.com/user-attachments/assets/ba7a88a3-bc4a-471f-897b-5422495e7291" alt="Binary map 3" width="400"/><br><em>hm3d Binary Map 3 (00827-BAbdmeyTvMZ)</em> |
| <img src="https://github.com/user-attachments/assets/07157798-0139-408f-8194-99da1bbaef0e" alt="Real map 4" width="400"/><br><em>hm3d Real Map 4 (00829-QaLdnwvtxbs)</em> | <img src="https://github.com/user-attachments/assets/520a663b-1b9e-4d1d-a734-3816e7f5c056" alt="Binary map 4" width="400"/><br><em>hm3d Binary Map 4 (00829-QaLdnwvtxbs)</em> |
| <img src="https://github.com/user-attachments/assets/f683540c-c888-488b-b562-40b7c070b795" alt="Real map 5" width="400"/><br><em>hm3d Real Map 5 (00848-ziup5kvtCCR)</em> | <img src="https://github.com/user-attachments/assets/b6ace63a-3098-4f9f-8239-876d0d32c2bd" alt="Binary map 5" width="400"/><br><em>hm3d Binary Map 5 (00848-ziup5kvtCCR)</em> |
| <img src="https://github.com/user-attachments/assets/7fb17c19-c586-45e2-8fb3-adf68fc5d193" alt="Real map 6" width="400"/><br><em>hm3d Real Map 6 (00853-5cdEh9F2hJL)</em> | <img src="https://github.com/user-attachments/assets/b5f5bbff-998e-4451-acb5-a67f70fef9dd" alt="Binary map 6" width="400"/><br><em>hm3d Binary Map 6 (00853-5cdEh9F2hJL)</em> |
| <img src="https://github.com/user-attachments/assets/5d438a40-6566-480a-bb2d-aa885d131a70" alt="Real map 7" width="400"/><br><em>hm3d Real Map 7 (00871-VBzV5z6i1WS)</em> | <img src="https://github.com/user-attachments/assets/76d92f83-b566-42b8-b7fe-6e3a0c272787" alt="Binary map 7" width="400"/><br><em>hm3d Binary Map 7 (00871-VBzV5z6i1WS)</em> |
| <img src="https://github.com/user-attachments/assets/69c66b1a-bf0b-4870-8458-995ed28132e9" alt="Real map 8" width="400"/><br><em>hm3d Real Map 8 (00876-mv2HUxq3B53)</em> | <img src="https://github.com/user-attachments/assets/7a0490fa-df60-413c-b0fe-e59918037115" alt="Binary map 8" width="400"/><br><em>hm3d Binary Map 8 (00876-mv2HUxq3B53)</em> |
| <img src="https://github.com/user-attachments/assets/0740999d-8ef6-42aa-804a-b27d030ed598" alt="Real map 9" width="400"/><br><em>hm3d Real Map 9 (00880-Nfvxx8J5NCo)</em> | <img src="https://github.com/user-attachments/assets/9ee64130-5e2d-41a0-8fcd-4a7e93daece1" alt="Binary map 9" width="400"/><br><em>hm3d Binary Map 9 (00880-Nfvxx8J5NCo)</em> |

## Acknowledgments

- [habitat-sim](https://github.com/facebookresearch/habitat-sim)
- [hm3d dataset](https://github.com/matterport/habitat-matterport-3dresearch)