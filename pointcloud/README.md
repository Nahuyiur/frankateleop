# frankateleop pointcloud tools

This folder keeps RGB-D stream recording, point-cloud export, and lightweight
reconstruction utilities together. The top-level shell scripts remain the user
entry points:

```bash
bash 18_record_rgb_pointclouds.sh
bash 19_view_recorded_rgb_pointclouds.sh /path/to/episode/0 --frame middle
```

## Data layout

The recorder stores sensor streams, not per-frame point clouds:

```text
episode/
  0.pkl.gz
  metadata.json
  <camera>.mp4
  depth/<camera>/000000.png
  depth/<camera>/000001.png
```

Each pkl frame only stores lightweight references such as
`<camera>_image_path`, `<camera>_depth_path`, `<camera>_depth_scale`,
`frame_index`, and `timestamp`.

Point clouds are derived when needed from:

```text
aligned depth PNG + RGB video frame + camera intrinsics
```

The derived points are in the camera color optical frame unless an explicit
extrinsic calibration is applied later.

## Modules

- `record_rgb_pointclouds.py`: camera-only RGB-D stream recorder used by script 18.
- `inspect_rgb_pointcloud_episode.py`: RGB/depth/PLY exporter used by script 19.
- `depth_proof.py`: shared RGB-D loading, depth PNG, and back-projection helpers.
- `verify_depth_episode.py`: regenerate depth proof artifacts for an episode.
- `visualize_rgbd_episode.py`: make explanatory RGB-D/point-cloud summary images.
- `reconstruct_rgbd_episode.py`: export dense PLY, organized depth mesh PLY, and
  rendered reconstruction checks.

## Useful direct commands

```bash
python -m pointcloud.verify_depth_episode /path/to/episode/0
python -m pointcloud.visualize_rgbd_episode /path/to/episode/0 --camera middle
python -m pointcloud.reconstruct_rgbd_episode /path/to/episode/0 --camera middle
```
