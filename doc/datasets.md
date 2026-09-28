# Dataset input formats

See the [README](../README.md#datasets) for commands to run TrackGraph on
uHumans2, Replica, ScanNet++, and HM3D.

## Replica

Set `dataset_root` to the Replica directory containing `cam_params.json` and scene
folders, or to a NICE-SLAM checkout containing `Datasets/Replica/`.
Each scene contains `traj.txt` and `results/frame*.jpg`, `results/depth*.png`.
The publisher reads camera intrinsics and the depth scale from `cam_params.json`.

## ScanNet++

The publisher reads the iPhone recordings under
`<dataset_root>/data/<scene>/iphone/` or `<dataset_root>/<scene>/iphone/`:

- `rgb.mkv`: RGB video.
- `depth.bin`: compressed depth stream.
- `pose_intrinsic_imu.json`: camera poses and intrinsics.

The adapter reads the iPhone streams and poses and resizes registered depth to
the configured RGB dimensions. The default output is 960 × 720 pixels; adjust
`target_width` and `target_height` on the publisher to change it.

## HM3D

The publisher expects prepared RGB-D trajectories under `<dataset_root>/<scene>/`:

- `rgb/*.png`: RGB frames.
- `depth/*.png`: depth frames in millimetres.
- `pose/*.txt`: flattened 4×4 camera-to-world matrices in the Habitat camera convention.

RGB, depth, and pose files must have matching counts and correspond when sorted
by filename. The adapter assumes a 90-degree horizontal field of view, as used
by the prepared trajectories.

Publisher options are defined in
[`hydra_ros/launch/datasets`](../hydra_ros/launch/datasets).
