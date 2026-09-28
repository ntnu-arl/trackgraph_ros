# TrackGraph configuration

Camera topics and TF frames belong in the
[robot launch file](../hydra_ros/launch/trackgraph/trackgraph_robot.launch.yaml).
Use the same RGB and registered-depth topics in the tracker's `robot.launch.yaml`.

| Configuration | Purpose |
| --- | --- |
| [Mapping robot preset](https://github.com/ntnu-arl/trackgraph/blob/main/config/trackgraph_config/robot.yaml) | Voxel resolution, map window, and scene graph settings |
| [ROS robot preset](../hydra_ros/config/trackgraph_config/robot.yaml) | Sensor range and input queues |
| [Common mapping settings](https://github.com/ntnu-arl/trackgraph/blob/main/config/trackgraph_config/trackgraph_common_config.yaml) | Segment association and open-vocabulary retrieval |
| [Tracker presets](https://github.com/ntnu-arl/instance_tracking/tree/main/instance_tracking_ros/config) | Segmentation, propagation, and CLIP models |

The robot and uHumans2 launches use the ViT-B OpenCLIP encoder in the tracker’s
`deployment` and `dev` presets. Replica, HM3D, and ScanNet++ use the ViT-H encoder
in `openlex_quality`. Keep the encoder expected by the mapper consistent with the
tracker; changing only one will reject feature messages.

To inspect all supported arguments:

```bash
ros2 launch hydra_ros trackgraph_robot.launch.yaml --show-args
ros2 launch instance_tracking_ros robot.launch.yaml --show-args
```

Default mapping outputs are stored under `~/.hydra/<log_name>`. Override
`log_path:=/path/to/output` when launching to choose another directory.
