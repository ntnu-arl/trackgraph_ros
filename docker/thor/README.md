# TrackGraph on Jetson Thor

This optional profile supplies ROS 2 Jazzy and the NVIDIA Python stack for Jetson
Thor. Install Docker and NVIDIA Container Toolkit on the host, and first complete
the source import in the [main setup guide](../../README.md#download-and-build).
Skip the native build and Python environment steps when using this container.

From the workspace on the host:

```bash
cd ~/trackgraph_ws
source src/hydra_ros/docker/thor/bash_additions.sh
build_container
# Optional, if datasets live outside the workspace:
export TRACKGRAPH_DATASETS_DIR=/absolute/path/to/datasets
# For RViz on an X11 display:
xhost +local:root
run_container
```

The helper derives the workspace path from its own location. Set `TRACKGRAPH_WS`
before sourcing to override it. Dataset mounts are optional; the helper requires
no particular camera or serial-device names. Additional Docker options can be
passed to `run_container`.

Inside the container:

```bash
cd "$HYDRA_WS"
colcon build --base-paths src --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DSEMANTIC_INFERENCE_USE_TRT=OFF
source install/setup.bash
```

Download the DINOv3 checkpoint into the host workspace's `models/dinov3/` as
described in the main guide. The image supplies tracker Python dependencies and
imports the tracker from the mounted source; no native `.venv` is needed.
The container keeps `/root/hydra_ws` as its compatibility path regardless of the
host workspace name. Optional external datasets appear at `/datasets`.

For each additional terminal on the host:

```bash
cd ~/trackgraph_ws
source src/hydra_ros/docker/thor/bash_additions.sh
attach_container
```

In that container shell, source `install/setup.bash` and use the same dataset,
robot, and query commands from the main guide. Rebuild the image when its
Dockerfile or dependencies change.
