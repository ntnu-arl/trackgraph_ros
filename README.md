# ROS 2 Wrapper for TʀᴀᴄᴋGʀᴀᴘʜ

This repository provides the ROS 2 interfaces for [TʀᴀᴄᴋGʀᴀᴘʜ: Online Open-Vocabulary 3D Scene Graphs via Image-Space Tracking](https://github.com/ntnu-arl/trackgraph), based on [Hydra](https://github.com/MIT-SPARK/Hydra).

- [Setup](#setup)
- [Datasets](#datasets)
- [Robot](#robot)
- [Segment queries](#segment-queries)
- [License](#license)

## Setup

Use **Ubuntu 24.04, ROS 2 Jazzy, and an NVIDIA GPU**. Install
[ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
and a compatible NVIDIA driver first. For Jetson Thor, use the optional
[container instructions](docker/thor/README.md) after importing the workspace.

### Download and build

```bash
sudo apt update
sudo apt install git python3-vcstool python3-venv ros-dev-tools
source /opt/ros/jazzy/setup.bash

export TRACKGRAPH_WS=~/trackgraph_ws
mkdir -p "$TRACKGRAPH_WS/src"
cd "$TRACKGRAPH_WS/src"
git clone --branch main https://github.com/ntnu-arl/trackgraph_ros.git hydra_ros
vcs import . < hydra_ros/install/packages.yaml

cd "$TRACKGRAPH_WS"
sudo rosdep init  # skip if rosdep is already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DSEMANTIC_INFERENCE_USE_TRT=OFF
source install/setup.bash
```

The manifest imports all dependencies, including the tracker, on the branches
required by TʀᴀᴄᴋGʀᴀᴘʜ.

### Tracker environment and weights

Create a virtual environment using the system Python so it can use ROS packages:

```bash
cd "$TRACKGRAPH_WS"
/usr/bin/python3 -m venv --system-site-packages .venv
touch .venv/COLCON_IGNORE
source .venv/bin/activate
python -m pip install --upgrade pip
```

Install CUDA-enabled `torch` and `torchvision` in this environment using the
[PyTorch instructions](https://pytorch.org/get-started/locally/) for your GPU and
driver, then install the tracker:

```bash
python -m pip install -e "src/instance_tracking/instance_tracking[open_vocab]"
python -c "import torch; assert torch.cuda.is_available(), 'CUDA is unavailable to PyTorch'"
mkdir -p models/dinov3
```

Request the **DINOv3 ViT-S+/16 LVD-1689M** weights from the official
[DINOv3 download instructions](https://github.com/facebookresearch/dinov3#pretrained-models)
and save the checkpoint here:

```text
<workspace>/models/dinov3/dinov3_vits16plus_pretrain_lvd1689m-4057cbaa.pth
```

Set `DINOV3_WEIGHTS_PATH` to its containing directory if stored elsewhere.
FastSAM and OpenCLIP download their checkpoints on first use; DINOv3 downloads
its architecture code, but requires the local weights above.

### Open a terminal

In each terminal used below (adjust the workspace path if needed):

```bash
cd ~/trackgraph_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

The mapper, tracker, and RGB-D source run separately. Wait for the tracker to finish
loading its models before starting dataset playback.

## Datasets

Run the following commands in separate terminals after sourcing the workspace
and Python environment.

<details>
<summary><strong>uHumans2</strong></summary>

Download the office bag from
[uHumans2](https://web.mit.edu/sparklab/datasets/uHumans2/) and convert it to ROS 2:

```bash
python -m pip install rosbags
rosbags-convert --src /path/to/office.bag --dst /path/to/office_scene
```

Run TʀᴀᴄᴋGʀᴀᴘʜ and play the bag:

```bash
# Terminal 1: mapping and RViz
ros2 launch hydra_ros trackgraph_uhumans2.launch.yaml

# Terminal 2: instance tracking
ros2 launch instance_tracking_ros uhumans2.launch.yaml

# Terminal 3: dataset playback
ros2 bag play /path/to/office_scene --clock \
  --qos-profile-overrides-path src/hydra_ros/config/rosbag_qos.yaml
```

</details>

<details>
<summary><strong>Replica</strong></summary>

Follow the [NICE-SLAM instructions](https://github.com/cvg/nice-slam#replica-1)
to download posed RGB-D data from Replica scenes.

```bash
# Terminal 1: mapping and RViz
ros2 launch hydra_ros trackgraph_replica.launch.yaml

# Terminal 2: instance tracking
ros2 launch instance_tracking_ros instance_tracking.launch.yaml \
  config_name:=openlex_quality namespace:=replica/left_cam use_sim_time:=true \
  rgb_topic:=rgb/image_raw depth_topic:=depth_registered/image_rect

# Terminal 3: dataset playback
ros2 launch hydra_ros publish_replica.launch.yaml \
  dataset_root:=/path/to/Replica scene_name:=room0
```

</details>

<details>
<summary><strong>ScanNet++</strong></summary>

Follow the [ScanNet++ download instructions](https://scannetpp.mlsg.cit.tum.de/scannetpp/#download-the-data)
to obtain the iPhone RGB-D recordings and camera poses.

```bash
# Terminal 1: mapping and RViz
ros2 launch hydra_ros trackgraph_scannetpp.launch.yaml

# Terminal 2: instance tracking
ros2 launch instance_tracking_ros instance_tracking.launch.yaml \
  config_name:=openlex_quality namespace:=scannetpp/left_cam use_sim_time:=true \
  rgb_topic:=rgb/image_raw depth_topic:=depth_registered/image_rect

# Terminal 3: dataset playback
ros2 launch hydra_ros publish_scannetpp.launch.yaml \
  dataset_root:=/path/to/scannetpp scene_name:=0a76e06478
```

</details>

<details>
<summary><strong>HM3D</strong></summary>

Follow the [HOV-SG dataset preparation instructions](https://github.com/hovsg/HOV-SG#habitat-matterport-3d-semantics)
to prepare posed RGB-D trajectories. The ground-truth graph generation step can be skipped.

```bash
# Terminal 1: mapping and RViz
ros2 launch hydra_ros trackgraph_hm3d.launch.yaml

# Terminal 2: instance tracking
ros2 launch instance_tracking_ros instance_tracking.launch.yaml \
  config_name:=openlex_quality namespace:=hm3d/left_cam use_sim_time:=true \
  rgb_topic:=rgb/image_raw depth_topic:=depth_registered/image_rect

# Terminal 3: dataset playback
ros2 launch hydra_ros publish_hm3d.launch.yaml \
  dataset_root:=/path/to/hm3dsem_walks/val scene_name:=00824-Dd4bFSTQ8gi
```

</details>

## Robot

The robot must publish registered RGB and depth images (`sensor_msgs/msg/Image`),
RGB camera intrinsics (`sensor_msgs/msg/CameraInfo`), and TF from the odometry frame
to the robot and camera. Depth must match the RGB resolution and intrinsics.

Set your camera topics and TF frames in
[`trackgraph_robot.launch.yaml`](hydra_ros/launch/trackgraph/trackgraph_robot.launch.yaml),
and set the same RGB and depth topics in the tracker's
[`robot.launch.yaml`](https://github.com/ntnu-arl/instance_tracking/blob/main/instance_tracking_ros/launch/robot.launch.yaml)
before launching:

```bash
# Terminal 1: mapping and RViz
ros2 launch hydra_ros trackgraph_robot.launch.yaml

# Terminal 2: instance tracking
ros2 launch instance_tracking_ros robot.launch.yaml
```

## Segment queries

Keep the mapper and tracker running. In another sourced terminal:

```bash
ros2 run hydra_ros query_open_vocab_objects --text "armchair" --top-k 5
ros2 run hydra_ros query_open_vocab_objects --image /path/to/object.jpg --top-k 5
```

Results appear in the terminal and RViz. The command also publishes ranked segment
centers on `/hydra/query_open_vocab_objects/inspection_targets` as a `PoseArray`.
These are inspection positions, not navigation goals. See
[query and visualization options](doc/queries.md) for multiple queries, output
files, and display controls.

## License

Released under BSD-3-Clause.
