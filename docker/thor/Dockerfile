FROM nvcr.io/nvidia/pytorch:25.10-py3 AS ros

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

# Fix NVIDIA HPC-X / UCC / UCX library ordering for torch import.
ENV LD_LIBRARY_PATH=/opt/hpcx/ucx/lib:/opt/hpcx/ucc/lib:/opt/hpcx/ompi/lib:${LD_LIBRARY_PATH}

WORKDIR /developer

RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /etc/apt/trusted.gpg.d/ros.gpg && \
    echo "deb http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y \
      ros-${ROS_DISTRO}-desktop \
      python3-rosdep \
      python3-colcon-common-extensions \
      ros-${ROS_DISTRO}-depth-image-proc \
      ros-${ROS_DISTRO}-rqt-tf-tree \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-pcl-ros     \
      ros-${ROS_DISTRO}-tf2-eigen   \
      ros-${ROS_DISTRO}-rviz2 \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

FROM ros AS trackgraph

RUN apt-get update && apt-get install -y \
    python3-pip python3-venv build-essential git curl wget tmux vim htop net-tools usbutils \
    nlohmann-json3-dev libgoogle-glog-dev libcli11-dev libeigen3-dev libzmq5-dev pipx \
    libjsoncpp-dev libspdlog-dev libcurl4-openssl-dev cmake && \
    pipx install rosbags && \
    rm -rf /var/lib/apt/lists/*

# Realsense sdk
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc | \
    gpg --dearmor | sudo tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo `lsb_release -cs` main" | \
    tee /etc/apt/sources.list.d/librealsense.list && \
    apt-get update && apt-get install -y librealsense2-utils librealsense2-dev && \
    rm -rf /var/lib/apt/lists/*

COPY src /tmp/deps/src

RUN apt-get update && \
    rosdep install --from-paths /tmp/deps/src --ignore-src --rosdistro=${ROS_DISTRO} \
      --skip-keys "python3-torch librealsense2" -r -y && \
    rm -rf /tmp/deps /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
      "numpy<2" termcolor torchmetrics ultralytics \
      "spark_config @ git+https://github.com/MIT-SPARK/Spark-Config.git"

# Open-vocabulary tracker dependencies. Keep these separate and dependency-free so
# pip never replaces NVIDIA's PyTorch / torchvision packages in the base image.
RUN python3 -m pip install --no-cache-dir --no-deps \
      open_clip_torch timm ftfy huggingface_hub

# Make runtime-mounted instance_tracking importable without repeated pip install -e.
ENV PYTHONPATH=/root/hydra_ws/src/instance_tracking/instance_tracking/python:${PYTHONPATH}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Compatibility path for configs that still use /hostwork/hydra_ws.
RUN mkdir -p /hostwork && ln -s /root/hydra_ws /hostwork/hydra_ws

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/hydra_ws/install/setup.bash ] && source /root/hydra_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export DINOV3_WEIGHTS_PATH=/root/hydra_ws/models/dinov3" >> /root/.bashrc && \
    echo "export PYTHONPATH=/root/hydra_ws/src/instance_tracking/instance_tracking/python:\${PYTHONPATH}" >> /root/.bashrc && \
    echo "alias cb='colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DSEMANTIC_INFERENCE_USE_TRT=OFF'" >> /root/.bashrc && \
    echo "alias qwe='source install/setup.bash'" >> /root/.bashrc
