#!/usr/bin/env bash
# Source this file from an imported TrackGraph workspace.
# Optional overrides: TRACKGRAPH_WS, TRACKGRAPH_DATASETS_DIR,
# TRACKGRAPH_IMAGE, TRACKGRAPH_CONTAINER. Existing ROS/package paths stay intact.

TRACKGRAPH_WS="${TRACKGRAPH_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
TRACKGRAPH_IMAGE="${TRACKGRAPH_IMAGE:-trackgraph-thor:dev}"
TRACKGRAPH_CONTAINER="${TRACKGRAPH_CONTAINER:-trackgraph-thor}"

# Replace aliases from older versions when re-sourcing this file.
unalias build_container run_container attach_container 2>/dev/null || true

build_container() {
  docker build -f "$TRACKGRAPH_WS/src/hydra_ros/docker/thor/Dockerfile" \
    -t "$TRACKGRAPH_IMAGE" "$TRACKGRAPH_WS" "$@"
}

run_container() {
  local -a mounts=(--volume "$TRACKGRAPH_WS:/root/hydra_ws")
  if [[ -n "${TRACKGRAPH_DATASETS_DIR:-}" ]]; then
    if [[ ! -d "$TRACKGRAPH_DATASETS_DIR" ]]; then
      echo "Dataset directory does not exist: $TRACKGRAPH_DATASETS_DIR" >&2
      return 1
    fi
    mounts+=(--volume "$TRACKGRAPH_DATASETS_DIR:/datasets:ro")
  fi
  if [[ -n "${DISPLAY:-}" ]]; then
    mounts+=(--volume /tmp/.X11-unix:/tmp/.X11-unix --env "DISPLAY=$DISPLAY")
  fi
  docker run --rm --name "$TRACKGRAPH_CONTAINER" --net=host --ipc=host \
    --privileged --runtime nvidia --ulimit memlock=-1 --ulimit stack=67108864 \
    --group-add dialout --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}" \
    --env HYDRA_WS=/root/hydra_ws --env DATASETS_DIR=/datasets \
    --env XDG_CACHE_HOME=/root/hydra_ws/.cache \
    --env TORCH_HOME=/root/hydra_ws/.cache/torch \
    --env HF_HOME=/root/hydra_ws/.cache/huggingface \
    "${mounts[@]}" --workdir /root/hydra_ws -it "$@" "$TRACKGRAPH_IMAGE" /bin/bash
}

attach_container() {
  docker exec -it "$TRACKGRAPH_CONTAINER" /bin/bash
}
