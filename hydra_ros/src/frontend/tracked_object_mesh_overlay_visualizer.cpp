/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/frontend/tracked_object_mesh_overlay_visualizer.h"

#include <algorithm>
#include <cctype>
#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/voxel_types.h>
#include <hydra/utils/mesh_utilities.h>
#include <hydra_visualizer/color/color_parsing.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <nlohmann/json.hpp>
#include <spark_dsg/colormaps.h>

#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilder::Sink,
                                   TrackedObjectMeshOverlayVisualizer,
                                   TrackedObjectMeshOverlayVisualizer::Config>(
        "TrackedObjectMeshOverlayVisualizer");

struct VertexOwnerInfo {
  spark_dsg::Color color;
  double alpha = 1.0;
  bool is_active = true;
  bool is_ignored = false;
};

struct TrackedObjectInfo {
  uint64_t object_uid = 0;
  std::vector<uint32_t> source_track_ids;
  std::unordered_map<uint32_t, spark_dsg::Color> source_track_colors;
  bool effective_ignore = false;
};

std::optional<nlohmann::json> getTrackedMetadata(const ObjectNodeAttributes& attrs) {
  const auto& metadata = attrs.metadata.get();
  if (!metadata.is_object()) {
    return std::nullopt;
  }

  const auto iter = metadata.find("hydra_tracked_object");
  if (iter == metadata.end() || !iter->is_object()) {
    return std::nullopt;
  }

  return *iter;
}

std::unordered_map<uint32_t, spark_dsg::Color> getSourceTrackColors(
    const ObjectNodeAttributes& attrs) {
  std::unordered_map<uint32_t, spark_dsg::Color> colors;
  const auto tracked = getTrackedMetadata(attrs);
  if (!tracked) {
    return colors;
  }

  const auto iter = tracked->find("source_track_colors");
  if (iter == tracked->end() || !iter->is_object()) {
    return colors;
  }

  for (const auto& [key, value] : iter->items()) {
    uint32_t track_id = 0;
    try {
      const auto parsed = std::stoul(key);
      if (parsed > std::numeric_limits<uint32_t>::max()) {
        continue;
      }
      track_id = static_cast<uint32_t>(parsed);
    } catch (const std::exception&) {
      continue;
    }

    if (!value.is_array() || value.size() < 3) {
      continue;
    }

    try {
      colors.emplace(track_id,
                     spark_dsg::Color(
                         static_cast<uint8_t>(std::clamp(value.at(0).get<int>(), 0, 255)),
                         static_cast<uint8_t>(std::clamp(value.at(1).get<int>(), 0, 255)),
                         static_cast<uint8_t>(std::clamp(value.at(2).get<int>(), 0, 255))));
    } catch (const std::exception&) {
      continue;
    }
  }

  return colors;
}

std::optional<TrackedObjectInfo> getTrackedObjectInfo(const ObjectNodeAttributes& attrs) {
  const auto* tracked = hydra::getTrackedObjectNodeAttributes(attrs);
  if (!tracked || tracked->source_track_ids.empty()) {
    return std::nullopt;
  }
  return TrackedObjectInfo{
      tracked->object_uid,
      tracked->source_track_ids,
      getSourceTrackColors(attrs),
      tracked->open_vocab_ignore_effective};
}

bool trackedMeshOverlayEnabled() {
  static const bool enabled = []() {
    const char* raw = std::getenv("HYDRA_ENABLE_TRACKED_OBJECT_MESH_OVERLAY");
    if (!raw) {
      return true;
    }

    std::string value(raw);
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

    return value != "0" && value != "false" && value != "off" && value != "no";
  }();

  return enabled;
}

spark_dsg::Color getTrackColor(uint32_t track_id) {
  return spark_dsg::colormaps::rainbowId(track_id);
}

}  // namespace

using kimera_pgmo_msgs::msg::Mesh;

void declare_config(TrackedObjectMeshOverlayVisualizer::Config& config) {
  using namespace config;
  name("TrackedObjectMeshOverlayVisualizer::Config");
  field(config.module_ns, "module_ns");
  field(config.topic, "topic");
  field(config.layer_id, "layer_id");
  enum_field(config.display_mode,
             "display_mode",
             {{TrackedObjectMeshOverlayVisualizer::DisplayMode::OWNER, "owner"},
              {TrackedObjectMeshOverlayVisualizer::DisplayMode::OWNER_STATE, "owner_state"},
              {TrackedObjectMeshOverlayVisualizer::DisplayMode::RAW_TRACK, "raw_track"}});
  field(config.active_only, "active_only");
  field(config.min_mesh_connections, "min_mesh_connections");
  field(config.vertex_stride, "vertex_stride");
  field(config.tracked_alpha, "tracked_alpha");
  field(config.raw_track_alpha, "raw_track_alpha");
  field(config.ignored_alpha, "ignored_alpha");
  field(config.ignored_color_blend_weight, "ignored_color_blend_weight");
  field(config.unassigned_alpha, "unassigned_alpha");
  field(config.unassigned_color, "unassigned_color");
  field(config.no_track_alpha, "no_track_alpha");
  field(config.no_track_color, "no_track_color");
  field(config.archived_owner_alpha, "archived_owner_alpha");
  field(config.archived_owner_color, "archived_owner_color");
  field(config.ignored_owner_color, "ignored_owner_color");

  check(config.tracked_alpha, GE, 0.0, "tracked_alpha");
  check(config.tracked_alpha, LE, 1.0, "tracked_alpha");
  check(config.raw_track_alpha, GE, 0.0, "raw_track_alpha");
  check(config.raw_track_alpha, LE, 1.0, "raw_track_alpha");
  check(config.ignored_alpha, GE, 0.0, "ignored_alpha");
  check(config.ignored_alpha, LE, 1.0, "ignored_alpha");
  check(config.ignored_color_blend_weight, GE, 0.0, "ignored_color_blend_weight");
  check(config.ignored_color_blend_weight, LE, 1.0, "ignored_color_blend_weight");
  check(config.unassigned_alpha, GE, 0.0, "unassigned_alpha");
  check(config.unassigned_alpha, LE, 1.0, "unassigned_alpha");
  check(config.no_track_alpha, GE, 0.0, "no_track_alpha");
  check(config.no_track_alpha, LE, 1.0, "no_track_alpha");
  check(config.archived_owner_alpha, GE, 0.0, "archived_owner_alpha");
  check(config.archived_owner_alpha, LE, 1.0, "archived_owner_alpha");
}

TrackedObjectMeshOverlayVisualizer::TrackedObjectMeshOverlayVisualizer(const Config& config)
    : config(config::checkValid(config)),
      nh_(ianvs::NodeHandle::this_node(config.module_ns)),
      pubs_(nh_) {}

std::string TrackedObjectMeshOverlayVisualizer::printInfo() const {
  return config::toString(config);
}

void TrackedObjectMeshOverlayVisualizer::call(uint64_t timestamp_ns,
                                              const DynamicSceneGraph& graph,
                                              const BackendInput& /*backend_input*/) const {
  if (!trackedMeshOverlayEnabled()) {
    (void)timestamp_ns;
    (void)graph;
    return;
  }

  pubs_.publish(config.topic, [&]() {
    auto msg = std::make_unique<Mesh>();
    msg->header.stamp = rclcpp::Time(timestamp_ns);
    msg->header.frame_id = GlobalInfo::instance().getFrames().odom;
    msg->ns = "tracked_object_mesh_overlay";

    if (!graph.hasMesh() || !graph.hasLayer(config.layer_id)) {
      return msg;
    }

    const auto mesh = graph.mesh();
    if (!mesh || mesh->empty()) {
      return msg;
    }

    const auto& objects = graph.getLayer(config.layer_id);
    if (!mesh->has_labels) {
      return msg;
    }

    std::unordered_map<uint32_t, size_t> track_counts;
    track_counts.reserve(mesh->numVertices());
    for (size_t i = 0; i < mesh->numVertices(); ++i) {
      const auto track_id = mesh->label(i);
      if (track_id == InstanceVoxel::NO_TRACK) {
        continue;
      }
      ++track_counts[track_id];
    }

    std::unordered_map<uint32_t, VertexOwnerInfo> owners;
    owners.reserve(objects.numNodes());
    std::unordered_map<uint32_t, spark_dsg::Color> track_colors;
    track_colors.reserve(objects.numNodes());

    const size_t stride = std::max<size_t>(1, config.vertex_stride);
    for (const auto& [node_id, node] : objects.nodes()) {
      (void)node_id;
      const auto* attrs = node->tryAttributes<ObjectNodeAttributes>();
      if (!attrs) {
        continue;
      }

      const auto tracked_info = getTrackedObjectInfo(*attrs);
      if (!tracked_info) {
        continue;
      }

      if (config.active_only && config.display_mode == DisplayMode::OWNER &&
          !attrs->is_active) {
        continue;
      }

      bool has_visible_track = false;
      for (const auto track_id : tracked_info->source_track_ids) {
        if (track_counts[track_id] < config.min_mesh_connections) {
          continue;
        }

        has_visible_track = true;
        const auto overlay_color = tracked_info->effective_ignore
                                       ? attrs->color.blend(
                                             config.unassigned_color,
                                             config.ignored_color_blend_weight)
                                       : attrs->color;
        const double overlay_alpha =
            tracked_info->effective_ignore ? config.ignored_alpha : config.tracked_alpha;
        const auto color_iter = tracked_info->source_track_colors.find(track_id);
        if (color_iter != tracked_info->source_track_colors.end()) {
          track_colors.try_emplace(track_id, color_iter->second);
        } else if (tracked_info->source_track_ids.size() == 1) {
          track_colors.try_emplace(track_id, attrs->color);
        }
        // First wins for deterministic conflict handling if duplicate object nodes exist.
        owners.try_emplace(track_id,
                           VertexOwnerInfo{overlay_color,
                                           overlay_alpha,
                                           attrs->is_active,
                                           tracked_info->effective_ignore});
      }

      if (!has_visible_track) {
        continue;
      }
    }

    msg->vertices.resize(mesh->numVertices());
    std::unordered_map<uint32_t, size_t> track_stride_counts;
    for (size_t i = 0; i < mesh->numVertices(); ++i) {
      const auto& p = mesh->pos(i);
      auto& vertex = msg->vertices[i];
      vertex.pos.x = p.x();
      vertex.pos.y = p.y();
      vertex.pos.z = p.z();
      vertex.has_color = true;

      const auto track_id = mesh->label(i);
      const bool stride_selected =
          config.display_mode == DisplayMode::OWNER &&
          track_id != InstanceVoxel::NO_TRACK &&
          ((track_stride_counts[track_id]++ % stride) == 0);
      const auto owner_iter = owners.find(track_id);

      if (config.display_mode == DisplayMode::RAW_TRACK) {
        if (track_id == InstanceVoxel::NO_TRACK) {
          vertex.color = visualizer::makeColorMsg(config.no_track_color, config.no_track_alpha);
        } else {
          const auto color_iter = track_colors.find(track_id);
          const auto color =
              color_iter == track_colors.end() ? getTrackColor(track_id) : color_iter->second;
          vertex.color = visualizer::makeColorMsg(color, config.raw_track_alpha);
        }
        continue;
      }

      if (config.display_mode == DisplayMode::OWNER_STATE) {
        if (track_id == InstanceVoxel::NO_TRACK) {
          vertex.color = visualizer::makeColorMsg(config.no_track_color, config.no_track_alpha);
        } else if (owner_iter == owners.end()) {
          vertex.color =
              visualizer::makeColorMsg(config.unassigned_color, config.unassigned_alpha);
        } else if (owner_iter->second.is_ignored) {
          vertex.color =
              visualizer::makeColorMsg(config.ignored_owner_color, config.ignored_alpha);
        } else if (!owner_iter->second.is_active) {
          vertex.color = visualizer::makeColorMsg(config.archived_owner_color,
                                                  config.archived_owner_alpha);
        } else {
          vertex.color = visualizer::makeColorMsg(owner_iter->second.color,
                                                  owner_iter->second.alpha);
        }
        continue;
      }

      if (stride_selected && owner_iter != owners.end()) {
        vertex.color = visualizer::makeColorMsg(owner_iter->second.color,
                                                owner_iter->second.alpha);
      } else if (track_id == InstanceVoxel::NO_TRACK) {
        vertex.color = visualizer::makeColorMsg(config.no_track_color, config.no_track_alpha);
      } else {
        vertex.color = visualizer::makeColorMsg(config.unassigned_color, config.unassigned_alpha);
      }
    }

    msg->triangles.resize(mesh->numFaces());
    for (size_t i = 0; i < mesh->numFaces(); ++i) {
      const auto& face = mesh->face(i);
      auto& triangle = msg->triangles[i].vertex_indices;
      triangle[0] = static_cast<uint32_t>(face[0]);
      triangle[1] = static_cast<uint32_t>(face[1]);
      triangle[2] = static_cast<uint32_t>(face[2]);
    }

    return msg;
  });
}

}  // namespace hydra
