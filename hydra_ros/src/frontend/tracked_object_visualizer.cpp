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
#include "hydra_ros/frontend/tracked_object_visualizer.h"

#include <algorithm>
#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/voxel_types.h>
#include <hydra/utils/mesh_utilities.h>
#include <hydra_visualizer/color/colormap_utilities.h>
#include <hydra_visualizer/color/color_parsing.h>

#include <limits>
#include <optional>
#include <unordered_map>
#include <vector>

namespace hydra {
namespace {

static const auto registration =
    config::RegistrationWithConfig<GraphBuilder::Sink,
                                   TrackedObjectVisualizer,
                                   TrackedObjectVisualizer::Config>(
        "TrackedObjectVisualizer");

struct VertexOwnerInfo {
  NodeId owner = 0;
  uint64_t object_uid = 0;
  uint32_t source_track_id = 0;
  spark_dsg::Color color;
  bool conflict = false;
};

struct TrackedObjectInfo {
  uint64_t object_uid = 0;
  std::vector<uint32_t> source_track_ids;
  bool effective_ignore = false;
};

std::optional<TrackedObjectInfo> getTrackedObjectInfo(const ObjectNodeAttributes& attrs) {
  const auto* tracked = hydra::getTrackedObjectNodeAttributes(attrs);
  if (!tracked || tracked->source_track_ids.empty()) {
    return std::nullopt;
  }
  return TrackedObjectInfo{
      tracked->object_uid, tracked->source_track_ids, tracked->open_vocab_ignore_effective};
}

struct LabelCentroidInfo {
  uint64_t object_uid = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double max_z = -std::numeric_limits<double>::infinity();
  size_t count = 0;
};

}  // namespace

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

void declare_config(TrackedObjectVisualizer::Config& config) {
  using namespace config;
  name("TrackedObjectVisualizer::Config");
  field(config.module_ns, "module_ns");
  field(config.topic, "topic");
  field(config.layer_id, "layer_id");
  field(config.point_scale, "point_scale");
  field(config.point_alpha, "point_alpha");
  field(config.use_spheres, "use_spheres");
  field(config.active_only, "active_only");
  field(config.min_mesh_connections, "min_mesh_connections");
  field(config.vertex_stride, "vertex_stride");
  field(config.show_conflicts, "show_conflicts");
  field(config.conflict_color, "conflict_color");
  field(config.draw_object_ids, "draw_object_ids");
  field(config.label_topic, "label_topic");
  field(config.label_height, "label_height");
  field(config.label_scale, "label_scale");
  field(config.label_alpha, "label_alpha");
  field(config.label_color, "label_color");
}

TrackedObjectVisualizer::TrackedObjectVisualizer(const Config& config)
    : config(config::checkValid(config)),
      nh_(ianvs::NodeHandle::this_node(config.module_ns)),
      pubs_(nh_),
      label_pubs_(nh_) {}

std::string TrackedObjectVisualizer::printInfo() const { return config::toString(config); }

void TrackedObjectVisualizer::call(uint64_t timestamp_ns,
                                   const DynamicSceneGraph& graph,
                                   const BackendInput& /*backend_input*/) const {
  struct CachedMessages {
    bool built = false;
    Marker::UniquePtr points;
    MarkerArray::UniquePtr labels;
  };

  CachedMessages cache;
  auto build_messages = [&]() {
    if (cache.built) {
      return;
    }
    cache.built = true;

    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(timestamp_ns);
    header.frame_id = GlobalInfo::instance().getFrames().odom;

    cache.points = std::make_unique<Marker>();
    auto& points_msg = *cache.points;
    points_msg.header = header;
    points_msg.ns = "tracked_object_mesh_vertices";
    points_msg.id = 0;
    points_msg.type = config.use_spheres ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
    points_msg.action = Marker::ADD;
    points_msg.pose.orientation.w = 1.0;
    points_msg.scale.x = config.point_scale;
    points_msg.scale.y = config.point_scale;
    points_msg.scale.z = config.point_scale;
    // Some RViz marker implementations ignore per-point alpha for *_LIST markers and use
    // the top-level marker alpha instead.
    points_msg.color.a = config.point_alpha;
    points_msg.color.r = 1.0;
    points_msg.color.g = 1.0;
    points_msg.color.b = 1.0;

    if (config.draw_object_ids) {
      cache.labels = std::make_unique<MarkerArray>();
      auto& clear_marker = cache.labels->markers.emplace_back();
      clear_marker.header = header;
      clear_marker.action = Marker::DELETEALL;
    }

    if (!graph.hasMesh()) {
      return;
    }

    const auto mesh = graph.mesh();
    const bool has_layer = graph.hasLayer(config.layer_id);
    if (!mesh || mesh->empty() || !mesh->has_labels || !has_layer) {
      return;
    }

    const auto& objects = graph.getLayer(config.layer_id);
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
    size_t passed_size_filter = 0;

    for (const auto& [node_id, node] : objects.nodes()) {
      const auto* attrs = node->tryAttributes<ObjectNodeAttributes>();
      if (!attrs) {
        continue;
      }

      const auto tracked_info = getTrackedObjectInfo(*attrs);
      if (!tracked_info) {
        continue;
      }

      if (tracked_info->effective_ignore) {
        continue;
      }

      if (config.active_only && !attrs->is_active) {
        continue;
      }

      bool has_visible_track = false;
      for (const auto track_id : tracked_info->source_track_ids) {
        if (track_counts[track_id] < config.min_mesh_connections) {
          continue;
        }

        has_visible_track = true;
        auto [it, inserted] = owners.emplace(
            track_id,
            VertexOwnerInfo{
                node_id, tracked_info->object_uid, track_id, attrs->color, false});
        if (!inserted && it->second.owner != node_id) {
          it->second.conflict = true;
        }
      }

      if (has_visible_track) {
        ++passed_size_filter;
      }
    }

    points_msg.points.reserve(mesh->numVertices());
    points_msg.colors.reserve(mesh->numVertices());

    std::unordered_map<NodeId, LabelCentroidInfo> label_centroids;
    if (config.draw_object_ids) {
      label_centroids.reserve(passed_size_filter);
    }

    const size_t stride = std::max<size_t>(1, config.vertex_stride);
    std::unordered_map<uint32_t, size_t> track_stride_counts;
    for (size_t vertex_idx = 0; vertex_idx < mesh->numVertices(); ++vertex_idx) {
      const auto track_id = mesh->label(vertex_idx);
      if (track_id == InstanceVoxel::NO_TRACK) {
        continue;
      }

      const auto info_iter = owners.find(track_id);
      if (info_iter == owners.end()) {
        continue;
      }

      if ((track_stride_counts[track_id]++ % stride) != 0) {
        continue;
      }

      const auto& info = info_iter->second;
      const auto& p = mesh->pos(vertex_idx);
      auto& point = points_msg.points.emplace_back();
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();

      if (info.conflict && config.show_conflicts) {
        points_msg.colors.push_back(
            visualizer::makeColorMsg(config.conflict_color, config.point_alpha));
      } else {
        points_msg.colors.push_back(visualizer::makeColorMsg(info.color, config.point_alpha));
      }

      if (config.draw_object_ids) {
        auto [centroid_it, inserted] =
            label_centroids.emplace(info.owner, LabelCentroidInfo{info.object_uid});
        auto& centroid = centroid_it->second;
        if (inserted) {
          centroid.object_uid = info.object_uid;
        }

        centroid.sum_x += p.x();
        centroid.sum_y += p.y();
        centroid.max_z = std::max<double>(centroid.max_z, p.z());
        ++centroid.count;
      }
    }

    if (config.draw_object_ids && cache.labels) {
      std::vector<LabelCentroidInfo> centroids;
      centroids.reserve(label_centroids.size());
      for (const auto& [node_id, centroid] : label_centroids) {
        (void)node_id;
        if (centroid.count == 0) {
          continue;
        }
        centroids.push_back(centroid);
      }

      std::sort(centroids.begin(),
                centroids.end(),
                [](const auto& lhs, const auto& rhs) {
                  return lhs.object_uid < rhs.object_uid;
                });

      int32_t marker_id = 0;
      for (const auto& centroid : centroids) {
        auto& marker = cache.labels->markers.emplace_back();
        marker.header = header;
        marker.ns = "tracked_object_ids";
        marker.id = marker_id++;
        marker.type = Marker::TEXT_VIEW_FACING;
        marker.action = Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = centroid.sum_x / static_cast<double>(centroid.count);
        marker.pose.position.y = centroid.sum_y / static_cast<double>(centroid.count);
        marker.pose.position.z = centroid.max_z + config.label_height;
        marker.scale.z = config.label_scale;
        marker.color = visualizer::makeColorMsg(config.label_color, config.label_alpha);
        marker.text = std::to_string(centroid.object_uid);
      }
    }
  };

  pubs_.publish(config.topic, [&]() {
    build_messages();
    return std::move(cache.points);
  });

  if (config.draw_object_ids) {
    label_pubs_.publish(config.label_topic, [&]() {
      build_messages();
      return std::move(cache.labels);
    });
  }
}

}  // namespace hydra
