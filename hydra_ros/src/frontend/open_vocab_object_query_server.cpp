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
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and
 * should not be interpreted as representing the official policies, either
 * expressed or implied, of the United States Air Force or the U.S. Government.
 * The U.S. Government is authorized to reproduce and distribute reprints for
 * Government purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include "hydra_ros/frontend/open_vocab_object_query_server.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <hydra/common/dsg_types.h>
#include <hydra/common/global_info.h>
#include <hydra/reconstruction/voxel_types.h>
#include <spark_dsg/node_attributes.h>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "hydra_visualizer/drawing.h"

namespace hydra {
namespace {

using hydra_msgs::msg::OpenVocabObjectQueryResult;
using spark_dsg::BoundingBox;
using spark_dsg::DsgLayers;
using spark_dsg::DynamicSceneGraph;
using spark_dsg::NodeId;
using spark_dsg::ObjectNodeAttributes;
using spark_dsg::TrackedObjectNodeAttributes;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr float kEps = 1.0e-9f;

struct QueryOptions {
  uint32_t top_k = 5;
  uint32_t min_mesh_vertices = 0;
  bool include_ignored = false;
  bool allow_encoder_mismatch = false;
};

std_msgs::msg::ColorRGBA makeRankColor(size_t rank, double alpha) {
  static constexpr std::array<std::array<float, 3>, 8> kPalette = {{
      {{0.12f, 0.85f, 0.30f}},
      {{0.95f, 0.62f, 0.12f}},
      {{0.20f, 0.62f, 1.00f}},
      {{0.92f, 0.24f, 0.72f}},
      {{0.92f, 0.88f, 0.18f}},
      {{0.55f, 0.42f, 0.95f}},
      {{0.16f, 0.80f, 0.78f}},
      {{0.95f, 0.34f, 0.24f}},
  }};

  const auto& rgb = kPalette.at(rank % kPalette.size());
  std_msgs::msg::ColorRGBA color;
  color.r = rgb[0];
  color.g = rgb[1];
  color.b = rgb[2];
  color.a = static_cast<float>(std::clamp(alpha, 0.0, 1.0));
  return color;
}

std_msgs::msg::ColorRGBA makeQueryColor(size_t query_index,
                                        size_t rank,
                                        double alpha) {
  auto color = makeRankColor(query_index, alpha);
  const auto brightness = static_cast<float>(std::max(0.55, 1.0 - 0.08 * rank));
  color.r *= brightness;
  color.g *= brightness;
  color.b *= brightness;
  return color;
}

std_msgs::msg::ColorRGBA makeRgba(const std::vector<double>& rgba) {
  std_msgs::msg::ColorRGBA color;
  if (rgba.size() != 4) {
    return color;
  }

  color.r = static_cast<float>(std::clamp(rgba[0], 0.0, 1.0));
  color.g = static_cast<float>(std::clamp(rgba[1], 0.0, 1.0));
  color.b = static_cast<float>(std::clamp(rgba[2], 0.0, 1.0));
  color.a = static_cast<float>(std::clamp(rgba[3], 0.0, 1.0));
  return color;
}

struct ScoreResult {
  OpenVocabObjectQueryResult msg;
};

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& value) {
  geometry_msgs::msg::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

std::optional<geometry_msgs::msg::Point> fallbackPoint(
    const OpenVocabObjectQueryResult& result) {
  const auto is_finite = [](const geometry_msgs::msg::Point& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
  };

  if (result.has_centroid && is_finite(result.centroid)) {
    return result.centroid;
  }

  if (result.bbox_valid && is_finite(result.bbox_center)) {
    return result.bbox_center;
  }

  return std::nullopt;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3f& value) {
  return toPoint(Eigen::Vector3d(value.cast<double>()));
}

geometry_msgs::msg::Vector3 toVector3(const Eigen::Vector3f& value) {
  geometry_msgs::msg::Vector3 vec;
  vec.x = value.x();
  vec.y = value.y();
  vec.z = value.z();
  return vec;
}

std::string bboxTypeToString(BoundingBox::Type type) {
  switch (type) {
    case BoundingBox::Type::AABB:
      return "AABB";
    case BoundingBox::Type::OBB:
      return "OBB";
    case BoundingBox::Type::RAABB:
      return "RAABB";
    case BoundingBox::Type::INVALID:
    default:
      return "INVALID";
  }
}

std::optional<nlohmann::json> trackedMetadata(const ObjectNodeAttributes& attrs) {
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

template <typename T>
std::optional<T> getJsonValue(const nlohmann::json& json, const std::string& key) {
  const auto iter = json.find(key);
  if (iter == json.end() || iter->is_null()) {
    return std::nullopt;
  }

  try {
    return iter->get<T>();
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::vector<uint32_t> getJsonTrackIds(const nlohmann::json& json,
                                      const std::string& key) {
  std::vector<uint32_t> ids;
  const auto iter = json.find(key);
  if (iter == json.end() || !iter->is_array()) {
    return ids;
  }

  for (const auto& entry : *iter) {
    if (!entry.is_number_unsigned() && !entry.is_number_integer()) {
      continue;
    }

    const auto value = entry.get<int64_t>();
    if (value < 0) {
      continue;
    }
    ids.push_back(static_cast<uint32_t>(value));
  }

  return ids;
}

std::vector<uint32_t> getSourceTrackIds(const spark_dsg::SceneGraphNode& node,
                                        const std::optional<nlohmann::json>& tracked) {
  if (const auto tracked_attrs = node.tryAttributes<TrackedObjectNodeAttributes>()) {
    if (!tracked_attrs->source_track_ids.empty()) {
      return tracked_attrs->source_track_ids;
    }
  }

  if (tracked) {
    return getJsonTrackIds(*tracked, "source_track_ids");
  }

  return {};
}

uint64_t getObjectUid(const spark_dsg::SceneGraphNode& node,
                      const std::optional<nlohmann::json>& tracked) {
  if (const auto tracked_attrs = node.tryAttributes<TrackedObjectNodeAttributes>()) {
    if (tracked_attrs->object_uid != 0u) {
      return tracked_attrs->object_uid;
    }
  }

  if (tracked) {
    return getJsonValue<uint64_t>(*tracked, "object_uid").value_or(0u);
  }

  return 0u;
}

bool getIgnored(const spark_dsg::SceneGraphNode& node,
                const std::optional<nlohmann::json>& tracked) {
  if (const auto tracked_attrs = node.tryAttributes<TrackedObjectNodeAttributes>()) {
    if (tracked_attrs->open_vocab_ignore_effective) {
      return true;
    }
  }

  if (tracked) {
    return getJsonValue<bool>(*tracked, "open_vocab_ignore_effective").value_or(false);
  }

  return false;
}

std::string getEncoderId(const std::optional<nlohmann::json>& tracked) {
  if (!tracked) {
    return "";
  }

  return getJsonValue<std::string>(*tracked, "open_vocab_encoder_id").value_or("");
}

std::vector<uint32_t> getFeatureSourceTrackIds(
    const std::optional<nlohmann::json>& tracked) {
  if (!tracked) {
    return {};
  }

  return getJsonTrackIds(*tracked, "open_vocab_feature_source_track_ids");
}

std_msgs::msg::ColorRGBA makeColorMsg(const spark_dsg::Color& color, double alpha) {
  std_msgs::msg::ColorRGBA msg;
  msg.r = static_cast<float>(color.r) / 255.0f;
  msg.g = static_cast<float>(color.g) / 255.0f;
  msg.b = static_cast<float>(color.b) / 255.0f;
  msg.a = static_cast<float>(std::clamp(alpha, 0.0, 1.0));
  return msg;
}

spark_dsg::Color makeColor(const std::vector<int>& rgb) {
  if (rgb.size() < 3) {
    return spark_dsg::Color(90, 90, 90);
  }

  return spark_dsg::Color(static_cast<uint8_t>(std::clamp(rgb[0], 0, 255)),
                          static_cast<uint8_t>(std::clamp(rgb[1], 0, 255)),
                          static_cast<uint8_t>(std::clamp(rgb[2], 0, 255)));
}

bool normalizeFeature(FeatureVector* feature) {
  if (!feature || feature->size() == 0 || !feature->allFinite()) {
    return false;
  }

  const float norm = feature->norm();
  if (!std::isfinite(norm) || norm <= kEps) {
    return false;
  }

  *feature /= norm;
  return true;
}

bool fillFeatureVector(const std::vector<float>& values,
                       FeatureVector* feature,
                       std::string* error) {
  if (!feature) {
    return false;
  }

  if (values.empty()) {
    if (error) {
      *error = "query_feature is empty";
    }
    return false;
  }

  feature->resize(static_cast<Eigen::Index>(values.size()));
  for (size_t i = 0; i < values.size(); ++i) {
    (*feature)(static_cast<Eigen::Index>(i)) = values.at(i);
  }

  if (!normalizeFeature(feature)) {
    if (error) {
      *error = "query feature is non-finite or has near-zero norm";
    }
    return false;
  }

  return true;
}

std::optional<ScoreResult> scoreObjectNode(const spark_dsg::SceneGraphNode& node,
                                           const FeatureVector& query_feature,
                                           const std::string& query_encoder_id,
                                           const QueryOptions& options,
                                           std::string* skip_reason,
                                           std::string* fatal_error) {
  const auto attrs = node.tryAttributes<ObjectNodeAttributes>();
  if (!attrs) {
    if (skip_reason) {
      *skip_reason = "not_object_attributes";
    }
    return std::nullopt;
  }

  const auto tracked = trackedMetadata(*attrs);
  const bool ignored = getIgnored(node, tracked);
  if (ignored && !options.include_ignored) {
    if (skip_reason) {
      *skip_reason = "ignored";
    }
    return std::nullopt;
  }

  if (attrs->mesh_connections.size() < options.min_mesh_vertices) {
    if (skip_reason) {
      *skip_reason = "too_few_mesh_vertices";
    }
    return std::nullopt;
  }

  const auto& matrix = attrs->open_vocab_features;
  if (matrix.rows() <= 0 || matrix.cols() <= 0 || matrix.size() == 0) {
    if (skip_reason) {
      *skip_reason = "missing_open_vocab_features";
    }
    return std::nullopt;
  }

  const auto object_encoder_id = getEncoderId(tracked);
  if (!options.allow_encoder_mismatch) {
    if (query_encoder_id.empty() && !object_encoder_id.empty()) {
      if (fatal_error) {
        *fatal_error = "query encoder id is empty but object " +
                       std::to_string(node.id) + " uses encoder '" +
                       object_encoder_id + "'";
      }
      return std::nullopt;
    }

    if (!query_encoder_id.empty() && object_encoder_id.empty()) {
      if (skip_reason) {
        *skip_reason = "missing_encoder_id";
      }
      return std::nullopt;
    }

    if (!query_encoder_id.empty() && !object_encoder_id.empty() &&
        query_encoder_id != object_encoder_id) {
      if (fatal_error) {
        *fatal_error = "encoder mismatch for object " + std::to_string(node.id) +
                       ": query='" + query_encoder_id + "' object='" +
                       object_encoder_id + "'";
      }
      return std::nullopt;
    }
  }

  if (matrix.rows() != query_feature.size()) {
    if (skip_reason) {
      *skip_reason = "feature_dimension_mismatch";
    }
    return std::nullopt;
  }

  int best_col = -1;
  float best_score = -std::numeric_limits<float>::infinity();
  FeatureVector mean = FeatureVector::Zero(matrix.rows());
  size_t valid_count = 0;
  for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
    const auto feature = matrix.col(col);
    if (!feature.allFinite()) {
      continue;
    }

    const float norm = feature.norm();
    if (!std::isfinite(norm) || norm <= kEps) {
      continue;
    }

    const float score = query_feature.dot(feature / norm);
    if (std::isfinite(score) && score > best_score) {
      best_score = score;
      best_col = static_cast<int>(col);
    }

    mean += feature;
    ++valid_count;
  }

  if (best_col < 0 || !std::isfinite(best_score)) {
    if (skip_reason) {
      *skip_reason = "invalid_open_vocab_features";
    }
    return std::nullopt;
  }

  ScoreResult result;
  auto& msg = result.msg;
  msg.node_id = static_cast<uint64_t>(node.id);
  msg.object_uid = getObjectUid(node, tracked);
  msg.score = best_score;
  msg.best_feature_column = best_col;
  msg.best_source_track_id = -1;
  msg.num_feature_columns = static_cast<uint32_t>(matrix.cols());
  msg.num_mesh_vertices = static_cast<uint32_t>(attrs->mesh_connections.size());
  msg.open_vocab_encoder_id = object_encoder_id;
  msg.is_active = attrs->is_active;
  msg.ignored = ignored;
  msg.has_centroid = attrs->position.allFinite();
  if (msg.has_centroid) {
    msg.centroid = toPoint(attrs->position);
  }

  const auto source_track_ids = getSourceTrackIds(node, tracked);
  msg.source_track_ids.reserve(source_track_ids.size());
  for (const auto id : source_track_ids) {
    msg.source_track_ids.push_back(static_cast<uint64_t>(id));
  }

  const auto feature_source_track_ids = getFeatureSourceTrackIds(tracked);
  if (best_col < static_cast<int>(feature_source_track_ids.size())) {
    msg.best_source_track_id = feature_source_track_ids.at(best_col);
  }

  msg.has_mean_score = false;
  msg.mean_score = 0.0f;
  if (valid_count > 0) {
    mean /= static_cast<float>(valid_count);
    if (normalizeFeature(&mean)) {
      msg.has_mean_score = true;
      msg.mean_score = query_feature.dot(mean);
    }
  }

  msg.bbox_valid = attrs->bounding_box.isValid();
  msg.bbox_type = bboxTypeToString(attrs->bounding_box.type);
  if (msg.bbox_valid) {
    msg.bbox_center = toPoint(attrs->bounding_box.world_P_center);
    msg.bbox_dimensions = toVector3(attrs->bounding_box.dimensions);
  }

  return result;
}

void fillSkipCounts(const std::map<std::string, uint32_t>& skipped,
                    OpenVocabObjectQueryServer::QueryResponse resp) {
  resp->skip_reasons.clear();
  resp->skip_counts.clear();
  for (const auto& [reason, count] : skipped) {
    resp->skip_reasons.push_back(reason);
    resp->skip_counts.push_back(count);
  }
}

std::string summarizeSkips(const std::map<std::string, uint32_t>& skipped) {
  if (skipped.empty()) {
    return "{}";
  }

  std::ostringstream ss;
  ss << "{";
  bool first = true;
  for (const auto& [reason, count] : skipped) {
    if (!first) {
      ss << ", ";
    }
    first = false;
    ss << reason << ": " << count;
  }
  ss << "}";
  return ss.str();
}

std::unordered_set<uint32_t> getResultTrackIds(
    const OpenVocabObjectQueryResult& result) {
  std::unordered_set<uint32_t> track_ids;
  for (const auto track_id : result.source_track_ids) {
    if (track_id > std::numeric_limits<uint32_t>::max()) {
      continue;
    }
    track_ids.insert(static_cast<uint32_t>(track_id));
  }

  if (result.best_source_track_id >= 0 &&
      result.best_source_track_id <=
          static_cast<int64_t>(std::numeric_limits<uint32_t>::max())) {
    track_ids.insert(static_cast<uint32_t>(result.best_source_track_id));
  }

  return track_ids;
}

std::vector<geometry_msgs::msg::Point> collectResultMeshPoints(
    const DynamicSceneGraph& graph,
    const OpenVocabObjectQueryResult& result,
    size_t vertex_stride,
    size_t max_points) {
  std::vector<geometry_msgs::msg::Point> points;
  if (!graph.hasMesh()) {
    return points;
  }

  const auto mesh = graph.mesh();
  if (!mesh || mesh->empty() || !mesh->has_labels) {
    return points;
  }

  const auto track_ids = getResultTrackIds(result);
  if (track_ids.empty()) {
    return points;
  }

  const size_t stride = std::max<size_t>(1, vertex_stride);
  std::unordered_map<uint32_t, size_t> stride_counts;
  stride_counts.reserve(track_ids.size());

  if (max_points > 0) {
    points.reserve(std::min<size_t>(mesh->numVertices(), max_points));
  }

  for (size_t i = 0; i < mesh->numVertices(); ++i) {
    const auto track_id = mesh->label(i);
    if (track_id == InstanceVoxel::NO_TRACK || !track_ids.count(track_id)) {
      continue;
    }

    if ((stride_counts[track_id]++ % stride) != 0) {
      continue;
    }

    const auto& p = mesh->pos(i);
    auto& point = points.emplace_back();
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();

    if (max_points > 0 && points.size() >= max_points) {
      break;
    }
  }

  return points;
}

geometry_msgs::msg::Point labelPosition(
    const std::vector<geometry_msgs::msg::Point>& points,
    const std::optional<geometry_msgs::msg::Point>& fallback,
    double label_height) {
  if (points.empty()) {
    auto point = fallback.value_or(geometry_msgs::msg::Point());
    point.z += label_height;
    return point;
  }

  geometry_msgs::msg::Point point;
  double max_z = -std::numeric_limits<double>::infinity();
  for (const auto& p : points) {
    point.x += p.x;
    point.y += p.y;
    max_z = std::max(max_z, p.z);
  }

  point.x /= static_cast<double>(points.size());
  point.y /= static_cast<double>(points.size());
  point.z = max_z + label_height;
  return point;
}

std::string makeResultLabel(const std::string& query,
                            const OpenVocabObjectQueryResult& result,
                            size_t rank) {
  std::ostringstream ss;
  ss << "#" << (rank + 1);
  if (!query.empty()) {
    ss << " " << query;
  }

  ss << "\nscore=" << std::fixed << std::setprecision(3) << result.score;
  if (result.object_uid != 0u) {
    ss << " uid=" << result.object_uid;
  }
  ss << " node=" << result.node_id;
  if (result.best_source_track_id >= 0) {
    ss << " track=" << result.best_source_track_id;
  }
  return ss.str();
}

}  // namespace

void declare_config(OpenVocabObjectQueryServer::Config::ResultMeshConfig& config) {
  using namespace config;
  name("OpenVocabObjectQueryServer::Config::ResultMeshConfig");
  field(config.enabled, "enabled");
  field(config.topic, "topic");
  field(config.mesh_namespace, "mesh_namespace");
  field(config.background_color_rgb, "background_color_rgb");
  field(config.background_alpha, "background_alpha");
  field(config.result_alpha, "result_alpha");
  field(config.ignored_color_blend_weight, "ignored_color_blend_weight");

  check(config.background_color_rgb.size(), EQ, 3ul, "background_color_rgb");
  check(config.background_alpha, GE, 0.0, "background_alpha");
  check(config.background_alpha, LE, 1.0, "background_alpha");
  check(config.result_alpha, GE, 0.0, "result_alpha");
  check(config.result_alpha, LE, 1.0, "result_alpha");
  check(config.ignored_color_blend_weight, GE, 0.0, "ignored_color_blend_weight");
  check(config.ignored_color_blend_weight, LE, 1.0, "ignored_color_blend_weight");
}

void declare_config(
    OpenVocabObjectQueryServer::Config::ResultHeatmapConfig& config) {
  using namespace config;
  name("OpenVocabObjectQueryServer::Config::ResultHeatmapConfig");
  field(config.enabled, "enabled");
  field(config.topic, "topic");
  field(config.mesh_namespace, "mesh_namespace");
  field(config.background_color_rgba, "background_color_rgba");
  field(config.outer_color_rgba, "outer_color_rgba");
  field(config.inner_color_rgba, "inner_color_rgba");
  field(config.core_color_rgba, "core_color_rgba");

  check(config.background_color_rgba.size(), EQ, 4ul, "background_color_rgba");
  check(config.outer_color_rgba.size(), EQ, 4ul, "outer_color_rgba");
  check(config.inner_color_rgba.size(), EQ, 4ul, "inner_color_rgba");
  check(config.core_color_rgba.size(), EQ, 4ul, "core_color_rgba");
}

void declare_config(
    OpenVocabObjectQueryServer::Config::ResultObjectLayerConfig& config) {
  using namespace config;
  name("OpenVocabObjectQueryServer::Config::ResultObjectLayerConfig");
  field(config.enabled, "enabled");
  field(config.topic, "topic");
  field(config.context_topic, "context_topic");
  field(config.descriptor_topic, "descriptor_topic");
  field(config.marker_namespace, "marker_namespace");
  field(config.z_offset, "z_offset");
  field(config.query_z_offset, "query_z_offset");
  field(config.node_scale, "node_scale");
  field(config.selected_node_scale, "selected_node_scale");
  field(config.glow_scale, "glow_scale");
  field(config.connector_scale, "connector_scale");
  field(config.selected_connector_scale, "selected_connector_scale");
  field(config.label_scale, "label_scale");
  field(config.context_color_rgba, "context_color_rgba");
  field(config.selected_color_rgba, "selected_color_rgba");
  field(config.glow_color_rgba, "glow_color_rgba");
  field(config.context_connector_color_rgba, "context_connector_color_rgba");

  check(config.node_scale, GT, 0.0, "node_scale");
  check(config.selected_node_scale, GT, 0.0, "selected_node_scale");
  check(config.glow_scale, GT, 0.0, "glow_scale");
  check(config.connector_scale, GT, 0.0, "connector_scale");
  check(config.selected_connector_scale, GT, 0.0, "selected_connector_scale");
  check(config.label_scale, GT, 0.0, "label_scale");
  check(config.query_z_offset, GE, 0.0, "query_z_offset");
  check(config.context_color_rgba.size(), EQ, 4ul, "context_color_rgba");
  check(config.selected_color_rgba.size(), EQ, 4ul, "selected_color_rgba");
  check(config.glow_color_rgba.size(), EQ, 4ul, "glow_color_rgba");
  check(config.context_connector_color_rgba.size(),
        EQ,
        4ul,
        "context_connector_color_rgba");
}

void declare_config(
    OpenVocabObjectQueryServer::Config::InspectionTargetsConfig& config) {
  using namespace config;
  name("OpenVocabObjectQueryServer::Config::InspectionTargetsConfig");
  field(config.enabled, "enabled");
  field(config.topic, "topic");
}

void declare_config(OpenVocabObjectQueryServer::Config& config) {
  using namespace config;
  name("OpenVocabObjectQueryServer::Config");
  field(config.enabled, "enabled");
  field(config.service_name, "service_name");
  field(config.text_encoder_service_name, "text_encoder_service_name");
  field(config.image_encoder_service_name, "image_encoder_service_name");
  field(config.text_encoder_timeout_s, "text_encoder_timeout_s");
  field(config.image_encoder_timeout_s, "image_encoder_timeout_s");
  field(config.default_top_k, "default_top_k");
  field(config.result_mesh, "result_mesh");
  field(config.result_heatmap, "result_heatmap");
  field(config.result_object_layer, "result_object_layer");
  field(config.inspection_targets, "inspection_targets");
  field(config.publish_markers, "publish_markers");
  field(config.marker_topic, "marker_topic");
  field(config.marker_frame_id, "marker_frame_id");
  field(config.marker_namespace, "marker_namespace");
  field(config.marker_use_spheres, "marker_use_spheres");
  field(config.marker_point_scale, "marker_point_scale");
  field(config.marker_alpha, "marker_alpha");
  field(config.marker_vertex_stride, "marker_vertex_stride");
  field(config.marker_max_points_per_result, "marker_max_points_per_result");
  field(config.marker_publish_labels, "marker_publish_labels");
  field(config.marker_label_height, "marker_label_height");
  field(config.marker_label_scale, "marker_label_scale");
  field(config.marker_query_z_offset, "marker_query_z_offset");
  field(config.verbosity, "verbosity");

  check(config.marker_query_z_offset, GE, 0.0, "marker_query_z_offset");
}

OpenVocabObjectQueryServer::OpenVocabObjectQueryServer(
    ianvs::NodeHandle nh,
    SharedDsgInfo::Ptr frontend_dsg,
    std::function<void()> stop_mapping)
    : config(config::checkValid(config::fromContext<Config>("open_vocab_object_query"))),
      frontend_dsg_(std::move(frontend_dsg)),
      stop_mapping_(std::move(stop_mapping)),
      resolved_text_encoder_service_name_(
          nh.resolve_name(config.text_encoder_service_name, true)),
      resolved_image_encoder_service_name_(
          nh.resolve_name(config.image_encoder_service_name, true)) {
  if (!config.enabled) {
    LOG(INFO) << "Hydra online open-vocabulary object query service disabled";
    return;
  }

  const auto resolved_service_name = nh.resolve_name(config.service_name, true);
  rclcpp::NodeOptions query_node_options;
  query_node_options.use_global_arguments(false);
  query_node_ = std::make_shared<rclcpp::Node>(
      "hydra_open_vocab_object_query_server", query_node_options);
  server_ = query_node_->create_service<QuerySrv>(
      resolved_service_name,
      std::bind(&OpenVocabObjectQueryServer::handleQuery,
                this,
                std::placeholders::_1,
                std::placeholders::_2),
      rclcpp::ServicesQoS());
  if (config.result_mesh.enabled) {
    const auto resolved_result_mesh_topic =
        nh.resolve_name(config.result_mesh.topic, true);
    result_mesh_pub_ =
        query_node_->create_publisher<kimera_pgmo_msgs::msg::Mesh>(
            resolved_result_mesh_topic, rclcpp::QoS(1).transient_local());
    LOG_IF(INFO, config.verbosity > 0)
        << "Hydra online open-vocabulary query result mesh enabled on '"
        << resolved_result_mesh_topic << "'";
  }
  if (config.result_heatmap.enabled) {
    const auto resolved_result_heatmap_topic =
        nh.resolve_name(config.result_heatmap.topic, true);
    result_heatmap_pub_ =
        query_node_->create_publisher<kimera_pgmo_msgs::msg::Mesh>(
            resolved_result_heatmap_topic, rclcpp::QoS(1).transient_local());
    LOG_IF(INFO, config.verbosity > 0)
        << "Hydra online open-vocabulary query heatmap enabled on '"
        << resolved_result_heatmap_topic << "'";
  }
  if (config.result_object_layer.enabled) {
    const auto resolved_result_object_layer_topic =
        nh.resolve_name(config.result_object_layer.topic, true);
    const auto resolved_result_object_context_topic =
        nh.resolve_name(config.result_object_layer.context_topic, true);
    const auto resolved_result_object_descriptor_topic =
        nh.resolve_name(config.result_object_layer.descriptor_topic, true);
    result_object_layer_pub_ = query_node_->create_publisher<MarkerArray>(
        resolved_result_object_layer_topic,
        rclcpp::QoS(1).reliable().transient_local());
    result_object_context_pub_ = query_node_->create_publisher<MarkerArray>(
        resolved_result_object_context_topic,
        rclcpp::QoS(1).reliable().transient_local());
    result_object_descriptor_pub_ = query_node_->create_publisher<MarkerArray>(
        resolved_result_object_descriptor_topic,
        rclcpp::QoS(1).reliable().transient_local());
    LOG_IF(INFO, config.verbosity > 0)
        << "Hydra online open-vocabulary query object highlights enabled on '"
        << resolved_result_object_layer_topic << "'; washed-out context on '"
        << resolved_result_object_context_topic << "'; descriptors on '"
        << resolved_result_object_descriptor_topic << "'";
  }
  if (config.inspection_targets.enabled) {
    const auto resolved_inspection_targets_topic =
        nh.resolve_name(config.inspection_targets.topic, true);
    inspection_targets_pub_ =
        query_node_->create_publisher<geometry_msgs::msg::PoseArray>(
            resolved_inspection_targets_topic,
            rclcpp::QoS(1).reliable().transient_local());
    LOG_IF(INFO, config.verbosity > 0)
        << "Hydra online open-vocabulary inspection targets enabled on '"
        << resolved_inspection_targets_topic << "'";
  }
  if (config.publish_markers) {
    const auto resolved_marker_topic = nh.resolve_name(config.marker_topic, true);
    marker_pub_ = query_node_->create_publisher<MarkerArray>(
        resolved_marker_topic, rclcpp::QoS(1).reliable().transient_local());
    LOG_IF(INFO, config.verbosity > 0)
        << "Hydra online open-vocabulary query markers ready at '"
        << resolved_marker_topic << "'";
  }
  query_executor_.add_node(query_node_);
  query_thread_ = std::thread([this]() { query_executor_.spin(); });

  LOG_IF(INFO, config.verbosity > 0)
      << "Hydra online open-vocabulary object query service ready at '"
      << resolved_service_name << "' using text encoder service '"
      << resolved_text_encoder_service_name_ << "' and image encoder service '"
      << resolved_image_encoder_service_name_ << "' on a dedicated executor";
}

OpenVocabObjectQueryServer::~OpenVocabObjectQueryServer() {
  if (!query_node_) {
    return;
  }

  query_executor_.cancel();
  if (query_thread_.joinable()) {
    query_thread_.join();
  }
  query_executor_.remove_node(query_node_);
}

bool OpenVocabObjectQueryServer::encodeText(const std::string& query,
                                            FeatureVector* feature,
                                            std::string* encoder_id,
                                            std::string* error) const {
  const auto trimmed = query;
  if (trimmed.empty()) {
    if (error) {
      *error = "query is empty and no query_feature was provided";
    }
    return false;
  }

  rclcpp::NodeOptions text_client_options;
  text_client_options.use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>(
      "hydra_open_vocab_query_text_client", text_client_options);
  auto client = node->create_client<EncodeTextSrv>(resolved_text_encoder_service_name_);
  const auto timeout =
      std::chrono::duration<double>(std::max(0.1, config.text_encoder_timeout_s));
  if (!client->wait_for_service(timeout)) {
    if (error) {
      *error = "timed out waiting for text encoder service '" +
               resolved_text_encoder_service_name_ + "'";
    }
    return false;
  }

  auto request = std::make_shared<EncodeTextSrv::Request>();
  request->prompt = trimmed;
  auto future = client->async_send_request(request).future.share();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto code = executor.spin_until_future_complete(future, timeout);
  executor.remove_node(node);
  if (code != rclcpp::FutureReturnCode::SUCCESS || !future.valid()) {
    if (error) {
      *error = "timed out waiting for text encoder response from '" +
               resolved_text_encoder_service_name_ + "'";
    }
    return false;
  }

  const auto response = future.get();
  if (!response || response->encoder_id.empty() || response->feature.empty()) {
    if (error) {
      *error = "text encoder service returned an empty feature";
    }
    return false;
  }

  if (encoder_id) {
    *encoder_id = response->encoder_id;
  }

  return fillFeatureVector(response->feature, feature, error);
}

bool OpenVocabObjectQueryServer::encodeImage(
    const sensor_msgs::msg::CompressedImage& image,
    FeatureVector* feature,
    std::string* encoder_id,
    std::string* error) const {
  if (image.data.empty()) {
    if (error) {
      *error = "query image is empty";
    }
    return false;
  }

  rclcpp::NodeOptions image_client_options;
  image_client_options.use_global_arguments(false);
  auto node = std::make_shared<rclcpp::Node>(
      "hydra_open_vocab_query_image_client", image_client_options);
  auto client =
      node->create_client<EncodeImageSrv>(resolved_image_encoder_service_name_);
  const auto timeout =
      std::chrono::duration<double>(std::max(0.1, config.image_encoder_timeout_s));
  if (!client->wait_for_service(timeout)) {
    if (error) {
      *error = "timed out waiting for image encoder service '" +
               resolved_image_encoder_service_name_ + "'";
    }
    return false;
  }

  auto request = std::make_shared<EncodeImageSrv::Request>();
  request->image = image;
  auto future = client->async_send_request(request).future.share();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto code = executor.spin_until_future_complete(future, timeout);
  executor.remove_node(node);
  if (code != rclcpp::FutureReturnCode::SUCCESS || !future.valid()) {
    if (error) {
      *error = "timed out waiting for image encoder response from '" +
               resolved_image_encoder_service_name_ + "'";
    }
    return false;
  }

  const auto response = future.get();
  if (!response || !response->success || response->encoder_id.empty() ||
      response->feature.empty()) {
    if (error) {
      *error = response && !response->message.empty()
                   ? response->message
                   : "image encoder service returned an empty feature";
    }
    return false;
  }

  if (encoder_id) {
    *encoder_id = response->encoder_id;
  }

  return fillFeatureVector(response->feature, feature, error);
}

void OpenVocabObjectQueryServer::publishResultMesh(
    const std::vector<VisualizationQueryResults>& queries) const {
  if (!result_mesh_pub_) {
    return;
  }

  kimera_pgmo_msgs::msg::Mesh msg;
  msg.header.stamp = query_node_ ? query_node_->now() : rclcpp::Time(0);
  msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
  msg.ns = config.result_mesh.mesh_namespace;

  if (!frontend_dsg_ || !frontend_dsg_->graph) {
    result_mesh_pub_->publish(msg);
    return;
  }

  std::lock_guard<std::mutex> lock(frontend_dsg_->mutex);
  const DynamicSceneGraph& graph = *frontend_dsg_->graph;
  if (!graph.hasMesh()) {
    result_mesh_pub_->publish(msg);
    return;
  }

  const auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    result_mesh_pub_->publish(msg);
    return;
  }

  const auto background_color = makeColor(config.result_mesh.background_color_rgb);
  struct ColorAccumulator {
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    size_t count = 0;
  };
  std::unordered_map<uint32_t, ColorAccumulator> result_track_colors;
  for (size_t query_index = 0; query_index < queries.size(); ++query_index) {
    const auto& results = queries.at(query_index).results;
    for (size_t rank = 0; rank < results.size(); ++rank) {
      const auto color = makeQueryColor(query_index, rank, 1.0);
      for (const auto track_id : getResultTrackIds(results.at(rank))) {
        auto& accumulated = result_track_colors[track_id];
        accumulated.r += color.r;
        accumulated.g += color.g;
        accumulated.b += color.b;
        ++accumulated.count;
      }
    }
  }

  msg.vertices.resize(mesh->numVertices());
  for (size_t i = 0; i < mesh->numVertices(); ++i) {
    const auto& p = mesh->pos(i);
    auto& vertex = msg.vertices[i];
    vertex.pos.x = p.x();
    vertex.pos.y = p.y();
    vertex.pos.z = p.z();
    vertex.has_color = true;

    vertex.color = makeColorMsg(background_color, config.result_mesh.background_alpha);
    if (mesh->has_labels) {
      const auto track_id = mesh->label(i);
      const auto color_iter = result_track_colors.find(track_id);
      if (track_id != InstanceVoxel::NO_TRACK && color_iter != result_track_colors.end()) {
        const auto count = static_cast<float>(color_iter->second.count);
        vertex.color.r = static_cast<float>(color_iter->second.r) / count;
        vertex.color.g = static_cast<float>(color_iter->second.g) / count;
        vertex.color.b = static_cast<float>(color_iter->second.b) / count;
        vertex.color.a = static_cast<float>(config.result_mesh.result_alpha);
      }
    }
  }

  msg.triangles.resize(mesh->numFaces());
  for (size_t i = 0; i < mesh->numFaces(); ++i) {
    const auto& face = mesh->face(i);
    auto& triangle = msg.triangles[i].vertex_indices;
    triangle[0] = static_cast<uint32_t>(face[0]);
    triangle[1] = static_cast<uint32_t>(face[1]);
    triangle[2] = static_cast<uint32_t>(face[2]);
  }

  result_mesh_pub_->publish(msg);
}

void OpenVocabObjectQueryServer::publishResultHeatmap(
    const std::vector<VisualizationQueryResults>& queries) const {
  if (!result_heatmap_pub_) {
    return;
  }

  kimera_pgmo_msgs::msg::Mesh msg;
  msg.header.stamp = query_node_ ? query_node_->now() : rclcpp::Time(0);
  msg.header.frame_id = GlobalInfo::instance().getFrames().odom;
  msg.ns = config.result_heatmap.mesh_namespace;

  if (!frontend_dsg_ || !frontend_dsg_->graph) {
    result_heatmap_pub_->publish(msg);
    return;
  }

  std::lock_guard<std::mutex> lock(frontend_dsg_->mutex);
  const auto& graph = *frontend_dsg_->graph;
  if (!graph.hasMesh()) {
    result_heatmap_pub_->publish(msg);
    return;
  }

  const auto mesh = graph.mesh();
  if (!mesh || mesh->empty()) {
    result_heatmap_pub_->publish(msg);
    return;
  }

  std::unordered_set<uint32_t> result_track_ids;
  for (const auto& query : queries) {
    for (const auto& result : query.results) {
      const auto track_ids = getResultTrackIds(result);
      result_track_ids.insert(track_ids.begin(), track_ids.end());
    }
  }

  std::vector<uint8_t> levels(mesh->numVertices(), 0);
  if (mesh->has_labels) {
    for (size_t i = 0; i < mesh->numVertices(); ++i) {
      const auto track_id = mesh->label(i);
      if (track_id != InstanceVoxel::NO_TRACK && result_track_ids.count(track_id)) {
        levels.at(i) = 3;
      }
    }
  }

  // Match the accumulated-query visualization: one warm triangle shoulder
  // around selected surfaces, followed by a translucent outer triangle halo.
  for (size_t i = 0; i < mesh->numFaces(); ++i) {
    const auto& face = mesh->face(i);
    if (levels.at(face[0]) != 3 && levels.at(face[1]) != 3 &&
        levels.at(face[2]) != 3) {
      continue;
    }
    for (size_t j = 0; j < 3; ++j) {
      const auto vertex_index = face[j];
      if (levels.at(vertex_index) == 0) {
        levels.at(vertex_index) = 2;
      }
    }
  }
  for (size_t i = 0; i < mesh->numFaces(); ++i) {
    const auto& face = mesh->face(i);
    if (levels.at(face[0]) < 2 && levels.at(face[1]) < 2 &&
        levels.at(face[2]) < 2) {
      continue;
    }
    for (size_t j = 0; j < 3; ++j) {
      const auto vertex_index = face[j];
      if (levels.at(vertex_index) == 0) {
        levels.at(vertex_index) = 1;
      }
    }
  }

  const std::array<std_msgs::msg::ColorRGBA, 4> colors = {
      makeRgba(config.result_heatmap.background_color_rgba),
      makeRgba(config.result_heatmap.outer_color_rgba),
      makeRgba(config.result_heatmap.inner_color_rgba),
      makeRgba(config.result_heatmap.core_color_rgba)};
  msg.vertices.resize(mesh->numVertices());
  for (size_t i = 0; i < mesh->numVertices(); ++i) {
    const auto& p = mesh->pos(i);
    auto& vertex = msg.vertices.at(i);
    vertex.pos.x = p.x();
    vertex.pos.y = p.y();
    vertex.pos.z = p.z();
    vertex.has_color = true;
    const auto level = levels.at(i);
    if (level != 0) {
      vertex.color = colors.at(level);
      continue;
    }

    if (!mesh->has_colors) {
      vertex.color = colors.at(0);
      continue;
    }

    // Match the accumulated-query "metric" style exactly: retain Hydra's
    // reconstructed RGB color and apply heat colors only to query/halo vertices.
    vertex.color = makeColorMsg(mesh->color(i), 0.98);
  }

  msg.triangles.resize(mesh->numFaces());
  for (size_t i = 0; i < mesh->numFaces(); ++i) {
    const auto& face = mesh->face(i);
    auto& triangle = msg.triangles.at(i).vertex_indices;
    triangle[0] = static_cast<uint32_t>(face[0]);
    triangle[1] = static_cast<uint32_t>(face[1]);
    triangle[2] = static_cast<uint32_t>(face[2]);
  }

  result_heatmap_pub_->publish(msg);
}

void OpenVocabObjectQueryServer::publishResultObjectLayer(
    const std::vector<VisualizationQueryResults>& queries) const {
  if ((!result_object_layer_pub_ && !result_object_context_pub_ &&
       !result_object_descriptor_pub_) ||
      !query_node_) {
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = query_node_->now();
  header.frame_id = GlobalInfo::instance().getFrames().odom;

  MarkerArray marker_array;
  auto& clear_marker = marker_array.markers.emplace_back();
  clear_marker.header = header;
  clear_marker.ns = config.result_object_layer.marker_namespace;
  clear_marker.id = 0;
  clear_marker.action = Marker::DELETEALL;

  MarkerArray context_marker_array;
  auto& clear_context_marker = context_marker_array.markers.emplace_back();
  clear_context_marker.header = header;
  clear_context_marker.ns = config.result_object_layer.marker_namespace + "_context";
  clear_context_marker.id = 0;
  clear_context_marker.action = Marker::DELETEALL;

  MarkerArray descriptor_marker_array;
  auto& clear_descriptor_marker = descriptor_marker_array.markers.emplace_back();
  clear_descriptor_marker.header = header;
  clear_descriptor_marker.ns =
      config.result_object_layer.marker_namespace + "_descriptors";
  clear_descriptor_marker.id = 0;
  clear_descriptor_marker.action = Marker::DELETEALL;

  if (frontend_dsg_ && frontend_dsg_->graph) {
    std::lock_guard<std::mutex> lock(frontend_dsg_->mutex);
    const auto& graph = *frontend_dsg_->graph;
    if (!graph.hasLayer(DsgLayers::OBJECTS)) {
      if (result_object_layer_pub_) {
        result_object_layer_pub_->publish(marker_array);
      }
      if (result_object_context_pub_) {
        result_object_context_pub_->publish(context_marker_array);
      }
      if (result_object_descriptor_pub_) {
        result_object_descriptor_pub_->publish(descriptor_marker_array);
      }
      return;
    }

    std::unordered_set<NodeId> selected_node_ids;
    for (const auto& query : queries) {
      for (const auto& result : query.results) {
        selected_node_ids.insert(static_cast<NodeId>(result.node_id));
      }
    }

    const auto make_list_marker = [&](const std::string& suffix,
                                      int32_t type,
                                      double scale,
                                      const std_msgs::msg::ColorRGBA& color) {
      Marker marker;
      marker.header = header;
      marker.ns = config.result_object_layer.marker_namespace + suffix;
      marker.id = 1;
      marker.type = type;
      marker.action = Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;
      marker.color = color;
      return marker;
    };

    const auto context_color = makeRgba(config.result_object_layer.context_color_rgba);
    const auto selected_color =
        makeRgba(config.result_object_layer.selected_color_rgba);
    const auto glow_color = makeRgba(config.result_object_layer.glow_color_rgba);
    const auto context_connector_color =
        makeRgba(config.result_object_layer.context_connector_color_rgba);

    auto context_nodes = make_list_marker(
        "_context_nodes", Marker::CUBE_LIST, config.result_object_layer.node_scale, context_color);
    auto selected_nodes = make_list_marker("_selected_nodes",
                                           Marker::CUBE_LIST,
                                           config.result_object_layer.selected_node_scale,
                                           selected_color);
    auto selected_glow = make_list_marker("_selected_glow",
                                          Marker::SPHERE_LIST,
                                          config.result_object_layer.glow_scale,
                                          glow_color);
    auto context_connectors = make_list_marker(
        "_context_connectors",
        Marker::LINE_LIST,
        config.result_object_layer.connector_scale,
        context_connector_color);
    auto selected_connectors = make_list_marker(
        "_selected_connectors",
        Marker::LINE_LIST,
        config.result_object_layer.selected_connector_scale,
        selected_color);

    std::unordered_map<NodeId, geometry_msgs::msg::Point> elevated_positions;
    const auto& object_layer = graph.getLayer(DsgLayers::OBJECTS);
    for (const auto& [node_id, node] : object_layer.nodes()) {
      const auto& position = node->attributes().position;
      if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
          !std::isfinite(position.z())) {
        continue;
      }

      auto true_position = toPoint(position);
      auto elevated_position = true_position;
      elevated_position.z += config.result_object_layer.z_offset;
      elevated_positions.emplace(node_id, elevated_position);

      if (!selected_node_ids.count(node_id)) {
        context_nodes.points.push_back(elevated_position);
        context_connectors.points.push_back(elevated_position);
        context_connectors.points.push_back(true_position);
        continue;
      }

      // Match the accumulated visualization: remove queried nodes from the
      // contextual CUBE_LIST and reinsert them once, enlarged, with a soft halo.
      selected_nodes.points.push_back(elevated_position);
      selected_glow.points.push_back(elevated_position);
      selected_connectors.points.push_back(elevated_position);
      selected_connectors.points.push_back(true_position);
    }

    if (!context_nodes.points.empty()) {
      context_marker_array.markers.push_back(std::move(context_nodes));
    }
    if (!context_connectors.points.empty()) {
      context_marker_array.markers.push_back(std::move(context_connectors));
    }
    if (!selected_glow.points.empty()) {
      marker_array.markers.push_back(std::move(selected_glow));
      marker_array.markers.push_back(std::move(selected_nodes));
      marker_array.markers.push_back(std::move(selected_connectors));
    }

    for (size_t query_index = 0; query_index < queries.size(); ++query_index) {
      const auto& query = queries.at(query_index);
      for (size_t rank = 0; rank < query.results.size(); ++rank) {
        const auto& result = query.results.at(rank);
        const auto node_id = static_cast<NodeId>(result.node_id);
        const auto position_iter = elevated_positions.find(node_id);
        if (position_iter == elevated_positions.end()) {
          continue;
        }

        auto& label = descriptor_marker_array.markers.emplace_back();
        label.header = header;
        label.ns = config.result_object_layer.marker_namespace + "_descriptors_q" +
                   std::to_string(query_index);
        label.id = static_cast<int32_t>(rank + 1);
        label.type = Marker::TEXT_VIEW_FACING;
        label.action = Marker::ADD;
        label.pose.orientation.w = 1.0;
        label.pose.position = position_iter->second;
        label.pose.position.z += config.marker_label_height +
                                 query_index * config.result_object_layer.query_z_offset;
        label.scale.z = config.result_object_layer.label_scale;
        label.color = selected_color;
        label.text = makeResultLabel(query.label, result, rank);
      }
    }
  }

  if (result_object_layer_pub_) {
    result_object_layer_pub_->publish(marker_array);
  }
  if (result_object_context_pub_) {
    result_object_context_pub_->publish(context_marker_array);
  }
  if (result_object_descriptor_pub_) {
    result_object_descriptor_pub_->publish(descriptor_marker_array);
  }
}

void OpenVocabObjectQueryServer::publishInspectionTargets(
    const std::vector<VisualizationQueryResults>& queries) const {
  if (!inspection_targets_pub_ || !query_node_) {
    return;
  }

  geometry_msgs::msg::PoseArray targets;
  targets.header.stamp = query_node_->now();
  targets.header.frame_id = GlobalInfo::instance().getFrames().odom;
  for (const auto& query : queries) {
    for (const auto& result : query.results) {
      const auto point = fallbackPoint(result);
      if (!point) {
        continue;
      }

      auto& pose = targets.poses.emplace_back();
      pose.position = *point;
      pose.orientation.w = 1.0;
    }
  }

  inspection_targets_pub_->publish(targets);
}

void OpenVocabObjectQueryServer::publishQueryMarkers(
    const std::vector<VisualizationQueryResults>& queries) const {
  if (!marker_pub_ || !query_node_) {
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = query_node_->now();
  header.frame_id = config.marker_frame_id.empty()
                        ? GlobalInfo::instance().getFrames().odom
                        : config.marker_frame_id;

  MarkerArray marker_array;
  auto& clear_marker = marker_array.markers.emplace_back();
  clear_marker.header = header;
  clear_marker.ns = config.marker_namespace;
  clear_marker.id = 0;
  clear_marker.action = Marker::DELETEALL;

  std::vector<std::vector<std::vector<geometry_msgs::msg::Point>>> result_points(
      queries.size());
  if (frontend_dsg_ && frontend_dsg_->graph) {
    std::lock_guard<std::mutex> lock(frontend_dsg_->mutex);
    const auto& graph = *frontend_dsg_->graph;
    for (size_t query_index = 0; query_index < queries.size(); ++query_index) {
      const auto& results = queries.at(query_index).results;
      auto& points = result_points.at(query_index);
      points.resize(results.size());
      for (size_t rank = 0; rank < results.size(); ++rank) {
        points.at(rank) = collectResultMeshPoints(
            graph,
            results.at(rank),
            config.marker_vertex_stride,
            config.marker_max_points_per_result);
      }
    }
  }

  int32_t marker_id = 1;
  for (size_t query_index = 0; query_index < queries.size(); ++query_index) {
    const auto& query = queries.at(query_index);
    for (size_t rank = 0; rank < query.results.size(); ++rank) {
      const auto& result = query.results.at(rank);
      auto points = std::move(result_points.at(query_index).at(rank));
      auto fallback = fallbackPoint(result);
      if (fallback) {
        fallback->z += query_index * config.marker_query_z_offset;
      }
      for (auto& point : points) {
        point.z += query_index * config.marker_query_z_offset;
      }
      if (points.empty() && fallback) {
        points.push_back(*fallback);
      }

      if (points.empty()) {
        continue;
      }

      const auto label_position =
          labelPosition(points, fallback, config.marker_label_height);

      auto& point_marker = marker_array.markers.emplace_back();
      point_marker.header = header;
      point_marker.ns = config.marker_namespace + "_q" +
                        std::to_string(query_index) + "_points";
      point_marker.id = marker_id++;
      point_marker.type =
          config.marker_use_spheres ? Marker::SPHERE_LIST : Marker::CUBE_LIST;
      point_marker.action = Marker::ADD;
      point_marker.pose.orientation.w = 1.0;
      point_marker.scale.x = config.marker_point_scale;
      point_marker.scale.y = config.marker_point_scale;
      point_marker.scale.z = config.marker_point_scale;
      point_marker.color = makeQueryColor(query_index, rank, config.marker_alpha);
      point_marker.points = std::move(points);

      if (!config.marker_publish_labels) {
        continue;
      }

      auto& label = marker_array.markers.emplace_back();
      label.header = header;
      label.ns = config.marker_namespace + "_q" +
                 std::to_string(query_index) + "_labels";
      label.id = marker_id++;
      label.type = Marker::TEXT_VIEW_FACING;
      label.action = Marker::ADD;
      label.pose.orientation.w = 1.0;
      label.pose.position = label_position;
      label.scale.z = config.marker_label_scale;
      label.color = makeQueryColor(query_index, rank, 1.0);
      label.text = makeResultLabel(query.label, result, rank);
    }
  }

  marker_pub_->publish(marker_array);
}

void OpenVocabObjectQueryServer::publishVisualizations() const {
  publishResultMesh(visualized_queries_);
  publishResultHeatmap(visualized_queries_);
  publishResultObjectLayer(visualized_queries_);
  publishInspectionTargets(visualized_queries_);
  publishQueryMarkers(visualized_queries_);
}

void OpenVocabObjectQueryServer::handleQuery(QueryRequest req, QueryResponse resp) {
  resp->success = false;
  resp->message.clear();
  resp->encoder_id.clear();
  resp->mapping_stopped = mapping_stopped_.load();
  resp->objects_total = 0;
  resp->objects_considered = 0;
  resp->results.clear();
  resp->skip_reasons.clear();
  resp->skip_counts.clear();

  try {
    if (req->stop_mapping && !mapping_stopped_.exchange(true)) {
      LOG(INFO) << "Stopping Hydra mapping before online open-vocabulary object query";
      if (stop_mapping_) {
        stop_mapping_();
      }
    }
    resp->mapping_stopped = mapping_stopped_.load();

    FeatureVector query_feature;
    std::string query_encoder_id = req->query_encoder_id;
    std::string error;
    if (!req->query_feature.empty()) {
      if (!fillFeatureVector(req->query_feature, &query_feature, &error)) {
        resp->message = error;
        return;
      }
    } else if (!req->query_image.data.empty()) {
      if (!req->query.empty()) {
        resp->message = "text and image query inputs are mutually exclusive";
        return;
      }
      if (!encodeImage(req->query_image,
                       &query_feature,
                       &query_encoder_id,
                       &error)) {
        resp->message = error;
        return;
      }
    } else if (!encodeText(req->query, &query_feature, &query_encoder_id, &error)) {
      resp->message = error;
      return;
    }
    resp->encoder_id = query_encoder_id;

    if (!frontend_dsg_ || !frontend_dsg_->graph) {
      resp->message = "frontend DSG is not available";
      return;
    }

    QueryOptions options;
    options.top_k = req->top_k == 0 ? config.default_top_k : req->top_k;
    options.min_mesh_vertices = req->min_mesh_vertices;
    options.include_ignored = req->include_ignored;
    options.allow_encoder_mismatch = req->allow_encoder_mismatch;

    std::vector<ScoreResult> scored;
    std::map<std::string, uint32_t> skipped;
    bool has_object_layer = true;
    {
      std::lock_guard<std::mutex> lock(frontend_dsg_->mutex);
      const DynamicSceneGraph& graph = *frontend_dsg_->graph;
      if (!graph.hasLayer(DsgLayers::OBJECTS)) {
        has_object_layer = false;
      } else {
        const auto& objects = graph.getLayer(DsgLayers::OBJECTS);
        resp->objects_total = static_cast<uint32_t>(objects.numNodes());
        for (const auto& [node_id, node] : objects.nodes()) {
          (void)node_id;
          std::string skip_reason;
          std::string fatal_error;
          auto result = scoreObjectNode(*node,
                                        query_feature,
                                        query_encoder_id,
                                        options,
                                        &skip_reason,
                                        &fatal_error);
          if (!fatal_error.empty()) {
            resp->message = fatal_error;
            fillSkipCounts(skipped, resp);
            return;
          }

          if (!skip_reason.empty()) {
            ++skipped[skip_reason];
            continue;
          }

          if (result) {
            scored.push_back(std::move(*result));
          }
        }
      }
    }

    const auto query_label = !req->query_image.data.empty()
                                 ? std::string("image query")
                                 : (!req->query.empty() ? req->query
                                                       : std::string("feature query"));
    const auto update_visualizations = [&]() {
      if (!req->append_to_visualization) {
        visualized_queries_.clear();
      }
      visualized_queries_.push_back({query_label, resp->results});
      publishVisualizations();
    };

    if (!has_object_layer) {
      resp->success = true;
      resp->message = "object layer is not present";
      update_visualizations();
      return;
    }

    resp->objects_considered = static_cast<uint32_t>(scored.size());
    std::sort(scored.begin(), scored.end(), [](const auto& lhs, const auto& rhs) {
      if (lhs.msg.score != rhs.msg.score) {
        return lhs.msg.score > rhs.msg.score;
      }
      if (lhs.msg.num_mesh_vertices != rhs.msg.num_mesh_vertices) {
        return lhs.msg.num_mesh_vertices > rhs.msg.num_mesh_vertices;
      }
      return lhs.msg.node_id < rhs.msg.node_id;
    });

    const auto count =
        std::min<size_t>(static_cast<size_t>(options.top_k), scored.size());
    resp->results.reserve(count);
    for (size_t i = 0; i < count; ++i) {
      resp->results.push_back(scored.at(i).msg);
    }
    fillSkipCounts(skipped, resp);

    resp->success = true;
    std::ostringstream message;
    message << "ranked " << scored.size() << " objects";
    if (!skipped.empty()) {
      message << "; skipped " << summarizeSkips(skipped);
    }
    resp->message = message.str();
    update_visualizations();
  } catch (const std::exception& e) {
    resp->success = false;
    resp->message = e.what();
  }
}

}  // namespace hydra
