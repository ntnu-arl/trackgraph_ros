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
 * and conclusions contained in this document are those of the authors and
 * should not be interpreted as representing the official policies, either
 * expressed or implied, of the United States Air Force or the U.S. Government.
 * The U.S. Government is authorized to reproduce and distribute reprints for
 * Government purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#pragma once

#include <hydra/common/shared_dsg_info.h>
#include <hydra/openset/openset_types.h>
#include <hydra_msgs/srv/query_open_vocab_objects.hpp>
#include <ianvs/node_handle.h>
#include <instance_tracking_msgs/srv/encode_open_vocab_image.hpp>
#include <instance_tracking_msgs/srv/encode_open_vocab_text.hpp>
#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace hydra {

class OpenVocabObjectQueryServer {
 public:
  using QuerySrv = hydra_msgs::srv::QueryOpenVocabObjects;
  using QueryRequest = QuerySrv::Request::SharedPtr;
  using QueryResponse = QuerySrv::Response::SharedPtr;
  using EncodeImageSrv = instance_tracking_msgs::srv::EncodeOpenVocabImage;
  using EncodeTextSrv = instance_tracking_msgs::srv::EncodeOpenVocabText;

  struct Config {
    struct ResultMeshConfig {
      bool enabled = false;
      std::string topic = "tracked_objects/open_vocab_query_mesh";
      std::string mesh_namespace = "tracked_open_vocab_query_mesh";
      std::vector<int> background_color_rgb{90, 90, 90};
      double background_alpha = 0.35;
      double result_alpha = 1.0;
      double ignored_color_blend_weight = 0.85;
    };

    struct ResultHeatmapConfig {
      bool enabled = false;
      std::string topic = "tracked_objects/open_vocab_query_heatmap";
      std::string mesh_namespace = "tracked_open_vocab_query_heatmap";
      std::vector<double> background_color_rgba{0.52, 0.53, 0.55, 0.28};
      std::vector<double> outer_color_rgba{0.72, 0.08, 0.15, 0.58};
      std::vector<double> inner_color_rgba{0.98, 0.47, 0.07, 0.86};
      std::vector<double> core_color_rgba{0.99, 0.98, 0.62, 1.0};
    };

    struct ResultObjectLayerConfig {
      bool enabled = false;
      std::string topic = "tracked_objects/open_vocab_query_object_layer";
      std::string context_topic = "tracked_objects/open_vocab_query_object_context";
      std::string descriptor_topic =
          "tracked_objects/open_vocab_query_object_descriptors";
      std::string marker_namespace = "open_vocab_query_object_layer";
      double z_offset = 10.0;
      // Only labels from later accumulated queries are offset. All object nodes
      // stay in one contextual layer so selected nodes remain among the others.
      double query_z_offset = 0.75;
      double node_scale = 0.40;
      double selected_node_scale = 0.54;
      double glow_scale = 0.82;
      double connector_scale = 0.018;
      double selected_connector_scale = 0.036;
      double label_scale = 0.24;
      std::vector<double> context_color_rgba{0.52, 0.53, 0.55, 0.22};
      std::vector<double> selected_color_rgba{0.99, 0.98, 0.62, 1.0};
      std::vector<double> glow_color_rgba{0.99, 0.98, 0.62, 0.20};
      std::vector<double> context_connector_color_rgba{0.52, 0.53, 0.55, 0.12};
    };

    struct InspectionTargetsConfig {
      bool enabled = true;
      std::string topic = "query_open_vocab_objects/inspection_targets";
    };

    bool enabled = true;
    std::string service_name = "query_open_vocab_objects";
    std::string text_encoder_service_name = "tracking/open_vocab/encode_text";
    std::string image_encoder_service_name = "tracking/open_vocab/encode_image";
    double text_encoder_timeout_s = 10.0;
    double image_encoder_timeout_s = 30.0;
    uint32_t default_top_k = 5;
    ResultMeshConfig result_mesh;
    ResultHeatmapConfig result_heatmap;
    ResultObjectLayerConfig result_object_layer;
    InspectionTargetsConfig inspection_targets;
    bool publish_markers = true;
    std::string marker_topic = "query_open_vocab_objects/markers";
    std::string marker_frame_id = "";
    std::string marker_namespace = "open_vocab_object_query";
    bool marker_use_spheres = true;
    double marker_point_scale = 0.18;
    double marker_alpha = 0.95;
    size_t marker_vertex_stride = 1;
    size_t marker_max_points_per_result = 10000;
    bool marker_publish_labels = true;
    double marker_label_height = 0.25;
    double marker_label_scale = 0.24;
    double marker_query_z_offset = 0.05;
    int verbosity = 1;
  } const config;

  OpenVocabObjectQueryServer(ianvs::NodeHandle nh,
                             SharedDsgInfo::Ptr frontend_dsg,
                             std::function<void()> stop_mapping);
  ~OpenVocabObjectQueryServer();

 private:
  struct VisualizationQueryResults {
    std::string label;
    std::vector<hydra_msgs::msg::OpenVocabObjectQueryResult> results;
  };

  void handleQuery(QueryRequest req, QueryResponse resp);

  void publishResultMesh(const std::vector<VisualizationQueryResults>& queries) const;
  void publishResultHeatmap(
      const std::vector<VisualizationQueryResults>& queries) const;
  void publishResultObjectLayer(
      const std::vector<VisualizationQueryResults>& queries) const;
  void publishInspectionTargets(
      const std::vector<VisualizationQueryResults>& queries) const;

  bool encodeText(const std::string& query,
                  FeatureVector* feature,
                  std::string* encoder_id,
                  std::string* error) const;
  bool encodeImage(const sensor_msgs::msg::CompressedImage& image,
                   FeatureVector* feature,
                   std::string* encoder_id,
                   std::string* error) const;
  void publishQueryMarkers(const std::vector<VisualizationQueryResults>& queries) const;
  void publishVisualizations() const;

  SharedDsgInfo::Ptr frontend_dsg_;
  std::function<void()> stop_mapping_;
  std::string resolved_text_encoder_service_name_;
  std::string resolved_image_encoder_service_name_;
  std::atomic<bool> mapping_stopped_{false};
  rclcpp::Node::SharedPtr query_node_;
  rclcpp::executors::SingleThreadedExecutor query_executor_;
  std::thread query_thread_;
  rclcpp::Service<QuerySrv>::SharedPtr server_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr result_mesh_pub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr result_heatmap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      result_object_layer_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      result_object_context_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      result_object_descriptor_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr
      inspection_targets_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::vector<VisualizationQueryResults> visualized_queries_;
};

void declare_config(OpenVocabObjectQueryServer::Config::ResultMeshConfig& config);
void declare_config(OpenVocabObjectQueryServer::Config::ResultHeatmapConfig& config);
void declare_config(
    OpenVocabObjectQueryServer::Config::ResultObjectLayerConfig& config);
void declare_config(
    OpenVocabObjectQueryServer::Config::InspectionTargetsConfig& config);
void declare_config(OpenVocabObjectQueryServer::Config& config);

}  // namespace hydra
