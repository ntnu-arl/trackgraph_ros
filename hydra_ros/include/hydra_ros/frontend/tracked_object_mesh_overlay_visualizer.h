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
#pragma once

#include <cstddef>
#include <cstdint>

#include <hydra/frontend/graph_builder.h>
#include <ianvs/lazy_publisher_group.h>
#include <spark_dsg/color.h>

#include <kimera_pgmo_msgs/msg/mesh.hpp>

namespace hydra {

class TrackedObjectMeshOverlayVisualizer : public GraphBuilder::Sink {
 public:
  enum class DisplayMode { OWNER = 0, OWNER_STATE = 1, RAW_TRACK = 2 };

  struct Config {
    std::string module_ns = "~/tracked_objects";
    std::string topic = "mesh_surface_overlay";
    std::string layer_id = DsgLayers::OBJECTS;
    DisplayMode display_mode = DisplayMode::OWNER;
    bool active_only = false;
    size_t min_mesh_connections = 1;
    size_t vertex_stride = 1;
    double tracked_alpha = 1.0;
    double raw_track_alpha = 1.0;
    double ignored_alpha = 0.20;
    double ignored_color_blend_weight = 0.85;
    double unassigned_alpha = 0.25;
    spark_dsg::Color unassigned_color = spark_dsg::Color(90, 90, 90);
    double no_track_alpha = 0.12;
    spark_dsg::Color no_track_color = spark_dsg::Color(45, 45, 45);
    double archived_owner_alpha = 0.60;
    spark_dsg::Color archived_owner_color = spark_dsg::Color(70, 150, 240);
    spark_dsg::Color ignored_owner_color = spark_dsg::Color(210, 80, 80);
  } const config;

  explicit TrackedObjectMeshOverlayVisualizer(const Config& config);
  ~TrackedObjectMeshOverlayVisualizer() = default;

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const BackendInput& backend_input) const override;

  std::string printInfo() const override;

 private:
  ianvs::NodeHandle nh_;
  ianvs::RosPublisherGroup<kimera_pgmo_msgs::msg::Mesh> pubs_;
};

void declare_config(TrackedObjectMeshOverlayVisualizer::Config& config);

}  // namespace hydra
