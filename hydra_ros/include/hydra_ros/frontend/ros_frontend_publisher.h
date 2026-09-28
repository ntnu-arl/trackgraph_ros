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
#include <hydra/frontend/graph_builder.h>
#include <pose_graph_tools_ros/conversions.h>

#include <map>
#include <mutex>

#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>
#include <kimera_pgmo_msgs/srv/mesh_delta_query.hpp>

#include "hydra_ros/utils/dsg_streaming_interface.h"

namespace hydra {

class RosFrontendPublisher : public GraphBuilder::Sink {
 public:
  using MeshDeltaSrv = kimera_pgmo_msgs::srv::MeshDeltaQuery;
  using MeshDeltaRequest = kimera_pgmo_msgs::srv::MeshDeltaQuery::Request::SharedPtr;
  using MeshDeltaResponse = kimera_pgmo_msgs::srv::MeshDeltaQuery::Response::SharedPtr;
  using MeshDeltaMsg = kimera_pgmo_msgs::msg::MeshDelta;

  struct Config {
    //! @brief Configuration for dsg publisher
    DsgSender::Config dsg_sender;
    size_t mesh_delta_queue_size = 100;  // Store mesh delta to resend. 0 for infinite
    //! @brief Periodically log retained mesh-delta counts and estimated bytes.
    bool log_mesh_delta_stats = false;
    //! @brief Number of received mesh deltas between statistics snapshots.
    size_t mesh_delta_stats_log_interval = 50;
  } const config;

  struct RetainedDeltaStatistics {
    size_t retained_count = 0;
    //! Resend-map object and sequence-capacity bytes. Excludes allocator/container
    //! overhead and any middleware/QoS copies.
    size_t estimated_retained_bytes = 0;
    size_t peak_retained_count = 0;
    size_t peak_estimated_retained_bytes = 0;
    size_t total_received = 0;
    size_t total_evicted = 0;
  };

  explicit RosFrontendPublisher(ianvs::NodeHandle);

  ~RosFrontendPublisher() override;

  void call(uint64_t timestamp_ns,
            const DynamicSceneGraph& graph,
            const BackendInput& backend_input) const override;

  std::string printInfo() const override { return "RosFrontendPublisher"; }

  //! @brief Return a thread-safe snapshot of retained mesh-delta statistics.
  RetainedDeltaStatistics retainedDeltaStatistics() const;

 protected:
  void processMeshDeltaQuery(const MeshDeltaRequest req, MeshDeltaResponse resp);

  static size_t estimateRetainedBytes(const MeshDeltaMsg& msg);

  RetainedDeltaStatistics retainedDeltaStatisticsLocked() const;

  void maybeLogRetainedDeltaStatistics(bool final = false) const;

  std::unique_ptr<DsgSender> dsg_sender_;
  pose_graph_tools::PoseGraphPublisher mesh_graph_pub_;
  rclcpp::Publisher<MeshDeltaMsg>::SharedPtr mesh_update_pub_;
  rclcpp::Service<MeshDeltaSrv>::SharedPtr mesh_delta_server_;

  mutable std::mutex stored_delta_mutex_;
  mutable std::map<uint16_t, MeshDeltaMsg::SharedPtr> stored_delta_;
  mutable size_t stored_delta_estimated_bytes_ = 0;
  mutable size_t peak_stored_delta_count_ = 0;
  mutable size_t peak_stored_delta_estimated_bytes_ = 0;
  mutable size_t total_received_deltas_ = 0;
  mutable size_t total_evicted_deltas_ = 0;
  mutable bool retained_delta_final_logged_ = false;
};

}  // namespace hydra
