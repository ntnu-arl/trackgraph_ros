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
#include "hydra_ros/frontend/ros_frontend_publisher.h"

#include <algorithm>
#include <vector>

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/common/global_info.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo_ros/conversion/mesh_delta.h>

namespace hydra {

using pose_graph_tools::PoseGraphTypeAdapter;
using BaseInterface = rclcpp::node_interfaces::NodeBaseInterface;
using rclcpp::CallbackGroupType;

namespace {

inline RosFrontendPublisher::Config get_config() {
  const auto odom_frame = GlobalInfo::instance().getFrames().odom;
  auto config = config::fromContext<RosFrontendPublisher::Config>("frontend");
  config.dsg_sender = config.dsg_sender.with_name("frontend").with_frame(odom_frame);
  return config;
}

template <typename Container>
size_t capacityBytes(const Container& values) {
  return values.capacity() * sizeof(typename Container::value_type);
}

}  // namespace

void declare_config(RosFrontendPublisher::Config& config) {
  using namespace config;
  name("RosFrontendPublisher::Config");
  field(config.dsg_sender, "");
  field(config.mesh_delta_queue_size, "mesh_delta_queue_size");
  field(config.log_mesh_delta_stats, "log_mesh_delta_stats");
  field(config.mesh_delta_stats_log_interval, "mesh_delta_stats_log_interval");
  checkCondition(config.mesh_delta_stats_log_interval > 0,
                 "mesh_delta_stats_log_interval must be greater than zero");
}

RosFrontendPublisher::RosFrontendPublisher(ianvs::NodeHandle nh)
    : config(config::checkValid(get_config())),
      dsg_sender_(new DsgSender(config.dsg_sender, nh)) {
  auto group = nh.as<BaseInterface>()->create_callback_group(
      CallbackGroupType::MutuallyExclusive);
  mesh_delta_server_ =
      nh.create_service<MeshDeltaSrv>("mesh_delta_query",
                                      &RosFrontendPublisher::processMeshDeltaQuery,
                                      this,
                                      rclcpp::ServicesQoS(),
                                      group);

  const auto qos = rclcpp::QoS(100).transient_local();
  mesh_graph_pub_ =
      nh.create_publisher<PoseGraphTypeAdapter>("mesh_graph_incremental", qos);
  mesh_update_pub_ = nh.create_publisher<MeshDeltaMsg>("full_mesh_update", qos);
}

RosFrontendPublisher::~RosFrontendPublisher() {
  maybeLogRetainedDeltaStatistics(true);
}

size_t RosFrontendPublisher::estimateRetainedBytes(const MeshDeltaMsg& msg) {
  // MeshDelta's variable-size fields are flat vectors. Capacity, rather than size,
  // reflects memory retained by those vectors after conversion. This deliberately
  // excludes allocator bookkeeping, the std::map node, shared_ptr control block,
  // and any copies retained in the ROS middleware's transient-local history.
  return sizeof(msg) + capacityBytes(msg.previous_indices) +
         capacityBytes(msg.current_indices) + capacityBytes(msg.vertex_updates) +
         capacityBytes(msg.face_updates) + capacityBytes(msg.face_archive_updates);
}

RosFrontendPublisher::RetainedDeltaStatistics
RosFrontendPublisher::retainedDeltaStatisticsLocked() const {
  RetainedDeltaStatistics stats;
  stats.retained_count = stored_delta_.size();
  stats.estimated_retained_bytes = stored_delta_estimated_bytes_;
  stats.peak_retained_count = peak_stored_delta_count_;
  stats.peak_estimated_retained_bytes = peak_stored_delta_estimated_bytes_;
  stats.total_received = total_received_deltas_;
  stats.total_evicted = total_evicted_deltas_;
  return stats;
}

RosFrontendPublisher::RetainedDeltaStatistics
RosFrontendPublisher::retainedDeltaStatistics() const {
  std::lock_guard<std::mutex> lock(stored_delta_mutex_);
  return retainedDeltaStatisticsLocked();
}

void RosFrontendPublisher::maybeLogRetainedDeltaStatistics(bool final) const {
  if (!config.log_mesh_delta_stats) {
    return;
  }

  RetainedDeltaStatistics stats;
  {
    std::lock_guard<std::mutex> lock(stored_delta_mutex_);
    if (final && retained_delta_final_logged_) {
      return;
    }

    if (!final && total_received_deltas_ != 1u &&
        total_received_deltas_ % config.mesh_delta_stats_log_interval != 0u) {
      return;
    }

    stats = retainedDeltaStatisticsLocked();
    retained_delta_final_logged_ = final;
  }

  LOG(INFO) << "[ros-frontend-mesh-deltas] snapshot: total_received="
            << stats.total_received << " retained_count=" << stats.retained_count
            << " estimated_retained_bytes=" << stats.estimated_retained_bytes
            << " peak_retained_count=" << stats.peak_retained_count
            << " peak_estimated_retained_bytes="
            << stats.peak_estimated_retained_bytes
            << " total_evicted=" << stats.total_evicted
            << " queue_limit=" << config.mesh_delta_queue_size << " final=" << final;
}

void RosFrontendPublisher::call(uint64_t timestamp_ns,
                                const DynamicSceneGraph& graph,
                                const BackendInput& backend_input) const {
  // TODO(nathan) make sure pgmo stamps the deformation graph
  mesh_graph_pub_->publish(backend_input.deformation_graph);

  if (backend_input.mesh_update) {
    backend_input.mesh_update->timestamp_ns = timestamp_ns;
    auto delta_msg = std::make_shared<MeshDeltaMsg>();
    kimera_pgmo::conversions::to_ros(*backend_input.mesh_update, *delta_msg);
    mesh_update_pub_->publish(*delta_msg);

    {
      std::lock_guard<std::mutex> lock(stored_delta_mutex_);
      ++total_received_deltas_;
      const auto result = stored_delta_.insert(
          {backend_input.mesh_update->info.sequence_number, delta_msg});
      if (result.second) {
        stored_delta_estimated_bytes_ += estimateRetainedBytes(*delta_msg);
      }

      if (config.mesh_delta_queue_size &&
          stored_delta_.size() > config.mesh_delta_queue_size) {
        stored_delta_estimated_bytes_ -= estimateRetainedBytes(*stored_delta_.begin()->second);
        stored_delta_.erase(stored_delta_.begin());
        ++total_evicted_deltas_;
      }

      peak_stored_delta_count_ =
          std::max(peak_stored_delta_count_, stored_delta_.size());
      peak_stored_delta_estimated_bytes_ =
          std::max(peak_stored_delta_estimated_bytes_, stored_delta_estimated_bytes_);
    }
    maybeLogRetainedDeltaStatistics();
  }

  dsg_sender_->sendGraph(graph, rclcpp::Time(timestamp_ns));
}

void RosFrontendPublisher::processMeshDeltaQuery(const MeshDeltaRequest req,
                                                 MeshDeltaResponse resp) {
  LOG(INFO) << "Received request for " << req->sequence_numbers.size() << " deltas...";
  std::vector<MeshDeltaMsg::SharedPtr> requested_deltas;
  requested_deltas.reserve(req->sequence_numbers.size());
  {
    // Keep the critical section to shared_ptr lookups. The messages are immutable,
    // so potentially large response copies do not need to block frontend insertion.
    std::lock_guard<std::mutex> lock(stored_delta_mutex_);
    for (const auto& seq : req->sequence_numbers) {
      const auto iter = stored_delta_.find(seq);
      requested_deltas.push_back(iter == stored_delta_.end() ? nullptr : iter->second);
    }
  }

  for (size_t i = 0; i < req->sequence_numbers.size(); ++i) {
    auto& msg = resp->deltas.emplace_back();
    if (!requested_deltas[i]) {
      LOG(ERROR) << "Mesh delta sequence " << req->sequence_numbers[i] << " not found";
      continue;
    }

    msg = *requested_deltas[i];
  }

  LOG(INFO) << "Responding with " << resp->deltas.size() << " deltas...";
}

}  // namespace hydra
