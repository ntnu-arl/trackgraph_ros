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
#include "hydra_ros/input/image_receiver.h"

#include <algorithm>
#include <config_utilities/config.h>
#include <glog/logging.h>

#include <cstdlib>
#include <cv_bridge/cv_bridge.hpp>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <sensor_msgs/image_encodings.hpp>
#include <unordered_map>
#include <unordered_set>

namespace hydra {

using instance_tracking_msgs::msg::TrackedInstances;
using semantic_inference_msgs::msg::FeatureImage;
using sensor_msgs::msg::Image;

namespace {

class TrackedSyncDiagnostics {
 public:
  static TrackedSyncDiagnostics& instance() {
    static TrackedSyncDiagnostics diagnostics;
    return diagnostics;
  }

  void recordReceive(const TrackedInstances& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    initializeLocked();

    const auto stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    ++tracked_messages_received_;
    if (msg.is_keyframe) {
      ++tracked_keyframes_received_;
      received_keyframes_[stamp_ns] = msg.frame_index;
      LOG(INFO) << "Received tracked keyframe frame_index=" << msg.frame_index
                << " stamp_ns=" << stamp_ns << " tracks=" << msg.tracks.size();
    }

    appendEventLocked("tracked_received", msg, stamp_ns);
    writeSummaryLocked();
  }

  void recordSync(const TrackedInstances& msg, int64_t timestamp_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    initializeLocked();

    const auto msg_stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    ++synced_packets_;
    if (msg.is_keyframe) {
      ++tracked_keyframes_synced_;
      synced_keyframes_.insert(msg_stamp_ns);
      LOG(INFO) << "Synchronized tracked keyframe frame_index=" << msg.frame_index
                << " msg_stamp_ns=" << msg_stamp_ns
                << " packet_stamp_ns=" << timestamp_ns << " tracks=" << msg.tracks.size();
    }

    appendEventLocked("synced_packet", msg, msg_stamp_ns, timestamp_ns);
    writeSummaryLocked();
  }

 private:
  TrackedSyncDiagnostics() = default;

  void initializeLocked() {
    if (initialized_) {
      return;
    }
    initialized_ = true;

    const char* root = std::getenv("JETSON_LOG_DIR");
    if (!root || std::string(root).empty()) {
      return;
    }

    try {
      const auto output_dir = std::filesystem::path(root) / "hydra_sync";
      std::filesystem::create_directories(output_dir);
      events_path_ = output_dir / "tracked_sync_events.jsonl";
      summary_path_ = output_dir / "tracked_sync_summary.json";
      enabled_ = true;
      LOG(INFO) << "Saving tracked input sync diagnostics under " << output_dir;
    } catch (const std::exception& e) {
      LOG(WARNING) << "Failed to initialize tracked sync diagnostics: " << e.what();
    }
  }

  void appendEventLocked(const std::string& event,
                         const TrackedInstances& msg,
                         int64_t stamp_ns,
                         std::optional<int64_t> packet_stamp_ns = std::nullopt) const {
    if (!enabled_) {
      return;
    }

    std::ofstream out(events_path_, std::ios::app);
    out << "{\"event\":\"" << event << "\","
        << "\"stamp_ns\":" << stamp_ns << ","
        << "\"packet_stamp_ns\":";
    if (packet_stamp_ns) {
      out << *packet_stamp_ns;
    } else {
      out << "null";
    }
    out << ","
        << "\"frame_index\":" << msg.frame_index << ","
        << "\"is_keyframe\":" << (msg.is_keyframe ? "true" : "false") << ","
        << "\"track_count\":" << msg.tracks.size() << "}\n";
  }

  void writeSummaryLocked() const {
    if (!enabled_) {
      return;
    }

    size_t unsynced_keyframes = 0;
    for (const auto& [stamp_ns, _] : received_keyframes_) {
      if (!synced_keyframes_.count(stamp_ns)) {
        ++unsynced_keyframes;
      }
    }

    std::ofstream out(summary_path_);
    out << "{\n"
        << "  \"tracked_messages_received\": " << tracked_messages_received_
        << ",\n"
        << "  \"tracked_keyframes_received\": " << tracked_keyframes_received_
        << ",\n"
        << "  \"synced_packets\": " << synced_packets_ << ",\n"
        << "  \"tracked_keyframes_synced\": " << tracked_keyframes_synced_
        << ",\n"
        << "  \"unsynced_keyframe_candidates\": " << unsynced_keyframes << "\n"
        << "}\n";
  }

  mutable std::mutex mutex_;
  bool initialized_ = false;
  bool enabled_ = false;
  std::filesystem::path events_path_;
  std::filesystem::path summary_path_;
  size_t tracked_messages_received_ = 0;
  size_t tracked_keyframes_received_ = 0;
  size_t synced_packets_ = 0;
  size_t tracked_keyframes_synced_ = 0;
  std::unordered_map<int64_t, int64_t> received_keyframes_;
  std::unordered_set<int64_t> synced_keyframes_;
};

}  // namespace

ColorSubscriber::ColorSubscriber() = default;

ColorSubscriber::ColorSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(nh, "rgb/image_raw", queue_size)) {}

ColorSubscriber::~ColorSubscriber() = default;

ColorSubscriber::Filter& ColorSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void ColorSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.color = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert color image: " << e.what();
  }
}

void ColorSubscriber::recordSync(const MsgType&, int64_t) const {}

DepthSubscriber::DepthSubscriber() = default;

DepthSubscriber::DepthSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(
          nh, "depth_registered/image_rect", queue_size)) {}

DepthSubscriber::~DepthSubscriber() = default;

DepthSubscriber::Filter& DepthSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void DepthSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.depth = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
  }
}

void DepthSubscriber::recordSync(const MsgType&, int64_t) const {}

LabelSubscriber::LabelSubscriber() = default;

LabelSubscriber::LabelSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<Image>>(nh, "semantic/image_raw", queue_size)) {}

LabelSubscriber::~LabelSubscriber() = default;

LabelSubscriber::Filter& LabelSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void LabelSubscriber::fillInput(const Image& img, ImageInputPacket& packet) const {
  try {
    packet.labels = cv_bridge::toCvCopy(img)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert label image: " << e.what();
  }
}

void LabelSubscriber::recordSync(const MsgType&, int64_t) const {}

FeatureSubscriber::FeatureSubscriber() = default;

FeatureSubscriber::FeatureSubscriber(ianvs::NodeHandle nh, uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<FeatureImage>>(
          nh, "semantic/image_raw", queue_size)) {}

FeatureSubscriber::~FeatureSubscriber() = default;

FeatureSubscriber::Filter& FeatureSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void FeatureSubscriber::fillInput(const MsgType& msg, ImageInputPacket& packet) const {
  try {
    packet.labels = cv_bridge::toCvCopy(msg.image)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert depth image: " << e.what();
  }

  CHECK_EQ(msg.mask_ids.size(), msg.features.size());
  for (size_t i = 0; i < msg.mask_ids.size(); ++i) {
    const auto& vec = msg.features[i].data;
    packet.label_features.emplace(
        msg.mask_ids[i],
        Eigen::Map<const hydra::FeatureVector>(vec.data(), vec.size()));
  }
}

void FeatureSubscriber::recordSync(const MsgType&, int64_t) const {}

TrackedInstancesSubscriber::TrackedInstancesSubscriber() = default;

TrackedInstancesSubscriber::TrackedInstancesSubscriber(ianvs::NodeHandle nh,
                                                       uint32_t queue_size)
    : impl_(std::make_shared<FilterSub<TrackedInstances>>(
          nh,
          "tracking/instances",
          queue_size,
          [](const TrackedInstances& msg) {
            TrackedSyncDiagnostics::instance().recordReceive(msg);
          })) {}

TrackedInstancesSubscriber::~TrackedInstancesSubscriber() = default;

TrackedInstancesSubscriber::Filter& TrackedInstancesSubscriber::getFilter() const {
  return *CHECK_NOTNULL(impl_);
}

void TrackedInstancesSubscriber::fillInput(const MsgType& msg,
                                           ImageInputPacket& packet) const {
  try {
    packet.instance_image =
        cv_bridge::toCvCopy(msg.masks, sensor_msgs::image_encodings::TYPE_16UC1)->image;
  } catch (const cv_bridge::Exception& e) {
    LOG(ERROR) << "Failed to convert tracked instance mask image: " << e.what();
  }

  packet.track_observations.clear();
  packet.track_observations.reserve(msg.tracks.size());
  for (const auto& track : msg.tracks) {
    TrackObservation obs;
    obs.track_id = track.track_id;
    obs.instance_id = track.instance_id;
    obs.confidence = track.confidence;
    obs.age = track.age;
    obs.frames_since_detection = track.frames_since_detection;
    if (!track.prototype.empty()) {
      obs.prototype = Eigen::Map<const hydra::FeatureVector>(track.prototype.data(),
                                                             track.prototype.size());
    }
    obs.has_display_color = track.has_display_color;
    if (track.has_display_color) {
      obs.display_color_r = static_cast<uint8_t>(
          std::clamp(track.display_color.r, 0.0f, 1.0f) * 255.0f);
      obs.display_color_g = static_cast<uint8_t>(
          std::clamp(track.display_color.g, 0.0f, 1.0f) * 255.0f);
      obs.display_color_b = static_cast<uint8_t>(
          std::clamp(track.display_color.b, 0.0f, 1.0f) * 255.0f);
    }
    packet.track_observations.push_back(obs);
  }

  packet.tracking_is_keyframe = msg.is_keyframe;

  if (!msg.confidence_image.data.empty()) {
    try {
      packet.tracking_confidence_image =
          cv_bridge::toCvCopy(msg.confidence_image,
                              sensor_msgs::image_encodings::TYPE_32FC1)->image;
    } catch (const cv_bridge::Exception& e) {
      LOG(WARNING) << "Failed to convert tracking confidence image: " << e.what();
    }
  }
}

void TrackedInstancesSubscriber::recordSync(const MsgType& msg,
                                            int64_t timestamp_ns) const {
  TrackedSyncDiagnostics::instance().recordSync(msg, timestamp_ns);
}

void declare_config(ClosedSetImageReceiver::Config& config) {
  using namespace config;
  name("ClosedSetImageReceiver::Config");
  base<RosDataReceiver::Config>(config);
}

ClosedSetImageReceiver::ClosedSetImageReceiver(const Config& config,
                                               const std::string& sensor_name)
    : ImageReceiverImpl<LabelSubscriber>(config, sensor_name) {}

void declare_config(OpenSetImageReceiver::Config& config) {
  using namespace config;
  name("OpenSetImageReceiver::Config");
  base<hydra::RosDataReceiver::Config>(config);
}

OpenSetImageReceiver::OpenSetImageReceiver(const Config& config,
                                           const std::string& sensor_name)
    : ImageReceiverImpl<FeatureSubscriber>(config, sensor_name) {}

void declare_config(TrackedImageReceiver::Config& config) {
  using namespace config;
  name("TrackedImageReceiver::Config");
  base<hydra::RosDataReceiver::Config>(config);
}

TrackedImageReceiver::TrackedImageReceiver(const Config& config,
                                           const std::string& sensor_name)
    : ImageReceiverImpl<TrackedInstancesSubscriber>(config, sensor_name) {}

namespace {

static const auto closed_registration =
    config::RegistrationWithConfig<DataReceiver,
                                   ClosedSetImageReceiver,
                                   ClosedSetImageReceiver::Config,
                                   std::string>("ClosedSetImageReceiver");

static const auto open_registration =
    config::RegistrationWithConfig<hydra::DataReceiver,
                                   OpenSetImageReceiver,
                                   OpenSetImageReceiver::Config,
                                   std::string>("OpenSetImageReceiver");

static const auto tracked_registration =
    config::RegistrationWithConfig<hydra::DataReceiver,
                                   TrackedImageReceiver,
                                   TrackedImageReceiver::Config,
                                   std::string>("TrackedImageReceiver");

}  // namespace

}  // namespace hydra
