#include "hydra_ros/input/open_vocab_feature_receiver.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>
#include <hydra/openset/openset_types.h>

namespace hydra {

using instance_tracking_msgs::msg::OpenVocabTrackFeatures;

void declare_config(OpenVocabFeatureReceiver::Config& config) {
  using namespace config;
  name("OpenVocabFeatureReceiver::Config");
  field(config.ns, "ns");
  field(config.enabled, "enabled");
  field(config.topic, "topic");
  field(config.queue_size, "queue_size");
  field(config.verbosity, "verbosity");
}

OpenVocabFeatureReceiver::OpenVocabFeatureReceiver(const Config& config,
                                                   SharedModuleState::Ptr state)
    : config(config::checkValid(config)), state_(std::move(state)) {}

void OpenVocabFeatureReceiver::start() {
  if (!config.enabled) {
    LOG_IF(INFO, config.verbosity >= 1)
        << "Open-vocabulary feature receiver disabled";
    return;
  }

  auto nh = ianvs::NodeHandle::this_node(config.ns);
  sub_ = nh.create_subscription<OpenVocabTrackFeatures>(
      config.topic, config.queue_size, &OpenVocabFeatureReceiver::callback, this);
  LOG_IF(INFO, config.verbosity >= 1)
      << "Subscribed to open-vocabulary features on '" << config.topic << "'";
}

void OpenVocabFeatureReceiver::stop() { sub_.reset(); }

std::string OpenVocabFeatureReceiver::printInfo() const {
  return config::toString(config);
}

void OpenVocabFeatureReceiver::callback(const OpenVocabTrackFeatures& msg) {
  if (!state_ || !state_->open_vocab_feature_cache) {
    return;
  }

  const auto timestamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  for (const auto& feature_msg : msg.features) {
    const auto& values = feature_msg.feature;
    if (values.empty()) {
      continue;
    }

    OpenVocabFeatureEntry entry;
    entry.feature =
        Eigen::Map<const FeatureVector>(values.data(), static_cast<Eigen::Index>(values.size()));
    entry.encoder_id = msg.encoder_id;
    entry.timestamp_ns = timestamp_ns;
    entry.frame_index = msg.frame_index;
    entry.source_instance_id = feature_msg.source_instance_id;
    state_->open_vocab_feature_cache->store(feature_msg.source_track_id, std::move(entry));
    LOG_IF(INFO, config.verbosity >= 2)
        << "Cached open-vocabulary feature for source_track_id="
        << feature_msg.source_track_id << " frame_index=" << msg.frame_index;
  }
}

}  // namespace hydra
