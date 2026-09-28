#pragma once

#include <config_utilities/factory.h>
#include <hydra/common/module.h>
#include <hydra/common/shared_module_state.h>
#include <ianvs/node_handle.h>

#include <instance_tracking_msgs/msg/open_vocab_track_features.hpp>

namespace hydra {

class OpenVocabFeatureReceiver : public Module {
 public:
  struct Config {
    std::string ns = "~/input";
    bool enabled = false;
    std::string topic = "tracking/open_vocab_features";
    size_t queue_size = 10;
    size_t verbosity = 0;
  } const config;

  OpenVocabFeatureReceiver(const Config& config, SharedModuleState::Ptr state);
  ~OpenVocabFeatureReceiver() override = default;

  void start() override;
  void stop() override;
  std::string printInfo() const override;

 private:
  void callback(const instance_tracking_msgs::msg::OpenVocabTrackFeatures& msg);

  SharedModuleState::Ptr state_;
  rclcpp::Subscription<instance_tracking_msgs::msg::OpenVocabTrackFeatures>::SharedPtr sub_;

  inline static const auto registration_ =
      config::RegistrationWithConfig<OpenVocabFeatureReceiver,
                                     OpenVocabFeatureReceiver,
                                     Config,
                                     SharedModuleState::Ptr>("OpenVocabFeatureReceiver");
};

void declare_config(OpenVocabFeatureReceiver::Config& config);

}  // namespace hydra
