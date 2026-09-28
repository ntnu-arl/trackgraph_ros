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
#include "hydra_ros/hydra_ros_pipeline.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <hydra/active_window/reconstruction_module.h>
#include <hydra/backend/backend_module.h>
#include <hydra/backend/zmq_interfaces.h>
#include <hydra/common/dsg_types.h>
#include <hydra/common/global_info.h>
#include <hydra/frontend/graph_builder.h>
#include <hydra/loop_closure/loop_closure_module.h>
#include <hydra/openset/openset_types.h>
#include <instance_tracking_msgs/srv/encode_open_vocab_text.hpp>
#include <pose_graph_tools_ros/conversions.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>

#include "hydra_ros/backend/ros_backend_publisher.h"
#include "hydra_ros/frontend/ros_frontend_publisher.h"
#include "hydra_ros/utils/bow_subscriber.h"
#include "hydra_ros/utils/external_loop_closure_subscriber.h"
#include "hydra_ros/utils/status_monitor.h"

namespace hydra {
using instance_tracking_msgs::srv::EncodeOpenVocabText;

namespace {

using namespace std::chrono_literals;
constexpr int kOpenVocabPromptBankSchemaVersion = 1;

template <typename T>
std::future_status wait_for_result(T& future,
                                   std::chrono::seconds timeout,
                                   ianvs::NodeHandle& nh) {
  // NOTE: the hydra node is not yet being spun during HydraRosPipeline::init(),
  // so the client's response callback will never fire unless we pump the
  // executor ourselves. Without this spin_some, async_send_request's future
  // stays pending forever and we time out even though the service has already
  // replied on the wire.
  auto base = nh.node().get<rclcpp::node_interfaces::NodeBaseInterface>();
  auto status = std::future_status::timeout;
  const auto start = std::chrono::steady_clock::now();
  while (status != std::future_status::ready && rclcpp::ok()) {
    if (std::chrono::steady_clock::now() - start > timeout) {
      break;
    }

    rclcpp::spin_some(base);
    status = future.wait_for(100ms);
  }

  return status;
}

void clearOpenVocabPromptBank(const OpenVocabPromptBank::Ptr& bank) {
  if (!bank) {
    return;
  }

  bank->encoder_id.clear();
  bank->embeddings.clear();
  bank->names.clear();
}

std::string joinPromptList(const std::vector<std::string>& prompts) {
  std::ostringstream ss;
  for (size_t i = 0; i < prompts.size(); ++i) {
    if (i != 0) {
      ss << ", ";
    }
    ss << "'" << prompts.at(i) << "'";
  }
  return ss.str();
}

bool appendPromptEmbedding(const std::string& prompt,
                          const std::string& encoder_id,
                          const std::vector<float>& feature_values,
                          const TrackedObjectOpenVocabConfig& open_vocab,
                          const std::string& source_description,
                          const OpenVocabPromptBank::Ptr& bank) {
  if (!bank) {
    return false;
  }

  if (encoder_id.empty() || feature_values.empty()) {
    LOG(ERROR) << source_description << " returned an empty embedding for prompt '" << prompt
               << "'";
    return false;
  }

  if (!open_vocab.expected_encoder_id.empty() &&
      encoder_id != open_vocab.expected_encoder_id) {
    LOG(ERROR) << source_description << " encoder_id='" << encoder_id
               << "' does not match tracked open-vocabulary expected_encoder_id='"
               << open_vocab.expected_encoder_id << "'";
    return false;
  }

  if (bank->encoder_id.empty()) {
    bank->encoder_id = encoder_id;
  } else if (bank->encoder_id != encoder_id) {
    LOG(ERROR) << source_description << " returned inconsistent encoder ids ('"
               << bank->encoder_id << "' vs '" << encoder_id << "')";
    return false;
  }

  const Eigen::Map<const FeatureVector> embedding(feature_values.data(), feature_values.size());
  if (!embedding.allFinite() || embedding.norm() <= 1.0e-9f) {
    LOG(ERROR) << source_description << " returned an invalid embedding for prompt '" << prompt
               << "'";
    return false;
  }

  bank->names.push_back(prompt);
  bank->embeddings.push_back(embedding.normalized());
  return true;
}

bool loadTrackedObjectIgnorePromptBankFromFile(
    const std::filesystem::path& prompt_bank_path,
    const TrackedObjectOpenVocabConfig& open_vocab,
    const TrackedObjectOpenVocabIgnoreFilterConfig& filter,
    const OpenVocabPromptBank::Ptr& bank) {
  if (!bank) {
    return false;
  }

  std::ifstream fin(prompt_bank_path);
  if (!fin) {
    LOG(WARNING) << "Failed to open tracked-object ignore prompt bank '"
                 << prompt_bank_path.string() << "'";
    return false;
  }

  nlohmann::json json;
  try {
    fin >> json;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Failed to parse tracked-object ignore prompt bank '"
               << prompt_bank_path.string() << "': " << e.what();
    return false;
  }

  std::vector<std::string> loaded_prompts;
  try {
    if (json.contains("schema_version") &&
        json.at("schema_version").get<int>() != kOpenVocabPromptBankSchemaVersion) {
      LOG(ERROR) << "Tracked-object ignore prompt bank '" << prompt_bank_path.string()
                 << "' has unsupported schema_version="
                 << json.at("schema_version").get<int>();
      return false;
    }

    if (!json.contains("encoder_id") || !json.at("encoder_id").is_string()) {
      LOG(ERROR) << "Tracked-object ignore prompt bank '" << prompt_bank_path.string()
                 << "' is missing a string encoder_id";
      return false;
    }

    if (!json.contains("entries") || !json.at("entries").is_array()) {
      LOG(ERROR) << "Tracked-object ignore prompt bank '" << prompt_bank_path.string()
                 << "' is missing an entries array";
      return false;
    }

    const auto encoder_id = json.at("encoder_id").get<std::string>();
    for (const auto& entry : json.at("entries")) {
      if (!entry.is_object() || !entry.contains("prompt") ||
          !entry.contains("embedding") || !entry.at("prompt").is_string() ||
          !entry.at("embedding").is_array()) {
        LOG(ERROR) << "Tracked-object ignore prompt bank '" << prompt_bank_path.string()
                   << "' contains an invalid entry";
        clearOpenVocabPromptBank(bank);
        return false;
      }

      const auto prompt = entry.at("prompt").get<std::string>();
      const auto embedding = entry.at("embedding").get<std::vector<float>>();
      loaded_prompts.push_back(prompt);
      if (!appendPromptEmbedding(prompt,
                                 encoder_id,
                                 embedding,
                                 open_vocab,
                                 "Tracked-object ignore prompt bank '" +
                                     prompt_bank_path.string() + "'",
                                 bank)) {
        clearOpenVocabPromptBank(bank);
        return false;
      }
    }
  } catch (const std::exception& e) {
    LOG(ERROR) << "Tracked-object ignore prompt bank '" << prompt_bank_path.string()
               << "' is malformed: " << e.what();
    clearOpenVocabPromptBank(bank);
    return false;
  }

  if (bank->empty()) {
    LOG(ERROR) << "Tracked-object ignore prompt bank '" << prompt_bank_path.string()
               << "' did not contain any usable embeddings";
    return false;
  }

  if (!filter.prompts.empty() && loaded_prompts != filter.prompts) {
    LOG(WARNING) << "Tracked-object ignore prompt bank '" << prompt_bank_path.string()
                 << "' contains prompts [" << joinPromptList(loaded_prompts)
                 << "] but the runtime config lists [" << joinPromptList(filter.prompts)
                 << "]. Using the file contents; regenerate the prompt bank if the prompt "
                    "set changed.";
  }

  LOG_IF(INFO, filter.verbosity > 0)
      << "Loaded tracked-object ignore prompt bank from '" << prompt_bank_path.string()
      << "' with " << bank->size() << " prompts using encoder '" << bank->encoder_id << "'";
  return true;
}

bool shouldUsePromptBankOnly(const TrackedObjectOpenVocabIgnoreFilterConfig& filter) {
  return filter.source == "prompt_bank";
}

bool shouldUseServiceOnly(const TrackedObjectOpenVocabIgnoreFilterConfig& filter) {
  return filter.source == "service";
}

bool initializeTrackedObjectIgnorePromptBankFromService(
    ianvs::NodeHandle& nh,
    const TrackedObjectOpenVocabConfig& open_vocab,
    const TrackedObjectOpenVocabIgnoreFilterConfig& filter,
    const OpenVocabPromptBank::Ptr& bank) {
  if (!bank) {
    return false;
  }

  const auto service_name = nh.resolve_name(filter.service_name, true);
  auto client = nh.create_client<EncodeOpenVocabText>(filter.service_name);
  LOG_IF(INFO, filter.verbosity > 0)
      << "Initializing tracked-object ignore prompt bank from '" << service_name << "'";

  // The tracker only advertises this service after it has loaded and warmed up
  // the CLIP encoder, which on a cold start can take ~30s or more. Wait long
  // enough to cover that, and log progress so it's obvious Hydra is blocked on
  // the tracker rather than hung.
  constexpr size_t kMaxAttempts = 180;
  bool service_ready = false;
  for (size_t attempt = 0; attempt < kMaxAttempts && rclcpp::ok(); ++attempt) {
    if (client->wait_for_service(1s)) {
      service_ready = true;
      break;
    }
    LOG_IF(INFO, filter.verbosity > 0 && (attempt + 1) % 5 == 0)
        << "Still waiting for tracker text-embedding service '" << service_name
        << "' (" << (attempt + 1) << "/" << kMaxAttempts << "s)";
  }

  if (!service_ready) {
    LOG(ERROR) << "Failed to reach tracker text-embedding service '" << service_name
               << "' during Hydra startup; tracked-object ignore filtering is disabled";
    return false;
  }

  for (const auto& prompt : filter.prompts) {
    auto request = std::make_unique<EncodeOpenVocabText::Request>();
    request->prompt = prompt;
    auto future = client->async_send_request(std::move(request)).future.share();
    if (wait_for_result(future, 5s, nh) != std::future_status::ready) {
      LOG(ERROR) << "Timed out while encoding tracked-object ignore prompt '" << prompt
                 << "' from '" << service_name
                 << "'; tracked-object ignore filtering is disabled";
      clearOpenVocabPromptBank(bank);
      return false;
    }

    const auto response = future.get();
    if (!appendPromptEmbedding(prompt,
                               response->encoder_id,
                               response->feature,
                               open_vocab,
                               "Tracker text-embedding service",
                               bank)) {
      clearOpenVocabPromptBank(bank);
      return false;
    }
  }

  LOG_IF(INFO, filter.verbosity > 0)
      << "Initialized tracked-object ignore prompt bank with " << bank->size()
      << " prompts using encoder '" << bank->encoder_id << "'";
  return true;
}

void initializeTrackedObjectIgnorePromptBank(ianvs::NodeHandle& nh,
                                             const SharedModuleState::Ptr& state) {
  if (!state || !state->open_vocab_ignore_prompt_bank) {
    return;
  }

  auto& bank = state->open_vocab_ignore_prompt_bank;
  clearOpenVocabPromptBank(bank);

  const auto frontend_config = config::fromContext<GraphBuilder::Config>("frontend");
  const auto& open_vocab = frontend_config.trackgraph_segment_updater_config.open_vocab;
  const auto& filter = open_vocab.ignore_filter;
  if (!filter.enabled) {
    return;
  }

  if (!open_vocab.enabled) {
    LOG(WARNING) << "Tracked-object ignore filter requested, but tracked open-vocabulary "
                    "features are disabled; all objects will remain visible";
    return;
  }

  if (filter.prompts.empty()) {
    LOG(WARNING) << "Tracked-object ignore filter requested without any prompts; "
                    "all objects will remain visible";
    return;
  }

  if (shouldUsePromptBankOnly(filter) && filter.prompt_bank_path.empty()) {
    LOG(ERROR) << "Tracked-object ignore filter source='prompt_bank' requires "
                  "ignore_filter.prompt_bank_path; tracked-object ignore filtering is disabled";
    return;
  }

  if (!shouldUseServiceOnly(filter) && !filter.prompt_bank_path.empty()) {
    const auto prompt_bank_path =
        std::filesystem::path(filter.prompt_bank_path).lexically_normal();
    if (loadTrackedObjectIgnorePromptBankFromFile(prompt_bank_path, open_vocab, filter, bank)) {
      return;
    }

    if (shouldUsePromptBankOnly(filter)) {
      LOG(ERROR) << "Tracked-object ignore filter source='prompt_bank' forbids fallback to "
                    "the live text-embedding service; tracked-object ignore filtering is "
                    "disabled";
      return;
    }

    clearOpenVocabPromptBank(bank);
    LOG(WARNING) << "Falling back to tracker text-embedding service because prompt bank '"
                 << prompt_bank_path.string() << "' could not be used";
  }

  if (shouldUsePromptBankOnly(filter)) {
    LOG(ERROR) << "Tracked-object ignore filter source='prompt_bank' was requested, but no "
                  "prompt bank was loaded; tracked-object ignore filtering is disabled";
    return;
  }

  initializeTrackedObjectIgnorePromptBankFromService(nh, open_vocab, filter, bank);
}

}  // namespace

void declare_config(HydraRosPipeline::Config& config) {
  using namespace config;
  name("HydraRosConfig");
  field(config.active_window, "active_window");
  field(config.frontend, "frontend");
  field(config.backend, "backend");
  field(config.enable_frontend_output, "enable_frontend_output");
  field(config.enable_zmq_interface, "enable_zmq_interface");
  field(config.input, "input");
  config.features.setOptional();
  field(config.features, "features");
  config.open_vocab_features.setOptional();
  field(config.open_vocab_features, "open_vocab_features");
  field(config.open_vocab_object_query, "open_vocab_object_query");
  field(config.verbosity, "verbosity");
  field(config.preprint_config, "preprint_config");
  field(config.status_monitor, "status_monitor");
}

HydraRosPipeline::HydraRosPipeline(int robot_id, int config_verbosity)
    : HydraPipeline(config::fromContext<PipelineConfig>(), robot_id, config_verbosity),
      config(config::checkValid(config::fromContext<Config>())) {
  if (config.preprint_config) {
    LOG(INFO) << "Using configuration to start Hydra\n" << config::toString(config);
  } else {
    LOG_IF(INFO, config.verbosity >= 1)
        << "Starting Hydra-ROS with input configuration\n"
        << config::toString(config.input);
  }
}

HydraRosPipeline::~HydraRosPipeline() {}

void HydraRosPipeline::init() {
  const auto& pipeline_config = GlobalInfo::instance().getConfig();

  auto nh = ianvs::NodeHandle::this_node("~");
  initializeTrackedObjectIgnorePromptBank(nh, shared_state_);
  backend_ = config.backend.create(backend_dsg_, shared_state_);
  modules_["backend"] = CHECK_NOTNULL(backend_);

  frontend_ = config.frontend.create(frontend_dsg_, shared_state_);
  modules_["frontend"] = CHECK_NOTNULL(frontend_);

  active_window_ = config.active_window.create(frontend_->queue());
  modules_["active_window"] = CHECK_NOTNULL(active_window_);

  if (pipeline_config.enable_lcd) {
    initLCD();
    bow_sub_.reset(new BowSubscriber(nh));
  }

  status_monitor_ = std::make_unique<StatusMonitor>(config.status_monitor, nh);
  external_loop_closure_sub_.reset(new ExternalLoopClosureSubscriber(nh));
  open_vocab_object_query_server_ = std::make_unique<OpenVocabObjectQueryServer>(
      nh, frontend_dsg_, [this]() { this->stop(); });

  auto bnh = nh / "backend";
  backend_->addSink(std::make_shared<RosBackendPublisher>(bnh));
  backend_->addSink(BackendModule::Sink::fromCallback(
      [this](uint64_t timestamp_ns, const auto&, const auto&) {
        status_monitor_->recordModuleCallback("backend",
                                              std::chrono::nanoseconds(timestamp_ns));
      }));

  active_window_->addSink(ActiveWindowModule::Sink::fromCallback(
      [this](uint64_t timestamp_ns, const auto&, const auto&) {
        status_monitor_->recordModuleCallback("active_window",
                                              std::chrono::nanoseconds(timestamp_ns));
      }));

  // TODO(nathan) make optional config
  if (config.enable_zmq_interface) {
    const auto zmq_config = config::fromContext<ZmqSink::Config>("backend/zmq_sink");
    backend_->addSink(std::make_shared<ZmqSink>(zmq_config));
  }

  if (config.enable_frontend_output) {
    CHECK(frontend_) << "Frontend module required!";
    frontend_->addSink(std::make_shared<RosFrontendPublisher>(nh / "frontend"));
  }

  input_module_ =
      std::make_shared<RosInputModule>(config.input, active_window_->queue());
  if (config.features) {
    modules_["features"] = config.features.create();  // has to come after input module
  }
  if (config.open_vocab_features) {
    modules_["open_vocab_features"] = config.open_vocab_features.create(shared_state_);
  }
}

void HydraRosPipeline::start() {
  stopped_ = false;
  HydraPipeline::start();
  status_monitor_->start();
}

void HydraRosPipeline::stop() {
  std::lock_guard<std::mutex> lock(stop_mutex_);
  if (stopped_.exchange(true)) {
    return;
  }

  // TODO(nathan) log remaining queue sizes here or in stop
  // enforce stop order to make sure every data packet is processed
  input_module_->stop();
  // TODO(nathan) push extracting active window objects to module stop
  active_window_->stop();
  frontend_->stop();
  backend_->stop();

  HydraPipeline::stop();
}

void HydraRosPipeline::save(const DataDirectory& output) const {
  HydraPipeline::save(output);
  if (!output) {
    return;
  }

  const auto log_dir = output.path();
  std::ofstream fout(log_dir / "hydra_ros_config.yaml");
  fout << config::toYaml(config);
  LOG(INFO) << "[Hydra] saved ROS pipeline configuration to "
            << (log_dir / "hydra_ros_config.yaml");
}

void HydraRosPipeline::initLCD() {
  // TODO(nathan) push to pipeline config?
  auto lcd_config = config::fromContext<LoopClosureConfig>();
  lcd_config.detector.num_semantic_classes = GlobalInfo::instance().getTotalLabels();
  LOG_IF(INFO, config.verbosity >= 2)
      << "Number of classes for LCD: " << lcd_config.detector.num_semantic_classes;

  config::checkValid(lcd_config);

  auto lcd = std::make_shared<LoopClosureModule>(lcd_config, shared_state_);
  modules_["lcd"] = lcd;
  // TODO(nathan) rework sensor-level LCD request
}

}  // namespace hydra
