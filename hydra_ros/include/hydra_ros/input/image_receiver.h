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
#include <ianvs/node_handle.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <functional>
#include <utility>

#include <rclcpp/time.hpp>
#include <instance_tracking_msgs/msg/tracked_instances.hpp>
#include <semantic_inference_msgs/msg/feature_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "hydra_ros/input/ros_data_receiver.h"

namespace hydra {

template <typename MsgT>
struct FilterSub : public message_filters::SimpleFilter<MsgT> {
  using Observer = std::function<void(const MsgT&)>;

  FilterSub(ianvs::NodeHandle nh,
            const std::string& topic,
            uint32_t queue_size,
            Observer observer = {})
      : observer_(std::move(observer)),
        subscriber(nh.create_subscription<MsgT>(
            topic, queue_size, [this](const typename MsgT::ConstSharedPtr& msg) {
              if (observer_) {
                observer_(*msg);
              }
              this->signalMessage(msg);
            })) {}

  Observer observer_;
  typename rclcpp::Subscription<MsgT>::SharedPtr subscriber;
};

struct ColorSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  ColorSubscriber();
  explicit ColorSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~ColorSubscriber();

  Filter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;
  void recordSync(const MsgType& msg, int64_t timestamp_ns) const;

 private:
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct DepthSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  DepthSubscriber();
  explicit DepthSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~DepthSubscriber();

  Filter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;
  void recordSync(const MsgType& msg, int64_t timestamp_ns) const;

 private:
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct LabelSubscriber {
 public:
  using MsgType = sensor_msgs::msg::Image;
  using Filter = message_filters::SimpleFilter<MsgType>;

  LabelSubscriber();
  explicit LabelSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~LabelSubscriber();

  Filter& getFilter() const;
  void fillInput(const sensor_msgs::msg::Image& img, ImageInputPacket& packet) const;
  void recordSync(const MsgType& msg, int64_t timestamp_ns) const;

 private:
  std::shared_ptr<FilterSub<sensor_msgs::msg::Image>> impl_;
};

struct FeatureSubscriber {
 public:
  using MsgType = semantic_inference_msgs::msg::FeatureImage;
  using Filter = message_filters::SimpleFilter<MsgType>;

  FeatureSubscriber();
  explicit FeatureSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~FeatureSubscriber();

  Filter& getFilter() const;
  void fillInput(const MsgType& img, ImageInputPacket& packet) const;
  void recordSync(const MsgType& msg, int64_t timestamp_ns) const;

 private:
  std::shared_ptr<FilterSub<semantic_inference_msgs::msg::FeatureImage>> impl_;
};

struct TrackedInstancesSubscriber {
 public:
  using MsgType = instance_tracking_msgs::msg::TrackedInstances;
  using Filter = message_filters::SimpleFilter<MsgType>;

  TrackedInstancesSubscriber();
  explicit TrackedInstancesSubscriber(ianvs::NodeHandle nh, uint32_t queue_size = 1);
  virtual ~TrackedInstancesSubscriber();

  Filter& getFilter() const;
  void fillInput(const MsgType& msg, ImageInputPacket& packet) const;
  void recordSync(const MsgType& msg, int64_t timestamp_ns) const;

 private:
  std::shared_ptr<FilterSub<MsgType>> impl_;
};

template <typename SemanticT>
class ImageReceiverImpl : public RosDataReceiver {
 public:
  using SemanticMsgPtr = typename SemanticT::MsgType::ConstSharedPtr;
  using Policy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
                                                      sensor_msgs::msg::Image,
                                                      typename SemanticT::MsgType>;
  using Synchronizer = message_filters::Synchronizer<Policy>;

  ImageReceiverImpl(const RosDataReceiver::Config& config,
                    const std::string& sensor_name);
  virtual ~ImageReceiverImpl() = default;

 protected:
  bool initImpl() override;

  void callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                const SemanticMsgPtr& labels);

  ColorSubscriber color_sub_;
  DepthSubscriber depth_sub_;
  SemanticT semantic_sub_;
  std::unique_ptr<Synchronizer> sync_;
};

template <typename SemanticT>
ImageReceiverImpl<SemanticT>::ImageReceiverImpl(const Config& config,
                                                const std::string& sensor_name)
    : RosDataReceiver(config, sensor_name) {}

template <typename SemanticT>
bool ImageReceiverImpl<SemanticT>::initImpl() {
  color_sub_ = ColorSubscriber(ianvs::NodeHandle::this_node(ns_), config.queue_size);
  depth_sub_ = DepthSubscriber(ianvs::NodeHandle::this_node(ns_), config.queue_size);
  semantic_sub_ = SemanticT(ianvs::NodeHandle::this_node(ns_), config.queue_size);
  sync_.reset(new Synchronizer(Policy(config.queue_size),
                               color_sub_.getFilter(),
                               depth_sub_.getFilter(),
                               semantic_sub_.getFilter()));
  sync_->registerCallback(&ImageReceiverImpl<SemanticT>::callback, this);
  return true;
}

template <typename SemanticT>
void ImageReceiverImpl<SemanticT>::callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& color,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth,
    const SemanticMsgPtr& labels) {
  const auto timestamp_ns = rclcpp::Time(color->header.stamp).nanoseconds();
  if (!checkInputTimestamp(timestamp_ns)) {
    return;
  }

  auto packet = std::make_shared<ImageInputPacket>(timestamp_ns, sensor_name_);
  color_sub_.fillInput(*color, *packet);
  depth_sub_.fillInput(*depth, *packet);
  semantic_sub_.recordSync(*labels, timestamp_ns);
  semantic_sub_.fillInput(*labels, *packet);
  queue.push(packet);
}

class ClosedSetImageReceiver : public ImageReceiverImpl<LabelSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  ClosedSetImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~ClosedSetImageReceiver() = default;
};

class OpenSetImageReceiver : public ImageReceiverImpl<FeatureSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  OpenSetImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~OpenSetImageReceiver() = default;
};

class TrackedImageReceiver : public ImageReceiverImpl<TrackedInstancesSubscriber> {
 public:
  struct Config : RosDataReceiver::Config {};
  TrackedImageReceiver(const Config& config, const std::string& sensor_name);
  virtual ~TrackedImageReceiver() = default;
};

void declare_config(OpenSetImageReceiver::Config& config);
void declare_config(TrackedImageReceiver::Config& config);

}  // namespace hydra
