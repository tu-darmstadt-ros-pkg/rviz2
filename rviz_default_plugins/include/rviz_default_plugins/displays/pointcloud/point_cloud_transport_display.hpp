// Copyright (c) 2023, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "get_transport_from_topic.hpp"
#include "point_cloud_transport/point_cloud_transport.hpp"
#include "point_cloud_transport/subscriber_filter.hpp"
#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/enum_property.hpp"

namespace rviz_default_plugins
{
namespace displays
{

template<class MessageType>
class PointCloud2TransportDisplay : public rviz_common::MessageFilterDisplay<MessageType>
{
  // No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
  /// Convenience typedef so subclasses don't have to use
  /// the long templated class name to refer to their super class.
  typedef PointCloud2TransportDisplay<MessageType> PC2RDClass;

  PointCloud2TransportDisplay()
  : transport_manually_set_(false),
    programmatic_transport_change_(false)
  {
    // Note: no SLOT here — transport changes are handled in onInitialize().
    transport_property_ = new rviz_common::properties::EnumProperty(
      "Transport", "raw",
      "Point cloud transport to use for subscribing to the selected topic.",
      this);
  }

  void onInitialize() override
  {
    rviz_common::MessageFilterDisplay<MessageType>::onInitialize();
    scanForTransportSubscriberPlugins();

    // Refresh the transport list every time the dropdown is opened.
    QObject::connect(
      transport_property_,
      &rviz_common::properties::EnumProperty::requestOptions,
      [this](rviz_common::properties::EnumProperty *) {
        fillTransportOptionList();
      });

    // When the user explicitly picks a transport, mark it as manually set
    // and resubscribe. Programmatic changes (auto-select) are excluded via the guard.
    QObject::connect(
      transport_property_,
      &rviz_common::properties::EnumProperty::changed,
      [this]() {
        if (!programmatic_transport_change_) {
          transport_manually_set_ = true;
          this->resetSubscription();
        }
      });
  }

  // Ensure subscriber_filter_ outlives tf_filter_ during destruction.
  ~PointCloud2TransportDisplay() override
  {
    unsubscribe();
  }

protected:
  void updateTopic() override
  {
    // Topic changed: reset manual flag so we can auto-select again.
    // Done before resubscribing so the auto-selected transport is used right away.
    transport_manually_set_ = false;
    fillTransportOptionList();
    rviz_common::MessageFilterDisplay<MessageType>::updateTopic();
  }

  void subscribe() override
  {
    if (!this->isEnabled()) {
      return;
    }

    if (this->topic_property_->isEmpty()) {
      this->setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      auto node_abstraction = this->rviz_ros_node_.lock();
      if (!node_abstraction) {
        this->setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                        QString("Error subscribing: ROS node is no longer available"));
        return;
      }
      rclcpp::Node::SharedPtr node = node_abstraction->get_raw_node();
      const std::string topic = this->topic_property_->getTopicStd();
      // A transport sub-topic (e.g. /cloud/draco) selected directly takes precedence
      // over the dropdown.
      std::string transport = getPointCloud2TransportFromTopic(topic);
      if (transport == "raw") {
        transport = transport_property_->getStdString();
      }
      subscriber_filter_ = std::make_shared<point_cloud_transport::SubscriberFilter>();
      subscriber_filter_->subscribe(
        rclcpp::node_interfaces::NodeInterfaces<
          rclcpp::node_interfaces::NodeBaseInterface,
          rclcpp::node_interfaces::NodeParametersInterface,
          rclcpp::node_interfaces::NodeTopicsInterface,
          rclcpp::node_interfaces::NodeLoggingInterface>(*node),
        getPointCloud2BaseTopicFromTopic(topic),
        transport,
        this->qos_profile);
      this->subscription_start_time_ = node->now();
      this->tf_filter_ =
        std::make_shared<tf2_ros::MessageFilter<MessageType,
          rviz_common::transformation::FrameTransformer>>(
              *this->context_->getFrameManager()->getTransformer(),
            this->fixed_frame_.toStdString(),
              static_cast<uint32_t>(this->message_queue_property_->getInt()), *node);
      this->tf_filter_->connectInput(*subscriber_filter_);
      this->tf_filter_->registerCallback(
          std::bind(&PointCloud2TransportDisplay<MessageType>::messageTaken, this,
            std::placeholders::_1));
      this->setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      this->setStatus(rviz_common::properties::StatusProperty::Error, "Topic",
                      QString("Error subscribing: ") + e.what());
    }
  }

  virtual void unsubscribe()
  {
    rviz_common::MessageFilterDisplay<MessageType>::unsubscribe();
    subscriber_filter_.reset();
  }

  /// Scan for available point_cloud_transport subscriber plugins.
  void scanForTransportSubscriberPlugins()
  {
    try {
      point_cloud_transport::PointCloudTransportLoader loader;
      // getLoadableTransports() returns {lookup_name: transport_name} for all
      // transports that can actually be loaded (verified internally).
      auto loadable = loader.getLoadableTransports();
      for (const auto & [_, transport_name] : loadable) {
        transport_plugin_types_.insert(transport_name);
      }
    } catch (...) {
      // If something goes wrong, at least "raw" is available.
    }
    // Always ensure "raw" is available.
    transport_plugin_types_.insert("raw");
  }

  /// Fill the transport dropdown with transports that are actually advertised
  /// for the currently selected base topic.
  void fillTransportOptionList()
  {
    std::string current_transport = transport_property_->getStdString();
    transport_property_->clearOptions();

    // Always add "raw" first.
    transport_property_->addOptionStd("raw");

    auto node = this->rviz_ros_node_.lock();
    if (!node) {
      return;
    }

    std::string base_topic =
      getPointCloud2BaseTopicFromTopic(this->topic_property_->getTopicStd());
    if (base_topic.empty()) {
      return;
    }

    // Query published topics and look for sub-topics of the base topic
    // that match known transport plugin names.
    std::map<std::string, std::vector<std::string>> published_topics =
      node->get_topic_names_and_types();

    std::set<std::string> advertised;
    for (const auto & [topic_name, topic_types] : published_topics) {
      (void) topic_types;
      // Check if topic_name is a direct child of base_topic (e.g. /cloud/draco).
      if (topic_name.size() > base_topic.size() + 1 &&
        topic_name.compare(0, base_topic.size(), base_topic) == 0 &&
        topic_name[base_topic.size()] == '/' &&
        topic_name.find('/', base_topic.size() + 1) == std::string::npos)
      {
        std::string transport_type = topic_name.substr(base_topic.size() + 1);
        if (transport_plugin_types_.count(transport_type) > 0 && transport_type != "raw") {
          transport_property_->addOptionStd(transport_type);
          advertised.insert(transport_type);
        }
      }
    }

    std::string target = "raw";
    if (!transport_manually_set_) {
      // Auto-select: pick the first preferred (compressed) transport that is advertised.
      const std::vector<std::string> preferred_order = {"cloudini", "draco", "zstd", "zlib"};
      for (const auto & preferred : preferred_order) {
        if (advertised.count(preferred) > 0) {
          target = preferred;
          break;
        }
      }
    } else {
      // Keep the user's selection, even if it is not (yet) advertised.
      target = current_transport;
    }
    programmatic_transport_change_ = true;
    transport_property_->setString(QString::fromStdString(target));
    programmatic_transport_change_ = false;
  }

  std::shared_ptr<point_cloud_transport::SubscriberFilter> subscriber_filter_;
  rviz_common::properties::EnumProperty * transport_property_;
  std::set<std::string> transport_plugin_types_;
  bool transport_manually_set_;
  bool programmatic_transport_change_;
};

}  //  end namespace displays
}  // end namespace rviz_default_plugins

#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_
