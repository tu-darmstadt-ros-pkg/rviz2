/*
* Copyright (c) 2023, Open Source Robotics Foundation, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the copyright holder nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
* LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_
#define RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_

#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "point_cloud_transport/point_cloud_transport.hpp"
#include "point_cloud_transport/subscriber_filter.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/ros_topic_display.hpp"

namespace rviz_default_plugins
{
namespace displays
{

template<class MessageType>
class PointCloud2TransportDisplay : public rviz_common::_RosTopicDisplay
{
// No Q_OBJECT macro here, moc does not support Q_OBJECT in a templated class.

public:
/// Convenience typedef so subclasses don't have to use
/// the long templated class name to refer to their super class.
  typedef PointCloud2TransportDisplay<MessageType> PC2RDClass;

  PointCloud2TransportDisplay()
  : messages_received_(0),
    transport_manually_set_(false),
    programmatic_transport_change_(false)
  {
    QString message_type = QString::fromStdString(rosidl_generator_traits::name<MessageType>());
    topic_property_->setMessageType(message_type);
    topic_property_->setDescription(message_type + " topic to subscribe to.");

    // Note: no SLOT here — transport changes are handled in onInitialize().
    transport_property_ = new rviz_common::properties::EnumProperty(
      "Transport", "raw",
      "Point cloud transport to use for subscribing to the selected topic.",
      this);
  }

/**
* When overriding this method, the onInitialize() method of this superclass has to be called.
* Otherwise, the ros node will not be initialized.
*/
  void onInitialize() override
  {
    _RosTopicDisplay::onInitialize();
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
          resetSubscription();
        }
      });
  }

  ~PointCloud2TransportDisplay() override
  {
    unsubscribe();
  }

  void reset() override
  {
    Display::reset();
    messages_received_ = 0;
  }

  void setTopic(const QString & topic, const QString & datatype) override
  {
    (void) datatype;
    topic_property_->setString(topic);
  }

protected:
  void updateTopic() override
  {
    // Topic changed: reset manual flag so we can auto-select again.
    transport_manually_set_ = false;
    fillTransportOptionList();
    resetSubscription();
  }

  virtual void subscribe()
  {
    if (!isEnabled()) {
      return;
    }

    if (topic_property_->isEmpty()) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: Empty topic name"));
      return;
    }

    try {
      std::string base_topic = topic_property_->getTopicStd();
      std::string transport = transport_property_->getStdString();

      subscription_ = std::make_shared<point_cloud_transport::SubscriberFilter>();
      subscription_->subscribe(
        rviz_ros_node_.lock()->get_raw_node(),
        base_topic,
        transport,
        qos_profile.get_rmw_qos_profile());
      subscription_start_time_ = rviz_ros_node_.lock()->get_raw_node()->now();
      subscription_callback_ = subscription_->registerCallback(
        std::bind(
          &PointCloud2TransportDisplay<MessageType>::incomingMessage, this, std::placeholders::_1));
      setStatus(rviz_common::properties::StatusProperty::Ok, "Topic", "OK");
    } catch (rclcpp::exceptions::InvalidTopicNameError & e) {
      setStatus(
        rviz_common::properties::StatusProperty::Error, "Topic",
        QString("Error subscribing: ") + e.what());
    }
  }

  void transformerChangedCallback() override
  {
    resetSubscription();
  }

  void resetSubscription()
  {
    unsubscribe();
    reset();
    subscribe();
    context_->queueRender();
  }

  virtual void unsubscribe()
  {
    subscription_.reset();
  }

  void onEnable() override
  {
    subscribe();
  }

  void onDisable() override
  {
    unsubscribe();
    reset();
  }

/// Incoming message callback.
/**
* Checks if the message pointer
* is valid, increments messages_received_, then calls
* processMessage().
*/
  void incomingMessage(const typename MessageType::ConstSharedPtr msg)
  {
    if (!msg) {
      return;
    }

    ++messages_received_;
    QString topic_str = QString::number(messages_received_) + " messages received";
    // Append topic subscription frequency if we can lock rviz_ros_node_.
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_interface =
      rviz_ros_node_.lock();
    if (node_interface != nullptr) {
      const double duration =
        (node_interface->get_raw_node()->now() - subscription_start_time_).seconds();
      const double subscription_frequency =
        static_cast<double>(messages_received_) / duration;
      topic_str += " at " + QString::number(subscription_frequency, 'f', 1) + " hz.";
    }
    setStatus(
      rviz_common::properties::StatusProperty::Ok,
      "Topic",
      topic_str);

    processMessage(msg);
  }


/// Implement this to process the contents of a message.
/**
* This is called by incomingMessage().
*/
  virtual void processMessage(typename MessageType::ConstSharedPtr msg) = 0;

  /// Scan for available point_cloud_transport subscriber plugins.
  void scanForTransportSubscriberPlugins()
  {
    try {
      point_cloud_transport::PointCloudTransportLoader loader;
      // getLoadableTransports() returns {lookup_name: transport_name} for all
      // transports that can actually be loaded (verified internally).
      auto loadable = loader.getLoadableTransports();
      for (const auto & [lookup_name, transport_name] : loadable) {
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
    transport_property_->clearOptions();

    std::string current_transport = transport_property_->getStdString();

    // Always add "raw" first.
    transport_property_->addOptionStd("raw");

    auto node = rviz_ros_node_.lock();
    if (!node) {
      return;
    }

    std::string base_topic = topic_property_->getTopicStd();
    if (base_topic.empty()) {
      return;
    }

    // Query published topics and look for sub-topics of the base topic
    // that match known transport plugin names.
    std::map<std::string, std::vector<std::string>> published_topics =
      node->get_topic_names_and_types();

    for (const auto & [topic_name, topic_types] : published_topics) {
      // Check if topic_name is a direct child of base_topic (e.g. /cloud/draco).
      if (topic_name.find(base_topic) == 0 &&
        topic_name != base_topic &&
        topic_name[base_topic.size()] == '/' &&
        topic_name.find('/', base_topic.size() + 1) == std::string::npos)
      {
        std::string transport_type = topic_name.substr(base_topic.size() + 1);
        if (transport_plugin_types_.count(transport_type) > 0 && transport_type != "raw") {
          transport_property_->addOptionStd(transport_type);
        }
      }
    }

    // Determine what transport to use.
    // Priority order for auto-selection (prefer compressed transports).
    const std::vector<std::string> preferred_order = {"cloudini", "draco", "zstd", "zlib"};

    if (!transport_manually_set_) {
      // Auto-select: pick the first preferred transport that is advertised.
      std::string auto_transport;
      for (const auto & preferred : preferred_order) {
        if (transport_plugin_types_.count(preferred) > 0) {
          // Check if it appears in the option list (i.e. it's advertised).
          for (const auto & [topic_name2, topic_types2] : published_topics) {
            if (topic_name2 == base_topic + "/" + preferred) {
              auto_transport = preferred;
              break;
            }
          }
          if (!auto_transport.empty()) {break;}
        }
      }
      std::string target = auto_transport.empty() ? "raw" : auto_transport;
      programmatic_transport_change_ = true;
      transport_property_->setString(QString::fromStdString(target));
      programmatic_transport_change_ = false;
    } else {
      // Restore previous selection if still in the list, otherwise fall back to raw.
      programmatic_transport_change_ = true;
      transport_property_->setString(QString::fromStdString(current_transport));
      programmatic_transport_change_ = false;
    }
  }

  uint32_t messages_received_;
  rclcpp::Time subscription_start_time_;

  std::shared_ptr<point_cloud_transport::SubscriberFilter> subscription_;
  message_filters::Connection subscription_callback_;

  rviz_common::properties::EnumProperty * transport_property_;
  std::set<std::string> transport_plugin_types_;
  bool transport_manually_set_;
  bool programmatic_transport_change_;
};

}  //  end namespace displays
}  // end namespace rviz_default_plugins


#endif  // RVIZ_DEFAULT_PLUGINS__DISPLAYS__POINTCLOUD__POINT_CLOUD_TRANSPORT_DISPLAY_HPP_
