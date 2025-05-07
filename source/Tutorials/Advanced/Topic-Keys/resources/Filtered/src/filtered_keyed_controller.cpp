// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <demo_keys_filtering_cpp/msg/keyed_sensor_data_msg.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

namespace demo_keys_filtering_cpp
{

class FilteredKeyedControllerNode : public rclcpp::Node
{
  static constexpr double SENSOR_TRIGGER = 60.0;

public:

  explicit FilteredKeyedControllerNode(const rclcpp::NodeOptions & options)
  : Node("filtered_keyed_controller", options)
  {
    // Create a callback function for when messages are received.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](demo_keys_filtering_cpp::msg::KeyedSensorDataMsg::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Received data from sensor [%d] measurement: [%f]",(int)msg->sensor_id, (float)msg->measurement);
      };

    // Initialize a subscription with a content filter to receive data from sensors 2 to 4
    rclcpp::SubscriptionOptions sub_options;
    sub_options.content_filter_options.filter_expression =
        "sensor_id >= 2 AND sensor_id <= 4 AND measurement > %0";
    sub_options.content_filter_options.expression_parameters = {
      std::to_string(SENSOR_TRIGGER)
    };

    // Create the subscription with the content filter options
    sub_ = create_subscription<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>("/robot/sensors",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        callback,
        sub_options);

    if (!sub_->is_cft_enabled()) {
      RCLCPP_WARN(
        this->get_logger(), "Content filter is not enabled since it's not supported");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "subscribed to topic \"%s\" with content filter options \"%s, {%s}\"",
        sub_->get_topic_name(),
        sub_options.content_filter_options.filter_expression.c_str(),
        rcpputils::join(sub_options.content_filter_options.expression_parameters, ", ").c_str());
    }
  }

private:
  rclcpp::Subscription<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>::SharedPtr sub_;
};

} // namespace demo_keys_filtering_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_keys_filtering_cpp::FilteredKeyedControllerNode)
