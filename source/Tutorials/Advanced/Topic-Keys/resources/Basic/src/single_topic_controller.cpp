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

#include <demo_keys_cpp/msg/sensor_data_msg.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

namespace demo_keys_cpp
{

class SingleTopicControllerNode : public rclcpp::Node
{
public:

  explicit SingleTopicControllerNode(const rclcpp::NodeOptions & options)
  : Node("single_topic_controller", options)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](demo_keys_cpp::msg::SensorDataMsg::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Received data from sensor [%d]",(int)msg->sensor_id);
      };

    sub_ = create_subscription<demo_keys_cpp::msg::SensorDataMsg>("/robot/sensors",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        callback);
  }

private:
  rclcpp::Subscription<demo_keys_cpp::msg::SensorDataMsg>::SharedPtr sub_;
};

} // namespace demo_keys_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_keys_cpp::SingleTopicControllerNode)
