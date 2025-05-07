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

class MultipleTopicControllerNode : public rclcpp::Node
{
  static constexpr size_t NUM_SENSORS = 10;

public:

  explicit MultipleTopicControllerNode(const rclcpp::NodeOptions & options)
  : Node("controller", options)
  {
    subs_.reserve(NUM_SENSORS);

    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto callback =
      [this](demo_keys_cpp::msg::SensorDataMsg::ConstSharedPtr msg) -> void
      {
        RCLCPP_INFO(this->get_logger(), "Received data from sensor [%d]",(int)msg->sensor_id);
      };

    for (size_t i = 1; i <= NUM_SENSORS; i++)
    {
      subs_.emplace_back(create_subscription<demo_keys_cpp::msg::SensorDataMsg>("/robot/sensor_" + std::to_string(i),
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
        callback));
    }
  }

private:
  std::vector<rclcpp::Subscription<demo_keys_cpp::msg::SensorDataMsg>::SharedPtr> subs_;
};

} // namespace demo_keys_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_keys_cpp::MultipleTopicControllerNode)
