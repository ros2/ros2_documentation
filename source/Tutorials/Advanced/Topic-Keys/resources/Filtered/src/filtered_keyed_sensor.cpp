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

#include <chrono>
#include <cstdio>
#include <memory>
#include <random>
#include <utility>

#include <demo_keys_filtering_cpp/msg/keyed_sensor_data_msg.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace demo_keys_filtering_cpp
{

class FilteredKeyedSensorNode : public rclcpp::Node
{
public:

  explicit FilteredKeyedSensorNode(const rclcpp::NodeOptions & options)
  : Node("keyed_sensor", options)
  {
    this->declare_parameter<int>("id", 0);
    id_ = this->get_parameter("id").as_int();

    period_ = std::chrono::milliseconds(id_ * 1000);
    measurement_ = distribution_(generator_);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>();
        msg_->sensor_id = id_;
        msg_->measurement = measurement_;
        msg_->data = "Sensor [" + std::to_string(id_) + "] publishing measurement " + std::to_string(measurement_);
        RCLCPP_INFO(this->get_logger(), "%s", msg_->data.c_str());
        measurement_ = distribution_(generator_);
        pub_->publish(std::move(msg_));
      };

    pub_ = this->create_publisher<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>(
        "/robot/sensors",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(period_, publish_message);
  }

private:
  uint16_t id_{0};
  std::chrono::milliseconds period_{1000};
  std::default_random_engine generator_;
  std::uniform_real_distribution<float> distribution_{50.0, 90.0};
  float measurement_{0};
  std::unique_ptr<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg> msg_;
  rclcpp::Publisher<demo_keys_filtering_cpp::msg::KeyedSensorDataMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace demo_keys_filtering_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_keys_filtering_cpp::FilteredKeyedSensorNode)

