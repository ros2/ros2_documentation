# Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.12)

project(demo_keys_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorDataMsg.idl"
  "msg/KeyedSensorDataMsg.idl"
)

ament_export_dependencies(rosidl_default_runtime)

# Prepare libraries
add_library(single_topic_sensor_component src/single_topic_sensor.cpp)
ament_target_dependencies(single_topic_sensor_component rclcpp rclcpp_components)

add_library(single_topic_controller_component src/single_topic_controller.cpp)
ament_target_dependencies(single_topic_controller_component rclcpp rclcpp_components)

add_library(multiple_topic_sensor_component src/multiple_topic_sensor.cpp)
ament_target_dependencies(multiple_topic_sensor_component rclcpp rclcpp_components)

add_library(multiple_topic_controller_component src/multiple_topic_controller.cpp)
ament_target_dependencies(multiple_topic_controller_component rclcpp rclcpp_components)

add_library(keyed_sensor_component src/keyed_sensor.cpp)
ament_target_dependencies(keyed_sensor_component rclcpp rclcpp_components)

add_library(keyed_controller_component src/keyed_controller.cpp)
ament_target_dependencies(keyed_controller_component rclcpp rclcpp_components)

# Declare components
rclcpp_components_register_node(
    single_topic_sensor_component
    PLUGIN "demo_keys_cpp::SingleTopicSensorNode"
    EXECUTABLE single_topic_sensor
)

rclcpp_components_register_node(
    single_topic_controller_component
    PLUGIN "demo_keys_cpp::SingleTopicControllerNode"
    EXECUTABLE single_topic_controller
)

rclcpp_components_register_node(
    multiple_topic_sensor_component
    PLUGIN "demo_keys_cpp::MultipleTopicSensorNode"
    EXECUTABLE multiple_topic_sensor
)

rclcpp_components_register_node(
    multiple_topic_controller_component
    PLUGIN "demo_keys_cpp::MultipleTopicControllerNode"
    EXECUTABLE multiple_topic_controller
)

rclcpp_components_register_node(
    keyed_sensor_component
    PLUGIN "demo_keys_cpp::KeyedSensorNode"
    EXECUTABLE keyed_sensor
)

rclcpp_components_register_node(
    keyed_controller_component
    PLUGIN "demo_keys_cpp::KeyedControllerNode"
    EXECUTABLE keyed_controller
)

# Attach to cpp generation target
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link against the generated source files
target_link_libraries(single_topic_sensor_component "${cpp_typesupport_target}")
target_link_libraries(single_topic_controller_component "${cpp_typesupport_target}")
target_link_libraries(multiple_topic_sensor_component "${cpp_typesupport_target}")
target_link_libraries(multiple_topic_controller_component "${cpp_typesupport_target}")
target_link_libraries(keyed_sensor_component "${cpp_typesupport_target}")
target_link_libraries(keyed_controller_component "${cpp_typesupport_target}")

# Install
install(TARGETS
  single_topic_sensor_component
  single_topic_controller_component
  multiple_topic_sensor_component
  multiple_topic_controller_component
  keyed_sensor_component
  keyed_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
