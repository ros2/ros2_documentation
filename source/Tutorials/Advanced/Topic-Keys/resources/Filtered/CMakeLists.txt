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

project(demo_keys_filtering_cpp)

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
  "msg/KeyedSensorDataMsg.idl"
)

ament_export_dependencies(rosidl_default_runtime)

# Prepare liraries
add_library(filtered_keyed_sensor_component src/filtered_keyed_sensor.cpp)
ament_target_dependencies(filtered_keyed_sensor_component rclcpp rclcpp_components)

add_library(filtered_keyed_controller_component src/filtered_keyed_controller.cpp)
ament_target_dependencies(filtered_keyed_controller_component rclcpp rclcpp_components)

# Declare components
rclcpp_components_register_node(
    filtered_keyed_sensor_component
    PLUGIN "demo_keys_filtering_cpp::FilteredKeyedSensorNode"
    EXECUTABLE filtered_keyed_sensor
)

rclcpp_components_register_node(
    filtered_keyed_controller_component
    PLUGIN "demo_keys_filtering_cpp::FilteredKeyedControllerNode"
    EXECUTABLE filtered_keyed_controller
)

# Attach to cpp generation target
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link against the generated source files
target_link_libraries(filtered_keyed_sensor_component "${cpp_typesupport_target}")
target_link_libraries(filtered_keyed_controller_component "${cpp_typesupport_target}")

# Install
install(TARGETS
  filtered_keyed_sensor_component
  filtered_keyed_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
