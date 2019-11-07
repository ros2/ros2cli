// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef ROS2COMPONENT_TEST_FIXTURES__LISTENER_COMPONENT_HPP_
#define ROS2COMPONENT_TEST_FIXTURES__LISTENER_COMPONENT_HPP_

#include "ros2component_test_fixtures/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2component_test_fixtures
{

class Listener : public rclcpp::Node
{
public:
  ROS2COMPONENT_TEST_FIXTURES_PUBLIC
  explicit Listener(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

}  // namespace ros2component_test_fixtures

#endif  // ROS2COMPONENT_TEST_FIXTURES__LISTENER_COMPONENT_HPP_
