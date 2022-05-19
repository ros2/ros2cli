// Copyright 2022 Open Source Robotics Foundation, Inc.
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
#include <functional>

#include "perf_tool_msgs/msg/bytes.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/subscription.hpp"

#include "rmw/features.h"

// Ignore -Wunused-value for clang.
// Based on https://github.com/pybind/pybind11/issues/2225
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-value"
#endif
#include <pybind11/pybind11.h>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#include <pybind11/stl.h>
#include <pybind11/chrono.h>

using namespace std::chrono_literals;

namespace perf_tool
{

struct PerfToolMessageInfo
{
  size_t message_id;
  std::chrono::nanoseconds latency;
};

class Server : public rclcpp::Node
{
public:
  explicit Server(
    const rclcpp::QoS & sub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{})
  : Node("perf_tool_server", options)
  {
    this->create_subscription<perf_tool_msgs::msg::Bytes>(
      "perf_topic", sub_qos,
      std::bind(&Server::handle_msg, this, std::placeholders::_1, std::placeholders::_2));
  }

  void handle_msg(
    const perf_tool_msgs::msg::Bytes & msg,
    const rclcpp::MessageInfo & msg_info)
  {
    PerfToolMessageInfo perf_info;
    perf_info.latency = this->calculate_latency(msg, msg_info);
    // we could use the publication sequence number instead, but not all rmw support it
    perf_info.message_id = msg.id;
  }

  std::chrono::nanoseconds
  calculate_latency(
    const perf_tool_msgs::msg::Bytes & msg,
    const rclcpp::MessageInfo & msg_info)
  {
    std::chrono::nanoseconds latency;
    const auto & rmw_msg_info = msg_info.get_rmw_message_info();
    if (rmw_msg_info.source_timestamp != 0 && rmw_msg_info.received_timestamp != 0) {
      // source and received timestamp supported
      // TODO(ivanpauno): Add a way to evaluate this in rmw_feature_supported() to avoid the if().
      RCLCPP_INFO_ONCE(
        this->get_logger(),
        "source and received timestamp supported, using them to calculate latency!");
      auto pub_timestamp = std::chrono::time_point<std::chrono::system_clock>{
        std::chrono::nanoseconds{rmw_msg_info.source_timestamp}};
      auto rec_timestamp = 
        std::chrono::time_point<std::chrono::system_clock>{
          std::chrono::nanoseconds{rmw_msg_info.received_timestamp}};
      latency = rec_timestamp - pub_timestamp;
    } else {
      auto rec_timestamp = std::chrono::system_clock::now();
      // TODO(ivanpauno): Maybe it's worth to always calculate this latency (?)
      // as it takes in account the executor overhead.
      RCLCPP_INFO_ONCE(
        this->get_logger(),
        "either source or received timestamp are not supported,"
        "using timestamps added to the message to calculate latency");
      auto pub_timestamp = 
        std::chrono::time_point<std::chrono::system_clock>{
          std::chrono::nanoseconds{msg.timestamp}};
      latency = rec_timestamp - pub_timestamp;
    }
    return latency;
  }

private:
  std::vector<PerfToolMessageInfo> collected_info_;
  rclcpp::Subscription<perf_tool_msgs::msg::Bytes>::SharedPtr sub_;
};

}  // namespace perf_tool

PYBIND11_MODULE(perf_tool_impl, m) {
  m.doc() = "Python wrapper of perf tool implementation";

  // pybind11::class_<perf_tool::Server, std::shared_ptr<perf_tool::Server>>(
  //   m, "Server")
  // .def(pybind11::init());
  // // .def("open", &rosbag2_py::Writer<rosbag2_cpp::writers::SequentialWriter>::open)
}
