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

#ifndef CLIENT_NODE_HPP_
#define CLIENT_NODE_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "netperf_tool_interfaces/msg/bytes.hpp"
#include "netperf_tool_interfaces/srv/get_results.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

namespace netperf_tool
{
struct ClientCollectedInfo
{
  std::vector<size_t> message_ids;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> message_published_times;
  std::vector<size_t> message_sizes;
};

struct ClientResults
{
  netperf_tool_interfaces::srv::GetResults::Response statistics;
  ClientCollectedInfo collected_info;
};

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(
    const rclcpp::NodeOptions & options,
    const rclcpp::QoS & pub_qos,
    size_t array_size,
    std::chrono::nanoseconds target_pub_period,
    std::chrono::nanoseconds server_timeout);

  bool
  wait_for_server();

  bool
  wait_for_server(std::chrono::nanoseconds timeout);

  void
  start_publishing();

  void
  stop_publishing();

  void
  sync_with_server(rclcpp::Executor & exec);

  ClientResults
  extract_results();

  std::string
  get_topic_name();

  std::string
  get_stringified_pub_gid();

  void
  pub_next_msg();

private:
  netperf_tool_interfaces::msg::Bytes msg_to_publish_;
  size_t serialized_msg_size_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  rclcpp::Publisher<netperf_tool_interfaces::msg::Bytes>::SharedPtr pub_;
  rclcpp::Client<netperf_tool_interfaces::srv::GetResults>::SharedPtr client_;
  ClientCollectedInfo collected_info_;
  mutable std::mutex collected_info_mutex_;
  netperf_tool_interfaces::srv::GetResults::Response::SharedPtr statistics_;
  std::chrono::nanoseconds target_pub_period_;
  std::chrono::nanoseconds server_timeout_;
};
}  // namespace netperf_tool
#endif  // CLIENT_NODE_HPP_
