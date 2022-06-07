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

#include "perf_tool_msgs/msg/bytes.hpp"
#include "perf_tool_msgs/srv/get_results.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

namespace perf_tool
{
struct ClientCollectedInfo
{
  std::vector<size_t> message_ids;
  std::vector<std::chrono::time_point<std::chrono::system_clock>> message_published_times;
  std::vector<size_t> message_sizes;
};

struct ClientResults
{
  perf_tool_msgs::srv::GetResults::Response statistics;
  ClientCollectedInfo collected_info;
};

class ClientNode : public rclcpp::Node
{
public:
  explicit ClientNode(
    size_t array_size,
    std::chrono::nanoseconds target_pub_period,
    const rclcpp::QoS & pub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  bool
  wait_for_server(std::chrono::nanoseconds timeout = std::chrono::seconds{5});

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
  size_t array_size_;
  std::vector<unsigned char> bytes_;
  uint64_t next_id{};
  std::shared_ptr<rclcpp::TimerBase> timer_;
  rclcpp::Publisher<perf_tool_msgs::msg::Bytes>::SharedPtr pub_;
  rclcpp::Client<perf_tool_msgs::srv::GetResults>::SharedPtr client_;
  ClientCollectedInfo collected_info_;
  mutable std::mutex collected_info_mutex_;
  rclcpp::Serialization<perf_tool_msgs::msg::Bytes> serializer_;
  perf_tool_msgs::srv::GetResults::Response::SharedPtr statistics_;
  std::chrono::nanoseconds target_pub_period_;
};
}  // namespace perf_tool
#endif  // CLIENT_NODE_HPP_
