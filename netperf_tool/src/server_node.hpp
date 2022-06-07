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

#ifndef SERVER_NODE_HPP_
#define SERVER_NODE_HPP_

#include <chrono>
#include <condition_variable>
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
#include "rclcpp/subscription.hpp"

namespace netperf_tool
{

struct ServerCollectedInfo
{
  std::vector<std::chrono::time_point<std::chrono::system_clock>> message_reception_time;
  std::vector<size_t> message_ids;
  std::vector<std::chrono::nanoseconds> message_latencies;
  std::vector<size_t> message_sizes;
};

struct ServerResults
{
  netperf_tool_interfaces::srv::GetResults::Response statistics;
  ServerCollectedInfo collected_info;
};

class ServerNode : public rclcpp::Node
{
public:
  explicit ServerNode(
    const rclcpp::QoS & sub_qos,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  void handle_msg(
    const rclcpp::SerializedMessage & serialized_msg,
    const rclcpp::MessageInfo & msg_info);

  void
  handle_get_results_request(
    netperf_tool_interfaces::srv::GetResults::Request::SharedPtr req,
    netperf_tool_interfaces::srv::GetResults::Response::SharedPtr rep);

  using PubGidToResultsMap = std::unordered_map<std::string, ServerResults>;

  PubGidToResultsMap
  extract_results();

  std::vector<std::string>
  extract_new_clients();

  void
  wait_for_results_available(std::chrono::nanoseconds timeout);

  std::string
  get_topic_name();

  void
  finalize_work(rclcpp::Executor &);

private:
  using PubGidToMessageData = std::unordered_map<std::string, ServerCollectedInfo>;
  // received messages info, agrouped by publisher gid
  PubGidToMessageData pub_gid_to_collected_info_map_;
  mutable std::mutex pub_gid_to_collected_info_map_mutex_;

  // when client ask for results, they are stored here together with the raw
  // message info.
  PubGidToResultsMap pub_gid_to_results_map_;
  // when a new client is detected, it's added here.
  std::vector<std::string> new_clients_;
  // mutex and cv to notify when new clients or results are available.
  mutable std::mutex notify_mutex_;
  std::condition_variable notify_cv_;

  rclcpp::Subscription<netperf_tool_interfaces::msg::Bytes>::SharedPtr sub_;
  rclcpp::Serialization<netperf_tool_interfaces::msg::Bytes> deserializer_;
  rclcpp::ServiceBase::SharedPtr srv_;
};
}  // namespace netperf_tool
#endif  // SERVER_NODE_HPP_
