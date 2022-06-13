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
struct ServerMessageInfo
{
  /// Timestamp of when the message was received.
  std::chrono::time_point<std::chrono::system_clock> reception_time;
  /// Message ids.
  size_t id;
  /// Single trip latency of the message.
  std::chrono::nanoseconds latency;
  /// Serialized message size.
  size_t serialized_size;
};

/// Raw data with info about received messages.
struct ServerCollectedInfo
{
  /// Collected info of each message.
  std::vector<ServerMessageInfo> message_infos;
};

/// The results of an experiment.
struct ServerResults
{
  /// Statistics, calculated when client makes a request.
  netperf_tool_interfaces::srv::GetResults::Response statistics;
  /// Raw message data collected from the received messages.
  ServerCollectedInfo collected_info;
};

/// Node used as server side of a ros2 netperf experiment.
class ServerNode : public rclcpp::Node
{
public:
  /// Construct.
  /**
   * \param[in] options Node options used to construct the node.
   * \param[in] sub_qos Qos profile to create the publisher used in the experiment.
   */
  explicit ServerNode(
    const rclcpp::NodeOptions & options,
    const rclcpp::QoS & sub_qos);

  /// Callback executed when a message is received.
  /**
   * \param[in] serialized_msg Serialized message received from client.
   * \param[in] msg_info Message info provided by the middleware.
   */
  void handle_msg(
    const rclcpp::SerializedMessage & serialized_msg,
    const rclcpp::MessageInfo & msg_info);

  /// Callback executed when client requests experiment results.
  /**
   * \param[in] req Request made by client.
   * \param[in] rep Response sent to client.
   */
  void
  handle_get_results_request(
    netperf_tool_interfaces::srv::GetResults::Request::SharedPtr req,
    netperf_tool_interfaces::srv::GetResults::Response::SharedPtr rep);

  /// Map from publisher gid to results.
  using PubGidToResultsMap = std::unordered_map<std::string, ServerResults>;

  /// Extract available results of experiments.
  /**
   * This method will clear the currently stored results, so calling it twice in a row
   * will result in empty results the second time.
   */
  PubGidToResultsMap
  extract_results();

  /// Extract publishers gids of new clients found.
  /**
   * This method will clear the currently stored gid, so calling it twice in a row
   * will result in empty results the second time.
   */
  std::vector<std::string>
  extract_new_clients();

  /// Extract until results of an experiment are available or a new client is found.
  void
  wait_for_results_available(std::chrono::nanoseconds timeout);

  /// Get publisher topic name.
  std::string
  get_topic_name();

private:
  /// Map fro publisher gid to raw message data collected.
  using PubGidToMessageData = std::unordered_map<std::string, ServerCollectedInfo>;
  /// Received messages info, agrouped by publisher gid.
  PubGidToMessageData pub_gid_to_collected_info_map_;
  /// Mutex protecting pub_gid_to_collected_info_map_.
  mutable std::mutex pub_gid_to_collected_info_map_mutex_;

  // When client ask for results, they are stored here together with the raw
  // message info.
  PubGidToResultsMap pub_gid_to_results_map_;
  // When a new client is detected, it's added here.
  std::vector<std::string> new_clients_;
  // Mutex and cv to notify when new clients or results are available.
  mutable std::mutex notify_mutex_;
  std::condition_variable notify_cv_;

  /// Subscription that receives messages from the netperf client.
  rclcpp::Subscription<netperf_tool_interfaces::msg::Bytes>::SharedPtr sub_;
  /// Used to deserialize received messages.
  rclcpp::Serialization<netperf_tool_interfaces::msg::Bytes> deserializer_;
  /// Service used to make experiment statistics available to netperf clients.
  rclcpp::ServiceBase::SharedPtr srv_;
};
}  // namespace netperf_tool
#endif  // SERVER_NODE_HPP_
