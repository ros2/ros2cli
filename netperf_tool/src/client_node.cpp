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

#include "client_node.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "netperf_tool_interfaces/msg/bytes.hpp"
#include "netperf_tool_interfaces/srv/get_results.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "utils.hpp"

namespace netperf_tool
{

using namespace std::chrono_literals;

ClientNode::ClientNode(
  const rclcpp::NodeOptions & options,
  const rclcpp::QoS & pub_qos,
  size_t array_size,
  std::chrono::nanoseconds target_pub_period,
  std::chrono::nanoseconds server_timeout)
: Node("client", "netperf_tool", options),
  target_pub_period_{target_pub_period},
  server_timeout_{server_timeout}
{
  pub_ = this->create_publisher<netperf_tool_interfaces::msg::Bytes>("test_topic", pub_qos);
  // always keep a message with the data preallocated
  msg_to_publish_.data.resize(array_size);
  msg_to_publish_.id = 0;
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<netperf_tool_interfaces::msg::Bytes> serializer;
  serializer.serialize_message(&msg_to_publish_, &serialized_msg);
  serialized_msg_size_ = serialized_msg.size();
  client_ = this->create_client<netperf_tool_interfaces::srv::GetResults>("get_results");
}

bool
ClientNode::wait_for_server()
{
  return client_->wait_for_service(server_timeout_);
}

void
ClientNode::start_publishing()
{
  timer_ = this->create_wall_timer(
    target_pub_period_,
    std::bind(&ClientNode::pub_next_msg, this));
}

void
ClientNode::stop_publishing()
{
  if (timer_) {
    timer_->cancel();
  }
}

void
ClientNode::sync_with_server(rclcpp::Executor & exec)
{
  if (!pub_->wait_for_all_acked(server_timeout_)) {
    RCLCPP_WARN(this->get_logger(), "Some messages were not acknowledged by the netperf server ...");
  }
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(
      this->get_logger(), "Perf server is not available anymore, cannot get the results back");
    return;
  }
  auto req = std::make_shared<netperf_tool_interfaces::srv::GetResults::Request>();
  req->publisher_gid = this->get_stringified_pub_gid();
  req->messages_total = this->collected_info_.message_ids.size();
  auto future = client_->async_send_request(req);
  exec.spin_until_future_complete(future);
  statistics_ = future.get();
}

ClientResults
ClientNode::extract_results()
{
  ClientResults results;
  if (!statistics_) {
    throw std::runtime_error{"you must first completely run the client before getting results"};
  }
  results.statistics = *statistics_;
  statistics_.reset();
  {
    std::lock_guard guard{collected_info_mutex_};
    results.collected_info = std::move(collected_info_);
  }
  return results;
}

std::string
ClientNode::get_topic_name()
{
  return pub_->get_topic_name();
}

std::string
ClientNode::get_stringified_pub_gid()
{
  return stringify_gid(pub_->get_gid());
}

void
ClientNode::pub_next_msg()
{
  auto now = std::chrono::system_clock::now();
  msg_to_publish_.timestamp = now.time_since_epoch().count();
  pub_->publish(msg_to_publish_);
  // always keep the vector preallocated, to avoid delays when the timer is triggered
  {
    std::lock_guard guard{collected_info_mutex_};
    collected_info_.message_ids.emplace_back(msg_to_publish_.id);
    collected_info_.message_published_times.emplace_back(now);
    collected_info_.message_sizes.emplace_back(serialized_msg_size_);
  }
  ++msg_to_publish_.id;
}
}  // namespace netperf_tool
