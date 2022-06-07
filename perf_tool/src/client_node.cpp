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


#include "perf_tool_msgs/msg/bytes.hpp"
#include "perf_tool_msgs/srv/get_results.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "utils.hpp"

namespace perf_tool
{

using namespace std::chrono_literals;

ClientNode::ClientNode(
  size_t array_size,
  std::chrono::nanoseconds target_pub_period,
  const rclcpp::QoS & pub_qos,
  const rclcpp::NodeOptions & options)
: Node("client", "perf_tool", options),
  array_size_{array_size},
  target_pub_period_{target_pub_period}
{
  pub_ = this->create_publisher<perf_tool_msgs::msg::Bytes>("test_topic", pub_qos);
  // always keep the vector preallocated, to avoid delays when the timer is triggered
  bytes_.resize(array_size);
  client_ = this->create_client<perf_tool_msgs::srv::GetResults>("get_results");
}

bool
ClientNode::wait_for_server(std::chrono::nanoseconds timeout)
{
  auto start = std::chrono::system_clock::now();
  while (std::chrono::system_clock::now() < start + timeout) {
    if (client_->service_is_ready()) {
      return true;
    }
    RCLCPP_INFO(this->get_logger(), "perf server not available yet, waiting...");
    std::this_thread::sleep_for(500ms);
  }
  RCLCPP_INFO_STREAM(
    this->get_logger(),
    "perf server not available after waiting ["
      << std::chrono::duration_cast<std::chrono::seconds>(timeout).count() << "s]" << std::endl);
  return false;
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
  if (!pub_->wait_for_all_acked(5s)) {
    RCLCPP_WARN(this->get_logger(), "Some messages were not acked by the perf server ...");
  }
  if (!client_->service_is_ready()) {
    RCLCPP_ERROR(
      this->get_logger(), "Perf server is not available anymore, cannot get the results back");
    return;
  }
  auto req = std::make_shared<perf_tool_msgs::srv::GetResults::Request>();
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
  perf_tool_msgs::msg::Bytes msg;
  rclcpp::SerializedMessage serialized_msg;
  msg.id = next_id;
  auto now = std::chrono::system_clock::now();
  msg.timestamp = now.time_since_epoch().count();
  msg.data = std::move(bytes_);
  serializer_.serialize_message(&msg, &serialized_msg);
  auto sent_bytes = serialized_msg.size();
  pub_->publish(msg);
  // always keep the vector preallocated, to avoid delays when the timer is triggered
  bytes_.resize(array_size_);
  {
    std::lock_guard guard{collected_info_mutex_};
    collected_info_.message_ids.emplace_back(msg.id);
    collected_info_.message_published_times.emplace_back(now);
    collected_info_.message_sizes.emplace_back(sent_bytes);
  }
  ++next_id;
}
}  // namespace perf_tool
